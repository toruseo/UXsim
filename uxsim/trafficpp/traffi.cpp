// clang-format off

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <random>
#include <map>
#include <algorithm>
#include <chrono>
#include <climits>
#include <exception>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "traffi.h"

using std::string, std::vector, std::deque, std::pair, std::map, std::unordered_map;
using std::round, std::floor, std::ceil;
using std::cout, std::endl;

// How many queue positions ahead the car-following and RUN passes prefetch a vehicle.
// Vehicles of a queue are scattered in memory, so without it each one stalls the pass on a cache miss.
static constexpr int QUEUE_PREFETCH_AHEAD = 4;

// Thread-local scratch for the outlink filters of route_next_link_choice, which runs in the parallel generate and RUN stages.
static thread_local vector<Link *> tls_route_outlinks;
static thread_local vector<Link *> tls_route_filtered;

// -----------------------------------------------------------------------
// MARK: Node 
// -----------------------------------------------------------------------

/**
 * @brief Create a node.
 * 
 * @param w The world to which the node belongs.
 * @param node_name The name of the node.
 * @param x The x-coordinate of the node.
 * @param y The y-coordinate of the node.
 * @param signal_intervals A list representing the signal at the node.
 * @param signal_offset The offset of the signal.
 */
Node::Node(World *w, const string &node_name, double x, double y, vector<double> signal_intervals, double signal_offset, double flow_capacity, int number_of_lanes)
    : w(w),
      id(w->node_id),
      name(node_name),
      x(x),
      y(y),
      signal_intervals(signal_intervals),
      signal_offset(signal_offset),
      flow_capacity(flow_capacity),
      number_of_lanes(number_of_lanes){
    w->nodes.push_back(this);
    // Per-node RNG stream (kind=1) and stat partial sum, seeded/sized by node id.
    w->node_rngs.push_back(make_entity_rng(w->random_seed, 1u, (unsigned int)id));
    w->node_stat_count.push_back(0.0);
    w->node_id++;
    w->nodes_map[node_name] = this;

    // Signal initialization matching Python's Node.__init__
    signal_phase = 0;
    signal_t = 0;
    double cycle_length = 0;
    for (auto s : this->signal_intervals) cycle_length += s;

    if (this->signal_intervals.size() > 1 || (this->signal_intervals.size() == 1 && this->signal_intervals[0] != 0)){
        double offset = cycle_length - this->signal_offset;
        int i = 0;
        while (true){
            if (offset < this->signal_intervals[i]){
                signal_phase = i;
                signal_t = offset;
                break;
            }
            offset -= this->signal_intervals[i];
            i++;
            if (i >= (int)this->signal_intervals.size()){
                i = 0;
            }
        }
    }

    // Reserve signal_log for expected timesteps to avoid reallocations
    signal_log.reserve(w->total_timesteps + 1);

    // flow capacity initialization (matching Python Node.__init__)
    if (flow_capacity >= 0.0){
        this->flow_capacity_remain = flow_capacity * w->tau * w->delta_n;
        if (number_of_lanes <= 0){
            this->number_of_lanes = (int)std::ceil(flow_capacity / 0.8);
        }
    } else {
        this->flow_capacity = -1.0;
        this->flow_capacity_remain = 10e10;
    }
}

/**
 * @brief Generate a single vehicle if possible.
 */
void Node::generate(){
    if (generation_queue.empty() || out_links.empty()){
        return;
    }

    // Total lanes across all outgoing links determines max generation attempts
    int total_lanes = 0;
    for (auto ol : out_links){
        total_lanes += ol->number_of_lanes;
    }

    for (int i = 0; i < total_lanes; i++){
        if (generation_queue.empty()){
            break;
        }
        Vehicle *veh = generation_queue.front();

        // Choose the next link (generate uses this node's RNG stream)
        veh->route_next_link_choice(out_links, w->node_rngs[id]);

        if (veh->route_next_link == nullptr){
            break;
        }
        Link *outlink = veh->route_next_link;

        // Multi-lane acceptance check
        bool can_accept = false;
        if ((int)outlink->vehicles.size() < outlink->number_of_lanes){
            can_accept = true;
        } else {
            int idx = (int)outlink->vehicles.size() - outlink->number_of_lanes;
            if (outlink->vehicles[idx]->x > outlink->delta_per_lane * w->delta_n){
                can_accept = true;
            }
        }
        if (outlink->capacity_in_remain < w->delta_n){
            can_accept = false;
        }

        if (!can_accept){
            break;
        }

        // Accept vehicle
        generation_queue.pop_front();

        veh->state = vsRUN;
        // Structural parity with Python: a generated vehicle enters VEHICLES_RUNNING.
        w->vehicles_running[veh->id] = veh;
        veh->link = outlink;
        veh->x = 0.0;
        veh->v = outlink->vmax;  // initial speed at link entry (matches Python)
        veh->record_travel_time(nullptr, (double)w->timestep * w->delta_t);

        // Track visited nodes for no_cyclic_routing and link count for specified_route
        const std::size_t required_traveled_nodes_size = w->nodes.size();
        if (veh->_traveled_nodes.size() < required_traveled_nodes_size){
            veh->_traveled_nodes.resize(required_traveled_nodes_size, false);
        }
        veh->_traveled_nodes[outlink->start_node->id] = true;
        veh->_traveled_link_count++;

        // Lane assignment
        if (!outlink->vehicles.empty()){
            veh->lane = (outlink->vehicles.back()->lane + 1) % outlink->number_of_lanes;
        } else {
            veh->lane = 0;
        }

        // Leader-Follower (leader is number_of_lanes positions back)
        veh->leader = nullptr;
        if ((int)outlink->vehicles.size() >= outlink->number_of_lanes){
            int leader_idx = (int)outlink->vehicles.size() - outlink->number_of_lanes;
            veh->leader = outlink->vehicles[leader_idx];
            veh->leader->follower = veh;
        }

        outlink->vehicles.push_back(veh);

        // Cumulative arrival curve
        outlink->arrival_curve[w->timestep] += w->delta_n;
        outlink->capacity_in_remain -= w->delta_n;
    }
}

/**
 * @brief Update the signal state of the node.
 */
void Node::signal_update(){
    // Match Python's signal_control() exactly
    if (signal_intervals.size() > 1){
        if (signal_t > signal_intervals[signal_phase]){
            signal_phase++;
            signal_t = 0;
            if ((size_t)signal_phase >= signal_intervals.size()){
                signal_phase = 0;
            }
        }
        signal_t += w->delta_t;
    }
    signal_log.push_back(signal_phase);
}

/**
 * @brief Automatically determine the flow capacity of this node from the capacities of connected links, matching Python's Node.adjust_node_capacity. Does not override an already-set flow_capacity.
 */
void Node::adjust_node_capacity(){
    if (flow_capacity < 0.0 && !in_links.empty() && !out_links.empty()){
        double capacity_in = in_links[0]->capacity;
        for (auto l : in_links) capacity_in = std::max(capacity_in, l->capacity);
        double capacity_out = out_links[0]->capacity;
        for (auto l : out_links) capacity_out = std::max(capacity_out, l->capacity);
        flow_capacity = (capacity_in + capacity_out) / 2.0;
        flow_capacity_remain = flow_capacity * w->delta_t;
        if (number_of_lanes <= 0){
            number_of_lanes = (int)std::ceil(flow_capacity / 0.8); //the number of lanes is determined by assuming 0.8 veh/s capacity per lane
        }
    }
}

void Node::flow_capacity_update(){
    if (flow_capacity >= 0.0){
        if (flow_capacity_remain < w->delta_n * (number_of_lanes > 0 ? number_of_lanes : 1)){
            flow_capacity_remain += flow_capacity * w->delta_t;
        }
    } else {
        flow_capacity_remain = 10e10;
    }
}

/**
 * @brief Transfer vehicles between links at the node.
 */
void Node::transfer(){
    // Aggregate the per-link arrival buffers (filled by the RUN system pass) into this node's
    // incoming_vehicles. in_links are iterated in a fixed order; sorting by vehicle id below
    // makes the final order independent of the aggregation order, reproducing the id-ordered
    // fill of the original update() loop (which pushed arrivals in vehicle-id order).
    for (auto inlink : in_links){
        for (auto veh : inlink->arrived_vehicles){
            incoming_vehicles.push_back(veh);
        }
        inlink->arrived_vehicles.clear();
    }
    if (incoming_vehicles.size() > 1){
        std::sort(incoming_vehicles.begin(), incoming_vehicles.end(),
                  [](Vehicle *a, Vehicle *b){ return a->id < b->id; });
    }
    // incoming_vehicles_requests mirrors incoming_vehicles (request == veh->route_next_link,
    // set in the RUN system pass and unchanged since). Kept for the observable node member layout.
    for (auto veh : incoming_vehicles){
        incoming_vehicles_requests.push_back(veh->route_next_link);
    }

    // Build outlink list with repetition per lane (multi-lane gets more transfer attempts)
    _buf_outlinks_expanded.clear();
    auto &outlinks_expanded = _buf_outlinks_expanded;
    for (auto veh : incoming_vehicles){
        if (veh->route_next_link != nullptr){
            if (!contains(outlinks_expanded, veh->route_next_link)){
                for (int i = 0; i < veh->route_next_link->number_of_lanes; i++){
                    outlinks_expanded.push_back(veh->route_next_link);
                }
            }
        }
    }

    // Shuffle for fairness (skip if hard_deterministic_mode)
    if (!w->hard_deterministic_mode){
        for (int i = (int)outlinks_expanded.size() - 1; i > 0; i--){
            std::uniform_int_distribution<int> dist(0, i);
            int j = dist(w->node_rngs[id]);
            std::swap(outlinks_expanded[i], outlinks_expanded[j]);
        }
    }

    for (auto outlink : outlinks_expanded){
        // Multi-lane acceptance check
        bool can_accept = false;
        if ((int)outlink->vehicles.size() < outlink->number_of_lanes){
            can_accept = true;
        } else {
            int idx = (int)outlink->vehicles.size() - outlink->number_of_lanes;
            if (outlink->vehicles[idx]->x > outlink->delta_per_lane * w->delta_n){
                can_accept = true;
            }
        }
        if (outlink->capacity_in_remain < w->delta_n){
            can_accept = false;
        }
        if (flow_capacity_remain < w->delta_n){
            can_accept = false;
        }
        if (!can_accept){
            continue;
        }

        // Collect merging vehicles: must be at front of their inlink AND heading to this outlink
        _buf_merging_vehs.clear();
        _buf_merge_priorities.clear();
        auto &merging_vehs = _buf_merging_vehs;
        auto &merge_priorities = _buf_merge_priorities;
        for (auto veh : incoming_vehicles){
            if (veh->route_next_link == outlink &&
                    veh == veh->link->vehicles.front() &&
                    veh->link->capacity_out_remain >= w->delta_n &&
                    (contains(veh->link->signal_group, signal_phase) || signal_intervals.size() <= 1)){
                merging_vehs.push_back(veh);
                merge_priorities.push_back(veh->link ? veh->link->merge_priority : 1.0);
            }
        }
        if (merging_vehs.empty()){
            continue;
        }

        // Pick one vehicle to merge
        Vehicle *chosen_veh = nullptr;
        if (w->hard_deterministic_mode){
            int best_idx = 0;
            for (size_t i = 1; i < merge_priorities.size(); i++){
                if (merge_priorities[i] > merge_priorities[best_idx]){
                    best_idx = i;
                }
            }
            chosen_veh = merging_vehs[best_idx];
        } else {
            chosen_veh = random_choice<Vehicle>(
                merging_vehs,
                [&](size_t i){ return merge_priorities[i]; },
                w->node_rngs[id]);
        }
        if (!chosen_veh){
            continue;
        }

        Link *inlink = chosen_veh->link;

        // Update capacity
        inlink->capacity_out_remain -= w->delta_n;
        outlink->capacity_in_remain -= w->delta_n;
        if (flow_capacity >= 0.0){
            flow_capacity_remain -= w->delta_n;
        }

        // Cumulative curves
        inlink->departure_curve[w->timestep] += w->delta_n;
        outlink->arrival_curve[w->timestep] += w->delta_n;

        // Record traveltime_real event (lazily materialized on read, matching Python's traveltime_actual slice update)
        {
            double current_tt = (double)w->timestep * w->delta_t - chosen_veh->arrival_time_link;
            int start_idx = (int)(chosen_veh->arrival_time_link / w->delta_t);
            if (start_idx < 0) start_idx = 0;
            inlink->traveltime_real_events.emplace_back(start_idx, current_tt);
        }

        chosen_veh->arrival_time_link = (double)w->timestep * w->delta_t;

        // Remove from old link
        inlink->vehicles.pop_front();

        chosen_veh->link = outlink;
        chosen_veh->x = 0.0;

        // Track visited nodes for no_cyclic_routing and link count for specified_route
        const std::size_t required_size = w->nodes.size();
        if (chosen_veh->_traveled_nodes.size() < required_size){
            chosen_veh->_traveled_nodes.resize(required_size, false);
        }
        chosen_veh->_traveled_nodes[outlink->start_node->id] = true;
        chosen_veh->_traveled_link_count++;

        if (chosen_veh->follower){
            chosen_veh->follower->leader = nullptr;
        }
        chosen_veh->follower = nullptr;

        // Lane assignment on new link
        if (!outlink->vehicles.empty()){
            chosen_veh->lane = (outlink->vehicles.back()->lane + 1) % outlink->number_of_lanes;
        } else {
            chosen_veh->lane = 0;
        }

        // Leader-Follower (leader is number_of_lanes positions back in the same lane)
        chosen_veh->leader = nullptr;
        if ((int)outlink->vehicles.size() >= outlink->number_of_lanes){
            int leader_idx = (int)outlink->vehicles.size() - outlink->number_of_lanes;
            chosen_veh->leader = outlink->vehicles[leader_idx];
            chosen_veh->leader->follower = chosen_veh;
        }

        // Move-remain processing: carry residual distance to next link
        double x_next_mr = chosen_veh->move_remain * outlink->vmax / inlink->vmax;
        if (chosen_veh->leader != nullptr){
            double x_cong = chosen_veh->leader->x_old - outlink->delta_per_lane * w->delta_n;
            if (x_cong < chosen_veh->x){
                x_cong = chosen_veh->x;
            }
            if (x_next_mr > x_cong){
                x_next_mr = x_cong;
            }
        }
        if (x_next_mr >= outlink->length){
            x_next_mr = outlink->length;
        }
        if (x_next_mr < 0.0){
            x_next_mr = 0.0;
        }
        chosen_veh->x = x_next_mr;
        chosen_veh->v += chosen_veh->x / w->delta_t;
        chosen_veh->move_remain = 0.0;

        // If the new front vehicle of inlink is waiting for trip end, let it end
        if (!inlink->vehicles.empty() && inlink->vehicles.front()->flag_waiting_for_trip_end){
            inlink->vehicles.front()->end_trip();
        }

        outlink->vehicles.push_back(chosen_veh);

        // Remove from incoming_vehicles
        remove_from_vector(incoming_vehicles, chosen_veh);
    }

    // Process trip-end-waiting vehicles at the front of each inlink
    for (auto inlink : in_links){
        for (int lane = 0; lane < inlink->number_of_lanes; lane++){
            if (!inlink->vehicles.empty() && inlink->vehicles.front()->flag_waiting_for_trip_end){
                inlink->vehicles.front()->end_trip();
            } else {
                break;
            }
        }
    }

    incoming_vehicles.clear();
    incoming_vehicles_requests.clear();
}

// -----------------------------------------------------------------------
// MARK: Link 
// -----------------------------------------------------------------------

/**
 * @brief Create a link.
 * 
 * @param w The world to which the link belongs.
 * @param link_name The name of the link.
 * @param start_node_name The name of the start node.
 * @param end_node_name The name of the end node.
 * @param vmax The free flow speed on the link.
 * @param kappa The jam density on the link.
 * @param length The length of the link.
 * @param merge_priority The priority of the link when merging.
 * @param capacity_out The capacity out of the link.
 * @param signal_group The signal group(s) to which the link belongs.
 */
Link::Link(
    World *w,
    const string &link_name,
    const string &start_node_name,
    const string &end_node_name,
    double vmax,
    double kappa,
    double length,
    int number_of_lanes,
    double merge_priority,
    double capacity_out,
    double capacity_in,
    vector<int> signal_group)
    : w(w),
      length(length),
      vmax(vmax),
      number_of_lanes(number_of_lanes),
      id(w->link_id),
      merge_priority(merge_priority),
      capacity_out(capacity_out),
      capacity_in(capacity_in),
      name(link_name),
      kappa(kappa),
      signal_group(signal_group),
      route_choice_penalty(0.0){

    if (this->kappa <= 0.0){
        this->kappa = 0.2;
    }
    this->delta = 1.0/this->kappa;
    this->delta_per_lane = this->delta * this->number_of_lanes;
    this->tau = w->tau / this->number_of_lanes;

    this->backward_wave_speed = 1/this->tau/this->kappa;
    this->capacity = this->vmax*this->backward_wave_speed*this->kappa/(this->vmax+this->backward_wave_speed);

    if (this->capacity_out < 0.0){
        this->capacity_out = this->capacity * 2;
    }
    this->capacity_out_remain = this->capacity_out*w->delta_t;

    if (this->capacity_in < 0.0){
        this->capacity_in = this->capacity * 2;
        this->capacity_in_remain = this->capacity * 2;  // large initial value = effectively unlimited
    } else {
        this->capacity_in_remain = this->capacity_in * w->delta_t;
    }

    start_node = w->nodes_map[start_node_name];
    end_node = w->nodes_map[end_node_name];

    arrival_curve.resize(w->total_timesteps, 0.0);
    departure_curve.resize(w->total_timesteps, 0.0);

    traveltime_real.resize(w->total_timesteps, this->length / this->vmax);
    traveltime_real_events.clear();
    traveltime_real_events_applied = 0;
    traveltime_instant.resize(w->total_timesteps, 0.0);

    // Insert self into global vectors
    start_node->out_links.push_back(this);
    end_node->in_links.push_back(this);

    w->links.push_back(this);
    // Per-link RNG stream (kind=2) and completed-trip count, seeded/sized by link id.
    w->link_rngs.push_back(make_entity_rng(w->random_seed, 2u, (unsigned int)id));
    w->link_trips_completed.push_back(0.0);
    w->link_id++;
    w->links_map[link_name] = this;
}

/**
 * @brief Update link state and capacity.
 */
void Link::update(){
    set_travel_time();

    if (w->timestep != 0){
        arrival_curve[w->timestep] = arrival_curve[w->timestep-1];
        departure_curve[w->timestep] = departure_curve[w->timestep-1];
    }

    if (capacity_out < 10e9 ){
        if (capacity_out_remain < w->delta_n * number_of_lanes){
            capacity_out_remain += capacity_out*w->delta_t;
        }
    } else {
        capacity_out_remain = 10e9;
    }

    if (capacity_in < 10e9 ){
        if (capacity_in_remain < w->delta_n * number_of_lanes){
            capacity_in_remain += capacity_in*w->delta_t;
        }
    } else {
        capacity_in_remain = 10e9;
    }
}

/**
 * @brief Set travel time based on current traffic conditions.
 */
void Link::change_free_flow_speed(double new_value){
    if (new_value >= 0){
        vmax = new_value;
        backward_wave_speed = 1.0/tau/kappa;
        capacity = vmax*backward_wave_speed*kappa/(vmax+backward_wave_speed);
        delta = 1.0/kappa;
    }
}

void Link::change_jam_density(double new_value){
    if (new_value >= 0){
        kappa = new_value;
        backward_wave_speed = 1.0/tau/kappa;
        capacity = vmax*backward_wave_speed*kappa/(vmax+backward_wave_speed);
        delta = 1.0/kappa;
        delta_per_lane = delta * number_of_lanes;
    }
}

int Link::count_vehicles_in_queue() const {
    int count = 0;
    for (auto veh : vehicles){
        if (veh->v < vmax){
            count++;
        }
    }
    return count;
}

void Link::set_travel_time(){
    // Interval skipping: only compute fresh at intervals, otherwise copy previous value
    if (w->timestep % w->instantaneous_TT_timestep_interval != 0){
        if (w->timestep > 0){
            traveltime_instant[w->timestep] = traveltime_instant[w->timestep - 1];
        }
        return;
    }
    // Instantaneous travel time = length / average speed
    if (!vehicles.empty()){
        double vsum = 0.0;
        for (auto veh : vehicles){
            vsum += veh->v;
        }
        double avg_v = vsum / (double)vehicles.size();
        if (avg_v > 0.0){
            traveltime_instant[w->timestep] = (double)length / avg_v;
        }else{
            traveltime_instant[w->timestep] = (double)length / (vmax / 100.0);
        }
    }else{
        traveltime_instant[w->timestep] = (double)length / (double)vmax;
    }
}

/**
 * @brief Materialize pending traveltime_real events into the array.
 *
 * Link-exit events are buffered as (start_idx, tt) pairs during the main loop (write-only phase) and replayed here on first read.
 * Replay rule: event k fills [s_k, min(s_{k+1}, n)) and the last pending event fills [s_k, n).
 * This is equivalent to the eager behavior where each event overwrites [s_k, n) in order: position i ends up with the value of the last event whose s_k <= i, regardless of start_idx monotonicity.
 */
void Link::ensure_traveltime_real() const {
    size_t n_events = traveltime_real_events.size();
    if (traveltime_real_events_applied >= n_events) return;
    size_t n = traveltime_real.size();
    for (size_t k = traveltime_real_events_applied; k < n_events; k++){
        size_t s = (size_t)traveltime_real_events[k].first; // clamped >= 0 at record time
        double tt = traveltime_real_events[k].second;
        size_t e = n;
        if (k + 1 < n_events){
            size_t s_next = (size_t)traveltime_real_events[k + 1].first;
            if (s_next < n) e = s_next;
        }
        for (size_t idx = s; idx < e; idx++){
            traveltime_real[idx] = tt;
        }
    }
    traveltime_real_events_applied = n_events;
}

double Link::estimate_congestion_externality(int ts){
    ensure_traveltime_real();
    int n = (int)traveltime_real.size();
    double dt = w->delta_t;
    double free_flow_tt = length / vmax * 1.01;

    // Step 1: Find congestion tail (forward scan)
    int ts_end = ts;
    for (int i = ts; i < n; i++){
        double tt_i = traveltime_real[i];
        int future_idx = i + (int)(tt_i / dt);
        if (future_idx >= n) break;
        if (tt_i > free_flow_tt && arrival_curve[future_idx] - departure_curve[future_idx] > 0){
            ts_end = i;
        } else {
            break;
        }
    }
    double veh_count = 0;
    if (ts >= 0 && ts < n && ts_end >= 0 && ts_end < n){
        veh_count = arrival_curve[ts_end] - arrival_curve[ts];
    }

    // Step 2: Find leader headway (backward scan)
    int ts_out = 0;
    if (ts >= 0 && ts < n){
        ts_out = (int)(ts + (int)(traveltime_real[ts] / dt)) + 1;
    }
    if (ts_out >= n) ts_out = n - 1;
    int ts_out_leader = ts_out;
    for (int i = ts_out; i > 0; i--){
        if (i < n && ts_out < n && departure_curve[i] < departure_curve[ts_out]){
            ts_out_leader = i;
            break;
        }
    }

    double headway_consumed = (double)(ts_out - ts_out_leader) * dt;
    return headway_consumed * veh_count;
}

double World::estimate_congestion_externality_route(const vector<Link*> &route, double departure_time){
    double t = departure_time;
    double exts = 0;
    for (auto ln : route){
        ln->ensure_traveltime_real();
        int ts = (int)(t / delta_t);
        exts += ln->estimate_congestion_externality(ts);
        // Advance t by the actual travel time on this link
        int n = (int)ln->traveltime_real.size();
        if (ts >= 0 && ts < n){
            t += ln->traveltime_real[ts];
        } else {
            t += ln->length / ln->vmax;
        }
    }
    return exts;
}

// -----------------------------------------------------------------------
// MARK: Vehicle
// -----------------------------------------------------------------------


/**
 * @brief Create a vehicle.
 * 
 * @param w The world to which the vehicle belongs.
 * @param vehicle_name The name of the vehicle.
 * @param departure_time The departure time of the vehicle.
 * @param orig_name The origin node.
 * @param dest_name The destination node.
 */
Vehicle::Vehicle(
    World *w,
    const string &vehicle_name,
    double departure_time,
    const string &orig_name,
    const string &dest_name)
    : link(nullptr),
      leader(nullptr),
      x(0.0),
      x_next(0.0),
      x_old(0.0),
      v(0.0),
      distance_traveled(0.0),
      state(vsHOME),
      id(w->vehicle_id),
      w(w),
      move_remain(0.0),
      lane(0),
      flag_waiting_for_trip_end(0),
      follower(nullptr),
      dest(nullptr),
      route_next_link(nullptr),
      route_choice_flag_on_link(0),
      trip_abort(1),
      arrival_time_link(0.0),
      name(vehicle_name),
      departure_time(departure_time),
      orig(nullptr),
      arrival_time(-1.0),
      travel_time(-1.0),
      flag_trip_aborted(0),
      route_adaptive(0.0),
      route_choice_uncertainty(0.0),
      route_choice_principle(rcpDUO){
    orig = w->nodes_map[orig_name];
    dest = w->nodes_map[dest_name];

    route_preference_size = (int)w->links.size();
    route_choice_uncertainty = w->route_choice_uncertainty;

    // Incremental tracking for no_cyclic_routing and specified_route
    _traveled_nodes.assign(w->nodes.size(), false);
    _traveled_link_count = 0;

    log_size = 0;
    log_first_ts = -1;
    log_last_ts = -1;
    log_wait_count = 0;
    log_end_count = 0;
    if (w->vehicle_log_mode) {
        // Reserve log arrays: total_timesteps + margin for extra log_data calls
        size_t log_cap = w->total_timesteps + 10;
        log_link.reserve(log_cap);
        log_x.reserve(log_cap);
        log_v.reserve(log_cap);
        log_lane.reserve(log_cap);
        log_s.reserve(log_cap);
    }

    w->vehicles.push_back(this);
    w->vehicle_id++;
    w->vehicles_map[vehicle_name] = this;
    // Structural parity with Python: a newly created vehicle is "living" (home) from birth.
    w->vehicles_living.push_back(this);
    w->vehicles_running.push_back(nullptr);
}

/**
 * @brief Apply Newell's car-following model (computes x_next only).
 *
 * Mirrors uxsim.py Vehicle.carfollow. Runs as an independent pass before the RUN update pass, so
 * the leader is read at its pre-move position (leader->x, unchanged this step): the result is
 * independent of the intra-link processing order and of the OpenMP thread count. The leader is
 * this->leader (the maintained same-link leader pointer, nullptr once the leader leaves).
 */
void Vehicle::car_follow_newell(const World *w){
    // free-flow
    x_next = x + link->vmax * w->delta_t;

    // congested (use delta_per_lane for multi-lane car following)
    if (leader != nullptr){
        double gap = leader->x - link->delta_per_lane * w->delta_n;
        if (gap < x){
            gap = x;
        }
        if (x_next > gap){
            x_next = gap;
        }
    }

    // non-decreasing
    if (x_next < x){
        x_next = x;
    }

    // clamp to link length, carry over residual as move_remain
    if (x_next > link->length){
        move_remain = x_next - link->length;
        x_next = link->length;
    }
}

/**
 * @brief Update the vehicle's state and position (HOME/WAIT/RUN/END state machine).
 *
 * Mirrors uxsim.py Vehicle.update. record_log runs first for every state; the RUN branch then
 * applies the already-computed x_next, and handles trip end / link transfer at the link end.
 * snap_leader (the pre-pass queue-position leader) is forwarded to log_data so the RUN-state
 * leader-spacing reproduces the original id-ordered read even though the RUN pass is per-link.
 */
inline void Vehicle::update(World *w, Vehicle *snap_leader){
    log_data(w, snap_leader);

    if (state == vsHOME){
        // depart when the current time reaches the departure time
        if ((double)w->timestep * w->delta_t >= departure_time){
            state = vsWAIT;
            orig->generation_queue.push_back(this);
        }
    } else if (state == vsWAIT){
        // wait in the vertical queue at the origin node (route pref update is World-level in C++)
    } else if (state == vsRUN){
        // drive within the link
        if (x == 0.0){
            route_choice_flag_on_link = 0;
        }

        v = (x_next - x) / w->delta_t;
        x_old = x;
        x = x_next;

        distance_traveled += x - x_old;

        // at the end of the link
        if (std::fabs(x - link->length) < 1e-9){
            if (link->end_node == dest){
                // prepare for trip end (wait if not at front of link)
                flag_waiting_for_trip_end = 1;
                if (link->vehicles.front() == this){
                    end_trip();
                }
            } else if (link->end_node->out_links.empty() && trip_abort == 1){
                // dead-end: abort trip
                flag_trip_aborted = 1;
                route_next_link = nullptr;
                flag_waiting_for_trip_end = 1;
                if (link->vehicles.front() == this){
                    end_trip();
                }
            } else {
                // request link transfer (RUN-path route choice uses this link's RNG stream; the
                // arrival is staged in this link's buffer, aggregated id-ordered in transfer)
                route_next_link_choice(link->end_node->out_links, w->link_rngs[link->id]);
                link->arrived_vehicles.push_back(this);
            }
        }
    }
    // vsEND / vsABORT: nothing
}

/**
 * @brief End the vehicle's trip.
 */
void Vehicle::end_trip(){
    state = vsEND;
    // Per-link completed-trip counter (end_trip runs in this link's exclusive context).
    w->link_trips_completed[link->id] += 1.0;
    link->departure_curve[w->timestep] += w->delta_n;

    // Record traveltime_real event (lazily materialized on read, matching Python's traveltime_actual slice update)
    {
        double current_tt = ((double)w->timestep + 1.0) * w->delta_t - arrival_time_link;
        int start_idx = (int)(arrival_time_link / w->delta_t);
        if (start_idx < 0) start_idx = 0;
        link->traveltime_real_events.emplace_back(start_idx, current_tt);
    }

    record_travel_time(link, (double)w->timestep * w->delta_t);

    arrival_time = (double)w->timestep * w->delta_t;
    travel_time = arrival_time - departure_time;

    // Structural parity with Python's VEHICLES_RUNNING.pop / VEHICLES_LIVING.pop.
    w->vehicles_running[id] = nullptr;
    w->vehicles_living[id] = nullptr;

    link->vehicles.pop_front();

    if (follower){
        follower->leader = nullptr;
    }
    link = nullptr;
    x = 0.0;
    flag_waiting_for_trip_end = 0;

    if (flag_trip_aborted){
        state = vsABORT;
        arrival_time = -1.0;
        travel_time = -1.0;
    }

    // Final log entry (END/ABORT tail; non-RUN state logs -1s, so no leader spacing needed)
    log_data(w, nullptr);
}

/**
 * @brief Choose the next link based on route choice principle.
 * 
 * @param linkset Available links to choose from.
 */
void Vehicle::route_next_link_choice(const vector<Link*>& linkset, std::mt19937 &rng){
    if (linkset.empty()){
        route_next_link = nullptr;
        route_choice_flag_on_link = 1;
        return;
    }

    // specified_route: force next link from the route plan.
    // Inconsistencies raise the same exceptions as Python: IndexError (out_of_range) when the route is exhausted, ValueError (invalid_argument) when the forced link is not an outgoing link.
    if (!specified_route.empty()){
        int traveled = _traveled_link_count;
        if (traveled >= (int)specified_route.size()){
            throw std::out_of_range("Vehicle " + name + ": specified route has no more links (index " + std::to_string(traveled) + " out of range for route of length " + std::to_string(specified_route.size()) + ")");
        }
        Link *forced = specified_route[traveled];
        if (!contains(linkset, forced)){
            throw std::invalid_argument("Vehicle " + name + ": specified route is inconsistent; link " + forced->name + " is not an outgoing link of the current node");
        }
        route_next_link = forced;
        route_choice_flag_on_link = 1;
        return;
    }

    // Filter by links_preferred (intersection with linkset)
    const vector<Link*> *current = &linkset;
    if (!links_preferred.empty()){
        auto &filtered = tls_route_filtered;
        filtered.clear();
        for (auto ln : *current){
            if (contains(links_preferred, ln)){
                filtered.push_back(ln);
            }
        }
        if (!filtered.empty()){
            tls_route_outlinks.swap(filtered);
            current = &tls_route_outlinks;
        }
    }

    // Filter out links_avoid (subtraction from linkset).
    // As in Python, the subtraction applies whenever any outlink is avoided, even if the result becomes empty; an empty result raises ValueError.
    if (!links_avoid.empty()){
        auto &filtered = tls_route_filtered;
        filtered.clear();
        for (auto ln : *current){
            if (!contains(links_avoid, ln)){
                filtered.push_back(ln);
            }
        }
        if (filtered.size() != current->size()){
            tls_route_outlinks.swap(filtered);
            current = &tls_route_outlinks;
        }
        if (current->empty()){
            throw std::invalid_argument("Vehicle " + name + ": links_avoid excludes all outgoing links of the current node");
        }
    }

    // Filter out links leading to already-traveled nodes (no_cyclic_routing)
    if (w->no_cyclic_routing){
        auto &filtered = tls_route_filtered;
        filtered.clear();
        for (auto ln : *current){
            auto end_node_id = ln->end_node->id;
            if (static_cast<size_t>(end_node_id) >= _traveled_nodes.size() || !_traveled_nodes[end_node_id]){
                filtered.push_back(ln);
            }
        }
        if (!filtered.empty()){
            tls_route_outlinks.swap(filtered);
            current = &tls_route_outlinks;
        }
    }

    const vector<Link*> &outlinks = *current;
    if (outlinks.empty()){
        route_next_link = nullptr;
        route_choice_flag_on_link = 1;
        return;
    }

    // Preference weights (O(1) vector access by link id)
    const double *pref = w->route_preference[dest->id].data();

    if (w->hard_deterministic_mode){
        // Pick link with highest preference
        int best_idx = 0;
        for (size_t i = 1; i < outlinks.size(); i++){
            if (pref[outlinks[i]->id] > pref[outlinks[best_idx]->id]){
                best_idx = i;
            }
        }
        route_next_link = outlinks[best_idx];
    } else {
        route_next_link = random_choice<Link>(
            outlinks,
            [&](size_t i){ return pref[outlinks[i]->id]; },
            rng);
    }
    route_choice_flag_on_link = 1;
}

void Vehicle::enforce_route(vector<Link*> route){
    specified_route = route;
    // For backward compatibility, also set links_preferred
    links_preferred = route;
}

/**
 * @brief Record travel time for the link.
 *
 * @param link The link to record travel time for.
 * @param t The current time.
 */
void Vehicle::record_travel_time(Link *link, double t){
    if (link != nullptr){
        link->traveltime_t.push_back(t);
        link->traveltime_tt.push_back(t - arrival_time_link);
    }
    arrival_time_link = t;
}

/**
 * @brief Append one entry to the vehicle log.
 * Uses reserve'd arrays — push_back is O(1) with no reallocation.
 * log_size tracks actual count (vector::size() is equivalent but log_size avoids any ambiguity if resize was used elsewhere).
 */
void Vehicle::log_append(Vehicle *snap_leader){
    if (log_size == 0) log_first_ts = (int)w->timestep;
    log_last_ts = (int)w->timestep;
    if (state == vsWAIT) log_wait_count++;
    if (state == vsEND || state == vsABORT) log_end_count++;
    if (state == vsRUN){
        // Leader spacing reproduces the original id-ordered read even though the RUN pass is
        // per-link front-to-back. snap_leader is the pre-pass queue-position leader: a lower-id
        // leader was updated before self, so self saw its post-move x (or, if it end_trip()'d,
        // no leader -> -1); a higher-id leader had not yet moved when self logged in id order,
        // so self saw its pre-move x (x_old). (Python's record_log reads s.leader.x - s.x.)
        double spacing = -1.0;
        if (snap_leader != nullptr){
            if (snap_leader->id < id){
                if (snap_leader->state != vsEND && snap_leader->state != vsABORT){
                    spacing = snap_leader->x - x;
                }
            } else {
                spacing = snap_leader->x_old - x;
            }
        }
        log_link.push_back(link ? link->id : -1);
        log_x.push_back(x);
        log_v.push_back(v);
        log_lane.push_back(lane);
        log_s.push_back(spacing);
    } else {
        log_link.push_back(-1);
        log_x.push_back(-1);
        log_v.push_back(-1);
        log_lane.push_back(-1);
        log_s.push_back(-1.0);
    }
    log_size++;
}

/**
 * @brief Count the statistic sample of this step and append to the vehicle log when logging is on.
 */
inline void Vehicle::log_data(World *w, Vehicle *snap_leader){
    // Accumulate stats for print_simple_results (regardless of log mode).
    // A waiting vehicle counts as a zero-speed sample for the average speed statistic, as in Python's record_log.
    if (state == vsRUN){
        // RUN samples accumulate on the current link's partial sums.
        if (link){
            link->ave_v_sum += v;
            link->ave_vratio_sum += v / link->vmax;
            link->stat_count += 1.0;
        }
    } else if (state == vsWAIT){
        // WAIT samples accumulate on the origin node (where the vehicle queues).
        w->node_stat_count[orig->id] += 1.0;
    }

    if (w->vehicle_log_mode) log_append(snap_leader);
}

/**
 * @brief Reconstruct log_t entry i.
 * Bit-identical to the eager version: entries run one-per-timestep from log_first_ts, except the final END/ABORT tail (recorded by end_trip()) which, on the update() path, duplicates the timestep of the immediately preceding RUN entry (log_last_ts).
 * Uses the same (double)ts * delta_t expression as the eager push.
 */
double Vehicle::log_t_at(size_t i) const {
    size_t tail = log_size - (size_t)log_end_count;
    long long ts = (i < tail) ? (long long)log_first_ts + (long long)i
                              : (long long)log_last_ts;
    return (double)ts * w->delta_t;
}

/**
 * @brief Reconstruct log_state entry i.
 * Log structure is always: HOME x1, WAIT x log_wait_count, RUN x r, [END|ABORT] x log_end_count.
 */
int Vehicle::log_state_at(size_t i) const {
    if (log_end_count > 0 && i >= log_size - (size_t)log_end_count) return state;
    if (i == 0) return vsHOME;
    if (i < (size_t)(1 + log_wait_count)) return vsWAIT;
    return vsRUN;
}

/**
 * @brief Return departure_time in seconds.
 * Note: In C++, departure_time is already stored in seconds.
 */
double Vehicle::departure_time_in_second() const {
    return departure_time;
}





// -----------------------------------------------------------------------
// MARK: World
// -----------------------------------------------------------------------

/**
 * @brief Create a World (simulation environment).
 * 
 * @param world_name The name of the world.
 * @param t_max The simulation duration.
 * @param delta_n The platoon size.
 * @param tau The reaction time.
 * @param duo_update_time The time interval for route choice update.
 * @param duo_update_weight The update weight for route choice.
 * @param route_choice_uncertainty The noise in route choice.
 * @param print_mode Whether print the simulation progress or not.
 * @param random_seed The random seed.
 * @param vehicle_log_mode Whether save vehicle data or not.
 */
World::World(
    const string &world_name,
    double t_max,
    double delta_n,
    double tau,
    double duo_update_time,
    double duo_update_weight,
    double route_choice_uncertainty,
    int print_mode,
    long long random_seed,
    bool vehicle_log_mode,
    bool hard_deterministic_mode,
    bool route_choice_update_gradual,
    bool no_cyclic_routing)
    : timestamp(std::chrono::high_resolution_clock::now().time_since_epoch().count()),
      name(world_name),
      t_max(t_max),
      delta_n(delta_n),
      tau(tau),
      duo_update_time(duo_update_time),
      duo_update_weight(duo_update_weight),
      print_mode(print_mode),
      hard_deterministic_mode(hard_deterministic_mode),
      route_choice_update_gradual(route_choice_update_gradual),
      no_cyclic_routing(no_cyclic_routing),
      delta_t(tau * delta_n),
      total_timesteps((int)(t_max / (tau * delta_n))),
      timestep_for_route_update((int)(duo_update_time / (tau * delta_n))),
      time(0),
      node_id(0),
      link_id(0),
      vehicle_id(0),
      timestep(0),
      route_choice_uncertainty(route_choice_uncertainty),
      random_seed(random_seed),
      vehicle_log_reserve_size(0),
      vehicle_log_mode(vehicle_log_mode),
      ave_v(0.0),
      ave_vratio(0.0),
      trips_total(0.0),
      trips_completed(0.0),
      flag_initialized(false),
      writer(&std::cout){
    // The route update interval has a minimum of 1 timestep, like Python's DELTAT_ROUTE.
    // Without the clamp, duo_update_time < delta_t would yield 0 and disable route updates entirely.
    if (timestep_for_route_update == 0){
        timestep_for_route_update = 1;
    }
    show_progress = 1;
    show_progress_deltat_timestep = (size_t)(600.0 / delta_t);  // Python World default: show_progress_deltat=600
    sim_start_time = std::chrono::steady_clock::now();
}

World::~World(){
    // Reset writer to avoid calling into Python stdout during destruction
    writer = &std::cout;
    for (auto veh : vehicles) delete veh;
    for (auto ln : links) delete ln;
    for (auto nd : nodes) delete nd;
    vehicles.clear();
    links.clear();
    nodes.clear();
    nodes_map.clear();
    links_map.clear();
    vehicles_map.clear();
    vehicles_living.clear();
    vehicles_running.clear();
}

void World::set_t_max(double new_t_max){
    t_max = new_t_max;
    total_timesteps = (size_t)(t_max / delta_t);
    for (auto ln : links){
        ln->arrival_curve.resize(total_timesteps, 0.0);
        ln->departure_curve.resize(total_timesteps, 0.0);
        ln->traveltime_real.resize(total_timesteps, ln->length / ln->vmax);
        ln->traveltime_instant.resize(total_timesteps, 0.0);
    }
}

void World::initialize_adj_matrix(){
    if (flag_initialized==false){
        if (adjust_node_capacity){
            for (auto nd : nodes){
                nd->adjust_node_capacity();
            }
        }
        adj_mat.resize(node_id, vector<int>(node_id, 0));
        adj_mat_time.resize(node_id, vector<double>(node_id, 0.0));
        for (auto ln : links){
            int i = ln->start_node->id;
            int j = ln->end_node->id;
            adj_mat[i][j] = 1;
            adj_mat_time[i][j] = ln->length / ln->vmax;
        }

        route_preference.assign(nodes.size(), vector<double>(links.size(), 0.0));
        route_pref_active.assign(nodes.size(), 0);
        flag_initialized = true;
    }
}

void World::update_adj_time_matrix(){
    double noise = route_choice_uncertainty;
    // Reset the cells written by links (running-average state).
    // Matches Python's route_search_all, which rebuilds adj_mat_time from a zero matrix each call.
    if (adj_mat_link_count.size() != nodes.size()){
        adj_mat_link_count.assign(nodes.size(), vector<int>(nodes.size(), 0));
    }
    for (auto ln : links){
        adj_mat_time[ln->start_node->id][ln->end_node->id] = 0.0;
        adj_mat_link_count[ln->start_node->id][ln->end_node->id] = 0;
    }
    for (auto ln : links){
        int i = ln->start_node->id;
        int j = ln->end_node->id;
        double tt = ln->traveltime_instant[timestep];
        if (tt <= 0.0){
            tt = ln->length / ln->vmax;
        }
        // Use dynamic toll timeseries if available, otherwise fall back to static penalty
        double penalty = ln->route_choice_penalty;
        if (!ln->toll_timeseries.empty() && timestep < (int)ln->toll_timeseries.size()){
            penalty = ln->toll_timeseries[timestep];
        }
        double new_link_tt;
        if (hard_deterministic_mode){
            new_link_tt = tt + penalty;
        } else {
            double noise_factor = random_range_float64(1.0, 1.0 + noise, link_rngs[ln->id]);
            new_link_tt = tt * noise_factor + penalty;
        }
        // If there are multiple links between the same nodes, average the travel time
        double n = (double)adj_mat_link_count[i][j];
        adj_mat_time[i][j] = adj_mat_time[i][j] * n / (n + 1.0) + new_link_tt / (n + 1.0);
        adj_mat_link_count[i][j] += 1;
        // If the inflow is prohibited, travel time is assumed to be infinite
        if (ln->capacity_in == 0.0){
            adj_mat_time[i][j] = std::numeric_limits<double>::infinity();
        }
    }
}

int World::resolve_num_threads() const {
#ifdef _OPENMP
    if (num_threads == -1) return omp_get_max_threads();
    return num_threads;
#else
    return 1;
#endif
}

// Queue entry of the route search, ordered by (distance, node id).
struct RouteQueueEntry {
    double d;
    int node;
};

static inline bool route_queue_less(const RouteQueueEntry &a, const RouteQueueEntry &b){
    return a.d < b.d || (a.d == b.d && a.node < b.node);
}

// Comparator of the heap functions, which build a max-heap, so that the heap pops the smallest entry.
struct RouteQueueLater {
    bool operator()(const RouteQueueEntry &a, const RouteQueueEntry &b) const {
        return route_queue_less(b, a);
    }
};

void World::route_search_all(const vector<vector<double>> &adj, double infty) {
    int nsize = (int)adj.size();
    if (std::fabs(infty) < 1e-9) {
        infty = std::numeric_limits<double>::infinity();
    }

    // Build the flat adjacency into reused scratch buffers.
    // The `adj[i][j] > 0.0` edge filter runs on every call, so positive cells that are not links act as edges.
    rsa_arc_start.resize(nsize + 1);
    rsa_arcs.clear();
    for (int i = 0; i < nsize; i++) {
        rsa_arc_start[i] = rsa_arcs.size();
        const auto &adji = adj[i];
        for (int j = 0; j < nsize; j++) {
            if (adji[j] > 0.0) {
                rsa_arcs.push_back({adji[j], j});
            }
        }
    }
    rsa_arc_start[nsize] = rsa_arcs.size();
    const size_t *arc_start = rsa_arc_start.data();
    const RouteArc *arcs = rsa_arcs.data();

    // Reset dist / next-hop buffers to infty / -1.
    rsa_dist.resize(nsize);
    rsa_next.resize(nsize);
    for (int i = 0; i < nsize; i++) {
        rsa_dist[i].assign(nsize, infty);
        rsa_next[i].assign(nsize, -1);
    }

    // Dijkstra from each source node.
    // Each source writes only its own rsa_dist[start] / rsa_next[start] rows and the adjacency is read-only here, so the source loop is parallel-safe with a per-thread queue.
    // The result is independent of thread count.
    #pragma omp parallel num_threads(resolve_num_threads())
    {
        // Every arc pushes at most once and the source adds one entry, so the buffer cannot overflow.
        vector<RouteQueueEntry> queue(rsa_arcs.size() + 1);
        RouteQueueEntry *q = queue.data();
        #pragma omp for schedule(static)
        for (int start = 0; start < nsize; start++) {
            double *dstart = rsa_dist[start].data();
            int *nhstart = rsa_next[start].data();
            dstart[start] = 0.0;
            nhstart[start] = start;

            // q[head..tail) holds the live entries in ascending (distance, node id) order, which is also a valid binary min-heap.
            // Road networks push in nearly ascending order, so inserting from the tail moves few entries.
            // When the entries searched and moved exceed their budget of 1024 plus 32 per insertion, the same array continues as a heap for the rest of this source.
            size_t head = 0;
            size_t tail = 1;
            q[0] = {0.0, start};
            bool sorted = true;
            long long shift_credit = 1024;

            while (head < tail) {
                RouteQueueEntry top;
                if (sorted) {
                    top = q[head++];
                } else {
                    std::pop_heap(q + head, q + tail, RouteQueueLater());
                    top = q[--tail];
                    // A push needs a strict improvement, so only the newest entry of a node matches dstart.
                    if (top.d > dstart[top.node]) continue;
                }
                int current = top.node;
                double dcur = top.d;
                int hop = nhstart[current];

                // Explore neighbors via the flat adjacency
                for (size_t e = arc_start[current]; e < arc_start[current + 1]; e++) {
                    int next = arcs[e].to;
                    double new_dist = dcur + arcs[e].weight;
                    double old_dist = dstart[next];
                    if (new_dist < old_dist) {
                        dstart[next] = new_dist;
                        // Update next hop
                        nhstart[next] = (current == start) ? next : hop;
                        RouteQueueEntry entry{new_dist, next};
                        if (sorted) {
                            // Distances pop in ascending order, so a settled node cannot improve and a node reached earlier is still in the array.
                            // Its entry is replaced, which keeps one entry per node in the array.
                            size_t i;
                            if (old_dist == infty) {
                                i = tail++;
                            } else {
                                i = tail - 1;
                                while (q[i].node != next) i--;
                            }
                            while (i > head && route_queue_less(entry, q[i - 1])) {
                                q[i] = q[i - 1];
                                i--;
                            }
                            q[i] = entry;
                            // tail - 1 - i counts the entries searched and the entries moved.
                            shift_credit += 32 - (long long)(tail - 1 - i);
                            if (shift_credit < 0) sorted = false;
                        } else {
                            q[tail++] = entry;
                            std::push_heap(q + head, q + tail, RouteQueueLater());
                        }
                    }
                }
            }
        }
    }
}

map<pair<int,int>, vector<vector<int>>>
World::enumerate_k_random_routes_cpp(int k, unsigned int seed){
    int nsize = node_id;

    // Build node_pair → link_id lookup (last link wins for multi-link pairs, matching Python)
    vector<vector<int>> node_pair_link(nsize, vector<int>(nsize, -1));
    for (auto ln : links){
        node_pair_link[ln->start_node->id][ln->end_node->id] = ln->id;
    }

    map<pair<int,int>, vector<vector<int>>> dict_routes;
    std::mt19937 local_rng(seed);
    std::uniform_real_distribution<double> rdist(0.0, 100.0);

    int counter = 0;
    int iteration = 0;
    double average_n_routes_old = 0;

    // Weighted adjacency matrix, allocated once and reused.
    // Every iteration overwrites exactly the same (i,j) link cells; non-link cells stay 0.0, so reuse is bit-identical to reallocating a zero-filled matrix each time.
    vector<vector<double>> adj(nsize, vector<double>(nsize, 0.0));

    while (true){
        // Build weighted adj matrix (free-flow on iter 0, random on subsequent)
        for (auto ln : links){
            int i = ln->start_node->id;
            int j = ln->end_node->id;
            double base_weight = ln->length / ln->vmax;
            if (iteration == 0){
                adj[i][j] = base_weight;
            } else {
                adj[i][j] = base_weight * rdist(local_rng);
            }
        }

        // All-pairs Dijkstra (results in rsa_next scratch buffer)
        route_search_all(adj, 0.0);
        const auto &next_hop = rsa_next;

        // Reconstruct paths and collect unique routes
        int total_routes = 0;
        int total_pairs = 0;
        for (int src = 0; src < nsize; src++){
            for (int dst = 0; dst < nsize; dst++){
                if (src == dst) continue;
                if (next_hop[src][dst] == -1) continue;

                auto &routes = dict_routes[{src, dst}];
                total_pairs++;
                total_routes += (int)routes.size();

                if ((int)routes.size() >= k) continue;

                // Reconstruct path as link IDs via next_hop chain
                vector<int> path;
                int cur = src;
                bool valid = true;
                while (cur != dst){
                    int nxt = next_hop[cur][dst];
                    if (nxt == -1 || nxt == cur){ valid = false; break; }
                    int lid = node_pair_link[cur][nxt];
                    if (lid == -1){ valid = false; break; }
                    path.push_back(lid);
                    cur = nxt;
                }

                if (!valid || path.empty()) continue;

                // Check uniqueness
                bool duplicate = false;
                for (auto &existing : routes){
                    if (existing == path){ duplicate = true; break; }
                }
                if (!duplicate){
                    routes.push_back(path);
                }
            }
        }

        // Convergence check (matches Python: average routes per OD pair)
        double average_n_routes = total_pairs > 0 ?
            (double)total_routes / (double)total_pairs : 0.0;
        if (average_n_routes == average_n_routes_old){
            counter++;
        } else {
            counter = 0;
            average_n_routes_old = average_n_routes;
        }

        iteration++;
        if (counter > 10) break;
    }

    return dict_routes;
}

void World::duo_update_preference(double weight){
    // Per-destination: route_preference[k] and route_pref_active[k] are exclusive to dest k and route_next is read-only here.
    // So the destination loop is parallel-safe and independent of thread count.
    int nnodes = (int)nodes.size();
    int nlinks = (int)links.size();
    vector<const int *> start_rows(nlinks);
    vector<int> end_ids(nlinks);
    for (int l = 0; l < nlinks; l++){
        start_rows[l] = route_next[links[l]->start_node->id].data();
        end_ids[l] = links[l]->end_node->id;
    }
    #pragma omp parallel for schedule(static) num_threads(resolve_num_threads())
    for (int k = 0; k < nnodes; k++){
        auto &pref = route_preference[k];

        // psum==0.0 iff route_preference[k] has never been reinforced.
        // Tracked incrementally via route_pref_active[k] to avoid an O(links) sum here.
        double w_tmp = weight;
        if (!route_pref_active[k]){
            w_tmp = 1; //initialize with deterministic shortest path
        }

        // For each link in the world, update preference
        bool reinforced = false;
        for (int lid = 0; lid < nlinks; lid++){
            if (start_rows[lid][k] == end_ids[lid]){
                pref[lid] = (1.0 - w_tmp) * pref[lid] + w_tmp;
                reinforced = true;
            }else{
                pref[lid] = (1.0 - w_tmp) * pref[lid];
            }
        }
        // A reinforced on-path link makes route_preference[k] strictly positive.
        if (reinforced) route_pref_active[k] = 1;
    }
}

/**
 * @brief Update route choice using dynamic user optimum.
 */
void World::route_choice_duo(){
    duo_update_preference(duo_update_weight);
}

void World::route_choice_duo_gradual(){
    // Gradual DUO update: scaled weight applied every timestep
    double weight0 = duo_update_weight * (delta_t / duo_update_time);
    duo_update_preference(weight0);
}

void World::print_scenario_stats(){
    if (print_mode == 1){
        (*writer) << "Scenario statistics:\n";
        (*writer) << "    duration: " << t_max << " s\n";
        (*writer) << "    timesteps: " << total_timesteps << "\n";
        (*writer) << "    nodes: " << nodes.size() << "\n";
        (*writer) << "    links: " << links.size() << "\n";
        (*writer) << "    vehicles: " << (int)vehicles.size() * (int)delta_n << " veh\n";
        (*writer) << "    platoon size: " << delta_n << " veh\n";
        (*writer) << "    platoons: " << vehicles.size() << "\n";
        (*writer) << "    vehicles: " << (double)vehicles.size() * delta_n << " veh\n";
    }
}

// Id-ordered reductions of the per-entity statistic partial sums. Summation is over index
// (== entity id) in ascending order so repeated reads are bit-identical.
double World::ave_v_sum() const {
    double s = 0.0;
    for (auto ln : links) s += ln->ave_v_sum;
    return s;
}
double World::ave_vratio_sum() const {
    double s = 0.0;
    for (auto ln : links) s += ln->ave_vratio_sum;
    return s;
}
double World::stat_sample_count() const {
    double s = 0.0;
    for (auto ln : links) s += ln->stat_count;
    for (size_t i = 0; i < node_stat_count.size(); i++) s += node_stat_count[i];
    return s;
}
double World::trips_completed_count() const {
    double s = 0.0;
    for (size_t i = 0; i < link_trips_completed.size(); i++) s += link_trips_completed[i];
    return s;
}

void World::print_simple_results(){
    // Compute from incrementally accumulated stats (no log scan needed)
    trips_total = (double)vehicles.size() * delta_n;
    trips_completed = trips_completed_count() * delta_n;
    double n_samples = stat_sample_count();
    if (n_samples > 0){
        ave_v = ave_v_sum() / n_samples;
        ave_vratio = ave_vratio_sum() / n_samples;
    }

    (*writer) << "Stats:\n";
    (*writer) << "    Average speed: " << ave_v << "\n";
    (*writer) << "    Average speed ratio: " << ave_vratio << "\n";
    (*writer) << "    Trips completion: "
              << trips_completed << " / " << trips_total << "\n";
}

void World::reset_sim_start_time(){
    sim_start_time = std::chrono::steady_clock::now();
}

// Print one progress line in the same format and semantics as Python's Analyzer.show_simulation_progress
void World::print_progress_line(double t_print){
    double platoon_count = 0;
    double v_sum = 0;
    for (auto ln : links){
        platoon_count += (double)ln->vehicles.size();
        for (auto veh : ln->vehicles){
            v_sum += veh->v;
        }
    }
    double sum_vehs = platoon_count * delta_n;
    double avev = platoon_count > 0 ? v_sum / platoon_count : 0;
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - sim_start_time).count();
    char buf[128];
    snprintf(buf, sizeof(buf), "%8.0f s| %8.0f vehs|  %4.1f m/s| %8.2f s", t_print, sum_vehs, avev, elapsed);
    (*writer) << buf << endl;
}

// -----------------------------------------------------------------------
//MARK: mainloop
// -----------------------------------------------------------------------

/**
 * @brief Main simulation loop.
 * 
 * @param duration_t Duration to run simulation.
 * @param until_t Time to run simulation until.
 */
void World::main_loop(double duration_t=-1, double until_t=-1){
    int start_ts, end_ts;
    start_ts = timestep;

    if (duration_t < 0 && until_t < 0){
        end_ts = total_timesteps;
    } else if (duration_t >= 0 && until_t < 0){
        end_ts = static_cast<size_t>(floor((duration_t+time)/delta_t)) + 1;
    } else if (duration_t < 0 && until_t >= 0){
        end_ts = static_cast<size_t>(floor(until_t/delta_t)) + 1;
    } else {
        throw std::runtime_error("Cannot specify both `duration_t` and `until_t` parameters for `World.main_loop`");
    }

    if (end_ts > total_timesteps){
        end_ts = total_timesteps;
    }
    if (end_ts <= start_ts){
        return;
    }

    // Bring the HOME departure buckets up to date.
    // Until Python writes Vehicle::state, only the HOME pass moves a vehicle out of HOME, so a call keeps the buckets and adds only the vehicles created since the previous call.
    // After such a write, each call rebuilds the buckets from the states at its entry.
    // The first call allocates the buckets, and a call rebuilds them when set_t_max has changed the horizon.
    // ts0 is the smallest timestep with (double)ts0*delta_t >= departure_time, then clamped to the chunk start (past-due vehicles depart on the first step of the chunk).
    if (vehicle_state_overridden || departure_buckets.size() != total_timesteps){
        departure_buckets.assign(total_timesteps, {});
        departures_scanned = 0;
    }
    for (; departures_scanned < vehicles.size(); departures_scanned++){
        Vehicle *veh = vehicles[departures_scanned];
        if (veh->state == vsHOME){
            int ts0 = (int)(veh->departure_time / delta_t);
            while ((double)ts0 * delta_t < veh->departure_time) ts0++;
            if (ts0 < start_ts) ts0 = start_ts;
            if (ts0 < total_timesteps) departure_buckets[ts0].push_back(veh);
        }
    }

    for (timestep = start_ts; timestep < end_ts; timestep++){
        time = timestep*delta_t;

        // Progress header and initial line at t=0 (same as Python's exec_simulation)
        if (print_mode == 1 && timestep == 0){
            (*writer) << "      time| # of vehicles| ave speed| computation time" << endl;
            print_progress_line(time);
        }

        int nlinks = (int)links.size();
        int nnodes = (int)nodes.size();

        // All entity-exclusive timestep stages run inside a single parallel region to keep the
        // OpenMP setup/barrier overhead low (one region per timestep instead of one per stage;
        // this keeps the 1-thread path lean). Each `omp for` has an implicit barrier, so the
        // stage order (update -> generate -> transfer -> RUN -> WAIT) and its data dependencies
        // are preserved. The serial transfer runs in an `omp single`. Exceptions thrown by
        // route_next_link_choice (specified_route) are captured per stage (smallest entity id
        // wins, for determinism) and rethrown after the region, since an exception must never
        // cross the OpenMP boundary.
        std::exception_ptr gen_exc; int gen_exc_id = INT_MAX;
        std::exception_ptr run_exc; int run_exc_id = INT_MAX;
        #pragma omp parallel num_threads(resolve_num_threads())
        {
        // Link updates (per-link): Link::update() touches only its own time-indexed arrays
        // and capacities, so it is entity-exclusive and independent of thread count.
        #pragma omp for schedule(static)
        for (int i = 0; i < nlinks; i++){
            links[i]->update();
        }

        // Node generate + signal + capacity (per-node): a node's out_links are its exclusive
        // property (a link has a single start_node), and generate() uses this node's RNG
        // stream; signal/capacity touch only this node.
        #pragma omp for schedule(static)
        for (int i = 0; i < nnodes; i++){
            Node *nd = nodes[i];
            try {
                nd->generate();
            } catch (...) {
                #pragma omp critical (aos_exc)
                { if (nd->id < gen_exc_id){ gen_exc_id = nd->id; gen_exc = std::current_exception(); } }
            }
            nd->signal_update();
            nd->flow_capacity_update();
        }

        // Node transfer: serial. Serial id-ordered node processing has a sequential visibility
        // semantics (a node's acceptance decisions are seen by later nodes through the shared
        // out-link queues), matching Python; parallelizing it would break bit identity.
        #pragma omp single
        {
            for (auto nd : nodes){
                nd->transfer();
            }
        }

        // Car-following pass (per-link): an independent sweep that computes each running
        // vehicle's x_next (Vehicle::car_follow_newell) before any movement, matching uxsim.py's
        // separate carfollow loop. Because no vehicle has moved yet, each leader is read at its
        // pre-move position, so the result is independent of intra-link order and thread count.
        // A vehicle is on exactly one link, and its leader is on the same link, so per-link is
        // exclusive. A separate omp for (barrier before the RUN pass) guarantees every x_next is
        // computed before the RUN pass starts applying movement.
        #pragma omp for schedule(static)
        for (int li = 0; li < nlinks; li++){
            Link *lk = links[li];
            Vehicle **queue = lk->vehicles.begin();
            int n = (int)lk->vehicles.size();
            for (int i = 0; i < n; i++){
                if (i + QUEUE_PREFETCH_AHEAD < n) UXSIM_PREFETCH(queue[i + QUEUE_PREFETCH_AHEAD]);
                queue[i]->car_follow_newell(this);
            }
        }

        // RUN update pass (per-link): a front-to-back sweep over each link's queue that calls Vehicle::update() (RUN branch: apply x_next, log, handle trip end / transfer).
        // Parallel per-link: each link touches only its own queue, its own vehicles' state and logs, its own curves/traveltime events/arrived buffers, its own per-link stat sums and RNG stream; the result is independent of thread count.
        // Only Node::generate and Node::transfer push to a link's queue, so during this pass end_trip() only advances the queue's front and the slots from `queue` on keep the queue as it was at the start of the pass.
        // snap_leader is the pre-pass queue-position leader (lower index = ahead), forwarded so log_data reproduces the id-ordered spacing read.
        // route_next_link_choice may throw (specified_route); capture and rethrow the smallest-link-id exception after the region.
        #pragma omp for schedule(static)
        for (int li = 0; li < nlinks; li++){
          Link *lk = links[li];
          try {
            Vehicle **queue = lk->vehicles.begin();
            int lanes = lk->number_of_lanes;
            int n = (int)lk->vehicles.size();
            for (int i = 0; i < n; i++){
                if (i + QUEUE_PREFETCH_AHEAD < n) UXSIM_PREFETCH(queue[i + QUEUE_PREFETCH_AHEAD]);
                Vehicle *veh = queue[i];
                Vehicle *snap_leader = (i - lanes >= 0) ? queue[i - lanes] : nullptr;
                veh->update(this, snap_leader);
            }
          } catch (...) {
            #pragma omp critical (aos_exc)
            { if (lk->id < run_exc_id){ run_exc_id = lk->id; run_exc = std::current_exception(); } }
          }
        }

        // WAIT system pass (per-node): vehicles still in a generation_queue (those not
        // generated this step) get update() -> log_data() (WAIT branch). A queued vehicle's
        // origin is this node, so its WAIT stat sample (node_stat_count[orig->id]) and per-vehicle
        // log are exclusive to this node; the pass is thread-count independent. Vehicles departing
        // this step are logged as HOME below and enter the queue only afterwards, so no
        // double-logging.
        // Without logging, update() of a WAIT vehicle only adds 1.0 to a sample count.
        // The counts hold integers and are only read as a sum, so adding the queue size to this node's count gives the same sum.
        #pragma omp for schedule(static)
        for (int i = 0; i < nnodes; i++){
            Node *nd = nodes[i];
            if (vehicle_log_mode || vehicle_state_overridden){
                for (auto veh : nd->generation_queue){
                    veh->update(this);
                }
            } else {
                node_stat_count[i] += (double)nd->generation_queue.size();
            }
        }
        } // end omp parallel region
        // Rethrow the deterministic (smallest-id) captured exception, if any.
        if (gen_exc) std::rethrow_exception(gen_exc);
        if (run_exc) std::rethrow_exception(run_exc);

        // HOME departure pass (serial): vehicles whose departure timestep is now get update()
        // -> log_data() (HOME entry) and the HOME branch transitions them to WAIT and appends
        // them to their origin generation_queue (id ascending, since `vehicles` is in id order).
        // Run after the WAIT pass so a vehicle departing this step is not logged twice.
        for (auto veh : departure_buckets[timestep]){
            veh->update(this);
        }

        // route choice update
        if (route_choice_update_gradual){
            // Gradual mode: route_search at DELTAT_ROUTE intervals, DUO_update every step
            if (timestep_for_route_update > 0 && timestep % timestep_for_route_update == 0){
                update_adj_time_matrix();
                route_search_all(adj_mat_time, 0.0);
                route_dist_record[timestep] = std::move(rsa_dist);
                route_next.swap(rsa_next);
            }
            // DUO update every step with scaled weight
            route_choice_duo_gradual();
        } else {
            // Normal mode: both route_search and DUO_update at DELTAT_ROUTE intervals
            if (timestep_for_route_update > 0 && timestep % timestep_for_route_update == 0){
                update_adj_time_matrix();
                route_search_all(adj_mat_time, 0.0);
                route_dist_record[timestep] = std::move(rsa_dist);
                route_next.swap(rsa_next);
                route_choice_duo();
            }
        }

        // Print progress at show_progress_deltat intervals (same as Python's exec_simulation)
        if (print_mode == 1 && show_progress == 1 && show_progress_deltat_timestep > 0 && timestep > 0 && timestep % show_progress_deltat_timestep == 0){
            print_progress_line(time);
        }
    }

    // Final progress line when the simulation reaches its end (same as Python's exec_simulation)
    if (print_mode == 1 && show_progress == 1 && timestep >= total_timesteps){
        print_progress_line(timestep * delta_t);
    }
}


bool World::check_simulation_ongoing(){
    if (timestep < total_timesteps){
        return true;
    } else {
        return false;
    }
}

// -----------------------------------------------------------------------
// MARK: World utils
// -----------------------------------------------------------------------

Node *World::get_node(const string &node_name){
    auto it = nodes_map.find(node_name);
    if (it != nodes_map.end()){
        return it->second;
    }
    (*writer) << "Error at function get_node(): `"
              << node_name << "` not found\n";
    throw std::runtime_error("get_node() error");
}

Link *World::get_link(const string &link_name){
    auto it = links_map.find(link_name);
    if (it != links_map.end()){
        return it->second;
    }
    (*writer) << "Error at function get_link(): `"
              << link_name << "` not found\n";
    throw std::runtime_error("get_link() error");
}


Vehicle *World::get_vehicle_by_index(int index) const {
    if (index >= 0 && index < static_cast<int>(vehicles.size())) {
        return vehicles[index];
    }
    throw std::out_of_range("get_vehicle_by_index: index out of range");
}

// for some reason, this was defined outside of World
inline void add_demand(
        World *w,
        const string &orig_name,
        const string &dest_name,
        double start_t,
        double end_t,
        double flow,
        vector<string> links_preferred_str = {}){
    // Iterate over integer timesteps, mirroring Python's adddemand: `for t in range(int(t_start/DELTAT), int(t_end/DELTAT))`.
    // This keeps the iteration count and the timestep-quantized departure times identical to Python even when t_start/t_end are not multiples of delta_t.
    int ts_start = (int)(start_t / w->delta_t);
    int ts_end = (int)(end_t / w->delta_t);
    double demand = 0.0;
    for (int ts = ts_start; ts < ts_end; ts++){
        demand += flow * w->delta_t;
        while (demand >= (double)w->delta_n){
            // create new vehicle
            Vehicle *v = new Vehicle(
                w,
                std::to_string(w->vehicle_id),
                (double)ts * w->delta_t,
                orig_name,
                dest_name);

            for (auto ln_str : links_preferred_str){
                v->links_preferred.push_back(w->links_map[ln_str]);
            }

            demand -= (double)w->delta_n;
        }
    }
}

/**
 * @brief Return all vehicle states as (name, state_int) pairs.
 */
std::vector<std::pair<std::string, int>> World::get_all_vehicle_states() const {
    std::vector<std::pair<std::string, int>> result;
    result.reserve(vehicles.size());
    for (auto *v : vehicles) {
        int effective_state = v->state;
        if (effective_state == vsEND && (v->flag_trip_aborted || (v->arrival_time < 0 && v->travel_time <= 0))) {
            effective_state = vsABORT;
        }
        result.emplace_back(v->name, effective_state);
    }
    return result;
}

/**
 * @brief Build enter_log data for all vehicles in one pass.
 * Returns (link_id, time, vehicle_index) for each link-entry event.
 * Avoids per-vehicle Python property access and list creation.
 */
std::vector<World::EnterLogEntry> World::build_enter_log_data() const {
    std::vector<EnterLogEntry> result;

    for (size_t vi = 0; vi < vehicles.size(); vi++) {
        const Vehicle *v = vehicles[vi];
        // Scan log_link for link transitions (same logic as build_full_log's log_t_link)
        int prev_link_id = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            int lid = v->log_link[i];
            if (lid >= 0 && lid != prev_link_id) {
                double t = v->log_t_at(i);
                result.push_back({lid, t, static_cast<int>(vi)});
                prev_link_id = lid;
            }
        }
    }

    return result;
}

/**
 * @brief Compact version of build_all_vehicle_logs_flat.
 * No home prepend — only actual log entries (log_size per vehicle).
 * Returns n_missing array so Python can reconstruct home entries if needed.
 */
World::CompactFlatLogs World::build_all_vehicle_logs_flat_compact() const {
    CompactFlatLogs fl;
    size_t nv = vehicles.size();
    fl.offsets.resize(nv + 1);
    fl.ltl_offsets.resize(nv + 1);
    fl.n_missing.resize(nv);

    // First pass: compute total sizes
    size_t total_log = 0;
    size_t total_ltl = 0;
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        int n_missing = 0;
        if (v->log_size > 0 && v->log_t_at(0) > 0) {
            n_missing = static_cast<int>(v->log_t_at(0) / delta_t);
        }
        fl.n_missing[vi] = n_missing;
        fl.offsets[vi] = total_log;
        total_log += v->log_size;

        // log_t_link count
        size_t ltl_count = 1;  // home entry
        int prev_lid = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            int lid = v->log_link[i];
            if (lid >= 0 && lid != prev_lid) {
                ltl_count++;
                prev_lid = lid;
            }
        }
        if (v->state == vsEND) ltl_count++;
        fl.ltl_offsets[vi] = total_ltl;
        total_ltl += ltl_count;
    }
    fl.offsets[nv] = total_log;
    fl.ltl_offsets[nv] = total_ltl;

    // Allocate
    fl.log_t.resize(total_log);
    fl.log_x.resize(total_log);
    fl.log_v.resize(total_log);
    fl.log_state.resize(total_log);
    fl.log_s.resize(total_log);
    fl.log_lane.resize(total_log);
    fl.log_link.resize(total_log);
    fl.ltl_t.resize(total_ltl);
    fl.ltl_id.resize(total_ltl);

    // Second pass: fill arrays (no home prepend)
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        size_t base = fl.offsets[vi];

        for (size_t i = 0; i < v->log_size; i++) {
            size_t idx = base + i;
            fl.log_t[idx] = v->log_t_at(i);
            int st = v->log_state_at(i);
            fl.log_state[idx] = st;
            bool is_run = (st == vsRUN);
            fl.log_x[idx] = is_run ? v->log_x[i] : -1;
            fl.log_v[idx] = is_run ? v->log_v[i] : -1;
            fl.log_s[idx] = v->log_s[i];
            fl.log_lane[idx] = v->log_lane[i];
            fl.log_link[idx] = v->log_link[i];
        }

        // log_t_link
        size_t ltl_base = fl.ltl_offsets[vi];
        size_t ltl_idx = 0;
        fl.ltl_t[ltl_base] = v->departure_time;
        fl.ltl_id[ltl_base] = Vehicle::LOG_T_LINK_HOME;
        ltl_idx++;

        int prev_lid = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            int lid = v->log_link[i];
            if (lid >= 0 && lid != prev_lid) {
                fl.ltl_t[ltl_base + ltl_idx] = v->log_t_at(i);
                fl.ltl_id[ltl_base + ltl_idx] = lid;
                ltl_idx++;
                prev_lid = lid;
            }
        }
        if (v->state == vsEND) {
            fl.ltl_t[ltl_base + ltl_idx] = (v->arrival_time >= 0) ? v->arrival_time : -1.0;
            fl.ltl_id[ltl_base + ltl_idx] = Vehicle::LOG_T_LINK_END;
            ltl_idx++;
        }
    }

    return fl;
}

