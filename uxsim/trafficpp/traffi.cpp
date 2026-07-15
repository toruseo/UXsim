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
#include <queue>
#include <climits>
#include <exception>

#include "traffi.h"

using std::string, std::vector, std::deque, std::pair, std::map, std::unordered_map;
using std::round, std::floor, std::ceil;
using std::cout, std::endl;

// Thread-local scratch for the RUN pass front-to-back queue snapshot. The RUN stage is
// parallelized per-link, so each thread needs its own snapshot buffer; a single main_loop
// local would race. Reused across links/timesteps to keep its capacity.
static thread_local vector<Vehicle *> tls_run_snapshot;

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
    _buf_seen_outlinks.clear();
    auto &outlinks_expanded = _buf_outlinks_expanded;
    for (auto veh : incoming_vehicles){
        if (veh->route_next_link != nullptr){
            if (_buf_seen_outlinks.count(veh->route_next_link) == 0){
                _buf_seen_outlinks.insert(veh->route_next_link);
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
                merge_priorities,
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
      id(w->link_id),
      name(link_name),
      length(length),
      number_of_lanes(number_of_lanes),
      vmax(vmax),
      kappa(kappa),
      merge_priority(merge_priority),
      capacity_out(capacity_out),
      capacity_in(capacity_in),
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
    // Per-link RNG stream (kind=2) and stat partial sums, seeded/sized by link id.
    w->link_rngs.push_back(make_entity_rng(w->random_seed, 2u, (unsigned int)id));
    w->link_ave_v_sum.push_back(0.0);
    w->link_ave_vratio_sum.push_back(0.0);
    w->link_stat_count.push_back(0.0);
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
    : w(w),
      id(w->vehicle_id),
      name(vehicle_name),
      departure_time(departure_time),
      orig(nullptr),
      dest(nullptr),
      link(nullptr),
      x(0.0),
      x_next(0.0),
      x_old(0.0),
      v(0.0),
      move_remain(0.0),
      lane(0),
      leader(nullptr),
      follower(nullptr),
      state(vsHOME),
      arrival_time(-1.0),
      travel_time(-1.0),
      arrival_time_link(0.0),
      route_next_link(nullptr),
      route_choice_flag_on_link(0),
      route_choice_principle(rcpDUO),
      route_adaptive(0.0),
      route_choice_uncertainty(0.0),
      flag_waiting_for_trip_end(0),
      flag_trip_aborted(0),
      trip_abort(1),
      distance_traveled(0.0){
    orig = w->nodes_map[orig_name];
    dest = w->nodes_map[dest_name];

    // Initialize link preference (vector indexed by link_id)
    route_preference.assign(w->links.size(), 0.0);
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

    // Final log entry
    log_data();
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
    _buf_outlinks.clear();
    _buf_outlinks.insert(_buf_outlinks.end(), linkset.begin(), linkset.end());
    auto &outlinks = _buf_outlinks;
    if (!links_preferred.empty()){
        _buf_filtered.clear();
        for (auto ln : outlinks){
            if (contains(links_preferred, ln)){
                _buf_filtered.push_back(ln);
            }
        }
        if (!_buf_filtered.empty()){
            outlinks.swap(_buf_filtered);
        }
    }

    // Filter out links_avoid (subtraction from linkset).
    // As in Python, the subtraction applies whenever any outlink is avoided, even if the result becomes empty; an empty result raises ValueError.
    if (!links_avoid.empty()){
        _buf_filtered.clear();
        for (auto ln : outlinks){
            if (!contains(links_avoid, ln)){
                _buf_filtered.push_back(ln);
            }
        }
        if (_buf_filtered.size() != outlinks.size()){
            outlinks.swap(_buf_filtered);
        }
        if (outlinks.empty()){
            throw std::invalid_argument("Vehicle " + name + ": links_avoid excludes all outgoing links of the current node");
        }
    }

    // Filter out links leading to already-traveled nodes (no_cyclic_routing)
    if (w->no_cyclic_routing){
        _buf_filtered.clear();
        for (auto ln : outlinks){
            auto end_node_id = ln->end_node->id;
            if (static_cast<size_t>(end_node_id) >= _traveled_nodes.size() || !_traveled_nodes[end_node_id]){
                _buf_filtered.push_back(ln);
            }
        }
        if (!_buf_filtered.empty()){
            outlinks.swap(_buf_filtered);
        }
    }

    if (outlinks.empty()){
        route_next_link = nullptr;
        route_choice_flag_on_link = 1;
        return;
    }

    // Build preference weights (O(1) vector access by link id)
    _buf_outlink_pref.clear();
    auto &outlink_pref = _buf_outlink_pref;
    for (auto ln : outlinks){
        outlink_pref.push_back(w->route_preference[dest->id][ln->id]);
    }

    if (w->hard_deterministic_mode){
        // Pick link with highest preference
        int best_idx = 0;
        for (size_t i = 1; i < outlink_pref.size(); i++){
            if (outlink_pref[i] > outlink_pref[best_idx]){
                best_idx = i;
            }
        }
        route_next_link = outlinks[best_idx];
    } else {
        route_next_link = random_choice<Link>(
            outlinks,
            outlink_pref,
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
 * @brief Log vehicle data for analysis.
 * Uses reserve'd arrays — push_back is O(1) with no reallocation.
 * log_size tracks actual count (vector::size() is equivalent but log_size avoids any ambiguity if resize was used elsewhere).
 */
void Vehicle::log_data(){
    // Accumulate stats for print_simple_results (regardless of log mode).
    // A waiting vehicle counts as a zero-speed sample for the average speed statistic, as in Python's record_log.
    if (state == vsRUN){
        // RUN samples accumulate on the current link's partial sums.
        if (link){
            w->link_ave_v_sum[link->id] += v;
            w->link_ave_vratio_sum[link->id] += v / link->vmax;
            w->link_stat_count[link->id] += 1.0;
        }
    } else if (state == vsWAIT){
        // WAIT samples accumulate on the origin node (where the vehicle queues).
        w->node_stat_count[orig->id] += 1.0;
    }

    if (w->vehicle_log_mode){
        if (log_size == 0) log_first_ts = (int)w->timestep;
        log_last_ts = (int)w->timestep;
        if (state == vsWAIT) log_wait_count++;
        if (state == vsEND || state == vsABORT) log_end_count++;
        if (state == vsRUN){
            log_link.push_back(link ? link->id : -1);
            log_x.push_back(x);
            log_v.push_back(v);
            log_lane.push_back(lane);
            log_s.push_back((leader != nullptr && leader->link == link) ? leader->x - x : -1.0);
        } else {
            log_link.push_back(-1);
            log_x.push_back(-1);
            log_v.push_back(-1);
            log_lane.push_back(-1);
            log_s.push_back(-1.0);
        }
        log_size++;
    }
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

void World::route_search_all(const vector<vector<double>> &adj, double infty) {
    int nsize = (int)adj.size();
    if (std::fabs(infty) < 1e-9) {
        infty = std::numeric_limits<double>::infinity();
    }

    // Build adjacency list into reused scratch buffer.
    // The `adj[i][j] > 0.0` edge filter is re-evaluated every call so the neighbour set/order is identical to the previous allocate-fresh implementation (bit-identical).
    rsa_adj_list.resize(nsize);
    for (int i = 0; i < nsize; i++) {
        auto &row = rsa_adj_list[i];
        row.clear();
        const auto &adji = adj[i];
        for (int j = 0; j < nsize; j++) {
            if (adji[j] > 0.0) {
                row.push_back({j, adji[j]});
            }
        }
    }

    // Reuse dist / next-hop scratch buffers (reset to infty / -1 each call).
    rsa_dist.resize(nsize);
    rsa_next.resize(nsize);
    for (int i = 0; i < nsize; i++) {
        rsa_dist[i].assign(nsize, infty);
        rsa_next[i].assign(nsize, -1);
    }

    using pdi = pair<double, int>;

    // Dijkstra from each source node. Each source writes only its own rsa_dist[start] /
    // rsa_next[start] rows, and rsa_adj_list is read-only here, so the source loop is
    // parallel-safe with per-thread visited flags and priority queue. The result is
    // independent of thread count.
    #pragma omp parallel
    {
        vector<char> visited(nsize);
        std::priority_queue<pdi, vector<pdi>, std::greater<pdi>> pq;
        #pragma omp for schedule(static)
        for (int start = 0; start < nsize; start++) {
            std::fill(visited.begin(), visited.end(), (char)0);
            auto &dstart = rsa_dist[start];
            auto &nhstart = rsa_next[start];
            dstart[start] = 0.0;
            nhstart[start] = start;
            pq.push({0.0, start});

            while (!pq.empty()) {
                auto [d, current] = pq.top();
                pq.pop();

                if (visited[current]) continue;
                visited[current] = 1;

                // Explore neighbors via adjacency list
                double dcur = dstart[current];
                for (const auto& [next, weight] : rsa_adj_list[current]) {
                    double new_dist = dcur + weight;
                    if (new_dist < dstart[next]) {
                        dstart[next] = new_dist;
                        // Update next hop
                        nhstart[next] = (current == start) ? next : nhstart[current];
                        pq.push({new_dist, next});
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

/**
 * @brief Update route choice using dynamic user optimum.
 */
void World::route_choice_duo(){
    // Per-destination: route_preference[k] and route_pref_active[k] are exclusive to dest k;
    // route_next and links are read-only here, so the destination loop is parallel-safe and
    // independent of thread count.
    int nnodes = (int)nodes.size();
    #pragma omp parallel for schedule(static)
    for (int di = 0; di < nnodes; di++){
        Node *dest = nodes[di];
        int k = dest->id;

        // psum==0.0 iff route_preference[k] has never been reinforced.
        // Tracked incrementally via route_pref_active[k] to avoid an O(links) sum here.
        auto duo_update_weight_tmp = duo_update_weight;
        if (!route_pref_active[k]){
            duo_update_weight_tmp = 1; //initialize with deterministic shortest path
        }

        // For each link in the world, update preference
        bool reinforced = false;
        for (auto ln : links){
            int i = ln->start_node->id;
            int j = ln->end_node->id;
            int lid = ln->id;
            if (route_next[i][k] == j){
                route_preference[k][lid] = (1.0 - duo_update_weight_tmp) * route_preference[k][lid] + duo_update_weight_tmp;
                reinforced = true;
            }else{
                route_preference[k][lid] = (1.0 - duo_update_weight_tmp) * route_preference[k][lid];
            }
        }
        // A reinforced on-path link makes route_preference[k] strictly positive.
        if (reinforced) route_pref_active[k] = 1;
    }
}

void World::route_choice_duo_gradual(){
    // Gradual DUO update: scaled weight applied every timestep
    double weight0 = duo_update_weight * (delta_t / duo_update_time);

    // Per-destination and thread-count independent (see route_choice_duo).
    int nnodes = (int)nodes.size();
    #pragma omp parallel for schedule(static)
    for (int di = 0; di < nnodes; di++){
        Node *dest = nodes[di];
        int k = dest->id;

        // psum==0.0 iff route_preference[k] has never been reinforced (see route_choice_duo).
        // Tracked via route_pref_active[k].
        double w_tmp = weight0;
        if (!route_pref_active[k]){
            w_tmp = 1;
        }

        bool reinforced = false;
        for (auto ln : links){
            int i = ln->start_node->id;
            int j = ln->end_node->id;
            int lid = ln->id;
            if (route_next[i][k] == j){
                route_preference[k][lid] = (1.0 - w_tmp) * route_preference[k][lid] + w_tmp;
                reinforced = true;
            }else{
                route_preference[k][lid] = (1.0 - w_tmp) * route_preference[k][lid];
            }
        }
        if (reinforced) route_pref_active[k] = 1;
    }
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
    for (size_t i = 0; i < link_ave_v_sum.size(); i++) s += link_ave_v_sum[i];
    return s;
}
double World::ave_vratio_sum() const {
    double s = 0.0;
    for (size_t i = 0; i < link_ave_vratio_sum.size(); i++) s += link_ave_vratio_sum[i];
    return s;
}
double World::stat_sample_count() const {
    double s = 0.0;
    for (size_t i = 0; i < link_stat_count.size(); i++) s += link_stat_count[i];
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

    // Rebuild HOME departure buckets: departure_buckets[ts] holds the HOME vehicles (id
    // ascending, since `vehicles` is in id order) whose first departing timestep is ts. ts0
    // is the smallest timestep with (double)ts0*delta_t >= departure_time, using the exact
    // same FP comparison as the original per-step HOME predicate, then clamped to the chunk
    // start (past-due vehicles depart on the first step of the chunk).
    departure_buckets.assign(total_timesteps, {});
    for (auto veh : vehicles){
        if (veh->state == vsHOME){
            int ts0 = (int)(veh->departure_time / delta_t);
            while ((double)ts0 * delta_t < veh->departure_time) ts0++;
            if (ts0 < start_ts) ts0 = start_ts;
            if (ts0 < (int)total_timesteps) departure_buckets[ts0].push_back(veh);
        }
    }

    // Scratch for the RUN pass (a stable front-to-back snapshot of a link's queue, since
    // end_trip() pops from the live deque during the pass) lives in a thread_local vector
    // (tls_run_snapshot) because the RUN stage is parallelized per-link.

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
        #pragma omp parallel
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

        // RUN system pass: a per-link, front-to-back sweep over each link's queue snapshot
        // that fuses the old car_follow_newell() and update() RUN branch. Parallel per-link: a
        // vehicle is on exactly one link, so each link touches only its own queue, its own
        // vehicles' state and logs, its own curves/traveltime events/arrived buffers, its own
        // per-link stat sums and RNG stream; the result is independent of thread count.
        // Leaders are ahead in the queue (lower index), so they are moved before their
        // followers, matching the old two-phase (car_follow-all then update-all) reads via
        // x_old. Per-link stat sums are hoisted to locals and flushed once after the sweep.
        // route_next_link_choice may throw (specified_route); capture and rethrow the
        // smallest-link-id exception after the region.
        #pragma omp for schedule(static)
        for (int li = 0; li < nlinks; li++){
          Link *lk = links[li];
          try {
            if (lk->vehicles.empty()) continue;
            vector<Vehicle *> &run_snapshot = tls_run_snapshot;
            run_snapshot.assign(lk->vehicles.begin(), lk->vehicles.end());
            int lanes = lk->number_of_lanes;
            double dpl_dn = lk->delta_per_lane * delta_n;
            double vmax_dt = lk->vmax * delta_t;
            double length = lk->length;
            double vmax = lk->vmax;
            int n = (int)run_snapshot.size();

            double ave_v_sum_local = 0.0;
            double ave_vratio_sum_local = 0.0;
            double stat_count_local = 0.0;

            for (int i = 0; i < n; i++){
                Vehicle *veh = run_snapshot[i];
                // Leader from the pre-pass queue snapshot: a leader that end_trip()'d earlier
                // in this sweep is still readable here (its x_old is untouched by end_trip).
                Vehicle *leader = (i - lanes >= 0) ? run_snapshot[i - lanes] : nullptr;

                // car_follow (Newell): self uses its current x, the leader uses x_old (its
                // pre-move position this step, already stored since it moved first).
                double self_x = veh->x;
                double xn = self_x + vmax_dt;
                if (leader){
                    double gap = leader->x_old - dpl_dn;
                    if (gap < self_x) gap = self_x;
                    if (xn > gap) xn = gap;
                }
                if (xn < self_x) xn = self_x;
                if (xn > length){
                    veh->move_remain = xn - length;
                    xn = length;
                }
                veh->x_next = xn;

                // log_data (RUN): spacing to the leader reproduces the id-ordered read of the
                // original update() loop. A lower-id leader moved before self, so self saw its
                // post-move x (or, if it end_trip()'d, no leader -> -1); a higher-id leader had
                // not moved when self logged, so self saw its pre-move x (x_old).
                double spacing = -1.0;
                if (leader){
                    if (leader->id < veh->id){
                        if (leader->state != vsEND && leader->state != vsABORT){
                            spacing = leader->x - self_x;
                        }
                    } else {
                        spacing = leader->x_old - self_x;
                    }
                }
                // Inline the RUN stat accumulation into link-local sums (flushed once below);
                // the log push is unchanged from log_data_run().
                ave_v_sum_local += veh->v;
                ave_vratio_sum_local += veh->v / vmax;
                stat_count_local += 1.0;
                if (vehicle_log_mode){
                    if (veh->log_size == 0) veh->log_first_ts = (int)timestep;
                    veh->log_last_ts = (int)timestep;
                    veh->log_link.push_back(lk->id);
                    veh->log_x.push_back(self_x);
                    veh->log_v.push_back(veh->v);
                    veh->log_lane.push_back(veh->lane);
                    veh->log_s.push_back(spacing);
                    veh->log_size++;
                }

                // move + link-end handling (identical to the old update() RUN branch).
                if (self_x == 0.0){
                    veh->route_choice_flag_on_link = 0;
                }
                veh->v = (xn - self_x) / delta_t;
                veh->x_old = self_x;
                veh->x = xn;
                veh->distance_traveled += xn - self_x;

                if (std::fabs(xn - length) < 1e-9){
                    if (lk->end_node == veh->dest){
                        veh->flag_waiting_for_trip_end = 1;
                        if (lk->vehicles.front() == veh){
                            veh->end_trip();
                        }
                    } else if (lk->end_node->out_links.empty() && veh->trip_abort == 1){
                        veh->flag_trip_aborted = 1;
                        veh->route_next_link = nullptr;
                        veh->flag_waiting_for_trip_end = 1;
                        if (lk->vehicles.front() == veh){
                            veh->end_trip();
                        }
                    } else {
                        veh->route_next_link_choice(lk->end_node->out_links, link_rngs[lk->id]);
                        lk->arrived_vehicles.push_back(veh);
                    }
                }
            }

            link_ave_v_sum[lk->id] += ave_v_sum_local;
            link_ave_vratio_sum[lk->id] += ave_vratio_sum_local;
            link_stat_count[lk->id] += stat_count_local;
          } catch (...) {
            #pragma omp critical (aos_exc)
            { if (lk->id < run_exc_id){ run_exc_id = lk->id; run_exc = std::current_exception(); } }
          }
        }

        // WAIT system pass (per-node): vehicles still in a generation_queue (those not
        // generated this step) log a WAIT entry. A queued vehicle's origin is this node, so its
        // WAIT stat sample (node_stat_count[orig->id]) and per-vehicle log are exclusive to
        // this node; the pass is thread-count independent. Vehicles departing this step are
        // logged as HOME below and enter the queue only afterwards, so no double-logging.
        #pragma omp for schedule(static)
        for (int i = 0; i < nnodes; i++){
            for (auto veh : nodes[i]->generation_queue){
                veh->log_data();
            }
        }
        } // end omp parallel region
        // Rethrow the deterministic (smallest-id) captured exception, if any.
        if (gen_exc) std::rethrow_exception(gen_exc);
        if (run_exc) std::rethrow_exception(run_exc);

        // HOME departure pass: vehicles whose departure timestep is now log a HOME entry,
        // transition to WAIT, and enter their origin generation_queue (id ascending).
        if (timestep < total_timesteps){
            for (auto veh : departure_buckets[timestep]){
                veh->log_data();
                veh->state = vsWAIT;
                veh->orig->generation_queue.push_back(veh);
            }
        }

        // route choice update
        if (route_choice_update_gradual){
            // Gradual mode: route_search at DELTAT_ROUTE intervals, DUO_update every step
            if (timestep_for_route_update > 0 && timestep % timestep_for_route_update == 0){
                update_adj_time_matrix();
                route_search_all(adj_mat_time, 0.0);
                route_dist = rsa_dist;
                route_next = rsa_next;
                route_dist_record[timestep] = route_dist;
            }
            // DUO update every step with scaled weight
            route_choice_duo_gradual();
        } else {
            // Normal mode: both route_search and DUO_update at DELTAT_ROUTE intervals
            if (timestep_for_route_update > 0 && timestep % timestep_for_route_update == 0){
                update_adj_time_matrix();
                route_search_all(adj_mat_time, 0.0);
                route_dist = rsa_dist;
                route_next = rsa_next;
                route_dist_record[timestep] = route_dist;
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
 * Efficient bulk query so Python can update VEHICLES_LIVING/RUNNING without calling state on each vehicle individually.
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

