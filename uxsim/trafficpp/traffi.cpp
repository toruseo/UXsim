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
#include <cstring>

#include "traffi.h"

using std::string, std::vector, std::deque, std::pair, std::map, std::unordered_map;
using std::round, std::floor, std::ceil;
using std::cout, std::endl;

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

        // Choose the next link
        veh->route_next_link_choice(out_links);

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
        veh->_traveled_nodes[outlink->start_node->id] = true;
        veh->_traveled_link_count++;

        w->vehicles_running[veh->id] = veh;

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
            int j = dist(w->rng);
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
                w->rng);
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

        // Update traveltime_real array (slice update matching Python's traveltime_actual)
        {
            double current_tt = (double)w->timestep * w->delta_t - chosen_veh->arrival_time_link;
            int start_idx = (int)(chosen_veh->arrival_time_link / w->delta_t);
            if (start_idx < 0) start_idx = 0;
            for (size_t idx = start_idx; idx < inlink->traveltime_real.size(); idx++){
                inlink->traveltime_real[idx] = current_tt;
            }
        }

        chosen_veh->arrival_time_link = (double)w->timestep * w->delta_t;

        // Remove from old link
        inlink->vehicles.pop_front();

        // Accumulate distance traveled (full link traversal)
        chosen_veh->distance_traveled += inlink->length;

        chosen_veh->link = outlink;
        chosen_veh->x = 0.0;

        // Track visited nodes for no_cyclic_routing and link count for specified_route
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
    traveltime_instant.resize(w->total_timesteps, 0.0);

    // Insert self into global vectors
    start_node->out_links.push_back(this);
    end_node->in_links.push_back(this);

    w->links.push_back(this);
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
        if (avg_v > vmax / 100.0){
            traveltime_instant[w->timestep] = (double)length / avg_v;
        }else{
            traveltime_instant[w->timestep] = (double)length / (vmax / 100.0);
        }
    }else{
        traveltime_instant[w->timestep] = (double)length / (double)vmax;
    }
}

double Link::estimate_congestion_externality(int ts){
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
      active_index(-1),
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
    if (w->vehicle_log_mode) {
        // Reserve log array: total_timesteps + margin for extra log_data calls
        size_t log_cap = w->total_timesteps + 10;
        log_entries.reserve(log_cap);
    }

    active_index = (int)w->active_vehicles.size();
    w->vehicles.push_back(this);
    w->active_vehicles.push_back(this);
    w->vehicles_living[id] = this;
    w->vehicle_id++;
    w->vehicles_map[vehicle_name] = this;
}

/**
 * @brief Update vehicle state and movement.
 */
void Vehicle::update(){

    if (state == vsHOME){
        if ((double)w->timestep * w->delta_t >= departure_time){
            log_data();
            state = vsWAIT;
            orig->generation_queue.push_back(this);
        }
    }else if (state == vsWAIT){
        log_data();
    }else if (state == vsRUN){
        log_data();
        if (x == 0.0){
            route_choice_flag_on_link = 0;
        }

        v = (x_next - x) / (w->delta_t);
        x_old = x;
        x = x_next;

        if (std::fabs(x - link->length) < 1e-9){
            if (link->end_node == dest){
                // Prepare for trip end (wait if not at front of link)
                flag_waiting_for_trip_end = 1;
                if (link->vehicles.front() == this){
                    end_trip();
                    log_data();
                }
            }else if (link->end_node->out_links.empty() && trip_abort == 1){
                // Dead-end: abort trip
                flag_trip_aborted = 1;
                route_next_link = nullptr;
                flag_waiting_for_trip_end = 1;
                if (link->vehicles.front() == this){
                    end_trip();
                    log_data();
                }
            }else{
                route_next_link_choice(link->end_node->out_links);
                link->end_node->incoming_vehicles.push_back(this);
                link->end_node->incoming_vehicles_requests.push_back(route_next_link);
            }
        }
    }else if (state == vsEND || state == vsABORT){
        // do nothing
    }
}

/**
 * @brief End the vehicle's trip.
 */
void Vehicle::end_trip(){
    state = vsEND;
    w->trips_completed_count += 1.0;
    // Accumulate distance traveled (final link)
    distance_traveled += link->length;
    link->departure_curve[w->timestep] += w->delta_n;

    // Update traveltime_real array (slice update matching Python's traveltime_actual)
    {
        double current_tt = ((double)w->timestep + 1.0) * w->delta_t - arrival_time_link;
        int start_idx = (int)(arrival_time_link / w->delta_t);
        if (start_idx < 0) start_idx = 0;
        for (size_t idx = start_idx; idx < link->traveltime_real.size(); idx++){
            link->traveltime_real[idx] = current_tt;
        }
    }

    record_travel_time(link, (double)w->timestep * w->delta_t);

    arrival_time = (double)w->timestep * w->delta_t;
    travel_time = arrival_time - departure_time;

    w->vehicles_living.erase(id);
    w->vehicles_running.erase(id);

    // Remove from active_vehicles using swap-and-pop (O(1))
    if (active_index >= 0 && active_index < (int)w->active_vehicles.size()) {
        int last = (int)w->active_vehicles.size() - 1;
        if (active_index != last) {
            Vehicle *swapped = w->active_vehicles[last];
            w->active_vehicles[active_index] = swapped;
            swapped->active_index = active_index;
        }
        w->active_vehicles.pop_back();
        active_index = -1;
    }

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
 * @brief Apply Newell's car-following model.
 */
void Vehicle::car_follow_newell(){
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
 * @brief Choose the next link based on route choice principle.
 * 
 * @param linkset Available links to choose from.
 */
void Vehicle::route_next_link_choice(const vector<Link*>& linkset){
    if (linkset.empty()){
        route_next_link = nullptr;
        route_choice_flag_on_link = 1;
        return;
    }

    // specified_route: force next link from the route plan
    if (!specified_route.empty()){
        int traveled = _traveled_link_count;
        if (traveled < (int)specified_route.size()){
            Link *forced = specified_route[traveled];
            if (contains(linkset, forced)){
                route_next_link = forced;
                route_choice_flag_on_link = 1;
                return;
            }
        }
        route_next_link = nullptr;
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

    // Filter out links_avoid (subtraction from linkset)
    if (!links_avoid.empty()){
        _buf_filtered.clear();
        for (auto ln : outlinks){
            if (!contains(links_avoid, ln)){
                _buf_filtered.push_back(ln);
            }
        }
        if (!_buf_filtered.empty()){
            outlinks.swap(_buf_filtered);
        }
    }

    // Filter out links leading to already-traveled nodes (no_cyclic_routing)
    if (w->no_cyclic_routing){
        _buf_filtered.clear();
        for (auto ln : outlinks){
            if (!_traveled_nodes[ln->end_node->id]){
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
            w->rng);
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
 * log_size tracks actual count (vector::size() is equivalent but log_size
 * avoids any ambiguity if resize was used elsewhere).
 */
void Vehicle::log_data(){
    // Accumulate stats for print_simple_results (regardless of log mode)
    if (state == vsRUN){
        w->ave_v_sum += v;
        if (link){
            w->ave_vratio_sum += v / link->vmax;
        }
        w->stat_sample_count += 1.0;
    }

    if (w->vehicle_log_mode){
        LogEntry entry;
        entry.t = (double)w->timestep * w->delta_t;
        entry.state = state;
        if (state == vsRUN){
            entry.link = link ? link->id : -1;
            entry.x = x;
            entry.v = v;
            entry.lane = lane;
        } else {
            entry.link = -1;
            entry.x = -1;
            entry.v = -1;
            entry.lane = -1;
        }
        log_entries.push_back(entry);
        log_size++;
    }
}

/**
 * @brief Return state as a human-readable string.
 */
std::string Vehicle::state_str() const {
    switch (state) {
        case vsHOME:  return "home";
        case vsWAIT:  return "wait";
        case vsRUN:   return "run";
        case vsEND:   return "end";
        case vsABORT: return "abort";
        default:      return "unknown";
    }
}

/**
 * @brief Return log_state as a vector of human-readable strings.
 * Converts each int state entry to its string representation.
 */
std::vector<std::string> Vehicle::log_state_str() const {
    static const char* state_names[] = {"home", "wait", "run", "end", "abort"};
    std::vector<std::string> result;
    result.reserve(log_size);
    for (size_t i = 0; i < log_size; i++) {
        int s = log_entries[i].state;
        if (s >= 0 && s <= 4) {
            result.emplace_back(state_names[s]);
        } else {
            result.emplace_back("unknown");
        }
    }
    return result;
}

/**
 * @brief Return log_link as a vector of link name strings.
 * Converts each int link ID to its name. Invalid IDs become "-1".
 */
std::vector<std::string> Vehicle::log_link_names() const {
    std::vector<std::string> result;
    result.reserve(log_size);
    for (size_t i = 0; i < log_size; i++) {
        int lid = log_entries[i].link;
        if (lid >= 0 && lid < static_cast<int>(w->links.size())) {
            result.emplace_back(w->links[lid]->name);
        } else {
            result.emplace_back("-1");
        }
    }
    return result;
}

/**
 * @brief Return departure_time in seconds.
 * Note: In C++, departure_time is already stored in seconds.
 */
double Vehicle::departure_time_in_second() const {
    return departure_time;
}

/**
 * @brief Build log_t_link: a list of (time, label) pairs representing link transitions.
 * label is "home" for departure, link name for link entries, "end" for trip end.
 */
std::vector<std::pair<double, std::string>> Vehicle::build_log_t_link() const {
    std::vector<std::pair<double, std::string>> result;
    // departure_time is already in seconds in C++
    result.emplace_back(departure_time, "home");

    // Scan log_entries for link transitions
    int prev_link_id = -999;
    int n_missing = 0;
    if (log_size > 0 && log_entries[0].t > 0) {
        n_missing = static_cast<int>(log_entries[0].t / w->delta_t);
    }
    for (size_t i = 0; i < log_size; i++) {
        int lid = log_entries[i].link;
        if (lid >= 0 && lid != prev_link_id) {
            double t = log_entries[i].t;
            std::string lname = (lid >= 0 && lid < static_cast<int>(w->links.size()))
                                ? w->links[lid]->name : "-1";
            result.emplace_back(t, lname);
            prev_link_id = lid;
        }
    }

    // Add "end" entry if trip completed
    if (state == vsEND) {
        double at = (arrival_time >= 0) ? arrival_time : -1.0;
        result.emplace_back(at, "end");
    }
    return result;
}

/**
 * @brief Build full log arrays with home-timestep prepend.
 * Prepends missing "home" timesteps so indices match Python convention
 * (Python records from T=0 every timestep; C++ starts at departure_time).
 */
Vehicle::FullLog Vehicle::build_full_log() const {
    FullLog fl;

    int n_missing = 0;
    if (log_size > 0 && log_entries[0].t > 0) {
        n_missing = static_cast<int>(log_entries[0].t / w->delta_t);
    }

    size_t total = n_missing + log_size;
    fl.log_t.reserve(total);
    fl.log_x.reserve(total);
    fl.log_v.reserve(total);
    fl.log_state.reserve(total);
    fl.log_s.reserve(total);
    fl.log_lane.reserve(total);
    fl.log_link.reserve(total);

    // Prepend home entries
    for (int i = 0; i < n_missing; i++) {
        fl.log_t.push_back(i * w->delta_t);
        fl.log_x.push_back(-1);
        fl.log_v.push_back(-1);
        fl.log_state.push_back(vsHOME);
        fl.log_s.push_back(-1);
        fl.log_lane.push_back(-1);
        fl.log_link.push_back(-1);
    }

    // Append C++ log entries with state-based adjustments
    for (size_t i = 0; i < log_size; i++) {
        const LogEntry &e = log_entries[i];
        fl.log_t.push_back(e.t);

        int st = e.state;
        fl.log_state.push_back(st);

        bool is_run = (st == vsRUN);
        fl.log_x.push_back(is_run ? e.x : -1);
        fl.log_v.push_back(is_run ? e.v : -1);
        fl.log_s.push_back(is_run ? 0 : -1);
        fl.log_lane.push_back(e.lane);
        fl.log_link.push_back(e.link);
    }

    // Build log_t_link from the full (prepended) log arrays — all ints, no strings
    fl.log_t_link.emplace_back(departure_time, LOG_T_LINK_HOME);
    int prev_link_id = -999;
    for (size_t i = 0; i < fl.log_link.size(); i++) {
        int lid = fl.log_link[i];
        if (lid >= 0 && lid != prev_link_id) {
            fl.log_t_link.emplace_back(fl.log_t[i], lid);
            prev_link_id = lid;
        }
    }
    if (state == vsEND) {
        fl.log_t_link.emplace_back((arrival_time >= 0) ? arrival_time : -1.0, LOG_T_LINK_END);
    }

    return fl;
}

/**
 * @brief Full version of build_all_vehicle_logs_flat.
 * Includes home entries prepended per vehicle (log_t=arange*delta_t, log_x/v/s=-1, log_state=0, log_lane/link=-1).
 * No n_missing in output — home entries are already included in the arrays.
 */
World::FullFlatLogs World::build_all_vehicle_logs_flat_full() const {
    FullFlatLogs fl;
    size_t nv = vehicles.size();
    fl.offsets.resize(nv + 1);
    fl.ltl_offsets.resize(nv + 1);

    // First pass: compute total sizes
    size_t total_log = 0;
    size_t total_ltl = 0;
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        int n_missing = 0;
        if (v->log_size > 0 && v->log_entries[0].t > 0) {
            n_missing = static_cast<int>(v->log_entries[0].t / delta_t);
        }
        size_t full_size = n_missing + v->log_size;
        fl.offsets[vi] = total_log;
        total_log += full_size;

        // log_t_link count
        size_t ltl_count = 1;  // home entry
        int prev_lid = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            int lid = v->log_entries[i].link;
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

    // Second pass: fill arrays with home entries + actual log entries
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        size_t base = fl.offsets[vi];
        int n_missing = 0;
        if (v->log_size > 0 && v->log_entries[0].t > 0) {
            n_missing = static_cast<int>(v->log_entries[0].t / delta_t);
        }

        // Home entries
        for (int j = 0; j < n_missing; j++) {
            size_t idx = base + j;
            fl.log_t[idx] = j * delta_t;
            fl.log_x[idx] = -1.0;
            fl.log_v[idx] = -1.0;
            fl.log_state[idx] = vsHOME;
            fl.log_s[idx] = -1.0;
            fl.log_lane[idx] = -1;
            fl.log_link[idx] = -1;
        }

        // Actual log entries (AoS → SoA)
        size_t actual_base = base + n_missing;
        for (size_t i = 0; i < v->log_size; i++) {
            size_t idx = actual_base + i;
            const LogEntry &e = v->log_entries[i];
            fl.log_t[idx] = e.t;
            int st = e.state;
            fl.log_state[idx] = st;
            bool is_run = (st == vsRUN);
            fl.log_x[idx] = is_run ? e.x : -1;
            fl.log_v[idx] = is_run ? e.v : -1;
            fl.log_s[idx] = is_run ? 0 : -1;
            fl.log_lane[idx] = e.lane;
            fl.log_link[idx] = e.link;
        }

        // log_t_link
        size_t ltl_base = fl.ltl_offsets[vi];
        size_t ltl_idx = 0;
        fl.ltl_t[ltl_base] = v->departure_time;
        fl.ltl_id[ltl_base] = Vehicle::LOG_T_LINK_HOME;
        ltl_idx++;

        int prev_lid = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            const LogEntry &e = v->log_entries[i];
            int lid = e.link;
            if (lid >= 0 && lid != prev_lid) {
                fl.ltl_t[ltl_base + ltl_idx] = e.t;
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
      ave_v_sum(0.0),
      ave_vratio_sum(0.0),
      stat_sample_count(0.0),
      trips_completed_count(0.0),
      rng((std::mt19937::result_type)random_seed),
      flag_initialized(false),
      writer(&std::cout){
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
    vehicles_living.clear();
    vehicles_running.clear();
    nodes_map.clear();
    links_map.clear();
    vehicles_map.clear();
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
        flag_initialized = true;
    }
}

void World::update_adj_time_matrix(){
    double noise = 0.01;
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
        if (hard_deterministic_mode){
            adj_mat_time[i][j] = tt + penalty;
        } else {
            double noise_factor = random_range_float64(1.0, 1.0 + noise, rng);
            adj_mat_time[i][j] = tt * noise_factor + penalty;
        }
    }
}

pair<vector<vector<double>>, vector<vector<int>>> 
  World::route_search_all(const vector<vector<double>> &adj, double infty) {
    int nsize = (int)adj.size();
    if (std::fabs(infty) < 1e-9) {
        infty = std::numeric_limits<double>::infinity();
    }

    // Build adjacency list: pair<neighbor, weight>
    vector<vector<pair<int, double>>> adj_list(nsize);
    for (int i = 0; i < nsize; i++) {
        for (int j = 0; j < nsize; j++) {
            if (adj[i][j] > 0.0) {
                adj_list[i].push_back({j, adj[i][j]});
            }
        }
    }

    vector<vector<double>> dist(nsize, vector<double>(nsize, infty));
    vector<vector<int>> next_hop(nsize, vector<int>(nsize, -1));
    
    using pdi = pair<double, int>;
    std::priority_queue<pdi, vector<pdi>, std::greater<pdi>> pq;

    // Dijkstra from each source node
    for (int start = 0; start < nsize; start++) {
        vector<bool> visited(nsize, false);
        dist[start][start] = 0.0;
        next_hop[start][start] = start;
        pq.push({0.0, start});
        
        while (!pq.empty()) {
            auto [d, current] = pq.top();
            pq.pop();
            
            if (visited[current]) continue;
            visited[current] = true;
            
            // Explore neighbors via adjacency list
            for (const auto& [next, weight] : adj_list[current]) {
                double new_dist = dist[start][current] + weight;
                if (new_dist < dist[start][next]) {
                    dist[start][next] = new_dist;
                    // Update next hop
                    next_hop[start][next] = (current == start) ? 
                                          next : next_hop[start][current];
                    pq.push({new_dist, next});
                }
            }
        }
    }
    
    return {dist, next_hop};
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

    while (true){
        // Build weighted adj matrix (free-flow on iter 0, random on subsequent)
        vector<vector<double>> adj(nsize, vector<double>(nsize, 0.0));
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

        // All-pairs Dijkstra
        auto [dists, next_hop] = route_search_all(adj, 0.0);

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
    for (auto dest : nodes){
        int k = dest->id;

        auto duo_update_weight_tmp = duo_update_weight;
        {
            double psum = 0.0;
            for (double v : route_preference[k]) psum += v;
            if (psum == 0.0){
                duo_update_weight_tmp = 1; //initialize with deterministic shortest path
            }
        }

        // For each link in the world, update preference
        for (auto ln : links){
            int i = ln->start_node->id;
            int j = ln->end_node->id;
            int lid = ln->id;
            if (route_next[i][k] == j){
                route_preference[k][lid] = (1.0 - duo_update_weight_tmp) * route_preference[k][lid] + duo_update_weight_tmp;
            }else{
                route_preference[k][lid] = (1.0 - duo_update_weight_tmp) * route_preference[k][lid];
            }
        }
    }
}

void World::route_choice_duo_gradual(){
    // Gradual DUO update: scaled weight applied every timestep
    double weight0 = duo_update_weight * (delta_t / duo_update_time);

    for (auto dest : nodes){
        int k = dest->id;

        double w_tmp = weight0;
        {
            double psum = 0.0;
            for (double v : route_preference[k]) psum += v;
            if (psum == 0.0){
                w_tmp = 1;
            }
        }

        for (auto ln : links){
            int i = ln->start_node->id;
            int j = ln->end_node->id;
            int lid = ln->id;
            if (route_next[i][k] == j){
                route_preference[k][lid] = (1.0 - w_tmp) * route_preference[k][lid] + w_tmp;
            }else{
                route_preference[k][lid] = (1.0 - w_tmp) * route_preference[k][lid];
            }
        }
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

void World::print_simple_results(){
    // Compute from incrementally accumulated stats (no log scan needed)
    trips_total = (double)vehicles.size() * delta_n;
    trips_completed = trips_completed_count * delta_n;
    if (stat_sample_count > 0){
        ave_v = ave_v_sum / stat_sample_count;
        ave_vratio = ave_vratio_sum / stat_sample_count;
    }

    (*writer) << "Stats:\n";
    (*writer) << "    Average speed: " << ave_v << "\n";
    (*writer) << "    Average speed ratio: " << ave_vratio << "\n";
    (*writer) << "    Trips completion: "
              << trips_completed << " / " << trips_total << "\n";
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

    for (timestep = start_ts; timestep < end_ts; timestep++){
        time = timestep*delta_t;

        // Link updates
        for (auto ln : links){
            ln->update();
        }

        // Node generate + update (matches Python: node.generate() then node.update())
        for (auto nd : nodes){
            nd->generate();
            nd->signal_update();
            nd->flow_capacity_update();
        }

        // Node transfer
        for (auto nd : nodes){
            nd->transfer();
        }

        // car-following
        int veh_count = 0;
        double speed_sum = 0;
        if (hard_deterministic_mode) {
            // Deterministic: iterate all vehicles in id order
            for (auto veh : vehicles){
                if (veh->state == vsRUN){
                    veh->car_follow_newell();
                    veh_count++;
                    speed_sum += veh->v;
                }
            }
        } else {
            // Non-deterministic: active vehicles only (much faster when many END)
            for (auto veh : active_vehicles){
                if (veh->state == vsRUN){
                    veh->car_follow_newell();
                    veh_count++;
                    speed_sum += veh->v;
                }
            }
        }
        double ave_speed = veh_count > 0 ? speed_sum / veh_count : 0;

        // vehicle update
        if (hard_deterministic_mode) {
            // Deterministic: iterate all vehicles in id order
            for (auto veh : vehicles){
                if (veh->state == vsHOME || veh->state == vsWAIT || veh->state == vsRUN){
                    veh->update();
                }
            }
        } else {
            // Non-deterministic: active vehicles only
            // end_trip() removes via swap-and-pop, so iterate by index.
            size_t ai = 0;
            while (ai < active_vehicles.size()){
                Vehicle *veh = active_vehicles[ai];
                veh->update();
                if (veh->active_index < 0) {
                    // veh was removed — swapped-in vehicle now at ai, re-process
                } else {
                    ai++;
                }
            }
        }        

        // route choice update
        if (route_choice_update_gradual){
            // Gradual mode: route_search at DELTAT_ROUTE intervals, DUO_update every step
            if (timestep_for_route_update > 0 && timestep % timestep_for_route_update == 0){
                update_adj_time_matrix();
                auto res = route_search_all(adj_mat_time, 0.0);
                route_dist = res.first;
                route_next = res.second;
                route_dist_record[timestep] = route_dist;
            }
            // DUO update every step with scaled weight
            route_choice_duo_gradual();
        } else {
            // Normal mode: both route_search and DUO_update at DELTAT_ROUTE intervals
            if (timestep_for_route_update > 0 && timestep % timestep_for_route_update == 0){
                update_adj_time_matrix();
                auto res = route_search_all(adj_mat_time, 0.0);
                route_dist = res.first;
                route_next = res.second;
                route_dist_record[timestep] = route_dist;
                route_choice_duo();
            }
        }

        // Print progress in steps
        if (print_mode == 1 && total_timesteps > 0 && timestep % (total_timesteps / 10 == 0 ? 1 : total_timesteps / 10) == 0){
            if (timestep == 0){
                (*writer) <<  "Simulating..." << endl;
                (*writer) <<  std::setw(10) << "time" 
                    << "|"<< std::setw(14) <<  "# of vehicles"
                    << "|"<< std::setw(11) << " ave speed" << endl;
            }
            (*writer) << std::setw(8) << std::fixed << std::setprecision(0) << time << " s"
                  << "|" << std::setw(10) << veh_count*delta_n << " veh"
                  << "|" << std::setw(7) << std::fixed << std::setprecision(2) << ave_speed << " m/s"
                  << endl;
        }        
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
    for (auto nd : nodes){
        if (nd->name == node_name){
            return nd;
        }
    }
    (*writer) << "Error at function get_node(): `"
              << node_name << "` not found\n";
    throw std::runtime_error("get_node() error");
}

Link *World::get_link(const string &link_name){
    for (auto ln : links){
        if (ln->name == link_name){
            return ln;
        }
    }
    (*writer) << "Error at function get_link(): `"
              << link_name << "` not found\n";
    throw std::runtime_error("get_link() error");
}


Vehicle *World::get_vehicle(const string &vehicle_name){
    for (auto vh : vehicles){
        if (vh->name == vehicle_name){
            return vh;
        }
    }
    (*writer) << "Error at function get_vehicle(): `"
              << vehicle_name << "` not found\n";
    throw std::runtime_error("get_vehicle() error");
}

Vehicle *World::get_vehicle_by_index(int index) const {
    if (index >= 0 && index < static_cast<int>(vehicles.size())) {
        return vehicles[index];
    }
    throw std::out_of_range("get_vehicle_by_index: index out of range");
}

Link *World::get_link_by_id(const int link_id){
    if (link_id >= 0 && link_id < (int)links.size() && links[link_id]->id == link_id){
        return links[link_id];
    }
    // Fallback: linear search (should not happen if IDs are sequential)
    for (auto ln : links){
        if (ln->id == link_id){
            return ln;
        }
    }
    (*writer) << "Error at function get_link_id(): `"
              << link_id << "` not found\n";
    throw std::runtime_error("get_link_id() error");
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
    double demand = 0.0;
    for (double t = start_t; t < end_t; t += w->delta_t){
        demand += flow * w->delta_t;
        while (demand >= (double)w->delta_n){
            // create new vehicle
            Vehicle *v = new Vehicle(
                w,
                std::to_string(w->vehicle_id),
                t,
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
 * @brief Return vehicles filtered by state.
 */
std::vector<Vehicle *> World::get_vehicles_by_state(int state) const {
    std::vector<Vehicle *> result;
    for (auto *v : vehicles) {
        if (v->state == state) {
            result.push_back(v);
        }
    }
    return result;
}

/**
 * @brief Return all vehicle states as (name, state_int) pairs.
 * Efficient bulk query so Python can update VEHICLES_LIVING/RUNNING
 * without calling state on each vehicle individually.
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
 * @brief Return effective state for each vehicle, indexed by order in vehicles vector.
 * Abort detection is applied. Safe for index-based matching with Python VEHICLES.values().
 */
std::vector<int> World::get_vehicle_states_by_index() const {
    std::vector<int> result;
    result.reserve(vehicles.size());
    for (auto *v : vehicles) {
        int effective_state = v->state;
        if (effective_state == vsEND && (v->flag_trip_aborted || (v->arrival_time < 0 && v->travel_time <= 0))) {
            effective_state = vsABORT;
        }
        result.push_back(effective_state);
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
        // Scan log_entries for link transitions (same logic as build_full_log's log_t_link)
        int prev_link_id = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            const LogEntry &e = v->log_entries[i];
            int lid = e.link;
            if (lid >= 0 && lid != prev_link_id) {
                result.push_back({lid, e.t, static_cast<int>(vi)});
                prev_link_id = lid;
            }
        }
    }

    return result;
}

/**
 * @brief Build flat SoA logs for all vehicles in one pass.
 * Avoids per-vehicle vector allocation and Python object creation overhead.
 */
World::FlatLogs World::build_all_vehicle_logs_flat() const {
    FlatLogs fl;
    size_t nv = vehicles.size();
    fl.offsets.resize(nv + 1);
    fl.ltl_offsets.resize(nv + 1);

    // First pass: compute total sizes for pre-allocation
    size_t total_log = 0;
    size_t total_ltl = 0;
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        int n_missing = 0;
        if (v->log_size > 0 && v->log_entries[0].t > 0) {
            n_missing = static_cast<int>(v->log_entries[0].t / delta_t);
        }
        size_t vlen = n_missing + v->log_size;
        fl.offsets[vi] = total_log;
        total_log += vlen;

        // log_t_link: 1 (home) + transitions + optional end
        size_t ltl_count = 1;  // home entry
        int prev_lid = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            int lid = v->log_entries[i].link;
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

    // Allocate all arrays at once
    fl.log_t.resize(total_log);
    fl.log_x.resize(total_log);
    fl.log_v.resize(total_log);
    fl.log_state.resize(total_log);
    fl.log_s.resize(total_log);
    fl.log_lane.resize(total_log);
    fl.log_link.resize(total_log);
    fl.ltl_t.resize(total_ltl);
    fl.ltl_id.resize(total_ltl);

    // Second pass: fill arrays
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        size_t base = fl.offsets[vi];
        int n_missing = 0;
        if (v->log_size > 0 && v->log_entries[0].t > 0) {
            n_missing = static_cast<int>(v->log_entries[0].t / delta_t);
        }

        // Prepend home entries
        for (int i = 0; i < n_missing; i++) {
            size_t idx = base + i;
            fl.log_t[idx] = i * delta_t;
            fl.log_x[idx] = -1;
            fl.log_v[idx] = -1;
            fl.log_state[idx] = vsHOME;
            fl.log_s[idx] = -1;
            fl.log_lane[idx] = -1;
            fl.log_link[idx] = -1;
        }

        // Append actual log entries (AoS → SoA)
        size_t log_base = base + n_missing;
        size_t ls = v->log_size;
        for (size_t i = 0; i < ls; i++) {
            size_t idx = log_base + i;
            const LogEntry &e = v->log_entries[i];
            fl.log_t[idx] = e.t;
            int st = e.state;
            fl.log_state[idx] = st;
            bool is_run = (st == vsRUN);
            fl.log_x[idx] = is_run ? e.x : -1;
            fl.log_v[idx] = is_run ? e.v : -1;
            fl.log_s[idx] = is_run ? 0 : -1;
            fl.log_lane[idx] = e.lane;
            fl.log_link[idx] = e.link;
        }

        // log_t_link
        size_t ltl_base = fl.ltl_offsets[vi];
        size_t ltl_idx = 0;
        fl.ltl_t[ltl_base] = v->departure_time;
        fl.ltl_id[ltl_base] = Vehicle::LOG_T_LINK_HOME;
        ltl_idx++;

        int prev_lid = -999;
        for (size_t i = 0; i < ls; i++) {
            const LogEntry &e = v->log_entries[i];
            int lid = e.link;
            if (lid >= 0 && lid != prev_lid) {
                fl.ltl_t[ltl_base + ltl_idx] = e.t;
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
        if (v->log_size > 0 && v->log_entries[0].t > 0) {
            n_missing = static_cast<int>(v->log_entries[0].t / delta_t);
        }
        fl.n_missing[vi] = n_missing;
        fl.offsets[vi] = total_log;
        total_log += v->log_size;

        // log_t_link count
        size_t ltl_count = 1;  // home entry
        int prev_lid = -999;
        for (size_t i = 0; i < v->log_size; i++) {
            int lid = v->log_entries[i].link;
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

    // Second pass: fill arrays (no home prepend, AoS → SoA)
    for (size_t vi = 0; vi < nv; vi++) {
        const Vehicle *v = vehicles[vi];
        size_t base = fl.offsets[vi];
        size_t ls = v->log_size;

        for (size_t i = 0; i < ls; i++) {
            size_t idx = base + i;
            const LogEntry &e = v->log_entries[i];
            fl.log_t[idx] = e.t;
            int st = e.state;
            fl.log_state[idx] = st;
            bool is_run = (st == vsRUN);
            fl.log_x[idx] = is_run ? e.x : -1;
            fl.log_v[idx] = is_run ? e.v : -1;
            fl.log_s[idx] = is_run ? 0 : -1;
            fl.log_lane[idx] = e.lane;
            fl.log_link[idx] = e.link;
        }

        // log_t_link
        size_t ltl_base = fl.ltl_offsets[vi];
        size_t ltl_idx = 0;
        fl.ltl_t[ltl_base] = v->departure_time;
        fl.ltl_id[ltl_base] = Vehicle::LOG_T_LINK_HOME;
        ltl_idx++;

        int prev_lid = -999;
        for (size_t i = 0; i < ls; i++) {
            const LogEntry &e = v->log_entries[i];
            int lid = e.link;
            if (lid >= 0 && lid != prev_lid) {
                fl.ltl_t[ltl_base + ltl_idx] = e.t;
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

