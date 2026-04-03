// clang-format off

// DTA solver batch operations — implementation.
// Separate from traffi.cpp to keep normal simulation code untouched.

#include "dta_solver.h"

// -----------------------------------------------------------------------
// Traveled route extraction
// -----------------------------------------------------------------------

TraveledRouteInfo dta_get_traveled_route(const Vehicle *veh) {
    TraveledRouteInfo info;
    info.arrival_time = -1.0;

    int prev_lid = -999;
    for (size_t i = 0; i < veh->log_size; i++){
        int lid = veh->log_link[i];
        if (lid >= 0 && lid != prev_lid){
            info.link_ids.push_back(lid);
            info.entry_times.push_back(veh->log_t[i]);
            prev_lid = lid;
        }
    }

    if (veh->state == vsEND && veh->arrival_time >= 0){
        info.arrival_time = veh->arrival_time;
    }
    return info;
}

// -----------------------------------------------------------------------
// Route cost computation
// -----------------------------------------------------------------------

double dta_compute_route_travel_time(const World *w, const std::vector<int> &route_link_ids,
                                     double departure_time, std::vector<double> *details) {
    double tt_total = 0.0;
    double t = departure_time;
    for (int lid : route_link_ids){
        Link *ln = w->links[lid];
        double ltt = dta_get_actual_travel_time(ln, t);
        tt_total += ltt;
        t += ltt;
        if (details) details->push_back(ltt);
    }
    return tt_total;
}

double dta_compute_route_cost_due(const World *w, const std::vector<int> &route_link_ids,
                                  double departure_time) {
    double tt_total = 0.0;
    double toll_total = 0.0;
    double t = departure_time;
    for (int lid : route_link_ids){
        Link *ln = w->links[lid];
        double ltt = dta_get_actual_travel_time(ln, t);
        toll_total += dta_get_toll(ln, t);
        tt_total += ltt;
        t += ltt;
    }
    return tt_total + toll_total;
}

double dta_compute_route_cost_dso(World *w, const std::vector<int> &route_link_ids,
                                  double departure_time) {
    double tt_total = 0.0;
    double ext_total = 0.0;
    double t = departure_time;
    for (int lid : route_link_ids){
        Link *ln = w->links[lid];
        double ltt = dta_get_actual_travel_time(ln, t);
        int ts = (int)(t / w->delta_t);
        ext_total += ln->estimate_congestion_externality(ts);
        tt_total += ltt;
        t += ltt;
    }
    return tt_total + ext_total;
}

// -----------------------------------------------------------------------
// Batch enforce routes
// -----------------------------------------------------------------------

void dta_batch_enforce_routes(World *w, const std::vector<std::vector<int>> &routes_per_vehicle) {
    size_t n = std::min(routes_per_vehicle.size(), w->vehicles.size());
    for (size_t i = 0; i < n; i++){
        const auto &route_ids = routes_per_vehicle[i];
        if (route_ids.empty()) continue;
        // Convert link IDs to Link pointers
        std::vector<Link*> route;
        route.reserve(route_ids.size());
        for (int lid : route_ids){
            if (lid >= 0 && lid < (int)w->links.size()){
                route.push_back(w->links[lid]);
            }
        }
        w->vehicles[i]->enforce_route(route);
    }
}

// -----------------------------------------------------------------------
// DUE route swap
// -----------------------------------------------------------------------

RouteSwapResult dta_route_swap_due(
    World *w,
    const std::map<std::pair<int,int>, std::vector<std::vector<int>>> &od_route_sets,
    double swap_prob,
    bool has_external_route_sets,
    unsigned int rng_seed)
{
    size_t nv = w->vehicles.size();
    RouteSwapResult result;
    result.routes_specified.resize(nv);
    result.route_actual.resize(nv);
    result.cost_actual.resize(nv, 0.0);
    result.n_swap = 0;
    result.potential_n_swap = 0;
    result.total_t_gap = 0;

    std::mt19937 local_rng(rng_seed);
    std::uniform_real_distribution<double> udist(0.0, 1.0);

    for (size_t vi = 0; vi < nv; vi++){
        Vehicle *veh = w->vehicles[vi];
        int orig_id = veh->orig->id;
        int dest_id = veh->dest->id;

        // Get traveled route
        auto traveled = dta_get_traveled_route(veh);
        result.route_actual[vi] = traveled.link_ids;

        // Compute travel time (arrival - first entry)
        if (!traveled.entry_times.empty() && traveled.arrival_time >= 0){
            result.cost_actual[vi] = traveled.arrival_time - traveled.entry_times[0];
        } else {
            result.cost_actual[vi] = -1;
        }

        // Skip vehicles that didn't finish their trip (leave empty = no enforce_route)
        if (veh->state != vsEND){
            continue;
        }

        // Look up route set for this OD pair
        auto it = od_route_sets.find({orig_id, dest_id});
        if (it == od_route_sets.end()){
            result.routes_specified[vi] = traveled.link_ids;
            continue;
        }
        const auto &route_set = it->second;

        // Check if external route_sets and traveled route is not in set
        if (has_external_route_sets){
            bool found = false;
            for (const auto &rs : route_set){
                if (rs == traveled.link_ids){
                    found = true;
                    break;
                }
            }
            if (!found && !route_set.empty()){
                result.routes_specified[vi] = route_set[0];
                continue;
            }
        }

        // Determine if this vehicle is a swap candidate
        bool flag_swap = (udist(local_rng) < swap_prob);

        // Compute current route cost (DUE: travel_time + tolls at actual entry times)
        double departure_t = traveled.entry_times.empty() ? veh->departure_time : traveled.entry_times[0];
        double cost_current = 0.0;
        {
            double t = departure_t;
            for (size_t j = 0; j < traveled.link_ids.size(); j++){
                Link *ln = w->links[traveled.link_ids[j]];
                double ltt = dta_get_actual_travel_time(ln, t);
                // Toll at actual entry time for current route
                double toll_t = (j < traveled.entry_times.size()) ? traveled.entry_times[j] : t;
                cost_current += dta_get_toll(ln, toll_t);
                cost_current += ltt;
                t += ltt;
            }
        }

        bool flag_route_changed = false;
        int route_changed_idx = -1;
        double t_gap = 0.0;
        double potential_n_swap_delta = 0.0;

        for (size_t ri = 0; ri < route_set.size(); ri++){
            double cost_alt = dta_compute_route_cost_due(w, route_set[ri], departure_t);
            if (cost_alt < cost_current){
                if (!flag_route_changed || cost_alt < cost_current){
                    t_gap = cost_current - cost_alt;
                    potential_n_swap_delta = w->delta_n;
                    if (flag_swap){
                        flag_route_changed = true;
                        route_changed_idx = (int)ri;
                        cost_current = cost_alt;
                    }
                }
            }
        }

        result.potential_n_swap += potential_n_swap_delta;
        result.total_t_gap += t_gap;

        if (flag_route_changed && route_changed_idx >= 0){
            result.routes_specified[vi] = route_set[route_changed_idx];
            result.n_swap += w->delta_n;
        } else {
            result.routes_specified[vi] = traveled.link_ids;
        }
    }

    return result;
}

// -----------------------------------------------------------------------
// DSO route swap
// -----------------------------------------------------------------------

RouteSwapResult dta_route_swap_dso(
    World *w,
    const std::map<std::pair<int,int>, std::vector<std::vector<int>>> &od_route_sets,
    double swap_prob,
    int swap_num,
    bool has_external_route_sets,
    unsigned int rng_seed)
{
    size_t nv = w->vehicles.size();
    RouteSwapResult result;
    result.routes_specified.resize(nv);
    result.route_actual.resize(nv);
    result.cost_actual.resize(nv, 0.0);
    result.n_swap = 0;
    result.potential_n_swap = 0;
    result.total_t_gap = 0;

    std::mt19937 local_rng(rng_seed);
    std::uniform_real_distribution<double> udist(0.0, 1.0);

    // Determine swap candidates
    std::vector<bool> is_swap_candidate(nv, false);
    if (swap_num >= 0){
        // Fixed number of swaps: sample swap_num indices
        std::vector<int> indices(nv);
        for (size_t i = 0; i < nv; i++) indices[i] = (int)i;
        std::shuffle(indices.begin(), indices.end(), local_rng);
        int actual_swap_num = std::min(swap_num, (int)nv);
        for (int i = 0; i < actual_swap_num; i++){
            is_swap_candidate[indices[i]] = true;
        }
    } else {
        // Probabilistic swap
        for (size_t i = 0; i < nv; i++){
            is_swap_candidate[i] = (udist(local_rng) < swap_prob);
        }
    }

    for (size_t vi = 0; vi < nv; vi++){
        Vehicle *veh = w->vehicles[vi];
        int orig_id = veh->orig->id;
        int dest_id = veh->dest->id;

        // Get traveled route
        auto traveled = dta_get_traveled_route(veh);
        result.route_actual[vi] = traveled.link_ids;

        // Compute travel time
        if (!traveled.entry_times.empty() && traveled.arrival_time >= 0){
            result.cost_actual[vi] = traveled.arrival_time - traveled.entry_times[0];
        } else {
            result.cost_actual[vi] = -1;
        }

        // Skip vehicles that didn't finish their trip (leave empty = no enforce_route)
        if (veh->state != vsEND){
            continue;
        }

        // Look up route set for this OD pair
        auto it = od_route_sets.find({orig_id, dest_id});
        if (it == od_route_sets.end()){
            result.routes_specified[vi] = traveled.link_ids;
            continue;
        }
        const auto &route_set = it->second;

        // Check if external route_sets and traveled route is not in set
        if (has_external_route_sets){
            bool found = false;
            for (const auto &rs : route_set){
                if (rs == traveled.link_ids){
                    found = true;
                    break;
                }
            }
            if (!found && !route_set.empty()){
                result.routes_specified[vi] = route_set[0];
                continue;
            }
        }

        bool flag_swap = is_swap_candidate[vi];

        // Compute current route cost (DSO: travel_time + congestion externality)
        double departure_t = traveled.entry_times.empty() ? veh->departure_time : traveled.entry_times[0];
        double cost_current = dta_compute_route_cost_dso(w, traveled.link_ids, departure_t);

        bool flag_route_changed = false;
        int route_changed_idx = -1;
        double t_gap = 0.0;
        double potential_n_swap_delta = 0.0;

        for (size_t ri = 0; ri < route_set.size(); ri++){
            double cost_alt = dta_compute_route_cost_dso(w, route_set[ri], departure_t);
            if (cost_alt < cost_current){
                if (!flag_route_changed || cost_alt < cost_current){
                    t_gap = cost_current - cost_alt;
                    potential_n_swap_delta = w->delta_n;
                    if (flag_swap){
                        flag_route_changed = true;
                        route_changed_idx = (int)ri;
                        cost_current = cost_alt;
                    }
                }
            }
        }

        result.potential_n_swap += potential_n_swap_delta;
        result.total_t_gap += t_gap;

        if (flag_route_changed && route_changed_idx >= 0){
            result.routes_specified[vi] = route_set[route_changed_idx];
            result.n_swap += w->delta_n;
        } else {
            result.routes_specified[vi] = traveled.link_ids;
        }
    }

    return result;
}
