// clang-format off
#pragma once

// DTA (Dynamic Traffic Assignment) solver batch operations for C++ engine.
// Separate from traffi.h/traffi.cpp to avoid impacting normal simulation.
// Uses World/Vehicle/Link/Node via their public interfaces only.

#include "traffi.h"

#include <map>
#include <vector>
#include <random>
#include <algorithm>

// -----------------------------------------------------------------------
// Helper functions (link-level)
// -----------------------------------------------------------------------

// Get toll value at time t (from toll_timeseries or route_choice_penalty)
inline double dta_get_toll(const Link *ln, double t) {
    if (!ln->toll_timeseries.empty()){
        int ts = (int)(t / ln->w->delta_t);
        if (ts < 0) ts = 0;
        if (ts >= (int)ln->toll_timeseries.size()) ts = (int)ln->toll_timeseries.size() - 1;
        return ln->toll_timeseries[ts];
    }
    return ln->route_choice_penalty;
}

// Get actual travel time at time t from traveltime_real
inline double dta_get_actual_travel_time(const Link *ln, double t) {
    int n = (int)ln->traveltime_real.size();
    if (n == 0) return ln->length / ln->vmax;
    int idx = (int)(t / ln->w->delta_t);
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;
    return ln->traveltime_real[idx];
}

// -----------------------------------------------------------------------
// Traveled route extraction
// -----------------------------------------------------------------------

struct TraveledRouteInfo {
    std::vector<int> link_ids;       // sequence of link IDs traveled
    std::vector<double> entry_times; // time of entering each link
    double arrival_time;             // arrival time at destination (-1 if not arrived)
};

// Extract traveled route from vehicle log data
TraveledRouteInfo dta_get_traveled_route(const Vehicle *veh);

// -----------------------------------------------------------------------
// Route cost computation
// -----------------------------------------------------------------------

// Compute actual travel time along a route from departure_time.
// If details is non-null, fills per-link travel times.
double dta_compute_route_travel_time(const World *w, const std::vector<int> &route_link_ids,
                                     double departure_time, std::vector<double> *details = nullptr);

// DUE cost: actual_travel_time + sum of tolls
double dta_compute_route_cost_due(const World *w, const std::vector<int> &route_link_ids,
                                  double departure_time);

// DSO cost: actual_travel_time + congestion externality
double dta_compute_route_cost_dso(World *w, const std::vector<int> &route_link_ids,
                                  double departure_time);

// -----------------------------------------------------------------------
// Batch operations
// -----------------------------------------------------------------------

// Batch enforce routes for all vehicles.
// routes_per_vehicle[i] = route (link_ids) for vehicle i. Empty = skip.
void dta_batch_enforce_routes(World *w, const std::vector<std::vector<int>> &routes_per_vehicle);

// -----------------------------------------------------------------------
// Route swap result
// -----------------------------------------------------------------------

struct RouteSwapResult {
    // Per-vehicle: route for next iteration (link_ids). Empty = keep traveled route.
    std::vector<std::vector<int>> routes_specified;
    // Per-vehicle: actually traveled route (link_ids)
    std::vector<std::vector<int>> route_actual;
    // Per-vehicle: travel cost (travel_time = arrival - departure)
    std::vector<double> cost_actual;
    // Aggregate statistics
    double n_swap;
    double potential_n_swap;
    double total_t_gap;
};

// DUE route swap: compute next-iteration routes for all vehicles.
// od_route_sets: (orig_node_id, dest_node_id) -> list of routes (each = vector of link_ids)
// has_external_route_sets: if true, vehicles whose traveled route is not in set get forced to first route
RouteSwapResult dta_route_swap_due(
    World *w,
    const std::map<std::pair<int,int>, std::vector<std::vector<int>>> &od_route_sets,
    double swap_prob,
    bool has_external_route_sets,
    unsigned int rng_seed);

// DSO route swap: uses marginal cost (private + externality).
// swap_num: if >= 0, exactly this many vehicles are swap candidates (overrides swap_prob)
RouteSwapResult dta_route_swap_dso(
    World *w,
    const std::map<std::pair<int,int>, std::vector<std::vector<int>>> &od_route_sets,
    double swap_prob,
    int swap_num,
    bool has_external_route_sets,
    unsigned int rng_seed);
