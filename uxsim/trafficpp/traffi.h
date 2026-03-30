// clang-format off
#pragma once

#include <iostream>
#include <iomanip>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <random>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <queue>
#include <set>

#ifdef UXSIM_USE_OPENMP
#include <omp.h>
#endif

#include "utils.h"

using std::string, std::vector, std::deque, std::pair, std::map, std::unordered_map, std::priority_queue, std::greater, std::cout, std::endl;

// Forward declarations
struct World;
struct Node;
struct Link;
struct Vehicle;


enum VehicleState : int {
    vsHOME  = 0,
    vsWAIT  = 1,
    vsRUN   = 2,
    vsEND   = 3,
    vsABORT = 4
};

enum RouteChoicePrinciple : int {
    rcpDUO = 0,
    rcpFIXED = 1
};

// -----------------------------------------------------------------------
// MARK: class Node
// -----------------------------------------------------------------------

struct Node {
    World *w;
    int id;
    string name;

    vector<Link *> in_links;
    vector<Link *> out_links;

    // Vehicles just arrived at this node (not on any link)
    vector<Vehicle *> incoming_vehicles;
    // Requested next-links by each incoming vehicle
    vector<Link *> incoming_vehicles_requests;

    // Vehicles waiting to be generated onto the outgoing link
    deque<Vehicle *> generation_queue;

    double x;
    double y;

    // signal
    vector<double> signal_intervals;
    double signal_offset;
    double signal_t;
    int signal_phase;
    vector<int> signal_log;

    // flow capacity (macroscopic/continuous representation for signal)
    double flow_capacity;
    double flow_capacity_remain;
    int number_of_lanes;

    Node(
        World *w,
        const string &node_name,
        double x,
        double y,
        vector<double> signal_intervals = {0},
        double signal_offset = 0,
        double flow_capacity = -1.0,
        int number_of_lanes = 0);

    void generate();
    void transfer();
    void signal_update();
    void flow_capacity_update();
};

// -----------------------------------------------------------------------
// MARK: class Link
// -----------------------------------------------------------------------

struct Link {
    World *w;

    int id;
    string name;
    double length;
    Node *start_node;
    Node *end_node;

    int number_of_lanes;
    double vmax;
    double delta;
    double delta_per_lane;
    double tau;
    double kappa;
    double capacity;
    double backward_wave_speed;
    deque<Vehicle *> vehicles;

    vector<double> traveltime_tt; // increments of time
    vector<double> traveltime_t;

    vector<double> arrival_curve;
    vector<double> departure_curve;
    vector<double> traveltime_real;
    vector<double> traveltime_instant;

    double merge_priority;

    double capacity_out;
    double capacity_out_remain;
    double capacity_in;
    double capacity_in_remain;

    //signal
    vector<int> signal_group;

    // route choice penalty (congestion pricing support)
    double route_choice_penalty;
    // dynamic toll timeseries (indexed by timestep, set from Python wrapper)
    vector<double> toll_timeseries;

    Link(
        World *w,
        const string &link_name,
        const string &start_node_name,
        const string &end_node_name,
        double vmax,
        double kappa,
        double length,
        int number_of_lanes,
        double merge_priority,
        double capacity_out=-1.0,
        double capacity_in=-1.0,
        vector<int> signal_group={0});

    void update();
    void set_travel_time();
    double estimate_congestion_externality(int timestep);
    void change_free_flow_speed(double new_value);
    void change_jam_density(double new_value);
    int count_vehicles_in_queue() const;

};

// -----------------------------------------------------------------------
// MARK: class Vehicle
// -----------------------------------------------------------------------

struct Vehicle {
    World *w;
    int id;
    string name;

    double departure_time;
    Node *orig;
    Node *dest;
    Link *link;

    double arrival_time;
    double travel_time;
    double distance_traveled;

    double x;
    double x_next;
    double x_old;
    double v;
    double move_remain;
    int lane;

    Vehicle *leader;
    Vehicle *follower;

    int state; // VehicleState enum: 0=home, 1=wait, 2=run, 3=end, 4=abort
    int flag_waiting_for_trip_end;
    int flag_trip_aborted;
    int trip_abort;

    double arrival_time_link;

    // Route logic
    Link *route_next_link;
    int route_choice_flag_on_link;
    double route_adaptive;
    double route_choice_uncertainty;
    map<Link *, double> route_preference;
    int route_choice_principle;
    vector<Link *> links_preferred;
    vector<Link *> links_avoid;
    vector<Link *> specified_route;

    // Logging
    vector<double> log_t;
    vector<int> log_state;
    vector<int> log_link;
    vector<double> log_x;
    vector<double> log_v;
    vector<int> log_lane;

    Vehicle(
        World *w,
        const string &vehicle_name,
        double departure_time,
        const string &orig_name,
        const string &dest_name);

    void update();
    void end_trip();
    void car_follow_newell();
    void route_next_link_choice(vector<Link*> linkset);
    void enforce_route(vector<Link*> route);
    void record_travel_time(Link *link, double t);
    void log_data();

    // Helper: return state as string ("home", "wait", "run", "end", "abort")
    std::string state_str() const;

    // Helper: return log_state as vector of strings
    std::vector<std::string> log_state_str() const;

    // Helper: return log_link as vector of link name strings (-1 → "-1")
    std::vector<std::string> log_link_names() const;

    // Helper: return departure_time in seconds (already in seconds in C++)
    double departure_time_in_second() const;

    // Helper: build log_t_link as vector of (time, label) pairs.
    // label is link name for link entries, "home" or "end" for special entries.
    std::vector<std::pair<double, std::string>> build_log_t_link() const;

    // Helper: build full log arrays with home-timestep prepend.
    // All data is int/double — no string allocation for maximum speed.
    // String conversion is done at the binding layer only when needed.
    struct FullLog {
        std::vector<double> log_t;
        std::vector<double> log_x;
        std::vector<double> log_v;
        std::vector<int> log_state;     // int: 0=home,1=wait,2=run,3=end,4=abort
        std::vector<double> log_s;
        std::vector<int> log_lane;
        std::vector<int> log_link;      // link ID, -1 for home/none
        // log_t_link: (time, link_id) pairs for link transitions.
        // Special IDs: -1 = "home", -2 = "end"
        std::vector<std::pair<double, int>> log_t_link;
    };
    static constexpr int LOG_T_LINK_HOME = -1;
    static constexpr int LOG_T_LINK_END  = -2;
    FullLog build_full_log() const;

};

// -----------------------------------------------------------------------
// MARK: class World
// -----------------------------------------------------------------------

struct World {
    // Simulation config 
    long long timestamp;
    string name;

    double t_max;
    double delta_n;
    double tau;
    double duo_update_time;
    double duo_update_weight;
    int print_mode;
    bool hard_deterministic_mode;
    bool route_choice_update_gradual;
    bool no_cyclic_routing;
    int instantaneous_TT_timestep_interval = 5;

    double delta_t;
    size_t total_timesteps;
    size_t timestep_for_route_update;

    int node_id;
    int link_id;
    int vehicle_id;

    // Collections of objects
    vector<Vehicle *> vehicles;         //all state
    vector<Link *> links;
    vector<Node *> nodes;
    unordered_map<int, Vehicle *> vehicles_living;  //home, wait, run // vehicles_living[id] = vehicle
    unordered_map<int, Vehicle *> vehicles_running; //run
    unordered_map<string, Node *> nodes_map;
    unordered_map<string, Link *> links_map;
    unordered_map<string, Vehicle *> vehicles_map;

    size_t timestep;    //simulation timestep
    double time;    //simulation time in second

    double route_adaptive;
    double route_choice_uncertainty;
    vector<map<Link *, double>> route_preference;   // route_preference[dest_id][link]: preference weight for link towards dest

    // Graph adjacency
    vector<vector<int>> adj_mat;
    vector<vector<double>> adj_mat_time;
    vector<vector<int>> route_next;
    vector<vector<double>> route_dist;
    map<int, vector<vector<double>>> route_dist_record;

    bool flag_initialized;

    // stats
    double ave_v;
    double ave_vratio;
    double trips_total;
    double trips_completed;

    // Randomness
    long long random_seed;
    std::mt19937 rng;

    std::ostream *writer;

    World(
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
        bool hard_deterministic_mode = false,
        bool route_choice_update_gradual = false,
        bool no_cyclic_routing = false);

    ~World();

    void initialize_adj_matrix();
    void update_adj_time_matrix();

    void route_choice_duo();
    void route_choice_duo_gradual();

    // All-pairs shortest path (Dijkstra from each source)
    pair<vector<vector<double>>, vector<vector<int>>>
        route_search_all(const vector<vector<double>> &adj, double infty);

    // Enumerate k random routes between all reachable node pairs
    map<pair<int,int>, vector<vector<int>>>
        enumerate_k_random_routes_cpp(int k, unsigned int seed);

    // Estimate congestion externality for a route (sum of per-link externalities)
    double estimate_congestion_externality_route(const vector<Link*> &route, double departure_time);

    void print_scenario_stats();
    void print_simple_results();
    void main_loop(double duration_t, double end_t);

    bool check_simulation_ongoing();

    Node *get_node(const string &node_name);
    Link *get_link(const string &link_name);
    Link *get_link_by_id(const int link_id);
    Vehicle *get_vehicle(const string &vehicle_name);
    Vehicle *get_vehicle_by_index(int index) const;

    size_t vehicle_log_reserve_size;
    bool vehicle_log_mode;

    // Helper: return vehicles filtered by state
    std::vector<Vehicle *> get_vehicles_by_state(int state) const;

    // Helper: return vehicle states as a vector of (name, state_int) pairs
    // Useful for Python to update VEHICLES_LIVING/RUNNING without per-vehicle calls
    std::vector<std::pair<std::string, int>> get_all_vehicle_states() const;

    // Helper: return effective state (with abort detection) for each vehicle,
    // indexed by vehicle order in World.vehicles vector.
    // Returns vector of int states: 0=home, 1=wait, 2=run, 3=end, 4=abort
    std::vector<int> get_vehicle_states_by_index() const;

};
