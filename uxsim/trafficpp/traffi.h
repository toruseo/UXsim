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
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <queue>
#include <set>

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

    // Reusable buffers for transfer() to avoid per-call allocation
    vector<Link *> _buf_outlinks_expanded;
    std::unordered_set<Link *> _buf_seen_outlinks;
    vector<Vehicle *> _buf_merging_vehs;
    vector<double> _buf_merge_priorities;

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

    // Vehicle queue as an index-based FIFO ring buffer (replaces deque<Vehicle*>).
    // veh_ring holds vehicle idx (into World SoA), sized to a power of two.
    // head_seq/tail_seq are monotonically increasing absolute sequence numbers; the
    // queue position of a vehicle is (its stored World::veh_queue_seq) - head_seq.
    vector<int> veh_ring;
    long long veh_head_seq;
    long long veh_tail_seq;
    int veh_ring_mask;

    // Per-link buffers collecting vehicles that reached this link's end this timestep and
    // will transfer to a next link. Filled by the RUN pass (this link's exclusive context)
    // and drained/aggregated into the end_node's incoming_vehicles at the start of
    // Node::transfer. Replaces the RUN pass pushing directly into the shared node buffer.
    vector<Vehicle *> arrived_vehicles;
    vector<Link *> arrived_requests;

    // Queue helpers. q_at_seq/q_front_idx/q_back_idx return a vehicle idx (into World SoA).
    int q_size() const { return (int)(veh_tail_seq - veh_head_seq); }
    bool q_empty() const { return veh_tail_seq == veh_head_seq; }
    int q_front_idx() const { return veh_ring[veh_head_seq & veh_ring_mask]; }
    int q_back_idx() const { return veh_ring[(veh_tail_seq - 1) & veh_ring_mask]; }
    int q_at_seq(long long seq) const { return veh_ring[seq & veh_ring_mask]; }
    void q_pop_front() { veh_head_seq++; }
    void q_push(int veh_idx);  // appends veh_idx, records its seq into World::veh_queue_seq

    vector<double> traveltime_tt; // increments of time
    vector<double> traveltime_t;

    vector<double> arrival_curve;
    vector<double> departure_curve;
    // traveltime_real is lazily materialized: link-exit events are buffered as (start_idx, travel_time) pairs and replayed into the array on first read via ensure_traveltime_real().
    // Members are mutable so that const read paths can trigger materialization.
    mutable vector<double> traveltime_real;
    mutable vector<pair<int, double>> traveltime_real_events;
    mutable size_t traveltime_real_events_applied;
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
    void ensure_traveltime_real() const;
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

    double arrival_time;
    double travel_time;
    double distance_traveled;

    // Position in World::vehicles and index into the World SoA hot-state arrays
    int idx;

    // leader/follower are derived from the current link's queue position instead of
    // being stored: the leader is the vehicle number_of_lanes positions ahead in the
    // link ring buffer, the follower is number_of_lanes positions behind. Both return
    // -1 / nullptr when no such vehicle is on the current link.
    int leader_idx() const;
    int follower_idx() const;
    Vehicle *leader() const;
    Vehicle *follower() const;

    // Hot state (x, x_next, x_old, v, move_remain, state, link, lane) lives in
    // World SoA arrays; these facade accessors read/write it by idx.
    double &x();
    double x() const;
    double &x_next();
    double x_next() const;
    double &x_old();
    double x_old() const;
    double &v();
    double v() const;
    double &move_remain();
    double move_remain() const;
    int &state();  // VehicleState enum: 0=home, 1=wait, 2=run, 3=end, 4=abort
    int state() const;
    int &lane();
    int lane() const;
    Link *link() const;
    void set_link(Link *ln);

    int flag_waiting_for_trip_end;
    int flag_trip_aborted;
    int trip_abort;

    double arrival_time_link;

    // Route logic
    Link *route_next_link;
    int route_choice_flag_on_link;
    double route_adaptive;
    double route_choice_uncertainty;
    vector<double> route_preference;  // indexed by link->id
    int route_choice_principle;
    vector<Link *> links_preferred;
    vector<Link *> links_avoid;
    vector<Link *> specified_route;

    // Incremental tracking for no_cyclic_routing (O(1) lookup instead of log_link scan)
    vector<bool> _traveled_nodes;       // _traveled_nodes[node_id] = true if visited
    int _traveled_link_count;           // number of distinct links traveled (for specified_route)

    // Logging (pre-allocated, indexed by log_size)
    vector<int> log_link;
    vector<double> log_x;
    vector<double> log_v;
    vector<int> log_lane;
    vector<double> log_s;  // spacing to leader on the same link (-1 if none)
    size_t log_size;  // current number of log entries (used instead of push_back)

    // log_t / log_state are fully reconstructible from these scalars.
    // The log structure is always HOME x1, WAIT x log_wait_count, RUN x r, [END|ABORT] x log_end_count, with one entry per timestep after departure; the only exception is trip end (the update() path records the END entry in the same timestep as the immediately preceding RUN entry, so log_last_ts may duplicate the previous timestep).
    int log_first_ts;
    int log_last_ts;
    int log_wait_count;
    int log_end_count;   // single END/ABORT entry at tail, recorded by end_trip()

    Vehicle(
        World *w,
        const string &vehicle_name,
        double departure_time,
        const string &orig_name,
        const string &dest_name);

    void end_trip();
    void route_next_link_choice(const vector<Link*>& linkset, std::mt19937 &rng);
    void enforce_route(vector<Link*> route);
    void record_travel_time(Link *link, double t);
    void reserve_log_arrays();
    void log_data();
    // RUN-state logging with an externally supplied leader spacing (used by the fused
    // RUN system pass, which derives the spacing from the link ring buffer directly).
    void log_data_run(double spacing);

    // Reconstruct log_t / log_state entry i from the scalar counters above.
    double log_t_at(size_t i) const;
    int log_state_at(size_t i) const;

    // Helper: return departure_time in seconds (already in seconds in C++)
    double departure_time_in_second() const;

    // Special link IDs for log_t_link entries
    static constexpr int LOG_T_LINK_HOME = -1;
    static constexpr int LOG_T_LINK_END  = -2;

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
    int show_progress;
    size_t show_progress_deltat_timestep;
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
    // Departure buckets: departure_buckets[ts] holds vehicle idx (id ascending) whose
    // first departing timestep is ts. Rebuilt per main_loop call, replacing the old
    // id-ordered update_order full-scan for HOME->WAIT transitions.
    vector<vector<int>> departure_buckets;

    // Vehicle hot state as SoA arrays, indexed by Vehicle::idx (parallel to `vehicles`)
    vector<double> veh_x;
    vector<double> veh_x_next;
    vector<double> veh_x_old;
    vector<double> veh_v;
    vector<double> veh_move_remain;
    vector<int> veh_state;
    vector<int> veh_link_id;  // -1 = not on any link
    vector<int> veh_lane;
    vector<long long> veh_queue_seq;  // link-ring sequence number at link entry; -1 = not queued
    vector<Link *> links;
    vector<Node *> nodes;
    unordered_map<string, Node *> nodes_map;
    unordered_map<string, Link *> links_map;
    unordered_map<string, Vehicle *> vehicles_map;

    size_t timestep;    //simulation timestep
    double time;    //simulation time in second

    double route_adaptive;
    double route_choice_uncertainty;
    vector<vector<double>> route_preference;   // route_preference[dest_id][link_id]: preference weight for link towards dest
    vector<char> route_pref_active;            // route_pref_active[dest_id]: true once sum(route_preference[dest]) != 0

    // Shared scratch for Vehicle::route_next_link_choice() (serial: called only from the
    // single-threaded generate/RUN passes). Moved off Vehicle to cut per-vehicle fixed cost.
    // To parallelize the RUN pass later, replace these with per-thread/thread_local scratch.
    vector<Link *> _buf_outlinks;
    vector<Link *> _buf_filtered;
    vector<double> _buf_outlink_pref;

    // Graph adjacency
    vector<vector<int>> adj_mat;
    vector<vector<double>> adj_mat_time;
    vector<vector<int>> adj_mat_link_count;    // scratch for multi-link averaging in update_adj_time_matrix
    vector<vector<int>> route_next;
    vector<vector<double>> route_dist;
    map<int, vector<vector<double>>> route_dist_record;

    // Scratch buffers reused across route_search_all() calls (avoid per-call V*V allocation).
    // Filled by route_search_all; callers read from these.
    vector<vector<pair<int, double>>> rsa_adj_list;  // adjacency list (rebuilt each call)
    vector<vector<double>> rsa_dist;                 // all-pairs distances
    vector<vector<int>> rsa_next;                    // all-pairs next-hop
    vector<char> rsa_visited;                        // per-source visited flags

    bool flag_initialized;

    // stats
    double ave_v;
    double ave_vratio;
    double trips_total;
    double trips_completed;

    // Incremental stats accumulators as per-entity partial sums (parallel-safe). RUN-state
    // samples accumulate on the vehicle's current link; WAIT-state samples on the origin
    // node. Read back via the id-ordered reductions below so repeated reads are identical.
    // Indexed by link id (link_*) / node id (node_*), grown in the Link/Node constructors.
    vector<double> link_ave_v_sum;
    vector<double> link_ave_vratio_sum;
    vector<double> link_stat_count;
    vector<double> link_trips_completed;
    vector<double> node_stat_count;

    // Fixed-order (id-ascending) reductions of the per-entity partial sums above.
    double ave_v_sum() const;
    double ave_vratio_sum() const;
    double stat_sample_count() const;
    double trips_completed_count() const;

    // Randomness. The single World rng is split into per-node and per-link streams, seeded
    // deterministically from (random_seed, kind, id) so results are independent of thread
    // count once Phase 8 parallelizes the per-node/per-link stages. node_rngs: Node::transfer
    // shuffle + merge lottery and generate-time route_next_link_choice. link_rngs: RUN-path
    // route_next_link_choice and update_adj_time_matrix per-link noise.
    long long random_seed;
    vector<std::mt19937> node_rngs;
    vector<std::mt19937> link_rngs;

    std::ostream *writer;

    // Wall-clock start of simulation, for the "computation time" column of the progress display (same semantics as Python's W.sim_start_time)
    std::chrono::steady_clock::time_point sim_start_time;

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

    // Update t_max/total_timesteps and resize per-link time-indexed arrays.
    // Must be called before simulation start; the world may be created with a placeholder horizon before the final TMAX is known.
    void set_t_max(double new_t_max);

    void route_choice_duo();
    void route_choice_duo_gradual();

    // All-pairs shortest path (Dijkstra from each source).
    // Results are written into rsa_dist / rsa_next (reused scratch buffers).
    void route_search_all(const vector<vector<double>> &adj, double infty);

    // Enumerate k random routes between all reachable node pairs
    map<pair<int,int>, vector<vector<int>>>
        enumerate_k_random_routes_cpp(int k, unsigned int seed);

    // Estimate congestion externality for a route (sum of per-link externalities)
    double estimate_congestion_externality_route(const vector<Link*> &route, double departure_time);

    void print_scenario_stats();
    void print_simple_results();
    void reset_sim_start_time();
    void print_progress_line(double t_print);
    void main_loop(double duration_t, double end_t);

    bool check_simulation_ongoing();

    Node *get_node(const string &node_name);
    Link *get_link(const string &link_name);
    Vehicle *get_vehicle_by_index(int index) const;

    size_t vehicle_log_reserve_size;
    bool vehicle_log_mode;

    // Helper: return vehicle states as a vector of (name, state_int) pairs
    // Useful for Python to update VEHICLES_LIVING/RUNNING without per-vehicle calls
    std::vector<std::pair<std::string, int>> get_all_vehicle_states() const;

    // Build enter_log data: returns (link_id, time, vehicle_index) triples for all vehicle link-entry events.
    // Used by _build_vehicles_enter_log.
    struct EnterLogEntry {
        int link_id;
        double time;
        int vehicle_index;
    };
    std::vector<EnterLogEntry> build_enter_log_data() const;

    // Compact flat SoA log for all vehicles. No home prepend — only actual log entries.
    // n_missing[i] = number of home timesteps before vehicle i's first log entry.
    struct CompactFlatLogs {
        std::vector<double> log_t;
        std::vector<double> log_x;
        std::vector<double> log_v;
        std::vector<int>    log_state;
        std::vector<double> log_s;
        std::vector<int>    log_lane;
        std::vector<int>    log_link;
        std::vector<size_t> offsets;      // size = vehicles.size() + 1
        std::vector<int>    n_missing;    // size = vehicles.size()
        std::vector<double> ltl_t;
        std::vector<int>    ltl_id;
        std::vector<size_t> ltl_offsets;  // size = vehicles.size() + 1
    };
    CompactFlatLogs build_all_vehicle_logs_flat_compact() const;

};

// -----------------------------------------------------------------------
// MARK: Vehicle hot-state accessors (SoA facade)
// -----------------------------------------------------------------------

inline double &Vehicle::x() { return w->veh_x[idx]; }
inline double Vehicle::x() const { return w->veh_x[idx]; }
inline double &Vehicle::x_next() { return w->veh_x_next[idx]; }
inline double Vehicle::x_next() const { return w->veh_x_next[idx]; }
inline double &Vehicle::x_old() { return w->veh_x_old[idx]; }
inline double Vehicle::x_old() const { return w->veh_x_old[idx]; }
inline double &Vehicle::v() { return w->veh_v[idx]; }
inline double Vehicle::v() const { return w->veh_v[idx]; }
inline double &Vehicle::move_remain() { return w->veh_move_remain[idx]; }
inline double Vehicle::move_remain() const { return w->veh_move_remain[idx]; }
inline int &Vehicle::state() { return w->veh_state[idx]; }
inline int Vehicle::state() const { return w->veh_state[idx]; }
inline int &Vehicle::lane() { return w->veh_lane[idx]; }
inline int Vehicle::lane() const { return w->veh_lane[idx]; }
inline Link *Vehicle::link() const { int lid = w->veh_link_id[idx]; return lid >= 0 ? w->links[lid] : nullptr; }
inline void Vehicle::set_link(Link *ln) { w->veh_link_id[idx] = ln ? ln->id : -1; }

// leader/follower derived from the current link ring buffer (see declarations).
inline int Vehicle::leader_idx() const {
    int lid = w->veh_link_id[idx];
    if (lid < 0) return -1;
    Link *lk = w->links[lid];
    long long seq_leader = w->veh_queue_seq[idx] - lk->number_of_lanes;
    return (seq_leader >= lk->veh_head_seq) ? lk->q_at_seq(seq_leader) : -1;
}
inline int Vehicle::follower_idx() const {
    int lid = w->veh_link_id[idx];
    if (lid < 0) return -1;
    Link *lk = w->links[lid];
    long long seq_follower = w->veh_queue_seq[idx] + lk->number_of_lanes;
    return (seq_follower < lk->veh_tail_seq) ? lk->q_at_seq(seq_follower) : -1;
}
inline Vehicle *Vehicle::leader() const { int li = leader_idx(); return li >= 0 ? w->vehicles[li] : nullptr; }
inline Vehicle *Vehicle::follower() const { int fi = follower_idx(); return fi >= 0 ? w->vehicles[fi] : nullptr; }
