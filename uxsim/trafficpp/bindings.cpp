// clang-format off

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/map.h>
#include <nanobind/ndarray.h>

#include <memory>
#include <string>
#include <iostream>
#include <streambuf>
#include <cstring>

#include "traffi.cpp"

namespace nb = nanobind;

// ----------------------------------------------------------------------
// Helper: create a numpy array that owns a copy of the data (for raw pointers)
// ----------------------------------------------------------------------
template<typename T>
nb::ndarray<nb::numpy, T, nb::ndim<1>> make_numpy_copy(const T* data, size_t n) {
    T* copy = new T[n];
    std::memcpy(copy, data, n * sizeof(T));
    nb::capsule owner(copy, [](void* p) noexcept { delete[] static_cast<T*>(p); });
    return nb::ndarray<nb::numpy, T, nb::ndim<1>>(copy, {n}, owner);
}

// ----------------------------------------------------------------------
// Helper: move a vector into a numpy array (zero-copy, vector owns data)
// ----------------------------------------------------------------------
template<typename T>
nb::ndarray<nb::numpy, T, nb::ndim<1>> make_numpy_move(std::vector<T>&& vec) {
    auto* v = new std::vector<T>(std::move(vec));
    nb::capsule owner(v, [](void* p) noexcept { delete static_cast<std::vector<T>*>(p); });
    return nb::ndarray<nb::numpy, T, nb::ndim<1>>(v->data(), {v->size()}, owner);
}

// ----------------------------------------------------------------------
// Interned Python strings for state names.
// Heap-allocated and explicitly cleaned up via atexit to avoid
// static destruction order issues (segfault at interpreter shutdown).
// ----------------------------------------------------------------------
struct InternedStrings {
    nb::str states[5];   // home, wait, run, end, abort
    nb::str unknown;
    nb::str minus1;

    void init() {
        states[0] = nb::str("home");
        states[1] = nb::str("wait");
        states[2] = nb::str("run");
        states[3] = nb::str("end");
        states[4] = nb::str("abort");
        unknown = nb::str("unknown");
        minus1 = nb::str("-1");
    }
};
static InternedStrings *g_intern = nullptr;


// ----------------------------------------------------------------------
// Custom streambuf that redirects C++ output to Python sys.stdout.
// Uses raw PyObject* instead of nb::object to avoid GIL issues
// during static destruction at Python interpreter shutdown.
// ----------------------------------------------------------------------
class py_stdout_redirect_buf : public std::streambuf {
public:
    py_stdout_redirect_buf() : py_stdout_raw(nullptr) {
        nb::object sys = nb::module_::import_("sys");
        nb::object stdout_obj = sys.attr("stdout");
        py_stdout_raw = stdout_obj.ptr();
        Py_XINCREF(py_stdout_raw);
    }

    ~py_stdout_redirect_buf() {
        // Only decref if Python interpreter is still alive
        if (Py_IsInitialized() && py_stdout_raw) {
            nb::gil_scoped_acquire gil;
            Py_XDECREF(py_stdout_raw);
        }
        py_stdout_raw = nullptr;
    }

protected:
    virtual int overflow(int c) override {
        if (c != EOF && Py_IsInitialized() && py_stdout_raw) {
            nb::gil_scoped_acquire gil;
            std::string s(1, static_cast<char>(c));
            PyObject *result = PyObject_CallMethod(py_stdout_raw, "write", "s", s.c_str());
            Py_XDECREF(result);
        }
        return c;
    }

    virtual std::streamsize xsputn(const char* s, std::streamsize n) override {
        if (Py_IsInitialized() && py_stdout_raw) {
            nb::gil_scoped_acquire gil;
            std::string str(s, n);
            PyObject *result = PyObject_CallMethod(py_stdout_raw, "write", "s", str.c_str());
            Py_XDECREF(result);
        }
        return n;
    }

private:
    PyObject *py_stdout_raw;
};

// ----------------------------------------------------------------------
// Helper to get a Python-redirected ostream.
// Uses heap allocation to control destruction order and avoid
// static destruction crashes at interpreter shutdown.
// ----------------------------------------------------------------------
static py_stdout_redirect_buf *g_custom_buf = nullptr;
static std::ostream *g_pyout = nullptr;

std::ostream* get_pyout() {
    if (!g_pyout) {
        g_custom_buf = new py_stdout_redirect_buf();
        g_pyout = new std::ostream(g_custom_buf);
    }
    return g_pyout;
}

// ----------------------------------------------------------------------
// create_world() の実装（World::writer を Python の sys.stdout 経由の ostream に切り替える）
// ----------------------------------------------------------------------
std::unique_ptr<World> create_world(
        const std::string &world_name,
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
        bool no_cyclic_routing = false){
    auto world = std::make_unique<World>(
        world_name,
        t_max,
        delta_n,
        tau,
        duo_update_time,
        duo_update_weight,
        route_choice_uncertainty,
        print_mode,
        random_seed,
        vehicle_log_mode,
        hard_deterministic_mode,
        route_choice_update_gradual,
        no_cyclic_routing);
    // World 内の出力先を Python の sys.stdout 経由に変更
    world->writer = get_pyout();
    return world;
}

// ----------------------------------------------------------------------
// シナリオ定義関数
// ----------------------------------------------------------------------
void add_node(World &world, const std::string &node_name, double x, double y,
        vector<double> signal_intervals = {0}, double signal_offset = 0,
        double flow_capacity = -1.0, int number_of_lanes = 0) {
    new Node(&world, node_name, x, y, signal_intervals, signal_offset, flow_capacity, number_of_lanes);
}

void add_link(
        World &world,
        const std::string &link_name,
        const std::string &start_node_name,
        const std::string &end_node_name,
        double vmax,
        double kappa,
        double length,
        int number_of_lanes,
        double merge_priority,
        double capacity_out,
        double capacity_in,
        vector<int> signal_group={0}){
    new Link(&world, link_name, start_node_name, end_node_name,
                        vmax, kappa, length, number_of_lanes, merge_priority, capacity_out, capacity_in, signal_group);
}

void add_demand(
    World *w,
    const std::string &orig_name,
    const std::string &dest_name,
    double start_t,
    double end_t,
    double flow,
    vector<string> links_preferred_str);

// ----------------------------------------------------------------------
// コンパイル日時を返す関数
// ----------------------------------------------------------------------
std::string get_compile_datetime() {
    return std::string("Compiled on ") + __DATE__ + " at " + __TIME__;
}

// ----------------------------------------------------------------------
// nanobind モジュール定義
// ----------------------------------------------------------------------
NB_MODULE(uxsim_cpp, m) {
    m.doc() = "uxsim_cpp: nanobind bindings for C++ mesoscopic traffic simulation (UXsim integrated)";

    //
    // MARK: Scenario
    //
    m.def("create_world", &create_world,
          nb::arg("world_name"),
          nb::arg("t_max"),
          nb::arg("delta_n"),
          nb::arg("tau"),
          nb::arg("duo_update_time"),
          nb::arg("duo_update_weight"),
          nb::arg("route_choice_uncertainty"),
          nb::arg("print_mode"),
          nb::arg("random_seed"),
          nb::arg("vehicle_log_mode"),
          nb::arg("hard_deterministic_mode") = false,
          nb::arg("route_choice_update_gradual") = false,
          nb::arg("no_cyclic_routing") = false,
          R"docstring(
          Create a World (simulation environment).

          Parameters
          ----------
          world_name : str
              The name of the world.
          t_max : float
              The simulation duration.
          delta_n : float
              The platoon size.
          tau : float
              The reaction time.
          duo_update_time : float
              The time interval for route choice update.
          duo_update_weight : float
              The update weight for route choice.
          route_choice_uncertainty : float
              The noise in route choice.
          print_mode : int
              Whether print the simulation progress or not.
          random_seed : int
              The random seed.
          vehicle_log_mode : bool
              Whether save vehicle data or not.

          Returns
          -------
          World
              World simulation object.
          )docstring");

    m.def("add_node", &add_node,
          nb::arg("world"),
          nb::arg("node_name"),
          nb::arg("x"),
          nb::arg("y"),
          nb::arg("signal_intervals") = vector<double>{0},
          nb::arg("signal_offset") = 0.0,
          nb::arg("flow_capacity") = -1.0,
          nb::arg("number_of_lanes") = 0,
          R"docstring(
          Add a node to the world.

          Parameters
          ----------
          world : World
              The world to which the node belongs.
          node_name : str
              The name of the node.
          x : float
              The x-coordinate of the node.
          y : float
              The y-coordinate of the node.
          signal_intervals : list of float
              A list representing the signal at the node.
          signal_offset : float
              The offset of the signal.
          flow_capacity : float
              The flow capacity of the node. -1 means unlimited.
          number_of_lanes : int
              The number of lanes at the node. 0 means auto-determined.
          )docstring");

    m.def("add_link", &add_link,
          nb::arg("world"),
          nb::arg("link_name"),
          nb::arg("start_node_name"),
          nb::arg("end_node_name"),
          nb::arg("vmax"),
          nb::arg("kappa"),
          nb::arg("length"),
          nb::arg("number_of_lanes"),
          nb::arg("merge_priority"),
          nb::arg("capacity_out"),
          nb::arg("capacity_in"),
          nb::arg("signal_group"),
          R"docstring(
          Add a link to the world.

          Parameters
          ----------
          world : World
              The world to which the link belongs.
          link_name : str
              The name of the link.
          start_node_name : str
              The name of the start node.
          end_node_name : str
              The name of the end node.
          vmax : float
              The free flow speed on the link.
          kappa : float
              The jam density on the link.
          length : float
              The length of the link.
          number_of_lanes : int
              The number of lanes on the link.
          merge_priority : float
              The priority of the link when merging.
          capacity_out : float
              The capacity out of the link.
          capacity_in : float
              The capacity into the link.
          signal_group : list of int
              The signal group(s) to which the link belongs.
          )docstring");

    m.def("add_demand", &add_demand,
          nb::arg("world"),
          nb::arg("orig_name"),
          nb::arg("dest_name"),
          nb::arg("start_t"),
          nb::arg("end_t"),
          nb::arg("flow"),
          nb::arg("links_preferred_str"),
          R"docstring(
          Add demand (vehicle generation) to the world.

          Parameters
          ----------
          world : World
              The world to which the demand belongs.
          orig_name : str
              The origin node.
          dest_name : str
              The destination node.
          start_t : float
              The start time of demand.
          end_t : float
              The end time of demand.
          flow : float
              The flow rate of vehicles.
          links_preferred_str : list of str
              The names of the links the vehicles prefer.
          )docstring");

    //
    // 2) MARK: World
    //
    nb::class_<World>(m, "World")
        .def("initialize_adj_matrix", &World::initialize_adj_matrix)
        .def("print_scenario_stats", &World::print_scenario_stats)
        .def("main_loop", &World::main_loop)
        .def("check_simulation_ongoing", &World::check_simulation_ongoing)
        .def("print_simple_results", &World::print_simple_results)
        .def("update_adj_time_matrix", &World::update_adj_time_matrix)
        .def("enumerate_k_random_routes_cpp", [](World &w, int k, unsigned int seed) -> nb::dict {
                 auto result = w.enumerate_k_random_routes_cpp(k, seed);
                 nb::dict out;
                 for (auto &[key, routes] : result){
                     // Use node names as keys to match Python API
                     nb::tuple py_key = nb::make_tuple(
                         nb::str(w.nodes[key.first]->name.c_str()),
                         nb::str(w.nodes[key.second]->name.c_str()));
                     nb::list py_routes;
                     for (auto &route : routes){
                         nb::list py_route;
                         for (int lid : route){
                             py_route.append(nb::str(w.links[lid]->name.c_str()));
                         }
                         py_routes.append(py_route);
                     }
                     out[py_key] = py_routes;
                 }
                 return out;
             },
             nb::arg("k"), nb::arg("seed"),
             "Enumerate k random routes between all node pairs (C++ accelerated)")
        .def("estimate_congestion_externality_route",
             [](World &w, nb::list py_route, double departure_time) -> double {
                 std::vector<Link*> route;
                 for (auto item : py_route){
                     std::string name = nb::cast<std::string>(item.attr("name"));
                     Link *ln = w.get_link(name);
                     if (ln) route.push_back(ln);
                 }
                 return w.estimate_congestion_externality_route(route, departure_time);
             },
             nb::arg("route"), nb::arg("departure_time"),
             "Estimate congestion externality for a route (list of Link objects)")
        .def("get_node", &World::get_node,
             nb::rv_policy::reference,
             "Get a Node by name (reference)")
        .def("get_link", &World::get_link,
             nb::rv_policy::reference,
             "Get a Link by name (reference)")
        .def("get_vehicle", &World::get_vehicle,
             nb::rv_policy::reference,
             "Get a Vehicle by name (reference)")
        .def("get_link_by_id", &World::get_link_by_id,
             nb::rv_policy::reference,
             "Get a Link by integer ID (reference)")
        .def("get_vehicle_by_index", &World::get_vehicle_by_index,
             nb::rv_policy::reference,
             "Get a Vehicle by index in vehicles vector (O(1), no list conversion)")
        // Aliases for explicit by-name lookup (same as get_node/get_link)
        .def("get_node_by_name", &World::get_node,
             nb::arg("name"),
             nb::rv_policy::reference,
             "Get a Node by name string (alias for get_node)")
        .def("get_link_by_name", &World::get_link,
             nb::arg("name"),
             nb::rv_policy::reference,
             "Get a Link by name string (alias for get_link)")
        .def_ro("VEHICLES", &World::vehicles,
                      "Vector of pointers to all Vehicles in the world.")
        .def_prop_ro("vehicle_count", [](const World &w) { return w.vehicles.size(); },
                      "Number of vehicles (O(1), avoids list conversion)")
        .def("link_vehicle_count", [](const World &w, int link_id) -> size_t {
                 if (link_id >= 0 && link_id < (int)w.links.size())
                     return w.links[link_id]->vehicles.size();
                 return 0;
             },
             nb::arg("link_id"),
             "Get vehicle count on a link by ID (O(1), no list conversion)")
        .def_ro("LINKS", &World::links,
                      "Vector of pointers to all Links in the world.")
        .def_ro("NODES", &World::nodes,
                      "Vector of pointers to all Nodes in the world.")
        .def_ro("timestep", &World::timestep)
        .def_ro("time", &World::time)
        .def_ro("delta_t", &World::delta_t)
        .def_ro("DELTAT", &World::delta_t)
        .def_ro("t_max", &World::t_max)
        .def_ro("TMAX", &World::t_max)
        .def_ro("name", &World::name)
        .def_ro("deltan", &World::delta_n)
        .def_rw("hard_deterministic_mode", &World::hard_deterministic_mode)
        .def_rw("route_choice_update_gradual", &World::route_choice_update_gradual)
        .def_rw("no_cyclic_routing", &World::no_cyclic_routing)
        .def_rw("instantaneous_TT_timestep_interval", &World::instantaneous_TT_timestep_interval)
        .def_ro("route_dist", &World::route_dist)
        .def_ro("route_dist_record", &World::route_dist_record)
        .def_ro("route_next", &World::route_next)
        .def_rw("adj_mat_time", &World::adj_mat_time)
        .def_rw("route_preference", &World::route_preference)
        .def_rw("vehicle_log_mode", &World::vehicle_log_mode)
        .def_ro("ave_v", &World::ave_v)
        .def_ro("ave_vratio", &World::ave_vratio)
        .def_ro("trips_total", &World::trips_total)
        .def_ro("trips_completed", &World::trips_completed)
        .def_ro("flag_initialized", &World::flag_initialized)
        .def_ro("node_id", &World::node_id)
        .def_ro("link_id", &World::link_id)
        .def_ro("vehicle_id", &World::vehicle_id)
        .def_ro("adj_mat", &World::adj_mat)
        .def_ro("random_seed", &World::random_seed)
        .def_ro("total_timesteps", &World::total_timesteps)
        .def_ro("delta_n", &World::delta_n)
        .def_rw("duo_update_time", &World::duo_update_time)
        .def_rw("duo_update_weight", &World::duo_update_weight)
        .def_rw("route_choice_uncertainty", &World::route_choice_uncertainty)
        .def_ro("vehicles_map", &World::vehicles_map)
        .def_ro("nodes_map", &World::nodes_map)
        .def_ro("links_map", &World::links_map)
        .def("get_vehicles_by_state", &World::get_vehicles_by_state,
             nb::arg("state"),
             nb::rv_policy::reference,
             "Get vehicles filtered by state (0=home,1=wait,2=run,3=end,4=abort)")
        .def("get_all_vehicle_states", &World::get_all_vehicle_states,
             "Get (name, state_int) pairs for all vehicles. "
             "Abort is detected and returned as state 4.")
        .def("get_vehicle_states_by_index", &World::get_vehicle_states_by_index,
             "Get effective state ints indexed by vehicle order. "
             "0=home, 1=wait, 2=run, 3=end, 4=abort. "
             "Index matches World.VEHICLES vector order.")
        .def("build_all_vehicle_logs", [](const World &w) -> nb::list {
                 // Batch: build full logs for ALL vehicles in one C++ call.
                 // All data as numpy arrays — zero-copy via vector ownership transfer.
                 nb::list result;
                 for (auto *v : w.vehicles) {
                     auto fl = v->build_full_log();
                     size_t m = fl.log_t_link.size();
                     nb::dict d;
                     d["log_t"] = make_numpy_move(std::move(fl.log_t));
                     d["log_x"] = make_numpy_move(std::move(fl.log_x));
                     d["log_v"] = make_numpy_move(std::move(fl.log_v));
                     d["log_s"] = make_numpy_move(std::move(fl.log_s));
                     d["log_lane"] = make_numpy_move(std::move(fl.log_lane));
                     d["log_link"] = make_numpy_move(std::move(fl.log_link));
                     d["log_state"] = make_numpy_move(std::move(fl.log_state));
                     // log_t_link as two parallel numpy arrays
                     std::vector<double> ltl_t(m);
                     std::vector<int> ltl_id(m);
                     for (size_t i = 0; i < m; i++) {
                         ltl_t[i] = fl.log_t_link[i].first;
                         ltl_id[i] = fl.log_t_link[i].second;
                     }
                     d["log_t_link_t"] = make_numpy_move(std::move(ltl_t));
                     d["log_t_link_id"] = make_numpy_move(std::move(ltl_id));
                     result.append(d);
                 }
                 return result;
             },
             "Build full logs for all vehicles in batch. Returns list of dicts with numpy arrays.")
        .def("build_all_vehicle_logs_flat", [](const World &w) -> nb::dict {
                 // Flat SoA: all vehicles' logs in contiguous arrays + offset arrays.
                 // Returns dict with 7 data arrays + offsets + 2 ltl arrays + ltl_offsets.
                 auto fl = w.build_all_vehicle_logs_flat();
                 nb::dict d;
                 d["log_t"] = make_numpy_move(std::move(fl.log_t));
                 d["log_x"] = make_numpy_move(std::move(fl.log_x));
                 d["log_v"] = make_numpy_move(std::move(fl.log_v));
                 d["log_state"] = make_numpy_move(std::move(fl.log_state));
                 d["log_s"] = make_numpy_move(std::move(fl.log_s));
                 d["log_lane"] = make_numpy_move(std::move(fl.log_lane));
                 d["log_link"] = make_numpy_move(std::move(fl.log_link));
                 // offsets as uint64 numpy array (size_t)
                 {
                     size_t n = fl.offsets.size();
                     auto *buf = new std::vector<int64_t>(n);
                     for (size_t i = 0; i < n; i++) (*buf)[i] = static_cast<int64_t>(fl.offsets[i]);
                     nb::capsule owner(buf, [](void* p) noexcept { delete static_cast<std::vector<int64_t>*>(p); });
                     d["offsets"] = nb::ndarray<nb::numpy, int64_t, nb::ndim<1>>(buf->data(), {n}, owner);
                 }
                 d["ltl_t"] = make_numpy_move(std::move(fl.ltl_t));
                 d["ltl_id"] = make_numpy_move(std::move(fl.ltl_id));
                 {
                     size_t n = fl.ltl_offsets.size();
                     auto *buf = new std::vector<int64_t>(n);
                     for (size_t i = 0; i < n; i++) (*buf)[i] = static_cast<int64_t>(fl.ltl_offsets[i]);
                     nb::capsule owner(buf, [](void* p) noexcept { delete static_cast<std::vector<int64_t>*>(p); });
                     d["ltl_offsets"] = nb::ndarray<nb::numpy, int64_t, nb::ndim<1>>(buf->data(), {n}, owner);
                 }
                 return d;
             },
             "Build flat SoA logs for all vehicles. Returns dict with contiguous arrays + offset arrays.")
        .def("build_enter_log_data", [](const World &w) -> nb::dict {
                 auto entries = w.build_enter_log_data();
                 size_t n = entries.size();
                 std::vector<int> link_ids(n);
                 std::vector<double> times(n);
                 std::vector<int> veh_indices(n);
                 for (size_t i = 0; i < n; i++) {
                     link_ids[i] = entries[i].link_id;
                     times[i] = entries[i].time;
                     veh_indices[i] = entries[i].vehicle_index;
                 }
                 nb::dict d;
                 d["link_id"] = make_numpy_move(std::move(link_ids));
                 d["time"] = make_numpy_move(std::move(times));
                 d["vehicle_index"] = make_numpy_move(std::move(veh_indices));
                 return d;
             },
             "Build enter_log data for all vehicles. Returns dict with link_id/time/vehicle_index arrays.")
        ;

    //
    // MARK: Node
    //
    nb::class_<Node>(m, "Node")
        .def(nb::init<World *, const std::string &, double, double>(),
             nb::arg("world"),
             nb::arg("node_name"),
             nb::arg("x"),
             nb::arg("y"))
        .def_ro("W", &Node::w)
        .def_ro("id", &Node::id)
        .def_ro("name", &Node::name)
        .def_rw("x", &Node::x)
        .def_rw("y", &Node::y)
        .def_rw("signal_intervals", &Node::signal_intervals)
        .def_rw("signal_offset", &Node::signal_offset)
        .def_rw("signal_t", &Node::signal_t)
        .def_rw("signal_phase", &Node::signal_phase)
        .def_ro("signal_log", &Node::signal_log)
        .def_rw("flow_capacity", &Node::flow_capacity)
        .def_rw("flow_capacity_remain", &Node::flow_capacity_remain)
        .def_rw("number_of_lanes", &Node::number_of_lanes)
        .def_ro("in_links", &Node::in_links)
        .def_ro("out_links", &Node::out_links)
        .def_ro("incoming_vehicles", &Node::incoming_vehicles)
        .def_prop_ro("generation_queue", [](const Node &n) {
                 nb::list result;
                 for (auto *v : n.generation_queue) result.append(v);
                 return result;
             })
        .def("generate", &Node::generate)
        .def("transfer", &Node::transfer)
        .def("signal_update", &Node::signal_update)
        .def("flow_capacity_update", &Node::flow_capacity_update)
        ;

    //
    // MARK: Link
    //
    nb::class_<Link>(m, "Link")
        .def(nb::init<World *, const std::string &, const std::string &, const std::string &,
                      double, double, double, int, double, double, double>(),
             nb::arg("world"),
             nb::arg("link_name"),
             nb::arg("start_node_name"),
             nb::arg("end_node_name"),
             nb::arg("vmax"),
             nb::arg("kappa"),
             nb::arg("length"),
             nb::arg("number_of_lanes"),
             nb::arg("merge_priority"),
             nb::arg("capacity_out"),
             nb::arg("capacity_in"))
        .def_ro("W", &Link::w)
        .def_ro("id", &Link::id)
        .def_ro("name", &Link::name)
        .def_rw("length", &Link::length)
        .def_rw("number_of_lanes", &Link::number_of_lanes)
        .def_rw("u", &Link::vmax)
        .def_rw("vmax", &Link::vmax)
        .def_rw("kappa", &Link::kappa)
        .def_rw("delta", &Link::delta)
        .def_rw("delta_per_lane", &Link::delta_per_lane)
        .def_rw("tau", &Link::tau)
        .def_rw("capacity", &Link::capacity)
        .def_rw("w", &Link::backward_wave_speed)
        .def_rw("merge_priority", &Link::merge_priority)
        .def_rw("capacity_out", &Link::capacity_out)
        .def_rw("capacity_in", &Link::capacity_in)
        .def_rw("signal_group", &Link::signal_group)
        .def_rw("route_choice_penalty", &Link::route_choice_penalty)
        .def("set_toll_timeseries", [](Link &self, std::vector<double> ts){
            self.toll_timeseries = std::move(ts);
        })
        .def("estimate_congestion_externality", &Link::estimate_congestion_externality,
             nb::arg("timestep"),
             "Estimate congestion externality for this link at given timestep")
        .def("get_toll_timeseries_size", [](Link &self) -> int {
            return (int)self.toll_timeseries.size();
        })
        .def("get_toll_at_timestep", [](Link &self, int ts) -> double {
            if (ts >= 0 && ts < (int)self.toll_timeseries.size())
                return self.toll_timeseries[ts];
            return -1.0;
        })
        // Aliases matching Python uxsim naming
        .def_rw("free_flow_speed", &Link::vmax)
        .def_rw("backward_wave_speed", &Link::backward_wave_speed)
        .def_rw("capacity_out_remain", &Link::capacity_out_remain)
        .def_rw("capacity_in_remain", &Link::capacity_in_remain)
        .def_ro("start_node", &Link::start_node)
        .def_ro("end_node", &Link::end_node)
        .def_prop_ro("vehicles", [](const Link &l) {
                 nb::list result;
                 for (auto *v : l.vehicles) result.append(v);
                 return result;
             })
        .def_ro("arrival_curve", &Link::arrival_curve)
        .def_ro("cum_arrival", &Link::arrival_curve)
        .def_ro("departure_curve", &Link::departure_curve)
        .def_ro("cum_departure", &Link::departure_curve)
        .def_ro("traveltime_real", &Link::traveltime_real)
        .def_ro("traveltime_actual", &Link::traveltime_real)
        .def_ro("traveltime_instant", &Link::traveltime_instant)
        .def("update", &Link::update)
        .def("set_travel_time", &Link::set_travel_time)
        .def("change_free_flow_speed", &Link::change_free_flow_speed, nb::arg("new_value"))
        .def("change_jam_density", &Link::change_jam_density, nb::arg("new_value"))
        .def("count_vehicles_in_queue", &Link::count_vehicles_in_queue)
        .def_prop_ro("vehicle_count", [](const Link &l) -> size_t {
                 return l.vehicles.size();
             },
             "Number of vehicles on this link (O(1), no list conversion)")
        .def_prop_ro("avg_speed", [](const Link &l) -> double {
                 if (l.vehicles.empty()) return l.vmax;
                 double sum = 0;
                 for (auto *v : l.vehicles) sum += v->v;
                 return sum / l.vehicles.size();
             },
             "Average speed of vehicles on this link (computed in C++)")
        // Numpy array accessors — avoid list() conversion overhead on Python side
        .def("get_cum_arrival_np", [](const Link &l) {
                 return make_numpy_copy(l.arrival_curve.data(), l.arrival_curve.size());
             },
             "Return cum_arrival as a numpy array (memcpy, no list conversion)")
        .def("get_cum_departure_np", [](const Link &l) {
                 return make_numpy_copy(l.departure_curve.data(), l.departure_curve.size());
             },
             "Return cum_departure as a numpy array (memcpy, no list conversion)")
        .def("get_traveltime_actual_np", [](const Link &l) {
                 return make_numpy_copy(l.traveltime_real.data(), l.traveltime_real.size());
             },
             "Return traveltime_actual (traveltime_real) as a numpy array")
        .def("get_traveltime_instant_np", [](const Link &l) {
                 return make_numpy_copy(l.traveltime_instant.data(), l.traveltime_instant.size());
             },
             "Return traveltime_instant as a numpy array")
        ;

    //
    // MARK: Vehicle
    //
    nb::class_<Vehicle>(m, "Vehicle")
        .def(nb::init<World *, const std::string &, double, const std::string &, const std::string &>(),
             nb::arg("world"),
             nb::arg("name"),
             nb::arg("departure_time"),
             nb::arg("orig_name"),
             nb::arg("dest_name"))
        .def_ro("W", &Vehicle::w)
        .def_ro("id", &Vehicle::id)
        .def_rw("name", &Vehicle::name)
        .def_ro("departure_time", &Vehicle::departure_time)
        .def_rw("orig", &Vehicle::orig)
        .def_rw("dest", &Vehicle::dest)
        .def_ro("link", &Vehicle::link)
        .def_ro("x", &Vehicle::x)
        .def_ro("x_next", &Vehicle::x_next)
        .def_ro("v", &Vehicle::v)
        .def_ro("lane", &Vehicle::lane)
        .def_ro("leader", &Vehicle::leader)
        .def_ro("follower", &Vehicle::follower)
        .def_rw("state", &Vehicle::state)
        .def_ro("arrival_time_link", &Vehicle::arrival_time_link)
        .def_rw("route_next_link", &Vehicle::route_next_link)
        .def_rw("route_choice_flag_on_link", &Vehicle::route_choice_flag_on_link)
        .def_rw("route_adaptive", &Vehicle::route_adaptive)
        .def_rw("route_preference", &Vehicle::route_preference)
        .def_rw("links_preferred", &Vehicle::links_preferred)
        .def_rw("links_avoid", &Vehicle::links_avoid)
        .def_rw("specified_route", &Vehicle::specified_route)
        .def("enforce_route", &Vehicle::enforce_route, nb::arg("route"))
        .def_rw("move_remain", &Vehicle::move_remain)
        .def_ro("x_old", &Vehicle::x_old)
        .def_rw("trip_abort", &Vehicle::trip_abort)
        .def_ro("flag_trip_aborted", &Vehicle::flag_trip_aborted)
        .def_ro("flag_waiting_for_trip_end", &Vehicle::flag_waiting_for_trip_end)
        .def_ro("log_t", &Vehicle::log_t)
        .def_ro("log_state", &Vehicle::log_state)
        .def_ro("log_link", &Vehicle::log_link)
        .def_ro("log_x", &Vehicle::log_x)
        .def_ro("log_v", &Vehicle::log_v)
        .def_ro("log_lane", &Vehicle::log_lane)
        .def_ro("arrival_time", &Vehicle::arrival_time)
        .def_ro("travel_time", &Vehicle::travel_time)
        .def_rw("distance_traveled", &Vehicle::distance_traveled)
        .def("state_str", &Vehicle::state_str,
             "Return state as string: home/wait/run/end/abort")
        .def_prop_ro("state_str_p",
             [](const Vehicle &v) { return v.state_str(); },
             "State as string property: home/wait/run/end/abort")
        .def("log_state_str", &Vehicle::log_state_str,
             "Return log_state as vector of strings")
        .def("log_link_names", &Vehicle::log_link_names,
             "Return log_link as vector of link name strings")
        .def_prop_ro("departure_time_in_second", &Vehicle::departure_time_in_second,
             "Departure time in seconds")
        .def_rw("route_choice_principle", &Vehicle::route_choice_principle)
        .def_rw("route_choice_uncertainty", &Vehicle::route_choice_uncertainty)
        .def("build_log_t_link", &Vehicle::build_log_t_link,
             "Build log_t_link: list of (time, label) pairs for link transitions")
        .def("build_full_log", [](const Vehicle &v) -> nb::dict {
                 // Per-vehicle version: returns Python lists with string conversions
                 // for backward compatibility with existing wrapper code.
                 // Optimization: use heap-allocated interned strings (g_intern) to
                 // reuse same PyObject, use make_numpy_copy for numeric arrays.
                 auto *si = g_intern;
                 auto fl = v.build_full_log();
                 size_t n = fl.log_t.size();
                 nb::dict d;
                 // Keep numeric data as Python lists (nb::cast) for backward
                 // compatibility — wrapper fallback path expects list, not ndarray.
                 d["log_t"] = nb::cast(fl.log_t);
                 d["log_x"] = nb::cast(fl.log_x);
                 d["log_v"] = nb::cast(fl.log_v);
                 d["log_s"] = nb::cast(fl.log_s);
                 d["log_lane"] = nb::cast(fl.log_lane);
                 d["log_link"] = nb::cast(fl.log_link);
                 // State int→string using interned Python strings (no new allocation)
                 nb::list state_strs;
                 for (size_t i = 0; i < n; i++) {
                     int s = fl.log_state[i];
                     state_strs.append((s >= 0 && s <= 4) ? si->states[s] : si->unknown);
                 }
                 d["log_state"] = state_strs;
                 // Build log_t_link with interned label strings
                 size_t m = fl.log_t_link.size();
                 nb::list ltl;
                 for (size_t i = 0; i < m; i++) {
                     double t = fl.log_t_link[i].first;
                     int lid = fl.log_t_link[i].second;
                     nb::list entry;
                     entry.append(nb::float_(t));
                     if (lid == Vehicle::LOG_T_LINK_HOME) {
                         entry.append(si->states[0]);  // "home"
                     } else if (lid == Vehicle::LOG_T_LINK_END) {
                         entry.append(si->states[3]);  // "end"
                     } else if (lid >= 0 && lid < static_cast<int>(v.w->links.size())) {
                         entry.append(nb::str(v.w->links[lid]->name.c_str()));
                     } else {
                         entry.append(si->minus1);
                     }
                     ltl.append(entry);
                 }
                 d["log_t_link"] = ltl;
                 return d;
             },
             "Build full log arrays with string conversions (backward compatible)")
        .def("build_full_log_np", [](const Vehicle &v) -> nb::dict {
                 // Per-vehicle numpy version: all data as numpy/int, no strings.
                 // Zero-copy via vector ownership transfer.
                 auto fl = v.build_full_log();
                 size_t m = fl.log_t_link.size();
                 nb::dict d;
                 d["log_t"] = make_numpy_move(std::move(fl.log_t));
                 d["log_x"] = make_numpy_move(std::move(fl.log_x));
                 d["log_v"] = make_numpy_move(std::move(fl.log_v));
                 d["log_s"] = make_numpy_move(std::move(fl.log_s));
                 d["log_lane"] = make_numpy_move(std::move(fl.log_lane));
                 d["log_link"] = make_numpy_move(std::move(fl.log_link));
                 d["log_state"] = make_numpy_move(std::move(fl.log_state));
                 // log_t_link as two parallel numpy arrays
                 std::vector<double> ltl_t(m);
                 std::vector<int> ltl_id(m);
                 for (size_t i = 0; i < m; i++) {
                     ltl_t[i] = fl.log_t_link[i].first;
                     ltl_id[i] = fl.log_t_link[i].second;
                 }
                 d["log_t_link_t"] = make_numpy_move(std::move(ltl_t));
                 d["log_t_link_id"] = make_numpy_move(std::move(ltl_id));
                 return d;
             },
             "Build full log arrays as numpy arrays, all int (no string conversion)")
        .def("get_log_state_strings", [](const Vehicle &v) -> nb::list {
                 // Build full log and convert state ints to strings using interned objects
                 auto *si = g_intern;
                 auto fl = v.build_full_log();
                 nb::list result;
                 for (size_t i = 0; i < fl.log_state.size(); i++) {
                     int s = fl.log_state[i];
                     result.append((s >= 0 && s <= 4) ? si->states[s] : si->unknown);
                 }
                 return result;
             },
             "Return full log_state as list of strings (home/wait/run/end/abort)")
        .def("get_log_t_link_data", [](const Vehicle &v) -> nb::tuple {
                 // Return log_t_link as (times_array, link_id_array) numpy tuple
                 // Special link_ids: -1=home, -2=end, >=0=link_id
                 auto fl = v.build_full_log();
                 size_t m = fl.log_t_link.size();
                 std::vector<double> times(m);
                 std::vector<int> ids(m);
                 for (size_t i = 0; i < m; i++) {
                     times[i] = fl.log_t_link[i].first;
                     ids[i] = fl.log_t_link[i].second;
                 }
                 return nb::make_tuple(
                     make_numpy_move(std::move(times)),
                     make_numpy_move(std::move(ids))
                 );
             },
             "Return log_t_link as tuple of (times, link_ids) numpy arrays. "
             "Special link_ids: -1=home, -2=end, >=0=link_id")
        ;

    m.def("get_compile_datetime", &get_compile_datetime, "Return the compile date and time");

    // Initialize interned strings for state names (heap-allocated)
    g_intern = new InternedStrings();
    g_intern->init();

    // Register cleanup to run before Python interpreter shuts down.
    // This ensures heap-allocated objects with Python references are
    // destroyed while Py_IsInitialized() still returns true.
    auto atexit = nb::module_::import_("atexit");
    atexit.attr("register")(nb::cpp_function([]() {
        delete g_intern;
        g_intern = nullptr;
        delete g_pyout;
        g_pyout = nullptr;
        delete g_custom_buf;
        g_custom_buf = nullptr;
    }));
}
