// clang-format off

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <memory>
#include <string>
#include <iostream>
#include <streambuf>

#include "traffi.cpp"

namespace py = pybind11;

// ----------------------------------------------------------------------
// Interned Python strings for state names.
// Heap-allocated and explicitly cleaned up via atexit to avoid
// static destruction order issues (segfault at interpreter shutdown).
// ----------------------------------------------------------------------
struct InternedStrings {
    py::str states[5];   // home, wait, run, end, abort
    py::str unknown;
    py::str minus1;

    void init() {
        states[0] = py::str("home");
        states[1] = py::str("wait");
        states[2] = py::str("run");
        states[3] = py::str("end");
        states[4] = py::str("abort");
        unknown = py::str("unknown");
        minus1 = py::str("-1");
    }
};
static InternedStrings *g_intern = nullptr;


// ----------------------------------------------------------------------
// Custom streambuf that redirects C++ output to Python sys.stdout.
// Uses raw PyObject* instead of py::object to avoid GIL issues
// during static destruction at Python interpreter shutdown.
// ----------------------------------------------------------------------
class py_stdout_redirect_buf : public std::streambuf {
public:
    py_stdout_redirect_buf() : py_stdout_raw(nullptr) {
        py::object sys = py::module_::import("sys");
        py::object stdout_obj = sys.attr("stdout");
        py_stdout_raw = stdout_obj.ptr();
        Py_XINCREF(py_stdout_raw);
    }

    ~py_stdout_redirect_buf() {
        // Only decref if Python interpreter is still alive
        if (Py_IsInitialized() && py_stdout_raw) {
            py::gil_scoped_acquire gil;
            Py_XDECREF(py_stdout_raw);
        }
        py_stdout_raw = nullptr;
    }

protected:
    virtual int overflow(int c) override {
        if (c != EOF && Py_IsInitialized() && py_stdout_raw) {
            py::gil_scoped_acquire gil;
            std::string s(1, static_cast<char>(c));
            PyObject *result = PyObject_CallMethod(py_stdout_raw, "write", "s", s.c_str());
            Py_XDECREF(result);
        }
        return c;
    }

    virtual std::streamsize xsputn(const char* s, std::streamsize n) override {
        if (Py_IsInitialized() && py_stdout_raw) {
            py::gil_scoped_acquire gil;
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
        bool route_choice_update_gradual = false){
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
        route_choice_update_gradual);
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
// Pybind11 モジュール定義
// ----------------------------------------------------------------------
PYBIND11_MODULE(uxsim_cpp, m) {
    m.doc() = "uxsim_cpp: pybind11 bindings for C++ mesoscopic traffic simulation (UXsim integrated)";

    //
    // MARK: Scenario
    //
    m.def("create_world", &create_world,
          py::arg("world_name"),
          py::arg("t_max"),
          py::arg("delta_n"),
          py::arg("tau"),
          py::arg("duo_update_time"),
          py::arg("duo_update_weight"),
          py::arg("route_choice_uncertainty"),
          py::arg("print_mode"),
          py::arg("random_seed"),
          py::arg("vehicle_log_mode"),
          py::arg("hard_deterministic_mode") = false,
          py::arg("route_choice_update_gradual") = false,
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
          py::arg("world"),
          py::arg("node_name"),
          py::arg("x"),
          py::arg("y"),
          py::arg("signal_intervals") = vector<double>{0},
          py::arg("signal_offset") = 0.0,
          py::arg("flow_capacity") = -1.0,
          py::arg("number_of_lanes") = 0,
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
          py::arg("world"),
          py::arg("link_name"),
          py::arg("start_node_name"),
          py::arg("end_node_name"),
          py::arg("vmax"),
          py::arg("kappa"),
          py::arg("length"),
          py::arg("number_of_lanes"),
          py::arg("merge_priority"),
          py::arg("capacity_out"),
          py::arg("capacity_in"),
          py::arg("signal_group"),
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
          py::arg("world"),
          py::arg("orig_name"),
          py::arg("dest_name"),
          py::arg("start_t"),
          py::arg("end_t"),
          py::arg("flow"),
          py::arg("links_preferred_str"),
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
    py::class_<World>(m, "World")
        .def("initialize_adj_matrix", &World::initialize_adj_matrix)
        .def("print_scenario_stats", &World::print_scenario_stats)
        .def("main_loop", &World::main_loop)
        .def("check_simulation_ongoing", &World::check_simulation_ongoing)
        .def("print_simple_results", &World::print_simple_results)
        .def("update_adj_time_matrix", &World::update_adj_time_matrix)
        .def("enumerate_k_random_routes_cpp", [](World &w, int k, unsigned int seed) -> py::dict {
                 auto result = w.enumerate_k_random_routes_cpp(k, seed);
                 py::dict out;
                 for (auto &[key, routes] : result){
                     // Use node names as keys to match Python API
                     py::tuple py_key = py::make_tuple(
                         py::str(w.nodes[key.first]->name),
                         py::str(w.nodes[key.second]->name));
                     py::list py_routes;
                     for (auto &route : routes){
                         py::list py_route;
                         for (int lid : route){
                             py_route.append(py::str(w.links[lid]->name));
                         }
                         py_routes.append(py_route);
                     }
                     out[py_key] = py_routes;
                 }
                 return out;
             },
             py::arg("k"), py::arg("seed"),
             "Enumerate k random routes between all node pairs (C++ accelerated)")
        .def("estimate_congestion_externality_route",
             [](World &w, py::list py_route, double departure_time) -> double {
                 std::vector<Link*> route;
                 for (auto item : py_route){
                     std::string name = py::str(item.attr("name"));
                     Link *ln = w.get_link(name);
                     if (ln) route.push_back(ln);
                 }
                 return w.estimate_congestion_externality_route(route, departure_time);
             },
             py::arg("route"), py::arg("departure_time"),
             "Estimate congestion externality for a route (list of Link objects)")
        .def("get_node", &World::get_node,
             py::return_value_policy::reference,
             "Get a Node by name (reference)")
        .def("get_link", &World::get_link,
             py::return_value_policy::reference,
             "Get a Link by name (reference)")
        .def("get_vehicle", &World::get_vehicle,
             py::return_value_policy::reference,
             "Get a Vehicle by name (reference)")
        .def("get_link_by_id", &World::get_link_by_id,
             py::return_value_policy::reference,
             "Get a Link by integer ID (reference)")
        .def("get_vehicle_by_index", &World::get_vehicle_by_index,
             py::return_value_policy::reference,
             "Get a Vehicle by index in vehicles vector (O(1), no list conversion)")
        // Aliases for explicit by-name lookup (same as get_node/get_link)
        .def("get_node_by_name", &World::get_node,
             py::arg("name"),
             py::return_value_policy::reference,
             "Get a Node by name string (alias for get_node)")
        .def("get_link_by_name", &World::get_link,
             py::arg("name"),
             py::return_value_policy::reference,
             "Get a Link by name string (alias for get_link)")
        .def_readonly("VEHICLES", &World::vehicles,
                      "Vector of pointers to all Vehicles in the world.")
        .def_readonly("LINKS", &World::links,
                      "Vector of pointers to all Links in the world.")
        .def_readonly("NODES", &World::nodes,
                      "Vector of pointers to all Nodes in the world.")
        .def_readonly("timestep", &World::timestep)
        .def_readonly("time", &World::time)
        .def_readonly("delta_t", &World::delta_t)
        .def_readonly("DELTAT", &World::delta_t)
        .def_readonly("t_max", &World::t_max)
        .def_readonly("TMAX", &World::t_max)
        .def_readonly("name", &World::name)
        .def_readonly("deltan", &World::delta_n)
        .def_readwrite("hard_deterministic_mode", &World::hard_deterministic_mode)
        .def_readwrite("route_choice_update_gradual", &World::route_choice_update_gradual)
        .def_readwrite("instantaneous_TT_timestep_interval", &World::instantaneous_TT_timestep_interval)
        .def_readonly("route_dist", &World::route_dist)
        .def_readonly("route_dist_record", &World::route_dist_record)
        .def_readonly("route_next", &World::route_next)
        .def_readwrite("adj_mat_time", &World::adj_mat_time)
        .def_readwrite("route_preference", &World::route_preference)
        .def_readwrite("vehicle_log_mode", &World::vehicle_log_mode)
        .def_readonly("ave_v", &World::ave_v)
        .def_readonly("ave_vratio", &World::ave_vratio)
        .def_readonly("trips_total", &World::trips_total)
        .def_readonly("trips_completed", &World::trips_completed)
        .def_readonly("flag_initialized", &World::flag_initialized)
        .def_readonly("node_id", &World::node_id)
        .def_readonly("link_id", &World::link_id)
        .def_readonly("vehicle_id", &World::vehicle_id)
        .def_readonly("adj_mat", &World::adj_mat)
        .def_readonly("random_seed", &World::random_seed)
        .def_readonly("total_timesteps", &World::total_timesteps)
        .def_readonly("delta_n", &World::delta_n)
        .def_readwrite("duo_update_time", &World::duo_update_time)
        .def_readwrite("duo_update_weight", &World::duo_update_weight)
        .def_readwrite("route_choice_uncertainty", &World::route_choice_uncertainty)
        .def_readonly("vehicles_map", &World::vehicles_map)
        .def_readonly("nodes_map", &World::nodes_map)
        .def_readonly("links_map", &World::links_map)
        .def("get_vehicles_by_state", &World::get_vehicles_by_state,
             py::arg("state"),
             py::return_value_policy::reference,
             "Get vehicles filtered by state (0=home,1=wait,2=run,3=end,4=abort)")
        .def("get_all_vehicle_states", &World::get_all_vehicle_states,
             "Get (name, state_int) pairs for all vehicles. "
             "Abort is detected and returned as state 4.")
        .def("get_vehicle_states_by_index", &World::get_vehicle_states_by_index,
             "Get effective state ints indexed by vehicle order. "
             "0=home, 1=wait, 2=run, 3=end, 4=abort. "
             "Index matches World.VEHICLES vector order.")
        .def("build_all_vehicle_logs", [](const World &w) -> py::list {
                 // Batch: build full logs for ALL vehicles in one C++ call.
                 // All data as numpy arrays — zero string allocation.
                 // log_state: int (0-4), log_link: int (link ID),
                 // log_t_link: two parallel numpy arrays (times, link_ids)
                 //   Special link_ids: -1=home, -2=end, >=0=link_id
                 py::list result;
                 for (auto *v : w.vehicles) {
                     auto fl = v->build_full_log();
                     size_t n = fl.log_t.size();
                     size_t m = fl.log_t_link.size();
                     py::dict d;
                     d["log_t"] = py::array_t<double>(n, fl.log_t.data());
                     d["log_x"] = py::array_t<double>(n, fl.log_x.data());
                     d["log_v"] = py::array_t<double>(n, fl.log_v.data());
                     d["log_s"] = py::array_t<double>(n, fl.log_s.data());
                     d["log_lane"] = py::array_t<int>(n, fl.log_lane.data());
                     d["log_link"] = py::array_t<int>(n, fl.log_link.data());
                     d["log_state"] = py::array_t<int>(n, fl.log_state.data());
                     // log_t_link as two parallel numpy arrays
                     std::vector<double> ltl_t(m);
                     std::vector<int> ltl_id(m);
                     for (size_t i = 0; i < m; i++) {
                         ltl_t[i] = fl.log_t_link[i].first;
                         ltl_id[i] = fl.log_t_link[i].second;
                     }
                     d["log_t_link_t"] = py::array_t<double>(m, ltl_t.data());
                     d["log_t_link_id"] = py::array_t<int>(m, ltl_id.data());
                     result.append(d);
                 }
                 return result;
             },
             "Build full logs for all vehicles in batch. Returns list of dicts with numpy arrays.")
        ;

    //
    // MARK: Node
    //
    py::class_<Node>(m, "Node")
        .def(py::init<World *, const std::string &, double, double>(),
             py::arg("world"),
             py::arg("node_name"),
             py::arg("x"),
             py::arg("y"))
        .def_readonly("W", &Node::w)
        .def_readonly("id", &Node::id)
        .def_readonly("name", &Node::name)
        .def_readwrite("x", &Node::x)
        .def_readwrite("y", &Node::y)
        .def_readwrite("signal_intervals", &Node::signal_intervals)
        .def_readwrite("signal_offset", &Node::signal_offset)
        .def_readwrite("signal_t", &Node::signal_t)
        .def_readwrite("signal_phase", &Node::signal_phase)
        .def_readonly("signal_log", &Node::signal_log)
        .def_readwrite("flow_capacity", &Node::flow_capacity)
        .def_readwrite("flow_capacity_remain", &Node::flow_capacity_remain)
        .def_readwrite("number_of_lanes", &Node::number_of_lanes)
        .def_readonly("in_links", &Node::in_links)
        .def_readonly("out_links", &Node::out_links)
        .def_readonly("incoming_vehicles", &Node::incoming_vehicles)
        .def_readonly("generation_queue", &Node::generation_queue)
        .def("generate", &Node::generate)
        .def("transfer", &Node::transfer)
        .def("signal_update", &Node::signal_update)
        .def("flow_capacity_update", &Node::flow_capacity_update)
        ;

    //
    // MARK: Link
    //
    py::class_<Link>(m, "Link")
        .def(py::init<World *, const std::string &, const std::string &, const std::string &,
                      double, double, double, int, double, double, double>(),
             py::arg("world"),
             py::arg("link_name"),
             py::arg("start_node_name"),
             py::arg("end_node_name"),
             py::arg("vmax"),
             py::arg("kappa"),
             py::arg("length"),
             py::arg("number_of_lanes"),
             py::arg("merge_priority"),
             py::arg("capacity_out"),
             py::arg("capacity_in"))
        .def_readonly("W", &Link::w)
        .def_readonly("id", &Link::id)
        .def_readonly("name", &Link::name)
        .def_readwrite("length", &Link::length)
        .def_readwrite("number_of_lanes", &Link::number_of_lanes)
        .def_readwrite("u", &Link::vmax)
        .def_readwrite("vmax", &Link::vmax)
        .def_readwrite("kappa", &Link::kappa)
        .def_readwrite("delta", &Link::delta)
        .def_readwrite("delta_per_lane", &Link::delta_per_lane)
        .def_readwrite("tau", &Link::tau)
        .def_readwrite("capacity", &Link::capacity)
        .def_readwrite("w", &Link::backward_wave_speed)
        .def_readwrite("merge_priority", &Link::merge_priority)
        .def_readwrite("capacity_out", &Link::capacity_out)
        .def_readwrite("capacity_in", &Link::capacity_in)
        .def_readwrite("signal_group", &Link::signal_group)
        .def_readwrite("route_choice_penalty", &Link::route_choice_penalty)
        .def("set_toll_timeseries", [](Link &self, std::vector<double> ts){
            self.toll_timeseries = std::move(ts);
        })
        .def("estimate_congestion_externality", &Link::estimate_congestion_externality,
             py::arg("timestep"),
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
        .def_readwrite("free_flow_speed", &Link::vmax)
        .def_readwrite("backward_wave_speed", &Link::backward_wave_speed)
        .def_readwrite("capacity_out_remain", &Link::capacity_out_remain)
        .def_readwrite("capacity_in_remain", &Link::capacity_in_remain)
        .def_readonly("start_node", &Link::start_node)
        .def_readonly("end_node", &Link::end_node)
        .def_readonly("vehicles", &Link::vehicles)
        .def_readonly("arrival_curve", &Link::arrival_curve)
        .def_readonly("cum_arrival", &Link::arrival_curve)
        .def_readonly("departure_curve", &Link::departure_curve)
        .def_readonly("cum_departure", &Link::departure_curve)
        .def_readonly("traveltime_real", &Link::traveltime_real)
        .def_readonly("traveltime_actual", &Link::traveltime_real)
        .def_readonly("traveltime_instant", &Link::traveltime_instant)
        .def("update", &Link::update)
        .def("set_travel_time", &Link::set_travel_time)
        .def("change_free_flow_speed", &Link::change_free_flow_speed, py::arg("new_value"))
        .def("change_jam_density", &Link::change_jam_density, py::arg("new_value"))
        .def("count_vehicles_in_queue", &Link::count_vehicles_in_queue)
        // Numpy array accessors — avoid list() conversion overhead on Python side
        .def("get_cum_arrival_np", [](const Link &l) -> py::array_t<double> {
                 return py::array_t<double>(l.arrival_curve.size(), l.arrival_curve.data());
             },
             "Return cum_arrival as a numpy array (memcpy, no list conversion)")
        .def("get_cum_departure_np", [](const Link &l) -> py::array_t<double> {
                 return py::array_t<double>(l.departure_curve.size(), l.departure_curve.data());
             },
             "Return cum_departure as a numpy array (memcpy, no list conversion)")
        .def("get_traveltime_actual_np", [](const Link &l) -> py::array_t<double> {
                 return py::array_t<double>(l.traveltime_real.size(), l.traveltime_real.data());
             },
             "Return traveltime_actual (traveltime_real) as a numpy array")
        .def("get_traveltime_instant_np", [](const Link &l) -> py::array_t<double> {
                 return py::array_t<double>(l.traveltime_instant.size(), l.traveltime_instant.data());
             },
             "Return traveltime_instant as a numpy array")
        ;

    //
    // MARK: Vehicle
    //
    py::class_<Vehicle>(m, "Vehicle")
        .def(py::init<World *, const std::string &, double, const std::string &, const std::string &>(),
             py::arg("world"),
             py::arg("name"),
             py::arg("departure_time"),
             py::arg("orig_name"),
             py::arg("dest_name"))
        .def_readonly("W", &Vehicle::w)
        .def_readonly("id", &Vehicle::id)
        .def_readwrite("name", &Vehicle::name)
        .def_readonly("departure_time", &Vehicle::departure_time)
        .def_readwrite("orig", &Vehicle::orig)
        .def_readwrite("dest", &Vehicle::dest)
        .def_readonly("link", &Vehicle::link)
        .def_readonly("x", &Vehicle::x)
        .def_readonly("x_next", &Vehicle::x_next)
        .def_readonly("v", &Vehicle::v)
        .def_readonly("lane", &Vehicle::lane)
        .def_readonly("leader", &Vehicle::leader)
        .def_readonly("follower", &Vehicle::follower)
        .def_readwrite("state", &Vehicle::state)
        .def_readonly("arrival_time_link", &Vehicle::arrival_time_link)
        .def_readwrite("route_next_link", &Vehicle::route_next_link)
        .def_readwrite("route_choice_flag_on_link", &Vehicle::route_choice_flag_on_link)
        .def_readwrite("route_adaptive", &Vehicle::route_adaptive)
        .def_readwrite("route_preference", &Vehicle::route_preference)
        .def_readwrite("links_preferred", &Vehicle::links_preferred)
        .def_readwrite("links_avoid", &Vehicle::links_avoid)
        .def_readwrite("specified_route", &Vehicle::specified_route)
        .def("enforce_route", &Vehicle::enforce_route, py::arg("route"))
        .def_readwrite("move_remain", &Vehicle::move_remain)
        .def_readonly("x_old", &Vehicle::x_old)
        .def_readwrite("trip_abort", &Vehicle::trip_abort)
        .def_readonly("flag_trip_aborted", &Vehicle::flag_trip_aborted)
        .def_readonly("flag_waiting_for_trip_end", &Vehicle::flag_waiting_for_trip_end)
        .def_readonly("log_t", &Vehicle::log_t)
        .def_readonly("log_state", &Vehicle::log_state)
        .def_readonly("log_link", &Vehicle::log_link)
        .def_readonly("log_x", &Vehicle::log_x)
        .def_readonly("log_v", &Vehicle::log_v)
        .def_readonly("log_lane", &Vehicle::log_lane)
        .def_readonly("arrival_time", &Vehicle::arrival_time)
        .def_readonly("travel_time", &Vehicle::travel_time)
        .def_readwrite("distance_traveled", &Vehicle::distance_traveled)
        .def("state_str", &Vehicle::state_str,
             "Return state as string: home/wait/run/end/abort")
        .def_property_readonly("state_str_p",
             [](const Vehicle &v) { return v.state_str(); },
             "State as string property: home/wait/run/end/abort")
        .def("log_state_str", &Vehicle::log_state_str,
             "Return log_state as vector of strings")
        .def("log_link_names", &Vehicle::log_link_names,
             "Return log_link as vector of link name strings")
        .def_property_readonly("departure_time_in_second", &Vehicle::departure_time_in_second,
             "Departure time in seconds")
        .def_readwrite("route_choice_principle", &Vehicle::route_choice_principle)
        .def_readwrite("route_choice_uncertainty", &Vehicle::route_choice_uncertainty)
        .def("build_log_t_link", &Vehicle::build_log_t_link,
             "Build log_t_link: list of (time, label) pairs for link transitions")
        .def("build_full_log", [](const Vehicle &v) -> py::dict {
                 // Per-vehicle version: returns Python lists with string conversions
                 // for backward compatibility with existing wrapper code.
                 // Optimization: use heap-allocated interned strings (g_intern) to
                 // reuse same PyObject, use py::array_t for numeric arrays.
                 auto *si = g_intern;
                 auto fl = v.build_full_log();
                 size_t n = fl.log_t.size();
                 py::dict d;
                 // Keep numeric data as Python lists (py::cast) for backward
                 // compatibility — wrapper fallback path expects list, not ndarray.
                 d["log_t"] = py::cast(fl.log_t);
                 d["log_x"] = py::cast(fl.log_x);
                 d["log_v"] = py::cast(fl.log_v);
                 d["log_s"] = py::cast(fl.log_s);
                 d["log_lane"] = py::cast(fl.log_lane);
                 d["log_link"] = py::cast(fl.log_link);
                 // State int→string using interned Python strings (no new allocation)
                 py::list state_strs(n);
                 for (size_t i = 0; i < n; i++) {
                     int s = fl.log_state[i];
                     state_strs[i] = (s >= 0 && s <= 4) ? si->states[s] : si->unknown;
                 }
                 d["log_state"] = state_strs;
                 // Build log_t_link with interned label strings
                 size_t m = fl.log_t_link.size();
                 py::list ltl(m);
                 for (size_t i = 0; i < m; i++) {
                     double t = fl.log_t_link[i].first;
                     int lid = fl.log_t_link[i].second;
                     py::list entry(2);
                     entry[0] = py::float_(t);
                     if (lid == Vehicle::LOG_T_LINK_HOME) {
                         entry[1] = si->states[0];  // "home"
                     } else if (lid == Vehicle::LOG_T_LINK_END) {
                         entry[1] = si->states[3];  // "end"
                     } else if (lid >= 0 && lid < static_cast<int>(v.w->links.size())) {
                         entry[1] = py::str(v.w->links[lid]->name);
                     } else {
                         entry[1] = si->minus1;
                     }
                     ltl[i] = entry;
                 }
                 d["log_t_link"] = ltl;
                 return d;
             },
             "Build full log arrays with string conversions (backward compatible)")
        .def("build_full_log_np", [](const Vehicle &v) -> py::dict {
                 // Per-vehicle numpy version: all data as numpy/int, no strings.
                 auto fl = v.build_full_log();
                 size_t n = fl.log_t.size();
                 size_t m = fl.log_t_link.size();
                 py::dict d;
                 d["log_t"] = py::array_t<double>(n, fl.log_t.data());
                 d["log_x"] = py::array_t<double>(n, fl.log_x.data());
                 d["log_v"] = py::array_t<double>(n, fl.log_v.data());
                 d["log_s"] = py::array_t<double>(n, fl.log_s.data());
                 d["log_lane"] = py::array_t<int>(n, fl.log_lane.data());
                 d["log_link"] = py::array_t<int>(n, fl.log_link.data());
                 d["log_state"] = py::array_t<int>(n, fl.log_state.data());
                 // log_t_link as two parallel numpy arrays (times, link_ids)
                 // Special IDs: -1=home, -2=end, >=0=link_id
                 std::vector<double> ltl_t(m);
                 std::vector<int> ltl_id(m);
                 for (size_t i = 0; i < m; i++) {
                     ltl_t[i] = fl.log_t_link[i].first;
                     ltl_id[i] = fl.log_t_link[i].second;
                 }
                 d["log_t_link_t"] = py::array_t<double>(m, ltl_t.data());
                 d["log_t_link_id"] = py::array_t<int>(m, ltl_id.data());
                 return d;
             },
             "Build full log arrays as numpy arrays, all int (no string conversion)")
        .def("get_log_state_strings", [](const Vehicle &v) -> py::list {
                 // Build full log and convert state ints to strings using interned objects
                 auto *si = g_intern;
                 auto fl = v.build_full_log();
                 py::list result(fl.log_state.size());
                 for (size_t i = 0; i < fl.log_state.size(); i++) {
                     int s = fl.log_state[i];
                     result[i] = (s >= 0 && s <= 4) ? si->states[s] : si->unknown;
                 }
                 return result;
             },
             "Return full log_state as list of strings (home/wait/run/end/abort)")
        .def("get_log_t_link_data", [](const Vehicle &v) -> py::tuple {
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
                 return py::make_tuple(
                     py::array_t<double>(m, times.data()),
                     py::array_t<int>(m, ids.data())
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
    auto atexit = py::module_::import("atexit");
    atexit.attr("register")(py::cpp_function([]() {
        delete g_intern;
        g_intern = nullptr;
        delete g_pyout;
        g_pyout = nullptr;
        delete g_custom_buf;
        g_custom_buf = nullptr;
    }));
}
