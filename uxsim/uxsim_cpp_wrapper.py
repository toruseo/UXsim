"""
UXsim C++ Engine Wrapper

Thin bridge between the C++ traffic simulation engine and Python.
No simulation logic — all computation is delegated to C++.
This module only handles:
  - Parameter forwarding to C++
  - Post-simulation state sync from C++ to Python data structures
  - Analyzer-compatible attribute exposure
"""

import random
import time
import string
import warnings
from collections import deque, OrderedDict
from collections import defaultdict as ddict

import numpy as np

from .analyzer import *
from .utils import *
from .scenario_reader_writer import *

# Import C++ extension module
try:
    from . import uxsim_cpp as trafficppy
except ImportError:
    raise ImportError(
        "C++ engine (uxsim_cpp) is not available. "
        "Build it with 'pip install -e .' (requires C++ compiler and CMake)."
    )

_create_world = trafficppy.create_world
_add_node = trafficppy.add_node
_add_link = trafficppy.add_link
_add_demand = trafficppy.add_demand


class CppNode:
    """Thin wrapper around C++ Node. Stores attributes for analyzer compatibility."""

    def __init__(self, W, name, x, y, signal=[0], signal_offset=0, signal_offset_old=None,
                 flow_capacity=None, number_of_lanes=None, auto_rename=False,
                 attribute=None, user_attribute=None, user_function=None):
        self.W = W
        self.attribute = attribute
        self.user_attribute = user_attribute
        self.user_function = user_function

        # Compute signal_offset from old-style param before forwarding to C++
        actual_signal_offset = signal_offset
        if signal_offset_old is not None:
            actual_signal_offset = sum(signal) - signal_offset_old

        # Python-only containers (CppLink objects, not C++ Link pointers)
        self.inlinks = dict()
        self.outlinks = dict()

        # ID and name registration
        self.id = len(self.W.NODES)
        self.name = name
        if self.name in self.W.NODES_NAME_DICT:
            if auto_rename:
                self.name = self.name + "_renamed" + "".join(
                    self.W.rng.choice(list(string.ascii_letters + string.digits), size=8))
            else:
                raise ValueError(f"Node name {self.name} already used by another node.")
        self.W.NODES.append(self)
        self.W.NODES_NAME_DICT[self.name] = self

        # Forward to C++
        _add_node(self.W._cpp_world, name, float(x), float(y), list(signal), float(actual_signal_offset),
                  float(flow_capacity) if flow_capacity is not None else -1.0,
                  int(number_of_lanes) if number_of_lanes is not None else -1)
        self._cpp_node = self.W._cpp_world.get_node(self.name)

        # Read back from C++ (single source of truth)
        self.x = self._cpp_node.x
        self.y = self._cpp_node.y
        self.signal = list(self._cpp_node.signal_intervals)
        self.signal_offset = self._cpp_node.signal_offset
        self.cycle_length = sum(self.signal)
        self.flow_capacity = self._cpp_node.flow_capacity
        self.number_of_lanes = self._cpp_node.number_of_lanes
        self.flag_lanes_automatically_determined = False

    def __repr__(self):
        return f"<Node {self.name}>"

    @property
    def signal_phase(self):
        return self._cpp_node.signal_phase
    @signal_phase.setter
    def signal_phase(self, value):
        self._cpp_node.signal_phase = value

    @property
    def signal_t(self):
        return self._cpp_node.signal_t
    @signal_t.setter
    def signal_t(self, value):
        self._cpp_node.signal_t = value

    @property
    def signal_log(self):
        return list(self._cpp_node.signal_log)
    @signal_log.setter
    def signal_log(self, value):
        self._cpp_node.signal_log = value


class CppLink:
    """Thin wrapper around C++ Link. Stores attributes for analyzer compatibility."""

    def __init__(self, W, name, start_node, end_node, length,
                 free_flow_speed=20, jam_density=0.2, jam_density_per_lane=None,
                 number_of_lanes=1, merge_priority=1, signal_group=[0],
                 capacity_out=None, capacity_in=None,
                 congestion_pricing=None, eular_dx=None,
                 attribute=None, user_attribute=None, user_function=None,
                 auto_rename=False):
        self.W = W
        self.start_node = self.W.get_node(start_node)
        self.end_node = self.W.get_node(end_node)
        self.number_of_lanes = int(number_of_lanes)

        # Signal
        self.signal_group = [signal_group] if isinstance(signal_group, int) else list(signal_group)

        # ID and name registration
        self.id = len(self.W.LINKS)
        self.name = name
        if self.name is None:
            self.name = self.start_node.name + "-" + self.end_node.name
        if self.name in self.W.LINKS_NAME_DICT:
            if auto_rename:
                self.name = self.name + "_renamed" + "".join(
                    self.W.rng.choice(list(string.ascii_letters + string.digits), size=8))
            else:
                raise ValueError(f"Link name {self.name} already used by another link.")
        self.W.LINKS.append(self)
        self.W.LINKS_NAME_DICT[self.name] = self
        self.start_node.outlinks[self.name] = self
        self.end_node.inlinks[self.name] = self

        self.attribute = attribute
        self.user_attribute = user_attribute
        self.user_function = user_function

        # Forward to C++ — compute kappa from jam_density_per_lane if needed
        kappa = jam_density
        if jam_density_per_lane is not None:
            kappa = jam_density_per_lane * number_of_lanes
        cpp_capacity_out = capacity_out if capacity_out is not None else -1
        cpp_capacity_in = capacity_in if capacity_in is not None else -1
        cpp_signal_group = list(self.signal_group)
        try:
            _add_link(self.W._cpp_world, self.name, self.start_node.name, self.end_node.name,
                      float(free_flow_speed), float(kappa), float(length), int(self.number_of_lanes),
                      float(merge_priority), float(cpp_capacity_out), float(cpp_capacity_in), cpp_signal_group)
        except TypeError:
            _add_link(self.W._cpp_world, self.name, self.start_node.name, self.end_node.name,
                      float(free_flow_speed), float(kappa), float(length), float(merge_priority), float(cpp_capacity_out), cpp_signal_group)
        self._cpp_link = self.W._cpp_world.get_link(self.name)

        # Read derived parameters from C++ (single source of truth)
        cpp = self._cpp_link
        self.length = cpp.length
        self.u = cpp.vmax
        self.kappa = cpp.kappa
        self.tau = cpp.tau
        self.w = cpp.w
        self.capacity = cpp.capacity
        self.delta = cpp.delta
        self.delta_per_lane = cpp.delta_per_lane
        self.free_flow_speed = self.u
        self.jam_density = self.kappa
        self.jam_density_per_lane = jam_density_per_lane
        self.merge_priority = cpp.merge_priority
        self.q_star = self.capacity
        self.k_star = self.capacity / self.u
        self.capacity_out = cpp.capacity_out
        self.capacity_in = cpp.capacity_in
        self._capacity_out_remain = cpp.capacity_out_remain
        self._capacity_in_remain = cpp.capacity_in_remain

        # Containers for analyzer compatibility
        self.vehicles = deque()
        self.vehicles_enter_log = {}
        self._traveltime_actual = []
        self.congestion_pricing = congestion_pricing
        self._route_choice_penalty = 0
        self.tss = []
        self.xss = []
        self.cs = []
        self.ls = []
        self.names = []

        # Edie state grid
        self.eular_dx = eular_dx
        if eular_dx is None:
            self.edie_dx = max(self.length / 10, self.u * self.W.DELTAT)
        else:
            self.edie_dx = eular_dx

    def __repr__(self):
        return f"<Link {self.name}>"

    def init_after_tmax_fix(self):
        """Initialize analyzer data structures after TMAX is known."""
        self.edie_dt = self.W.EULAR_DT
        self.k_mat = np.zeros([int(self.W.TMAX / self.edie_dt) + 1, int(self.length / self.edie_dx)])
        self.q_mat = np.zeros(self.k_mat.shape)
        self.v_mat = np.zeros(self.k_mat.shape)
        self.tn_mat = np.zeros(self.k_mat.shape)
        self.dn_mat = np.zeros(self.k_mat.shape)
        self.an = self.edie_dt * self.edie_dx
        self.traveltime_actual = np.array([self.length / self.u for _ in range(self.W.TSIZE)])
        self._free_flow_tt = self.length / self.u
        self._deltat = self.W.DELTAT

    @property
    def cum_arrival(self):
        if self.W._simulation_done:
            if not '_cached_cum_arrival' in self.__dict__:
                self._cached_cum_arrival = self._cpp_link.get_cum_arrival_np()
            return self._cached_cum_arrival
        return self._cpp_link.get_cum_arrival_np()

    @property
    def cum_departure(self):
        if self.W._simulation_done:
            if not '_cached_cum_departure' in self.__dict__:
                self._cached_cum_departure = self._cpp_link.get_cum_departure_np()
            return self._cached_cum_departure
        return self._cpp_link.get_cum_departure_np()

    @property
    def traveltime_instant(self):
        if self.W._simulation_done:
            if not '_cached_traveltime_instant' in self.__dict__:
                self._cached_traveltime_instant = self._cpp_link.get_traveltime_instant_np()
            return self._cached_traveltime_instant
        return self._cpp_link.get_traveltime_instant_np()

    @property
    def traveltime_actual(self):
        """Return traveltime_actual as numpy array from C++."""
        if self.W._simulation_done:
            if not '_cached_traveltime_actual' in self.__dict__:
                self._cached_traveltime_actual = self._cpp_link.get_traveltime_actual_np()
            return self._cached_traveltime_actual
        arr = self._cpp_link.get_traveltime_actual_np()
        if len(arr) > 0:
            return arr
        # Fallback to Python-initialized default (pre-simulation)
        if '_traveltime_actual' in self.__dict__:
            return self._traveltime_actual
        return np.array([])
    @traveltime_actual.setter
    def traveltime_actual(self, value):
        self._traveltime_actual = value

    @property
    def capacity_in_remain(self):
        return self._cpp_link.capacity_in_remain
    @capacity_in_remain.setter
    def capacity_in_remain(self, value):
        self._capacity_in_remain = value
        self._cpp_link.capacity_in_remain = float(value)

    @property
    def capacity_out_remain(self):
        return self._cpp_link.capacity_out_remain
    @capacity_out_remain.setter
    def capacity_out_remain(self, value):
        self._capacity_out_remain = value
        self._cpp_link.capacity_out_remain = float(value)

    # --- Data access methods for analyzer (read directly from C++) ---

    @property
    def speed(self):
        cpp_link = self._cpp_link
        if cpp_link.vehicle_count == 0:
            return self.u
        return cpp_link.avg_speed

    @property
    def density(self):
        return self._cpp_link.vehicle_count * self.W.DELTAN / self.length

    @property
    def flow(self):
        return self.density * self.speed

    @property
    def num_vehicles(self):
        return self._cpp_link.vehicle_count * self.W.DELTAN

    @property
    def num_vehicles_queue(self):
        return self._cpp_link.count_vehicles_in_queue() * self.W.DELTAN

    def arrival_count(self, t):
        if len(self.cum_arrival) == 0:
            return 0
        idx = min(max(int(t / self.W.DELTAT), 0), len(self.cum_arrival) - 1)
        return self.cum_arrival[idx]

    def departure_count(self, t):
        if len(self.cum_departure) == 0:
            return 0
        idx = min(max(int(t / self.W.DELTAT), 0), len(self.cum_departure) - 1)
        return self.cum_departure[idx]

    def inflow_between(self, t0, t1):
        return (self.arrival_count(t1) - self.arrival_count(t0)) / (t1 - t0) if t1 != t0 else 0

    def average_travel_time_between(self, t0, t1):
        t_list = [t for t in self.vehicles_enter_log.keys() if t0 <= t < t1]
        if len(t_list) == 0:
            return self.length / self.u
        return np.average([self.actual_travel_time(t) for t in t_list])

    def num_vehicles_t(self, t):
        return self.arrival_count(t) - self.departure_count(t)

    def average_density(self, t):
        return self.num_vehicles_t(t) / self.length

    def instant_travel_time(self, t):
        arr = self.traveltime_instant
        n = len(arr)
        if n == 0:
            return self._free_flow_tt
        idx = int(t / self._deltat)
        if idx < 0:
            idx = 0
        elif idx >= n:
            idx = n - 1
        return arr[idx]

    def actual_travel_time(self, t):
        arr = self.traveltime_actual
        n = len(arr)
        if n == 0:
            return self._free_flow_tt
        idx = int(t / self._deltat)
        if idx < 0:
            idx = 0
        elif idx >= n:
            idx = n - 1
        return arr[idx]

    def average_speed(self, t):
        tt = self.actual_travel_time(t)
        return self.length / tt if tt > 0 else self.u

    def average_flow(self, t):
        return self.average_density(t) * self.average_speed(t)

    @property
    def route_choice_penalty(self):
        return self._route_choice_penalty

    @route_choice_penalty.setter
    def route_choice_penalty(self, value):
        self._route_choice_penalty = value
        self._cpp_link.route_choice_penalty = float(value)

    def get_toll(self, t):
        return self.congestion_pricing(t) if self.congestion_pricing else self.route_choice_penalty

    def change_free_flow_speed(self, new_value):
        """Forward to C++ Link.change_free_flow_speed()."""
        self._cpp_link.change_free_flow_speed(float(new_value))
        self._read_from_cpp()

    def change_jam_density(self, new_value):
        """Forward to C++ Link.change_jam_density()."""
        self._cpp_link.change_jam_density(float(new_value))
        self._read_from_cpp()

    def _read_from_cpp(self):
        """Read all flow parameters from C++ link."""
        cpp = self._cpp_link
        self.u = cpp.vmax
        self.kappa = cpp.kappa
        self.tau = cpp.tau
        self.w = cpp.w
        self.capacity = cpp.capacity
        self.delta = cpp.delta
        self.delta_per_lane = cpp.delta_per_lane
        self.free_flow_speed = self.u
        self.jam_density = self.kappa
        self.q_star = self.capacity
        self.k_star = self.capacity / self.u if self.u > 0 else 0



class CppVehicle:
    """Proxy wrapper around C++ Vehicle.  Reads most attributes directly from
    C++ via __getattr__; only Python-only data lives on the Python object.
    Instances are created via __new__ in _register_new_cpp_vehicles — no __init__."""

    # State int→str mapping
    _STATE_MAP = {0: "home", 1: "wait", 2: "run", 3: "end", 4: "abort"}
    _STATE_TO_INT = {"home": 0, "wait": 1, "run": 2, "end": 3, "abort": 4}

    def __repr__(self):
        return f"<Vehicle {self.name}>"

    def __getattr__(self, name):
        """Proxy attribute access to C++ vehicle when not found on Python object."""
        if name.startswith('_'):
            raise AttributeError(name)
        try:
            return getattr(self.__dict__['_cpp_vehicle'], name)
        except (KeyError, AttributeError):
            raise AttributeError(f"'CppVehicle' object has no attribute '{name}'")

    # --- Properties that need conversion between C++ and Python ---

    @property
    def state(self):
        cpp = self._cpp_vehicle
        s = self._STATE_MAP.get(cpp.state, "end")
        if s == "end":
            if cpp.flag_trip_aborted or (cpp.arrival_time < 0 and cpp.travel_time <= 0):
                return "abort"
        return s

    @state.setter
    def state(self, value):
        self._cpp_vehicle.state = self._STATE_TO_INT[value] if isinstance(value, str) else int(value)

    @property
    def departure_time(self):
        return int(self._cpp_vehicle.departure_time / self.W.DELTAT)

    @property
    def departure_time_in_second(self):
        return self._cpp_vehicle.departure_time

    @property
    def arrival_time(self):
        cpp = self._cpp_vehicle
        at = cpp.arrival_time
        if at >= 0:
            if self.state == "abort":
                return -1
            return int(at / self.W.DELTAT)
        return -1

    @property
    def travel_time(self):
        if self.state == "abort":
            return -1
        return self._cpp_vehicle.travel_time

    @property
    def flag_trip_aborted(self):
        cpp = self._cpp_vehicle
        if cpp.flag_trip_aborted:
            return 1
        if self._STATE_MAP.get(cpp.state, "end") == "end" and cpp.arrival_time < 0 and cpp.travel_time <= 0:
            return 1
        return 0

    @property
    def link(self):
        cpp_link = self._cpp_vehicle.link
        if cpp_link is not None:
            link_id = cpp_link.id
            links = self.W.LINKS
            if 0 <= link_id < len(links):
                return links[link_id]
        return None

    @property
    def link_arrival_time(self):
        return self._cpp_vehicle.arrival_time_link

    @property
    def distance_traveled(self):
        return self._cpp_vehicle.distance_traveled
    @distance_traveled.setter
    def distance_traveled(self, value):
        self._cpp_vehicle.distance_traveled = float(value)

    # --- Log properties (lazily computed and cached) ---

    # State int→str for log arrays
    _LOG_STATE_MAP = ["home", "wait", "run", "end", "abort"]

    def _ensure_log_raw(self):
        """Ensure raw log data is loaded. Triggers batch fetch on first call."""
        if self._log_cache is not None:
            return
        # Batch-fetch all vehicle raw logs at once
        self.W._build_all_vehicle_log_caches()
        # If batch didn't populate us (shouldn't happen), fallback to individual
        if self._log_cache is None:
            self._log_cache = self._cpp_vehicle.build_full_log_np()

    @property
    def log_t(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        return self._log_cache['log_t']

    @property
    def log_x(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        return self._log_cache['log_x']

    @property
    def log_s(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        return self._log_cache['log_s']

    @property
    def log_v(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        return self._log_cache['log_v']

    @property
    def log_lane(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        return self._log_cache['log_lane']

    @property
    def log_state(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        cache = self._log_cache
        if '_log_state_converted' not in cache:
            state_map = CppVehicle._LOG_STATE_MAP
            n_states = len(state_map)
            cache['_log_state_converted'] = [state_map[s] if 0 <= s < n_states else "end"
                                             for s in cache['log_state']]
        return cache['_log_state_converted']

    @property
    def log_link(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        cache = self._log_cache
        if '_log_link_converted' not in cache:
            links_list = self.W.LINKS
            n_links = len(links_list)
            cache['_log_link_converted'] = [links_list[idx] if 0 <= idx < n_links else -1
                                            for idx in cache['log_link']]
        return cache['_log_link_converted']

    @property
    def log_t_link(self):
        if self._log_cache is None:
            self._ensure_log_raw()
        cache = self._log_cache
        if '_log_t_link_converted' not in cache:
            links_list = self.W.LINKS
            n_links = len(links_list)
            t_arr = cache['log_t_link_t']
            id_arr = cache['log_t_link_id']
            n_t_link = len(t_arr)
            log_t_link = []
            for i in range(n_t_link):
                lid = int(id_arr[i])
                if lid == -1:
                    log_t_link.append([t_arr[i], "home"])
                elif lid == -2:
                    log_t_link.append([t_arr[i], "end"])
                elif 0 <= lid < n_links:
                    log_t_link.append([t_arr[i], links_list[lid]])
                else:
                    log_t_link.append([t_arr[i], -1])
            cache['_log_t_link_converted'] = log_t_link
        return cache['_log_t_link_converted']

    def traveled_route(self, include_arrival_time=True, include_departure_time=False):
        """Returns (Route, times_list) matching uxsim.Vehicle.traveled_route."""
        route_links = []
        ts = []
        for log in self.log_t_link:
            t, l = log[0], log[1]
            if l == "home" and include_departure_time:
                ts.append(t)
            if isinstance(l, CppLink):
                ts.append(t)
                route_links.append(l)
        if self.log_t_link:
            t, l = self.log_t_link[-1]
            if include_arrival_time:
                ts.append(t if l == "end" else -1)
        return CppRoute(self.W, route_links, trust_input=True), ts

    def enforce_route(self, route, set_avoid=False):
        """Forward to C++ Vehicle.enforce_route."""
        # Always resolve links by name in current World to avoid cross-world pointer bugs
        py_links = [self.W.get_link(l) for l in route]
        self.specified_route = py_links
        # Forward to C++ with raw C++ Link objects from current World
        try:
            cpp_links = [l._cpp_link for l in py_links]
            self._cpp_vehicle.enforce_route(cpp_links)
        except (AttributeError, TypeError):
            pass

    def set_links_prefer(self, links):
        self.links_prefer = [self.W.get_link(l) for l in links]
        try:
            self._cpp_vehicle.links_preferred = [self.W.get_link(l)._cpp_link for l in links]
        except (AttributeError, TypeError):
            pass

    def set_links_avoid(self, links):
        self.links_avoid = [self.W.get_link(l) for l in links]
        try:
            self._cpp_vehicle.links_avoid = [self.W.get_link(l)._cpp_link for l in links]
        except (AttributeError, TypeError):
            pass

    def add_dest(self, dest, order=-1):
        dest = self.W.get_node(dest)
        if order == -1:
            self.dest_list.append(dest)
        else:
            self.dest_list.insert(order, dest)

    def add_dests(self, dests):
        for d in dests:
            self.add_dest(d)

    def get_xy_coords(self, t=-1):
        if self.link is None:
            return self.orig.x, self.orig.y
        ratio = self.x / self.link.length if self.link.length > 0 else 0
        return (self.link.start_node.x + ratio * (self.link.end_node.x - self.link.start_node.x),
                self.link.start_node.y + ratio * (self.link.end_node.y - self.link.start_node.y))



class CppRoute:
    """Route compatible with uxsim.Route."""

    def __init__(self, W, links, name="", trust_input=False):
        self.W = W
        self.name = name
        self.links = []
        if trust_input:
            self.links = [W.get_link(l) if isinstance(l, str) else l for l in links]
        else:
            self.links = [W.get_link(l) for l in links]
            for i in range(len(self.links) - 1):
                if self.links[i].end_node != self.links[i + 1].start_node:
                    raise ValueError(f"Route links not connected: {self.links[i].name} -> {self.links[i + 1].name}")
        self.links_name = [l.name for l in self.links]

    def __repr__(self):
        return f"<Route {self.name}: {self.links_name}>"
    def __iter__(self):
        return iter(self.links)
    def __len__(self):
        return len(self.links)
    def __getitem__(self, idx):
        return self.links[idx]
    def __eq__(self, other):
        return self.links_name == other.links_name if hasattr(other, 'links_name') else NotImplemented

    def actual_travel_time(self, t, return_details=False):
        tt, ct, details = 0, t, []
        for link in self.links:
            ltt = link.actual_travel_time(ct)
            tt += ltt
            ct += ltt
            details.append(ltt)
        if return_details:
            return tt, details
        return tt


class CppRouteChoice:
    """Thin bridge exposing C++ route computation results as ROUTECHOICE-compatible object."""

    def __init__(self, W):
        self.W = W

    @property
    def dist(self):
        return np.array(self.W._cpp_world.route_dist)

    @property
    def next(self):
        return np.array(self.W._cpp_world.route_next)

    @property
    def dist_record(self):
        """Return dist record from C++, converting values to numpy arrays."""
        try:
            raw = self.W._cpp_world.route_dist_record
            return {k: np.array(v) for k, v in raw.items()}
        except AttributeError:
            pass
        # Fallback: current dist for all DUO timesteps
        dist = self.dist
        record = {}
        duo_interval = max(1, int(self.W.DUO_UPDATE_TIME / self.W.DELTAT))
        tsize = getattr(self.W, 'TSIZE', int(getattr(self.W, 'TMAX', 3600) / self.W.DELTAT))
        for ts in range(0, tsize, duo_interval):
            record[ts] = dist
        return record


class CppWorld:
    """Thin wrapper around C++ World. Delegates simulation to C++ engine."""

    def __init__(self, name="", deltan=5, reaction_time=1,
                 duo_update_time=600, duo_update_weight=0.5, duo_noise=0.01,
                 route_choice_principle="homogeneous_DUO", route_choice_update_gradual=False,
                 instantaneous_TT_timestep_interval=5,
                 eular_dt=120, eular_dx=100,
                 random_seed=None,
                 print_mode=1, save_mode=1, show_mode=0, show_progress=1, show_progress_deltat=600,
                 tmax=None,
                 vehicle_logging_timestep_interval=1,
                 reduce_memory_delete_vehicle_route_pref=False,
                 hard_deterministic_mode=False,
                 no_cyclic_routing=False,
                 meta_data=None, user_attribute=None, user_function=None):

        self.W = self  # self-reference for W.W access pattern
        self.rng = np.random.default_rng(seed=random_seed)
        self.random_seed = random_seed
        self.TMAX = tmax
        self.DELTAN = deltan
        self.REACTION_TIME = reaction_time
        self.DUO_UPDATE_TIME = duo_update_time
        self.DUO_UPDATE_WEIGHT = duo_update_weight
        self.DUO_NOISE = duo_noise
        self.EULAR_DT = eular_dt
        self.DELTAT = self.REACTION_TIME * self.DELTAN
        self.DELTAT_ROUTE = max(1, int(self.DUO_UPDATE_TIME / self.DELTAT))

        self.VEHICLES = OrderedDict()
        self.VEHICLES_LIVING = OrderedDict()
        self.VEHICLES_RUNNING = OrderedDict()
        self.NODES = []
        self.LINKS = []
        self.NODES_NAME_DICT = {}
        self.LINKS_NAME_DICT = {}

        self.vehicle_logging_timestep_interval = vehicle_logging_timestep_interval
        self.route_choice_principle = route_choice_principle
        self.route_choice_update_gradual = route_choice_update_gradual
        self.instantaneous_TT_timestep_interval = min(int(instantaneous_TT_timestep_interval), self.DELTAT_ROUTE)
        self.route_pref_for_vehs = None
        self.show_progress = show_progress
        self.show_progress_deltat_timestep = int(show_progress_deltat / self.DELTAT)
        self.reduce_memory_delete_vehicle_route_pref = reduce_memory_delete_vehicle_route_pref
        self.name = name
        self.hard_deterministic_mode = hard_deterministic_mode
        self.no_cyclic_routing = no_cyclic_routing
        self.meta_data = meta_data if meta_data is not None else {}
        self.network_info = ddict(list)
        self.demand_info = ddict(list)
        self.finalized = 0
        self.world_start_time = time.time()
        self.print_mode = print_mode
        self.print = print if print_mode else (lambda *a, **k: None)
        self.save_mode = save_mode
        self.show_mode = show_mode
        self.user_attribute = user_attribute
        self.user_function = user_function

        self._cpp_world = None
        self._cpp_world_created = False
        self._cpp_vehicle_log_mode = 1 if vehicle_logging_timestep_interval > 0 else 0
        if vehicle_logging_timestep_interval not in (0, 1, -1):
            warnings.warn("vehicle_logging_timestep_interval is not 0, 1, or -1. C++ mode only supports 0 (no logging) and 1 (full logging). The value will be treated as 1 (logging enabled).", stacklevel=2)
        self._simulation_done = False
        self._cpp_veh_registered_count = 0

    def _ensure_cpp_world(self):
        if self._cpp_world_created:
            return
        tmax = self.TMAX if self.TMAX is not None else 7200
        seed = self.random_seed if self.random_seed is not None else random.getrandbits(32)
        try:
            self._cpp_world = _create_world(self.name, float(tmax), float(self.DELTAN), float(self.REACTION_TIME),
                float(self.DUO_UPDATE_TIME), float(self.DUO_UPDATE_WEIGHT), float(self.DUO_NOISE), int(self.print_mode), int(seed),
                bool(self._cpp_vehicle_log_mode), bool(self.hard_deterministic_mode),
                route_choice_update_gradual=bool(self.route_choice_update_gradual),
                no_cyclic_routing=bool(self.no_cyclic_routing))
        except TypeError:
            self._cpp_world = _create_world(self.name, float(tmax), float(self.DELTAN), float(self.REACTION_TIME),
                float(self.DUO_UPDATE_TIME), float(self.DUO_UPDATE_WEIGHT), float(self.DUO_NOISE), int(self.print_mode), int(seed),
                bool(self._cpp_vehicle_log_mode))
        self._cpp_world_created = True
        self._cpp_world.route_choice_update_gradual = self.route_choice_update_gradual
        self._cpp_world.no_cyclic_routing = self.no_cyclic_routing

    # --- Scenario definition ---

    def addNode(self, name, x, y, signal=[0], signal_offset=0, signal_offset_old=None,
                flow_capacity=None, number_of_lanes=None, auto_rename=False,
                attribute=None, user_attribute=None, user_function=None):
        self._ensure_cpp_world()
        return CppNode(self, name, x, y, signal=signal, signal_offset=signal_offset,
                       signal_offset_old=signal_offset_old, flow_capacity=flow_capacity,
                       number_of_lanes=number_of_lanes, auto_rename=auto_rename,
                       attribute=attribute, user_attribute=user_attribute, user_function=user_function)

    def addLink(self, name, start_node, end_node, length,
                free_flow_speed=20, jam_density=0.2, jam_density_per_lane=None,
                number_of_lanes=1, merge_priority=1, signal_group=[0],
                capacity_out=None, capacity_in=None,
                congestion_pricing=None, eular_dx=None,
                attribute=None, user_attribute=None, user_function=None, auto_rename=False):
        self._ensure_cpp_world()
        return CppLink(self, name, start_node, end_node, length,
                       free_flow_speed=free_flow_speed, jam_density=jam_density,
                       jam_density_per_lane=jam_density_per_lane, number_of_lanes=number_of_lanes,
                       merge_priority=merge_priority, signal_group=signal_group,
                       capacity_out=capacity_out, capacity_in=capacity_in,
                       congestion_pricing=congestion_pricing, eular_dx=eular_dx,
                       attribute=attribute, user_attribute=user_attribute,
                       user_function=user_function, auto_rename=auto_rename)

    def addVehicle(self, *args, direct_call=True, **kwargs):
        """Add a single vehicle by forwarding to C++ as a one-platoon demand."""
        mode = kwargs.get('mode', 'single_trip')
        if mode == 'taxi':
            raise NotImplementedError("taxi mode is not supported in C++ mode")
        self._ensure_cpp_world()
        orig = args[0] if len(args) > 0 else kwargs.get('orig')
        dest = args[1] if len(args) > 1 else kwargs.get('dest')
        departure_time = args[2] if len(args) > 2 else kwargs.get('departure_time', 0)
        orig_name = orig.name if hasattr(orig, 'name') else str(orig)
        dest_name = dest.name if hasattr(dest, 'name') else str(dest)
        dep_is_ts = kwargs.get('departure_time_is_time_step', 0)
        t = departure_time * self.DELTAT if dep_is_ts else float(departure_time)

        # Resolve links_prefer to C++ link name strings
        links_prefer = kwargs.get('links_prefer', [])
        cpp_links_pref = []
        for l in links_prefer:
            name = l.name if hasattr(l, 'name') else str(l)
            cpp_links_pref.append(name)

        n_before = self._cpp_world.vehicle_count
        _add_demand(self._cpp_world, orig_name, dest_name,
                   float(t), float(t + self.DELTAT), float(self.DELTAN / self.DELTAT), cpp_links_pref)
        self._max_demand_t = max(getattr(self, '_max_demand_t', 0), t + self.DELTAT)

        # Set links_avoid on newly created C++ vehicles
        links_avoid = kwargs.get('links_avoid', [])
        if links_avoid:
            cpp_avoid = []
            for l in links_avoid:
                link_obj = self.get_link(l) if isinstance(l, str) else l
                try:
                    cpp_avoid.append(link_obj._cpp_link)
                except AttributeError:
                    pass
            n_after = self._cpp_world.vehicle_count
            for i in range(n_before, n_after):
                try:
                    self._cpp_world.get_vehicle_by_index(i).links_avoid = cpp_avoid
                except (AttributeError, TypeError):
                    pass

        self._register_new_cpp_vehicles()

    def _resolve_node_name(self, node):
        """Get node name string from Node object or string."""
        if hasattr(node, 'name'):
            return node.name
        return str(node)

    @demand_info_record
    def adddemand(self, orig, dest, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        self._ensure_cpp_world()
        if volume > 0:
            flow = volume / (t_end - t_start)
        _add_demand(self._cpp_world, self._resolve_node_name(orig), self._resolve_node_name(dest),
                   float(t_start), float(t_end), float(flow), [])
        self._max_demand_t = max(getattr(self, '_max_demand_t', 0), t_end)
        self._register_new_cpp_vehicles()

    @demand_info_record
    def adddemand_point2point(self, x_orig, y_orig, x_dest, y_dest, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        self.adddemand(self.get_nearest_node(x_orig, y_orig), self.get_nearest_node(x_dest, y_dest),
                       t_start, t_end, flow, volume, attribute, direct_call=False)

    @demand_info_record
    def adddemand_area2area(self, x_orig, y_orig, radious_orig, x_dest, y_dest, radious_dest,
                            t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        warnings.warn("`adddemand_area2area()` is not recommended. Use `adddemand_area2area2()`.", FutureWarning)
        origs = self.get_nodes_in_area(x_orig, y_orig, radious_orig)
        dests = self.get_nodes_in_area(x_dest, y_dest, radious_dest)
        origs = [o for o in origs if o.outlinks] or [self.get_nearest_node(x_orig, y_orig)]
        dests = [d for d in dests if d.inlinks] or [self.get_nearest_node(x_dest, y_dest)]
        if flow != -1: flow /= len(origs) * len(dests)
        if volume != -1: volume /= len(origs) * len(dests)
        for o in origs:
            for d in dests:
                self.adddemand(o, d, t_start, t_end, flow, volume, attribute, direct_call=False)

    @demand_info_record
    def adddemand_nodes2nodes(self, origs, dests, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        warnings.warn("`adddemand_nodes2nodes()` is not recommended. Use `adddemand_nodes2nodes2()`.", FutureWarning)
        origs = [self.get_node(o) for o in origs if self.get_node(o).outlinks]
        dests = [self.get_node(d) for d in dests if self.get_node(d).inlinks]
        if flow != -1: flow /= len(origs) * len(dests)
        if volume != -1: volume /= len(origs) * len(dests)
        for o in origs:
            for d in dests:
                self.adddemand(o, d, t_start, t_end, flow, volume, attribute, direct_call=False)

    @demand_info_record
    def adddemand_area2area2(self, x_orig, y_orig, radious_orig, x_dest, y_dest, radious_dest,
                             t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        origs = self.get_nodes_in_area(x_orig, y_orig, radious_orig)
        dests = self.get_nodes_in_area(x_dest, y_dest, radious_dest)
        origs = [o for o in origs if o.outlinks] or [self.get_nearest_node(x_orig, y_orig)]
        dests = [d for d in dests if d.inlinks] or [self.get_nearest_node(x_dest, y_dest)]
        if flow >= 0 and volume == -1:
            volume = flow * (t_end - t_start)
        size = int(volume / self.DELTAN)
        ts = np.linspace(t_start, t_end, size)
        os = self.rng.choice(origs, size=size)
        ds = self.rng.choice(dests, size=size)
        for t, o, d in zip(ts, os, ds):
            self.addVehicle(o, d, t, direct_call=False)

    @demand_info_record
    def adddemand_nodes2nodes2(self, origs, dests, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        origs = [self.get_node(o) for o in origs if self.get_node(o).outlinks]
        dests = [self.get_node(d) for d in dests if self.get_node(d).inlinks]
        if flow >= 0 and volume == -1:
            volume = flow * (t_end - t_start)
        size = int(volume / self.DELTAN)
        ts = np.linspace(t_start, t_end, size)
        os = self.rng.choice(origs, size=size)
        ds = self.rng.choice(dests, size=size)
        for t, o, d in zip(ts, os, ds):
            d2 = d
            if o == d:
                d2 = self.rng.choice([dd for dd in dests if dd != d])
            self.addVehicle(o, d2, t, direct_call=False)

    # --- Simulation ---

    def finalize_scenario(self, tmax=None):
        """Initialize C++ world and prepare analyzer."""
        if self.TMAX is None:
            self.TMAX = tmax if tmax else (getattr(self, '_max_demand_t', 0) // 1800 + 2) * 1800
        self._ensure_cpp_world()
        self._cpp_world.initialize_adj_matrix()
        self._setup_analyzer()

        # Pre-compute congestion pricing tolls and pass to C++ as timeseries
        for link in self.LINKS:
            if link.congestion_pricing is not None:
                tolls = [float(link.congestion_pricing(t * self.DELTAT)) for t in range(self.TSIZE)]
                link._cpp_link.set_toll_timeseries(tolls)

        self.finalized = 1
        self.sim_start_time = time.time()

    def _setup_analyzer(self):
        """One-time setup of Python data structures required by analyzer.py.
        This is NOT simulation logic — it initializes empty containers that
        analyzer.py reads/writes during post-simulation analysis.
        TODO: Move these to C++ or analyzer.py to eliminate from wrapper.
        """
        self.T = 0
        self.TIME = 0
        self.TSIZE = int(self.TMAX / self.DELTAT)
        self.Q_AREA = ddict(lambda: np.zeros(int(self.TMAX / self.EULAR_DT)))
        self.K_AREA = ddict(lambda: np.zeros(int(self.TMAX / self.EULAR_DT)))
        for l in self.LINKS:
            l.init_after_tmax_fix()
        self.ADJ_MAT = np.zeros([len(self.NODES), len(self.NODES)])
        self.ADJ_MAT_LINKS = dict()
        self.NODE_PAIR_LINKS = dict()
        for link in self.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            self.ADJ_MAT[i, j] = 1
            self.ADJ_MAT_LINKS[i, j] = link
            self.NODE_PAIR_LINKS[link.start_node.name, link.end_node.name] = link
        self.ROUTECHOICE = CppRouteChoice(self)
        self.analyzer = Analyzer(self)

    def exec_simulation(self, until_t=None, duration_t=None, duration_t2=None):
        """Run C++ simulation and sync results to Python."""
        if not self.finalized:
            self.finalize_scenario()

        # Convert Python duration params to C++ until_t
        if until_t is not None:
            cpp_until = until_t
        elif duration_t2 is not None:
            cpp_until = self.TIME + duration_t2
        elif duration_t is not None:
            cpp_until = self.TIME + duration_t + self.DELTAT
        else:
            cpp_until = self.TMAX

        self._cpp_world.main_loop(float(-1), float(cpp_until))
        self._sync_from_cpp()

        # Update Python-side time tracking from C++ state
        self.T = self._cpp_world.timestep
        self.TIME = self._cpp_world.time

        if self.T >= self.TSIZE:
            self.simulation_terminated()
            return 1
        return 0

    def _register_new_cpp_vehicles(self):
        """Register any new C++ vehicles into Python VEHICLES dict.
        Creates lightweight proxy objects — most attributes are read from C++ on demand."""
        start_idx = self._cpp_veh_registered_count
        total = self._cpp_world.vehicle_count
        n_new = total - start_idx
        if n_new <= 0:
            return
        nodes_name_dict = self.NODES_NAME_DICT
        vehicles = self.VEHICLES
        vehicles_living = self.VEHICLES_LIVING
        rcp = self.route_choice_principle
        get_veh = self._cpp_world.get_vehicle_by_index
        veh_id = len(vehicles)
        # Batch-generate random colors
        colors = self.rng.random(size=(n_new, 3))
        for j in range(n_new):
            i = start_idx + j
            cpp_veh = get_veh(i)
            name = cpp_veh.name
            py_veh = CppVehicle.__new__(CppVehicle)
            py_veh.W = self
            py_veh.name = name
            py_veh.id = veh_id
            veh_id += 1
            py_veh._cpp_vehicle = cpp_veh
            py_veh._log_cache = None
            py_veh.orig = nodes_name_dict.get(cpp_veh.orig.name) if cpp_veh.orig else None
            py_veh.dest = nodes_name_dict.get(cpp_veh.dest.name) if cpp_veh.dest else None
            c = colors[j]
            py_veh.color = (float(c[0]), float(c[1]), float(c[2]))
            # Python-only attributes (not in C++)
            py_veh.attribute = None
            py_veh.user_attribute = None
            py_veh.user_function = None
            py_veh.mode = 'single_trip'
            py_veh.dest_list = []
            py_veh.node_event = {}
            py_veh.route_pref = {}
            py_veh.links_prefer = []
            py_veh.links_avoid = []
            py_veh.specified_route = None
            py_veh.route_choice_principle = rcp
            py_veh.link_old = None
            vehicles[name] = py_veh
            vehicles_living[name] = py_veh
        self._cpp_veh_registered_count = total

    def _sync_from_cpp(self):
        """Update VEHICLES_LIVING / VEHICLES_RUNNING from C++ state.
        Log caches are never explicitly invalidated — they start as None and are
        built lazily on first access (e.g. when analyzer reads log_t)."""
        # Batch-update VEHICLES_LIVING / VEHICLES_RUNNING using C++ bulk query
        try:
            states = self._cpp_world.get_all_vehicle_states()
        except AttributeError:
            states = None

        if states is not None:
            vehicles = self.VEHICLES
            living = self.VEHICLES_LIVING
            running = self.VEHICLES_RUNNING
            # states is list of (cpp_name, state_int); Python names are sequential "0","1",...
            for i, (_, st_int) in enumerate(states):
                name = str(i)
                veh = vehicles.get(name)
                if veh is None:
                    continue
                if st_int >= 3:  # end(3) or abort(4)
                    living.pop(name, None)
                    running.pop(name, None)
                elif st_int == 2:  # run
                    living[name] = veh
                    running[name] = veh
                else:  # home(0) or wait(1)
                    living[name] = veh
                    running.pop(name, None)
        else:
            # Fallback: read state per-vehicle via property
            for name, veh in list(self.VEHICLES.items()):
                st = veh.state
                if st in ("end", "abort"):
                    self.VEHICLES_LIVING.pop(name, None)
                    self.VEHICLES_RUNNING.pop(name, None)
                elif st == "run":
                    self.VEHICLES_LIVING[name] = veh
                    self.VEHICLES_RUNNING[name] = veh
                else:
                    self.VEHICLES_LIVING[name] = veh
                    self.VEHICLES_RUNNING.pop(name, None)

    # --- Query methods ---

    def check_simulation_ongoing(self):
        return True if not self.finalized else self.T <= self.TSIZE - 1

    def _build_all_vehicle_log_caches(self):
        """Batch-fetch raw log data from C++ for all vehicles.
        Uses compact flat SoA API (no home prepend) for speed.
        Falls back to legacy APIs for older C++ builds."""
        try:
            flat = self._cpp_world.build_all_vehicle_logs_flat_compact()
        except AttributeError:
            try:
                flat = self._cpp_world.build_all_vehicle_logs_flat()
            except AttributeError:
                try:
                    logs = self._cpp_world.build_all_vehicle_logs()
                except AttributeError:
                    return
                vehicles = list(self.VEHICLES.values())
                for veh, log in zip(vehicles, logs):
                    if veh._log_cache is None:
                        veh._log_cache = log
                return
        offsets = flat['offsets']
        ltl_offsets = flat['ltl_offsets']
        all_log_t = flat['log_t']
        all_log_x = flat['log_x']
        all_log_v = flat['log_v']
        all_log_state = flat['log_state']
        all_log_s = flat['log_s']
        all_log_lane = flat['log_lane']
        all_log_link = flat['log_link']
        all_ltl_t = flat['ltl_t']
        all_ltl_id = flat['ltl_id']
        all_n_missing = flat.get('n_missing')
        delta_t = self.DELTAT
        vehicles = list(self.VEHICLES.values())
        for i, veh in enumerate(vehicles):
            if veh._log_cache is not None:
                continue
            s, e = int(offsets[i]), int(offsets[i + 1])
            ls, le = int(ltl_offsets[i]), int(ltl_offsets[i + 1])
            log_t = all_log_t[s:e]
            log_x = all_log_x[s:e]
            log_v = all_log_v[s:e]
            log_state = all_log_state[s:e]
            log_s = all_log_s[s:e]
            log_lane = all_log_lane[s:e]
            log_link = all_log_link[s:e]
            # Prepend home entries if compact format (n_missing available)
            if all_n_missing is not None:
                nm = int(all_n_missing[i])
                if nm > 0:
                    home_t = np.arange(nm, dtype=np.float64) * delta_t
                    home_fill_f = np.full(nm, -1.0)
                    home_fill_i = np.full(nm, -1, dtype=np.int32)
                    home_state = np.zeros(nm, dtype=np.int32)  # 0 = home
                    log_t = np.concatenate([home_t, log_t])
                    log_x = np.concatenate([home_fill_f, log_x])
                    log_v = np.concatenate([home_fill_f, log_v])
                    log_state = np.concatenate([home_state, log_state])
                    log_s = np.concatenate([home_fill_f, log_s])
                    log_lane = np.concatenate([home_fill_i, log_lane])
                    log_link = np.concatenate([home_fill_i, log_link])
            veh._log_cache = {
                'log_t': log_t,
                'log_x': log_x,
                'log_v': log_v,
                'log_state': log_state,
                'log_s': log_s,
                'log_lane': log_lane,
                'log_link': log_link,
                'log_t_link_t': all_ltl_t[ls:le],
                'log_t_link_id': all_ltl_id[ls:le],
            }

    def simulation_terminated(self):
        self.print(" simulation finished")
        self._simulation_done = True
        self._build_vehicles_enter_log()
        self._build_all_vehicle_log_caches()
        self.analyzer.basic_analysis()

    def _build_vehicles_enter_log(self):
        """Reconstruct each link's vehicles_enter_log from C++ bulk data."""
        for link in self.LINKS:
            link.vehicles_enter_log = {}
        try:
            data = self._cpp_world.build_enter_log_data()
            link_ids = data['link_id']
            times = data['time']
            veh_indices = data['vehicle_index']
            links_list = self.LINKS
            n_links = len(links_list)
            vehicles_list = list(self.VEHICLES.values())
            for i in range(len(link_ids)):
                lid = int(link_ids[i])
                if 0 <= lid < n_links:
                    links_list[lid].vehicles_enter_log[float(times[i])] = vehicles_list[int(veh_indices[i])]
        except AttributeError:
            # Fallback for older C++ builds without build_enter_log_data
            for veh in self.VEHICLES.values():
                for t, l in veh.log_t_link:
                    if isinstance(l, CppLink):
                        l.vehicles_enter_log[t] = veh

    def get_node(self, node):
        if node is None:
            return None
        if isinstance(node, CppNode):
            return node
        if isinstance(node, (str, np.str_)):
            return self.NODES_NAME_DICT.get(node)
        if hasattr(node, 'name'):
            return self.NODES_NAME_DICT.get(node.name)
        raise Exception(f"'{node}' is not Node in this World")

    def get_link(self, link):
        if link is None:
            return None
        if isinstance(link, CppLink):
            if link.W is self:
                return link
            return self.LINKS_NAME_DICT.get(link.name)
        if isinstance(link, (str, np.str_)):
            return self.LINKS_NAME_DICT.get(link)
        if hasattr(link, 'name'):
            return self.LINKS_NAME_DICT.get(link.name)
        raise Exception(f"'{link}' is not Link in this World")

    def get_nearest_node(self, x, y):
        return min(self.NODES, key=lambda n: (n.x - x)**2 + (n.y - y)**2) if self.NODES else None

    def get_nodes_in_area(self, x, y, r):
        return [n for n in self.NODES if (n.x - x)**2 + (n.y - y)**2 < r**2]

    def defRoute(self, links, name="", trust_input=False):
        return CppRoute(self, links, name=name, trust_input=trust_input)

    # --- I/O ---

    def load_scenario_from_csv(self, fname_node, fname_link, fname_demand, tmax=None):
        import csv as csv_mod
        self.generate_Nodes_from_csv(fname_node)
        self.generate_Links_from_csv(fname_link)
        self.generate_demand_from_csv(fname_demand)
        if tmax is not None:
            self.TMAX = tmax

    def generate_Nodes_from_csv(self, fname):
        import csv as csv_mod
        with open(fname) as f:
            for r in csv_mod.reader(f):
                if r[1] != "x":
                    self.addNode(r[0], float(r[1]), float(r[2]))

    def generate_Links_from_csv(self, fname):
        import csv as csv_mod
        with open(fname) as f:
            for r in csv_mod.reader(f):
                if r[3] != "length":
                    self.addLink(r[0], r[1], r[2], length=float(r[3]), free_flow_speed=float(r[4]),
                                jam_density=float(r[5]), merge_priority=float(r[6]))

    def generate_demand_from_csv(self, fname):
        import csv as csv_mod
        with open(fname) as f:
            for r in csv_mod.reader(f):
                if r[2] != "start_t":
                    try:
                        self.adddemand(r[0], r[1], float(r[2]), float(r[3]), float(r[4]), float(r[5]))
                    except:
                        self.adddemand(r[0], r[1], float(r[2]), float(r[3]), float(r[4]))

    def save_scenario(self, fname, network=True, demand=True):
        raise NotImplementedError("save_scenario is not yet supported in C++ mode.")

    def load_scenario(self, fname, network=True, demand=True):
        raise NotImplementedError("load_scenario is not yet supported in C++ mode.")

    def print_scenario_stats(self):
        label = "simulation setting:" if self.finalized else "simulation setting (not finalized):"
        print(label)
        print(" scenario name:", self.name)
        print(" simulation duration:\t", self.TMAX, "s")
        print(" number of vehicles:\t", len(self.VEHICLES) * self.DELTAN, "veh")
        print(" total road length:\t", sum(l.length for l in self.LINKS), "m")
        print(" timestep size:\t", self.DELTAT, "s")
        print(" platoon size:\t\t", self.DELTAN, "veh")
        print(" number of timesteps:\t", self.TSIZE if self.finalized else (self.TMAX / self.DELTAT if self.TMAX else "N/A"))
        print(" number of platoons:\t", len(self.VEHICLES))
        print(" number of links:\t", len(self.LINKS))
        print(" number of nodes:\t", len(self.NODES))
        print(" setup time:\t\t", f"{time.time() - self.world_start_time:.2f}", "s")

    def on_time(self, time_val):
        return self.T == int(time_val / self.DELTAT)

    def change_print_mode(self, print_mode):
        self.print_mode = print_mode
        self.print = print if print_mode else (lambda *a, **k: None)

    @catch_exceptions_and_warn()
    def show_network(self, width=1, left_handed=1, figsize=(6, 6), network_font_size=10, node_size=6, show_id=True):
        import matplotlib.pyplot as plt
        import os
        os.makedirs(f"out{self.name}", exist_ok=True)
        plt.rcParams["font.family"] = get_font_for_matplotlib()
        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")
        for n in self.NODES:
            plt.plot(n.x, n.y, "o", c="gray", ms=node_size, zorder=10)
            if network_font_size > 0:
                plt.text(n.x, n.y, f"{n.id}: {n.name}" if show_id else n.name, c="g",
                        horizontalalignment="center", verticalalignment="top", zorder=20, fontsize=network_font_size)
        for l in self.LINKS:
            x1, y1 = l.start_node.x, l.start_node.y
            x2, y2 = l.end_node.x, l.end_node.y
            vx, vy = (y1 - y2) * 0.05, (x2 - x1) * 0.05
            if not left_handed: vx, vy = -vx, -vy
            xm1, ym1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
            xm2, ym2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
            plt.plot([x1, xm1, xm2, x2], [y1, ym1, ym2, y2], "gray", lw=width, zorder=6, solid_capstyle="butt")
            if network_font_size > 0:
                plt.text(xm1, ym1, f"{l.id}: {l.name}" if show_id else l.name, c="b", zorder=20, fontsize=network_font_size)
        xs = [n.x for n in self.NODES]
        ys = [n.y for n in self.NODES]
        b = max((max(xs)-min(xs))/10, (max(ys)-min(ys))/10)
        plt.xlim([min(xs)-b, max(xs)+b])
        plt.ylim([min(ys)-b, max(ys)+b])
        plt.tight_layout()
        if self.save_mode: plt.savefig(f"out{self.name}/network.png")
        if self.show_mode: plt.show()
        else: plt.close("all")

    def copy(self):
        raise NotImplementedError("copy() is not supported in C++ mode")

    def save(self, fname):
        raise NotImplementedError("save() is not supported in C++ mode")
