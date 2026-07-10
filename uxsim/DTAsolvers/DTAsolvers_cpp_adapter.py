"""
C++ engine adapter for Dynamic Traffic Assignment solvers.

This module contains everything specific to the C++ engine (``World(cpp=True)``):
the C++ accelerated solver classes (:class:`CppSolverDUE`, :class:`CppSolverDSO_D2D`)
and their helper functions (id<->name conversion, nested-CSR route handling,
lightweight finalize, etc.). The pure-Python solvers live in ``DTAsolvers.py``.

``SolverDUE(func_World, cpp=True)`` / ``SolverDSO_D2D(func_World, cpp=True)`` return
instances of the classes defined here via a lazy import in the base class ``__new__``,
so this module is not imported when only the pure-Python solvers are used.
"""
import copy
import random
import time
import warnings
from collections import defaultdict

import numpy as np

from .DTAsolvers import SolverDUE, SolverDSO_D2D


def _ensure_cpp_world(W, solver_name="CppSolver"):
    """Verify that W is a CppWorld instance (has _cpp_world attribute).

    Raises ValueError if func_World did not return a World(cpp=True).
    """
    if not hasattr(W, '_cpp_world'):
        raise ValueError(
            f"{solver_name} requires func_World to return World(cpp=True, ...). "
            f"Got {type(W).__name__} without C++ engine. "
            f"Add cpp=True to your World() constructor inside func_World."
        )


def _lightweight_finalize(W, cached_analyzer=None):
    """Lightweight finalize_scenario for DTA intermediate iterations.

    Performs essential initialization (TMAX, adj_matrix, TSIZE) but reuses
    a cached Analyzer object to skip expensive font loading.

    Parameters
    ----------
    W : CppWorld
    cached_analyzer : Analyzer or None
        Analyzer from a previous iteration to reuse. If None, does full init.

    Returns
    -------
    Analyzer : the (possibly reused) Analyzer object now attached to W.
    """
    if W.TMAX is None:
        W.TMAX = (getattr(W, '_max_demand_t', 0) // 1800 + 2) * 1800
    W._ensure_cpp_world()
    W._cpp_world.initialize_adj_matrix()

    # Minimal _setup_analyzer equivalent
    W.T = 0
    W.TIME = 0
    W.TSIZE = int(W.TMAX / W.DELTAT)
    W.Q_AREA = defaultdict(lambda: np.zeros(int(W.TMAX / W.EULAR_DT)))
    W.K_AREA = defaultdict(lambda: np.zeros(int(W.TMAX / W.EULAR_DT)))
    for l in W.LINKS:
        l.init_after_tmax_fix()
    W.ADJ_MAT = np.zeros([len(W.NODES), len(W.NODES)])
    W.ADJ_MAT_LINKS = dict()
    W.NODE_PAIR_LINKS = dict()
    for link in W.LINKS:
        ii = link.start_node.id
        jj = link.end_node.id
        W.ADJ_MAT[ii, jj] = 1
        W.ADJ_MAT_LINKS[ii, jj] = link
        W.NODE_PAIR_LINKS[link.start_node.name, link.end_node.name] = link

    from ..uxsim_cpp_wrapper import CppRouteChoice
    W.ROUTECHOICE = CppRouteChoice(W)

    if cached_analyzer is not None:
        # Reuse existing Analyzer, just rebind to new World and reset stats
        analyzer = cached_analyzer
        analyzer.W = W
        analyzer.average_speed = 0
        analyzer.average_speed_count = 0
        analyzer.trip_completed = 0
        analyzer.trip_all = 0
        analyzer.total_travel_time = 0
        analyzer.average_travel_time = 0
        analyzer.flag_edie_state_computed = 0
        analyzer.flag_trajectory_computed = 0
        analyzer.flag_pandas_convert = 0
        analyzer.flag_od_analysis = 0
        W.analyzer = analyzer
    else:
        from ..analyzer import Analyzer
        W.analyzer = Analyzer(W)

    # Pre-compute congestion pricing tolls
    for link in W.LINKS:
        if link.congestion_pricing is not None:
            tolls = [float(link.congestion_pricing(t * W.DELTAT)) for t in range(W.TSIZE)]
            link._cpp_link.set_toll_timeseries(tolls)

    W.finalized = 1
    W.sim_start_time = time.time()

    return W.analyzer


def _build_id_lookup_arrays(W):
    """Build id->name lookup arrays/dicts (indexed by C++ id) for a CppWorld.

    Returns (link_name_to_id, link_id_to_name, node_name_to_id, node_id_to_name)
    where the *_id_to_name are dicts keyed by C++ id.
    """
    link_name_to_id = {link.name: link.id for link in W.LINKS}
    link_id_to_name = {link.id: link.name for link in W.LINKS}
    node_name_to_id = {node.name: node.id for node in W.NODES}
    node_id_to_name = {node.id: node.name for node in W.NODES}
    return link_name_to_id, link_id_to_name, node_name_to_id, node_id_to_name


class _LazyODRoutes(dict):
    """Lazy {(o_name, d_name): [[link_name, ...], ...]} built on first access.

    The C++ solvers consume OD route sets purely as link IDs, so the name-form
    dict is only needed if the solution World is later reused (e.g. as an
    initial_solution_World for a Python solver, or by ALNS). Deferring its
    construction avoids the per-solve name-building cost in the common case.
    Subclasses dict so ``isinstance(x, dict)`` and normal dict access hold.
    """
    __slots__ = ("_csr", "_nid2n", "_lid2n", "_built")

    def __init__(self, csr, node_id_to_name, link_id_to_name):
        super().__init__()
        self._csr = csr
        self._nid2n = node_id_to_name
        self._lid2n = link_id_to_name
        self._built = False

    def _build(self):
        if self._built:
            return
        self._built = True  # set first to avoid recursion via dict methods
        super().update(_build_od_name_dict_from_csr(self._csr, self._nid2n, self._lid2n))

    def __getitem__(self, k):
        self._build()
        return super().__getitem__(k)

    def __contains__(self, k):
        self._build()
        return super().__contains__(k)

    def __iter__(self):
        self._build()
        return super().__iter__()

    def __len__(self):
        self._build()
        return super().__len__()

    def get(self, k, default=None):
        self._build()
        return super().get(k, default)

    def keys(self):
        self._build()
        return super().keys()

    def values(self):
        self._build()
        return super().values()

    def items(self):
        self._build()
        return super().items()


def _build_od_name_dict_from_csr(csr, node_id_to_name, link_id_to_name):
    """Build {(o_name, d_name): [[link_name, ...], ...]} from nested-CSR arrays.

    This reuses interned name string objects from the id->name dicts (no new
    string allocation per element), so it is far cheaper than the per-element
    nb::str construction of the name-based public API. Built once per solve.
    """
    od_o = csr['od_o']
    od_d = csr['od_d']
    route_offsets = csr['route_offsets']
    link_offsets = csr['link_offsets']
    link_ids = csr['link_ids']
    dict_od_to_routes = defaultdict(list)
    for p in range(len(od_o)):
        o = node_id_to_name[int(od_o[p])]
        d = node_id_to_name[int(od_d[p])]
        r0 = int(route_offsets[p])
        r1 = int(route_offsets[p + 1])
        routes = []
        for r in range(r0, r1):
            s = int(link_offsets[r])
            e = int(link_offsets[r + 1])
            routes.append([link_id_to_name[int(lid)] for lid in link_ids[s:e]])
        dict_od_to_routes[o, d] = routes
    return dict_od_to_routes


def _convert_flat_result_to_names(result, veh_names, link_id_to_name):
    """Convert flat-CSR C++ route_swap result to name-keyed route_actual/cost_actual.

    routes_specified is intentionally NOT converted to names here: it is kept in
    flat ID-CSR form (rs_link_ids/rs_offsets) and fed directly back to
    batch_enforce_routes_csr, avoiding a name<->id round-trip.

    Returns (route_actual_dict, cost_actual_dict, n_swap, potential_n_swap, total_t_gap).
    """
    ra_ids = result['ra_link_ids']
    ra_off = result['ra_offsets']
    ca = result['cost_actual']

    route_actual = {}
    for idx, veh_name in enumerate(veh_names):
        s = int(ra_off[idx])
        e = int(ra_off[idx + 1])
        route_actual[veh_name] = [link_id_to_name[int(lid)] for lid in ra_ids[s:e]]

    cost_actual = {veh_name: float(ca[idx]) for idx, veh_name in enumerate(veh_names)}

    return (route_actual, cost_actual,
            float(result['n_swap']), float(result['potential_n_swap']),
            float(result['total_t_gap']))


def _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id, link_name_to_id):
    """Convert dict_od_to_routes from str names to int IDs for C++.

    Parameters
    ----------
    dict_od_to_routes : dict
        {(o_name, d_name): [[link_name, ...], ...]}
    node_name_to_id : dict
        {node_name: node_id}
    link_name_to_id : dict
        {link_name: link_id}

    Returns
    -------
    dict : {(node_id_o, node_id_d): [[link_id, ...], ...]}
    """
    od_route_sets = {}
    for (o_name, d_name), routes in dict_od_to_routes.items():
        o_id = node_name_to_id[o_name]
        d_id = node_name_to_id[d_name]
        od_route_sets[(o_id, d_id)] = [
            [link_name_to_id[ln] for ln in route] for route in routes
        ]
    return od_route_sets


class CppSolverDUE(SolverDUE):
    """C++ accelerated DUE solver. Created via ``SolverDUE(func_World, cpp=True)``."""

    _is_cpp_solver = True

    def __init__(s, func_World, **kwargs):
        s.func_World = func_World
        s.W_sol = None
        s.W_intermid_solution = None
        s.dfs_link = []

    def solve(s, max_iter, n_routes_per_od=10, swap_prob=0.05, route_sets=None, print_progress=True):
        s.start_time = time.time()

        W_orig = s.func_World()
        _ensure_cpp_world(W_orig, "CppSolverDUE")
        if print_progress:
            W_orig.print_scenario_stats()

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            dict_od_to_vehid[veh.orig.name, veh.dest.name].append(key)

        if W_orig.finalized == False:
            W_orig.finalize_scenario()

        # Build id<->name lookups once (node/link ids are stable across
        # func_World() reconstructions, so W_orig's mapping is valid for all iters).
        (link_name_to_id, link_id_to_name, node_name_to_id,
         node_id_to_name) = _build_id_lookup_arrays(W_orig)

        # Draw the enumeration seed exactly as enumerate_k_random_routes() would,
        # to preserve global-RNG consumption order (and thus bit-identical results).
        enum_seed = random.randint(0, 2**31 - 1)

        if route_sets is not None:
            dict_od_to_routes = {}
            for key, routes in route_sets.items():
                o = W_orig.get_node(key[0]).name
                d = W_orig.get_node(key[1]).name
                dict_od_to_routes[o, d] = [[W_orig.get_link(l).name for l in route] for route in routes]
            od_route_sets_ids = _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id, link_name_to_id)
            od_route_store = W_orig._cpp_world.make_od_route_set_store(od_route_sets_ids)
        else:
            # Enumerate + register routes entirely in C++ (link IDs, no name round-trip).
            od_route_store = W_orig._cpp_world.enumerate_od_route_sets_ids_cpp(n_routes_per_od, enum_seed)
            # Build the name-form dict lazily (only if the solution is reused).
            dict_od_to_routes = _LazyODRoutes(
                od_route_store.to_csr(), node_id_to_name, link_id_to_name)

        if print_progress:
            print(f"number of OD pairs: {od_route_store.n_od_pairs}, number of routes: {od_route_store.n_routes}")

        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.t_gaps = []
        s.route_log = []
        s.cost_log = []

        cached_analyzer = None
        prev_rs_link_ids = None
        prev_rs_offsets = None

        print("solving DUE...")
        for i in range(max_iter):
            W = s.func_World()
            is_last_iter = (i == max_iter - 1)

            cached_analyzer = _lightweight_finalize(W, cached_analyzer)

            # Skip expensive log cache building on intermediate iterations;
            # CppWorld log mode is set at __init__ time, so vehicle_logging_timestep_interval
            # cannot be changed after construction — _skip_log_on_terminate handles this instead.
            if not is_last_iter:
                W._skip_log_on_terminate = True

            if i != 0:
                W._cpp_world.batch_enforce_routes_csr(prev_rs_link_ids, prev_rs_offsets)

            W.exec_simulation()
            W.analyzer.print_simple_stats()

            unfinished_trips = W.analyzer.trip_all - W.analyzer.trip_completed
            if unfinished_trips > 0:
                warnings.warn(f"Warning: {unfinished_trips} / {W.analyzer.trip_all} vehicles have not finished their trips. The DUE solver assumes that all vehicles finish their trips during the simulation duration. Consider increasing the simulation time limit or checking the network configuration.", UserWarning)

            W.dict_od_to_routes = dict_od_to_routes
            s.W_intermid_solution = W

            s.dfs_link.append(W.analyzer.link_to_pandas())

            rng_seed = random.randint(0, 2**31 - 1)
            cpp_result = W._cpp_world.route_swap_due(
                od_route_store, swap_prob, route_sets is not None, rng_seed
            )
            veh_names = list(W.VEHICLES.keys())
            route_actual, cost_actual, n_swap, potential_n_swap, total_t_gap = \
                _convert_flat_result_to_names(cpp_result, veh_names, link_id_to_name)
            # Keep next-iteration routes in flat ID-CSR form for direct enforcement.
            prev_rs_link_ids = cpp_result['rs_link_ids']
            prev_rs_offsets = cpp_result['rs_offsets']

            t_gap_per_vehicle = total_t_gap / len(W.VEHICLES)
            if print_progress:
                print(f' iter {i}: time gap: {t_gap_per_vehicle:.1f}, potential route change: {potential_n_swap}, route change: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)
            s.ttts.append(int(W.analyzer.total_travel_time))
            s.n_swaps.append(n_swap)
            s.potential_swaps.append(potential_n_swap)
            s.t_gaps.append(t_gap_per_vehicle)

        s.end_time = time.time()

        print("DUE summary:")
        last_iters = int(max_iter / 4)
        print(f" total travel time: initial {s.ttts[0]:.1f} -> average of last {last_iters} iters {np.average(s.ttts[-last_iters:]):.1f}")
        print(f" number of potential route changes: initial {s.potential_swaps[0]:.1f} -> average of last {last_iters} iters {np.average(s.potential_swaps[-last_iters:]):.1f}")
        print(f" route travel cost gap: initial {s.t_gaps[0]:.1f} -> average of last {last_iters} iters {np.average(s.t_gaps[-last_iters:]):.1f}")
        print(f" computation time: {s.end_time - s.start_time:.1f} seconds")

        s.W_sol = W
        return s.W_sol


class CppSolverDSO_D2D(SolverDSO_D2D):
    """C++ accelerated DSO solver. Created via ``SolverDSO_D2D(func_World, cpp=True)``."""

    _is_cpp_solver = True

    def __init__(s, func_World, **kwargs):
        s.func_World = func_World
        s.W_sol = None
        s.W_intermid_solution = None
        s.dfs_link = []

    def solve(s, max_iter, n_routes_per_od=10, swap_prob=0.05, swap_num=None, route_sets=None, print_progress=True):
        s.start_time = time.time()

        W_orig = s.func_World()
        _ensure_cpp_world(W_orig, "CppSolverDSO_D2D")
        if print_progress:
            W_orig.print_scenario_stats()

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            dict_od_to_vehid[veh.orig.name, veh.dest.name].append(key)

        if W_orig.finalized == False:
            W_orig.finalize_scenario()

        # Build id<->name lookups once (ids stable across func_World() reconstructions).
        (link_name_to_id, link_id_to_name, node_name_to_id,
         node_id_to_name) = _build_id_lookup_arrays(W_orig)

        # Draw the enumeration seed exactly as enumerate_k_random_routes() would,
        # to preserve global-RNG consumption order (and thus bit-identical results).
        enum_seed = random.randint(0, 2**31 - 1)

        if route_sets is not None:
            dict_od_to_routes = {}
            for key, routes in route_sets.items():
                o = W_orig.get_node(key[0]).name
                d = W_orig.get_node(key[1]).name
                dict_od_to_routes[o, d] = [[W_orig.get_link(l).name for l in route] for route in routes]
            od_route_sets_ids = _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id, link_name_to_id)
            od_route_store = W_orig._cpp_world.make_od_route_set_store(od_route_sets_ids)
        else:
            # Enumerate + register routes entirely in C++ (link IDs, no name round-trip).
            od_route_store = W_orig._cpp_world.enumerate_od_route_sets_ids_cpp(n_routes_per_od, enum_seed)
            # Build the name-form dict lazily (only if the solution is reused).
            dict_od_to_routes = _LazyODRoutes(
                od_route_store.to_csr(), node_id_to_name, link_id_to_name)

        if print_progress:
            print(f"number of OD pairs: {od_route_store.n_od_pairs}, number of routes: {od_route_store.n_routes}")

        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.t_gaps = []
        s.route_log = []
        s.cost_log = []

        cached_analyzer = None
        prev_rs_link_ids = None
        prev_rs_offsets = None
        best_avg_tt = None

        print("solving DSO...")
        for i in range(max_iter):
            W = s.func_World()
            is_last_iter = (i == max_iter - 1)

            cached_analyzer = _lightweight_finalize(W, cached_analyzer)

            # CppWorld log mode is fixed at __init__; use _skip_log_on_terminate instead
            if not is_last_iter:
                W._skip_log_on_terminate = True

            if i != 0:
                W._cpp_world.batch_enforce_routes_csr(prev_rs_link_ids, prev_rs_offsets)

            W.exec_simulation()
            W.analyzer.print_simple_stats()

            unfinished_trips = W.analyzer.trip_all - W.analyzer.trip_completed
            if unfinished_trips > 0:
                warnings.warn(f"Warning: {unfinished_trips} / {W.analyzer.trip_all} vehicles have not finished their trips. The DSO solver assumes that all vehicles finish their trips during the simulation duration. Consider increasing the simulation time limit or checking the network configuration.", UserWarning)

            W.dict_od_to_routes = dict_od_to_routes

            if unfinished_trips == 0:
                # The Analyzer object is shared across iterations (rebound in
                # _lightweight_finalize), so compare against a saved scalar and
                # detach the best W's analyzer by shallow copy to freeze its stats.
                avg_tt = W.analyzer.average_travel_time
                if s.W_intermid_solution is None or avg_tt < best_avg_tt:
                    best_avg_tt = avg_tt
                    W.analyzer = copy.copy(W.analyzer)
                    s.W_intermid_solution = W

            s.dfs_link.append(W.analyzer.link_to_pandas())

            rng_seed = random.randint(0, 2**31 - 1)
            cpp_swap_num = swap_num if swap_num is not None else -1
            cpp_result = W._cpp_world.route_swap_dso(
                od_route_store, swap_prob, cpp_swap_num, route_sets is not None, rng_seed
            )
            veh_names = list(W.VEHICLES.keys())
            route_actual, cost_actual, n_swap, potential_n_swap, total_t_gap = \
                _convert_flat_result_to_names(cpp_result, veh_names, link_id_to_name)
            # Keep next-iteration routes in flat ID-CSR form for direct enforcement.
            prev_rs_link_ids = cpp_result['rs_link_ids']
            prev_rs_offsets = cpp_result['rs_offsets']

            t_gap_per_vehicle = total_t_gap / len(W.VEHICLES)
            if print_progress:
                print(f' iter {i}: marginal time gap: {t_gap_per_vehicle:.1f}, potential route change: {potential_n_swap}, route change: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)
            s.ttts.append(int(W.analyzer.total_travel_time))
            s.n_swaps.append(n_swap)
            s.potential_swaps.append(potential_n_swap)
            s.t_gaps.append(t_gap_per_vehicle)

        s.end_time = time.time()

        print("DSO summary:")
        last_iters = int(max_iter / 4)
        print(f" total travel time: initial {s.ttts[0]:.1f} -> minimum {np.min(s.ttts):.1f}")
        print(f" number of potential route changes: initial {s.potential_swaps[0]:.1f} -> minimum {np.min(s.potential_swaps):.1f}")
        print(f" route marginal travel time gap: initial {s.t_gaps[0]:.1f} -> minimum {np.min(s.t_gaps):.1f}")
        print(f" computation time: {s.end_time - s.start_time:.1f} seconds")

        s.W_sol = s.W_intermid_solution
        return s.W_sol
