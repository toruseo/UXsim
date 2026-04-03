"""
Submodule for Dynamic Traffic Assignment solvers.
"""
import random
import time
import warnings
from pprint import pprint
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

from ..Utilities import enumerate_k_shortest_routes, enumerate_k_random_routes, estimate_congestion_externality_route
from ..utils import *
from . import ALNS

import warnings


def _make_cpp_func_world(func_World):
    """Wrap func_World to inject cpp=True into World() calls.

    Temporarily monkey-patches World.__new__ so that any World(...)
    created inside func_World automatically gets cpp=True.
    """
    def wrapper():
        from ..uxsim import World
        _orig_new = World.__new__
        def _cpp_new(cls, *args, cpp=False, **kwargs):
            return _orig_new(cls, *args, cpp=True, **kwargs)
        World.__new__ = _cpp_new
        try:
            return func_World()
        finally:
            World.__new__ = _orig_new
    return wrapper


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


def _build_name_id_maps(W):
    """Build name<->ID mapping dicts for a CppWorld. Called once per iteration."""
    link_name_to_id = {link.name: link.id for link in W.LINKS}
    link_id_to_name = [link.name for link in W.LINKS]
    node_name_to_id = {node.name: node.id for node in W.NODES}
    return link_name_to_id, link_id_to_name, node_name_to_id


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


def _convert_cpp_result_to_names(result, W, link_id_to_name):
    """Convert C++ route_swap result from int IDs to str names.

    Parameters
    ----------
    result : dict
        C++ result with routes_specified/route_actual as list[list[int]],
        cost_actual as numpy array
    W : CppWorld
        World object (for vehicle name list)
    link_id_to_name : list
        link_id -> link_name mapping

    Returns
    -------
    dict with routes_specified/route_actual as dict[str, list[str]],
         cost_actual as dict[str, float], and scalar metrics
    """
    veh_names = list(W.VEHICLES.keys())
    rs = result['routes_specified']
    ra = result['route_actual']
    ca = result['cost_actual']

    routes_specified_data = {}
    for idx, route_ids in enumerate(rs):
        if route_ids:
            routes_specified_data[veh_names[idx]] = [link_id_to_name[lid] for lid in route_ids]

    route_actual = {}
    for idx, route_ids in enumerate(ra):
        route_actual[veh_names[idx]] = [link_id_to_name[lid] for lid in route_ids]

    cost_actual = {}
    for idx, veh_name in enumerate(veh_names):
        cost_actual[veh_name] = float(ca[idx])

    return {
        'routes_specified': routes_specified_data,
        'route_actual': route_actual,
        'cost_actual': cost_actual,
        'n_swap': float(result['n_swap']),
        'potential_n_swap': float(result['potential_n_swap']),
        'total_t_gap': float(result['total_t_gap']),
    }


def _build_enforce_routes_input(W, routes_specified_data, link_name_to_id):
    """Convert routes_specified_data to list[list[int]] for C++ batch_enforce_routes.

    Parameters
    ----------
    W : CppWorld
    routes_specified_data : dict[str, list[str]]
        {veh_name: [link_name, ...]}
    link_name_to_id : dict
        {link_name: link_id}

    Returns
    -------
    list[list[int]] : one entry per vehicle in VEHICLES order. Empty list = skip.
    """
    routes_per_vehicle = []
    for veh_name in W.VEHICLES:
        if veh_name in routes_specified_data:
            routes_per_vehicle.append(
                [link_name_to_id[ln] for ln in routes_specified_data[veh_name]]
            )
        else:
            routes_per_vehicle.append([])
    return routes_per_vehicle


def _route_swap_due_python(W, dict_od_to_routes, dict_od_to_vehid, swap_prob, has_fixed_route_sets):
    """Python fallback for DUE route swap logic.

    This function extracts the route swap loop from SolverDUE.solve() so that
    it can be replaced by a C++ batch call later. The logic is identical to
    the original Python loop.

    Parameters
    ----------
    W : World or CppWorld
        World object after simulation execution
    dict_od_to_routes : dict
        {(o_name, d_name): [[link_name, ...], ...]}
    dict_od_to_vehid : dict
        {(o_name, d_name): [veh_key, ...]}
    swap_prob : float
        probability of route swap per vehicle
    has_fixed_route_sets : bool
        whether route_sets was user-provided

    Returns
    -------
    dict with keys: routes_specified, route_actual, cost_actual,
                    n_swap, potential_n_swap, total_t_gap
    """
    # Build Route objects for this World
    route_set = defaultdict(lambda: [])
    for o, d in dict_od_to_vehid.keys():
        for r in dict_od_to_routes[o, d]:
            route_set[o, d].append(W.defRoute(r))

    routes_specified_data = {}
    route_actual = {}
    cost_actual = {}
    n_swap = 0
    total_t_gap = 0
    potential_n_swap = 0

    for key, veh in W.VEHICLES.items():
        flag_swap = random.random() < swap_prob
        o = veh.orig.name
        d = veh.dest.name
        r, ts = veh.traveled_route()
        travel_time = ts[-1] - ts[0]

        route_actual[key] = [rr.name for rr in r]
        cost_actual[key] = travel_time

        if veh.state != "end":
            continue

        if has_fixed_route_sets and r not in route_set[o, d]:
            routes_specified_data[key] = route_set[o, d][0].links_name
            continue

        flag_route_changed = False
        route_changed_links = None
        t_gap = 0

        cost_current = r.actual_travel_time(ts[0]) + sum([l.get_toll(ts[i]) for i, l in enumerate(r)])

        potential_n_swap_updated = potential_n_swap
        for alt_route in route_set[o, d]:
            alt_route_tts = alt_route.actual_travel_time(ts[0], return_details=True)[1]
            cost_alt = alt_route.actual_travel_time(ts[0]) + sum([l.get_toll(ts[0] + sum(alt_route_tts[:i])) for i, l in enumerate(alt_route)])
            if cost_alt < cost_current:
                if flag_route_changed == False or (cost_alt < cost_current):
                    t_gap = cost_current - cost_alt
                    potential_n_swap_updated = potential_n_swap + W.DELTAN
                    if flag_swap:
                        flag_route_changed = True
                        route_changed_links = alt_route.links_name
                        cost_current = cost_alt

        potential_n_swap = potential_n_swap_updated

        total_t_gap += t_gap
        routes_specified_data[key] = [rr.name for rr in r]
        if flag_route_changed:
            n_swap += W.DELTAN
            routes_specified_data[key] = route_changed_links

    return {
        'routes_specified': routes_specified_data,
        'route_actual': route_actual,
        'cost_actual': cost_actual,
        'n_swap': n_swap,
        'potential_n_swap': potential_n_swap,
        'total_t_gap': total_t_gap,
    }


def _route_swap_dso_python(W, dict_od_to_routes, dict_od_to_vehid, swap_prob, swap_num, has_fixed_route_sets):
    """Python fallback for DSO route swap logic.

    This function extracts the route swap loop from SolverDSO_D2D.solve() so
    that it can be replaced by a C++ batch call later. The logic is identical
    to the original Python loop, using marginal cost (private + externality).

    Parameters
    ----------
    W : World or CppWorld
        World object after simulation execution
    dict_od_to_routes : dict
        {(o_name, d_name): [[link_name, ...], ...]}
    dict_od_to_vehid : dict
        {(o_name, d_name): [veh_key, ...]}
    swap_prob : float
        probability of route swap per vehicle
    swap_num : int or None
        fixed number of vehicles to swap (overrides swap_prob if set)
    has_fixed_route_sets : bool
        whether route_sets was user-provided

    Returns
    -------
    dict with keys: routes_specified, route_actual, cost_actual,
                    n_swap, potential_n_swap, total_t_gap
    """
    # Build Route objects for this World
    route_set = defaultdict(lambda: [])
    for o, d in dict_od_to_vehid.keys():
        for r in dict_od_to_routes[o, d]:
            route_set[o, d].append(W.defRoute(r))

    routes_specified_data = {}
    route_actual = {}
    cost_actual = {}
    n_swap = 0
    total_t_gap = 0
    potential_n_swap = 0

    keys_swap = [key for key in W.VEHICLES.keys() if random.random() < swap_prob]
    if swap_num is not None:
        keys_swap = random.sample(list(W.VEHICLES.keys()), swap_num)

    for key, veh in W.VEHICLES.items():
        flag_swap = key in keys_swap
        o = veh.orig.name
        d = veh.dest.name
        r, ts = veh.traveled_route()
        travel_time = ts[-1] - ts[0]

        route_actual[key] = [rr.name for rr in r]
        cost_actual[key] = travel_time

        if veh.state != "end":
            continue

        if has_fixed_route_sets and r not in route_set[o, d]:
            routes_specified_data[key] = route_set[o, d][0].links_name
            continue

        flag_route_changed = False
        route_changed_links = None
        t_gap = 0

        ext = estimate_congestion_externality_route(W, r, ts[0])
        private_cost = r.actual_travel_time(ts[0])
        cost_current = private_cost + ext

        potential_n_swap_updated = potential_n_swap

        for alt_route in route_set[o, d]:
            ext = estimate_congestion_externality_route(W, alt_route, ts[0])
            private_cost = alt_route.actual_travel_time(ts[0])
            cost_alt = private_cost + ext

            if cost_alt < cost_current:
                if flag_route_changed == False or (cost_alt < cost_current):
                    t_gap = cost_current - cost_alt
                    potential_n_swap_updated = potential_n_swap + W.DELTAN
                    if flag_swap:
                        flag_route_changed = True
                        route_changed_links = alt_route.links_name
                        cost_current = cost_alt

        potential_n_swap = potential_n_swap_updated

        total_t_gap += t_gap
        routes_specified_data[key] = [rr.name for rr in r]
        if flag_route_changed:
            n_swap += W.DELTAN
            routes_specified_data[key] = route_changed_links

    return {
        'routes_specified': routes_specified_data,
        'route_actual': route_actual,
        'cost_actual': cost_actual,
        'n_swap': n_swap,
        'potential_n_swap': potential_n_swap,
        'total_t_gap': total_t_gap,
    }


class SolverDUE:

    def __new__(cls, func_World, cpp=False):
        if cpp and cls is SolverDUE:
            return CppSolverDUE(func_World)
        return super().__new__(cls)

    def __init__(s, func_World, cpp=False):
        """
        Solve quasi Dynamic User Equilibrium (DUE) problem using day-to-day dynamics.

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
        cpp : bool
            if True, return a CppSolverDUE instance that uses C++ batch operations

        Notes
        -----
            This function computes a near dynamic user equilibrium state as a steady state of day-to-day dynamical routing game.

            Specifically, on day `i`, vehicles choose their route based on actual travel time on day `i-1` with the same departure time.
            If there are shorter travel time route, they will change with probability `swap_prob`.
            This process is repeated until `max_iter` day.
            It is expected that this process eventually reach a steady state.
            Due to the problem complexity, it does not necessarily reach or converge to Nash equilibrium or any other stationary points.
            However, in the literature, it is argued that the steady state can be considered as a reasonable proxy for Nash equilibrium or dynamic equilibrium state.
            There are some theoretical background for it; but intuitively speaking, the steady state can be considered as a realistic state that people's rational behavior will reach.

            This method is based on the following literature:

            - Ishihara, M., & Iryo, T. (2015). Dynamic Traffic Assignment by Markov Chain. Journal of Japan Society of Civil Engineers, Ser. D3 (Infrastructure Planning and Management), 71(5), I_503-I_509. (in Japanese). https://doi.org/10.2208/jscejipm.71.I_503
            - Iryo, T., Urata, J., & Kawase, R. (2024). Traffic Flow Simulator and Travel Demand Simulators for Assessing Congestion on Roads After a Major Earthquake. In APPLICATION OF HIGH-PERFORMANCE COMPUTING TO EARTHQUAKE-RELATED PROBLEMS (pp. 413-447). https://doi.org/10.1142/9781800614635_0007
            - Iryo, T., Watling, D., & Hazelton, M. (2024). Estimating Markov Chain Mixing Times: Convergence Rate Towards Equilibrium of a Stochastic Process Traffic Assignment Model. Transportation Science. https://doi.org/10.1287/trsc.2024.0523
        """
        if isinstance(s, CppSolverDUE):
            return  # CppSolverDUE.__init__ handles its own init
        s.func_World = func_World
        s.W_sol = None  #final solution
        s.W_intermid_solution = None    #latest solution in the iterative process. Can be used when a user terminates the solution algorithm
        s.dfs_link = []

        #warnings.warn("DTA solver is experimental and may not work as expected. It is functional but unstable.")
    
    def solve(s, max_iter, n_routes_per_od=10, swap_prob=0.05, route_sets=None, print_progress=True):
        """
        Solve quasi Dynamic User Equilibrium (DUE) problem using day-to-day dynamics.

        Parameters
        ----------
        max_iter : int
            maximum number of iterations
        n_routes_per_od : int
            number of routes to enumerate for each OD pair
        swap_prob : float
            probability of route swap
        route_sets : dict or None
            Predefined route sets for each vehicle. If provided, these routes will be used instead of enumerating new routes.
            If None, routes will be enumerated automatically. If provided, `n_routes_per_od` is ignored.
            The format:

            >>> {
            >>>     [origin_node1, destination_node1]: [
            >>>         [linkA1, linkA2, ...],
            >>>         [linkB1, linkB2, ...],
            >>>         ...
            >>>     ],
            >>>     [origin_node2, destination_node2]:
            >>>         ...
            >>>     ],
            >>>     ...
            >>> }

            See test scrpt for an example.

        print_progress : bool
            whether to print the information

        Returns
        -------
        W : World
            World object with quasi DUE solution (if properly converged)

        Notes
        -----
        `self.W_sol` is the final solution.
        `self.W_intermid_solution` is the latest solution in the iterative process. Can be used when a user terminates the solution algorithm.
        """
        s.start_time = time.time()

        W_orig = s.func_World()
        if print_progress:
            W_orig.print_scenario_stats()

        # enumerate routes for each OD pair
        n_routes_per_od = n_routes_per_od

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        # dict_od_to_routes = {}
        # for o,d in dict_od_to_vehid.keys():
        #     routes = enumerate_k_shortest_routes(W_orig, o, d, k=n_routes_per_od)
        #     dict_od_to_routes[o,d] = routes

        if W_orig.finalized == False:
            W_orig.finalize_scenario()
        dict_od_to_routes = enumerate_k_random_routes(W_orig, k=n_routes_per_od)

        if route_sets != None:
            dict_od_to_routes = {}
            for key, routes in route_sets.items():
                o = W_orig.get_node(key[0]).name
                d = W_orig.get_node(key[1]).name
                dict_od_to_routes[o,d] = []
                for route in routes:
                    dict_od_to_routes[o,d].append([W_orig.get_link(l).name for l in route])

        if print_progress:
            print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

        # day-to-day dynamics
        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.t_gaps = []
        s.route_log = []
        s.cost_log = []
        swap_prob = swap_prob
        max_iter = max_iter

        print("solving DUE...")
        for i in range(max_iter):
            W = s.func_World()
            if i != max_iter-1:
                W.vehicle_logging_timestep_interval = -1

            if i != 0:
                for key in W.VEHICLES:
                    if key in routes_specified:
                        W.VEHICLES[key].enforce_route(routes_specified[key])

            route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
            for o,d in dict_od_to_vehid.keys():
                for r in dict_od_to_routes[o,d]:
                    route_set[o,d].append(W.defRoute(r))

            # simulation
            W.exec_simulation()

            # results
            W.analyzer.print_simple_stats()
            #W.analyzer.network_average()

            # trip completion check
            unfinished_trips = W.analyzer.trip_all - W.analyzer.trip_completed
            if unfinished_trips > 0:
                warnings.warn(f"Warning: {unfinished_trips} / {W.analyzer.trip_all} vehicles have not finished their trips. The DUE solver assumes that all vehicles finish their trips during the simulation duration. Consider increasing the simulation time limit or checking the network configuration.", UserWarning)

            # attach route choice set to W object for later re-use at different solvers like DSO-GA
            W.dict_od_to_routes = dict_od_to_routes

            s.W_intermid_solution = W

            s.dfs_link.append(W.analyzer.link_to_pandas())

            # route swap
            routes_specified = {}
            route_actual = {}
            cost_actual = {}
            n_swap = 0
            total_t_gap = 0
            potential_n_swap = 0
            for key,veh in W.VEHICLES.items():
                flag_swap = random.random() < swap_prob
                o = veh.orig.name
                d = veh.dest.name
                r, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]

                route_actual[key] = [rr.name for rr in r]
                cost_actual[key] = travel_time

                if veh.state != "end":
                    continue

                #route_setsが所与で，route_setの経路を何らかの理由で走っていない場合（初期解で経路指定が整合的でない場合がメイン），強制的に設定する
                if route_sets != None and r not in route_set[o,d]:
                    routes_specified[key] = route_set[o,d][0]
                    continue

                flag_route_changed = False
                route_changed = None
                t_gap = 0

                cost_current = r.actual_travel_time(ts[0]) + sum([l.get_toll(ts[i]) for i,l in enumerate(r)])

                potential_n_swap_updated = potential_n_swap
                for alt_route in route_set[o,d]:
                    alt_route_tts = alt_route.actual_travel_time(ts[0], return_details=True)[1]
                    cost_alt = alt_route.actual_travel_time(ts[0]) + sum([l.get_toll(ts[0]+sum(alt_route_tts[:i])) for i,l in enumerate(alt_route)])
                    if cost_alt < cost_current:
                        if flag_route_changed == False or (cost_alt < cost_current):
                            t_gap = cost_current - cost_alt
                            potential_n_swap_updated = potential_n_swap + W.DELTAN
                            if flag_swap:
                                flag_route_changed = True
                                route_changed = alt_route
                                cost_current = cost_alt

                potential_n_swap = potential_n_swap_updated

                total_t_gap += t_gap
                routes_specified[key] = r
                if flag_route_changed:
                    n_swap += W.DELTAN
                    routes_specified[key] = route_changed

            t_gap_per_vehicle = total_t_gap/len(W.VEHICLES)
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
        last_iters = int(max_iter/4)
        print(f" total travel time: initial {s.ttts[0]:.1f} -> average of last {last_iters} iters {np.average(s.ttts[-last_iters:]):.1f}")
        print(f" number of potential route changes: initial {s.potential_swaps[0]:.1f} -> average of last {last_iters} iters {np.average(s.potential_swaps[-last_iters:]):.1f}")
        print(f" route travel cost gap: initial {s.t_gaps[0]:.1f} -> average of last {last_iters} iters {np.average(s.t_gaps[-last_iters:]):.1f}")
        print(f" computation time: {s.end_time - s.start_time:.1f} seconds")

        s.W_sol = W
        return s.W_sol

    def plot_convergence(s):
        """
        Plots convergence metrics for a Dynamic Traffic Assignment (DTA) solution.
        This function creates three separate plots:
        1. Total travel time across iterations
        2. Number of route changes (swaps) across iterations
        3. Travel time gap between chosen routes and minimum cost routes across iterations 
        """

        # iteration plot
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(s.ttts)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of route change")
        plt.plot(s.n_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("travel cost difference between chosen route and minimum cost route")
        plt.plot(s.t_gaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.show()

        # plt.figure()
        # plot_multiple_y(ys=[s.ttts, s.n_swaps, s.potential_swaps, s.total_t_gaps, np.array(s.total_t_gaps)/np.array(s.potential_swaps)], labels=["total travel time", "number of route change", "number of potential route change", "time gap for potential route change", "time gap per potential route change"])
        # plt.xlabel("iter")
        # plt.show()

    def plot_link_stats(s):
        """
        Generate two plots to visualize the evolution of link-level traffic statistics across iterations.
        The first plot shows traffic volume changes for each link over iterations.
        The second plot shows average travel time changes for each link over iterations.
        """

        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        """
        Plot travel time statistics for vehicles based on their origin and destination.
        This function visualizes the average travel time and standard deviation for vehicles
        matching the specified origin and destination criteria. The data is plotted against 
        the departure time of each vehicle.

        Parameters
        ----------
        orig : str, optional
            Filter vehicles by origin. If None, vehicles from all origins are included.
        dest : str, optional
            Filter vehicles by destination. If None, vehicles to all destinations are included.
            
        Notes
        -----
        - The function uses the second half of the available data (from length/2 to length)
          from the cost_log to compute statistics.
        - The plot shows departure time on the x-axis and average travel time on the y-axis,
          with error bars representing the standard deviation.
        """

        ave_TT = []
        std_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                length = len(s.route_log)
                ts = [s.cost_log[day][vehid] for day in range(int(length/2), length)]
                ave_TT.append(np.average(ts))
                std_TT.append(np.std(ts))
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.errorbar(x=depature_time, y=ave_TT, yerr=std_TT, 
                fmt='bx', ecolor="#aaaaff", capsize=0, label=r"travel time (mean $\pm$ std)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()


class SolverDSO_D2D:

    def __new__(cls, func_World, cpp=False):
        if cpp and cls is SolverDSO_D2D:
            return CppSolverDSO_D2D(func_World)
        return super().__new__(cls)

    def __init__(s, func_World, cpp=False):
        """
        Solve quasi Dynamic System Optimum (DSO) problem using day-to-day dynamics.

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
        cpp : bool
            if True, return a CppSolverDSO_D2D instance that uses C++ batch operations

        Notes
        -----
            This function computes a near DSO state as a steady state of day-to-day dynamical routing game.

            This is a modification of Iryo's DUE algorithm, in which each traveler minimize marginal travel time (i.e., private cost + externality) instead of private cost as in DUE.
            Since the externality (i.e., the total system cost without the ego vehicle) is costly to compute directly, it is estimated based on the queueing principle of the traffic flow model.
            Apart from these point, this algorithm is almost the same to `SolverDUE` and thus expected to have the desirable properties discussed in the papers cited above.

            This algorithm is a loose generalization of "DSO game" of the following paper.

            - Satsukawa, K., Wada, K., & Watling, D. (2022). Dynamic system optimal traffic assignment with atomic users: Convergence and stability. Transportation Research Part B: Methodological, 155, 188-209.

            In this paper, it is theoretically guaranteed that their DSO game algorithm converges to the global optimal DSO state. However, it may take a long time. This `SolverDSO_D2D` speeds up the solution process significantly, but it weakens the convergence guarantee.
        """
        if isinstance(s, CppSolverDSO_D2D):
            return
        s.func_World = func_World
        s.W_sol = None  #final solution
        s.W_intermid_solution = None    #latest solution in the iterative process. Can be used when a user terminates the solution algorithm
        s.dfs_link = []

    def solve(s, max_iter, n_routes_per_od=10, swap_prob=0.05, swap_num=None, route_sets=None, print_progress=True):
        """
        Solve quasi DSO problem using day-to-day dynamics.

        Parameters
        ----------
        max_iter : int
            maximum number of iterations
        n_routes_per_od : int
            number of routes to enumerate for each OD pair
        swap_prob : float
            probability of route swap
        swap_num : int or None
            number of vehicles to swap route each iteration. If specified, `swap_prob` is ignored.
        route_sets : dict or None
            Predefined route sets for each vehicle. If provided, these routes will be used instead of enumerating new routes.
            If None, routes will be enumerated automatically. If provided, `n_routes_per_od` is ignored.
            The format:

            >>> {
            >>>     [origin_node1, destination_node1]: [
            >>>         [linkA1, linkA2, ...],
            >>>         [linkB1, linkB2, ...],
            >>>         ...
            >>>     ],
            >>>     [origin_node2, destination_node2]:
            >>>         ...
            >>>     ],
            >>>     ...
            >>> }

            See test scrpt for an example.

        print_progress : bool
            whether to print the information

        Returns
        -------
        W : World
            World object with quasi DUE solution (if properly converged)

        Notes
        -----
        `self.W_sol` is the final solution.
        `self.W_intermid_solution` is the latest solution in the iterative process. Can be used when a user terminates the solution algorithm.
        """
        s.start_time = time.time()

        W_orig = s.func_World()
        if print_progress:
            W_orig.print_scenario_stats()

        # enumerate routes for each OD pair
        n_routes_per_od = n_routes_per_od

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        # dict_od_to_routes = {}
        # for o,d in dict_od_to_vehid.keys():
        #     routes = enumerate_k_shortest_routes(W_orig, o, d, k=n_routes_per_od)
        #     dict_od_to_routes[o,d] = routes

        if W_orig.finalized == False:
            W_orig.finalize_scenario()
        dict_od_to_routes = enumerate_k_random_routes(W_orig, k=n_routes_per_od)

        if route_sets != None:
            dict_od_to_routes = {}
            for key, routes in route_sets.items():
                o = W_orig.get_node(key[0]).name
                d = W_orig.get_node(key[1]).name
                dict_od_to_routes[o,d] = []
                for route in routes:
                    dict_od_to_routes[o,d].append([W_orig.get_link(l).name for l in route])

        if print_progress:
            print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

        # day-to-day dynamics
        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.t_gaps = []
        s.route_log = []
        s.cost_log = []
        swap_prob = swap_prob
        max_iter = max_iter

        print("solving DSO...")
        for i in range(max_iter):
            W = s.func_World()
            if i != max_iter-1:
                W.vehicle_logging_timestep_interval = -1

            if i != 0:
                for key in W.VEHICLES:
                    if key in routes_specified:
                        W.VEHICLES[key].enforce_route(routes_specified[key])

            route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
            for o,d in dict_od_to_vehid.keys():
                for r in dict_od_to_routes[o,d]:
                    route_set[o,d].append(W.defRoute(r))

            # simulation
            W.exec_simulation()

            # results
            W.analyzer.print_simple_stats()
            #W.analyzer.network_average()

            # trip completion check
            unfinished_trips = W.analyzer.trip_all - W.analyzer.trip_completed
            if unfinished_trips > 0:
                warnings.warn(f"Warning: {unfinished_trips} / {W.analyzer.trip_all} vehicles have not finished their trips. The DSO solver assumes that all vehicles finish their trips during the simulation duration. Consider increasing the simulation time limit or checking the network configuration.", UserWarning)

            # attach route choice set to W object for later re-use at different solvers like DSO-GA
            W.dict_od_to_routes = dict_od_to_routes

            if unfinished_trips == 0:
                if s.W_intermid_solution == None:
                    s.W_intermid_solution = W
                elif W.analyzer.average_travel_time < s.W_intermid_solution.analyzer.average_travel_time:
                    s.W_intermid_solution = W

            s.dfs_link.append(W.analyzer.link_to_pandas())

            # route swap
            routes_specified = {}
            route_actual = {}
            cost_actual = {}
            n_swap = 0
            total_t_gap = 0
            potential_n_swap = 0

            keys_swap = [key for key in W.VEHICLES.keys() if random.random() < swap_prob]
            if swap_num != None:
                keys_swap = random.sample(list(W.VEHICLES.keys()), swap_num)

            for key,veh in W.VEHICLES.items():
                flag_swap = key in keys_swap
                o = veh.orig.name
                d = veh.dest.name
                r, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]

                route_actual[key] = [rr.name for rr in r]
                cost_actual[key] = travel_time

                if veh.state != "end":
                    continue

                #route_setsが所与で，route_setの経路を何らかの理由で走っていない場合（初期解で経路指定が整合的でない場合がメイン），強制的に設定する
                if route_sets != None and r not in route_set[o,d]:
                    routes_specified[key] = route_set[o,d][0]
                    continue

                flag_route_changed = False
                route_changed = None
                t_gap = 0

                ext = estimate_congestion_externality_route(W, r, ts[0])
                private_cost = r.actual_travel_time(ts[0])
                cost_current = private_cost+ext

                potential_n_swap_updated = potential_n_swap

                for alt_route in route_set[o,d]:
                    ext = estimate_congestion_externality_route(W, alt_route, ts[0])
                    private_cost = alt_route.actual_travel_time(ts[0])
                    cost_alt = private_cost+ext

                    if cost_alt < cost_current:
                        if flag_route_changed == False or (cost_alt < cost_current):
                            t_gap = cost_current - cost_alt
                            potential_n_swap_updated = potential_n_swap + W.DELTAN
                            if flag_swap:
                                flag_route_changed = True
                                route_changed = alt_route
                                cost_current = cost_alt

                potential_n_swap = potential_n_swap_updated

                total_t_gap += t_gap
                routes_specified[key] = r
                if flag_route_changed:
                    n_swap += W.DELTAN
                    routes_specified[key] = route_changed

            t_gap_per_vehicle = total_t_gap/len(W.VEHICLES)
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
        last_iters = int(max_iter/4)
        print(f" total travel time: initial {s.ttts[0]:.1f} -> minimum {np.min(s.ttts):.1f}")
        print(f" number of potential route changes: initial {s.potential_swaps[0]:.1f} -> minimum {np.min(s.potential_swaps):.1f}")
        print(f" route marginal travel time gap: initial {s.t_gaps[0]:.1f} -> minimum {np.min(s.t_gaps):.1f}")
        print(f" computation time: {s.end_time - s.start_time:.1f} seconds")

        s.W_sol = s.W_intermid_solution
        return s.W_sol
    
    def plot_convergence(s):
        """
        Plots convergence metrics for a Dynamic Traffic Assignment (DTA) solution.
        This function creates three separate plots:
        1. Total travel time across iterations
        2. Number of route changes (swaps) across iterations
        3. Travel time gap between chosen routes and minimum cost routes across iterations 
        """

        # iteration plot
        plt.figure(figsize=(6,6))
        plt.subplot(311)
        plt.ylabel("total travel time")
        plt.plot(s.ttts)
        plt.xlabel("iter")

        plt.subplot(312)
        plt.ylabel("number of route change")
        plt.plot(s.n_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.subplot(313)
        plt.ylabel("route marginal\n travel time gap")
        plt.plot(s.t_gaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.show()

        # plt.figure()
        # plot_multiple_y(ys=[s.ttts, s.n_swaps, s.potential_swaps, s.total_t_gaps, np.array(s.total_t_gaps)/np.array(s.potential_swaps)], labels=["total travel time", "number of route change", "number of potential route change", "time gap for potential route change", "time gap per potential route change"])
        # plt.xlabel("iter")
        # plt.show()

    def plot_link_stats(s):
        """
        Generate two plots to visualize the evolution of link-level traffic statistics across iterations.
        The first plot shows traffic volume changes for each link over iterations.
        The second plot shows average travel time changes for each link over iterations.
        """

        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        """
        Plot travel time statistics for vehicles based on their origin and destination.
        This function visualizes the average travel time and standard deviation for vehicles
        matching the specified origin and destination criteria. The data is plotted against 
        the departure time of each vehicle.

        Parameters
        ----------
        orig : str, optional
            Filter vehicles by origin. If None, vehicles from all origins are included.
        dest : str, optional
            Filter vehicles by destination. If None, vehicles to all destinations are included.
            
        Notes
        -----
        - The function uses the second half of the available data (from length/2 to length)
          from the cost_log to compute statistics.
        - The plot shows departure time on the x-axis and average travel time on the y-axis,
          with error bars representing the standard deviation.
        """

        sol_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)
                r, ts = s.W_sol.VEHICLES[vehid].traveled_route()
                sol_TT.append(ts[-1]-ts[0])

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.plot(depature_time, sol_TT, "bx", label=r"travel time (in DSO)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()


class SolverDSO_GA:
    def __init__(s, func_World):
        """
        Solve Dynamic System Optimum (DSO) problem using genetic algorithm.

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
        
        Notes
        -----
            This is based on the standard genetic algorithm. For some scenarios, this method works well.
        """
        s.func_World = func_World
        s.W_sol = None  #final solution
        s.W_intermid_solution = None    #latest solution in the iterative process. Can be used when a user terminates the solution 
        s.dfs_link = []
                
        warnings.warn(
            "`SolverDSO_GA` is deprecated. Please consider to use `SolverDSO_D2D`, which is much more efficient.",
            DeprecationWarning
        )

    
    def solve(s, max_iter, n_routes_per_od=10, pop_size=50, elite_size=2, mutation_occur_rate=0.1, mutation_gene_rate=0.05, n_crossover_points=2, print_progress=True, initial_solution_World=None):
        """
        Solve Dynamic System Optimum (DSO) problem using genetic algorithm. The objective function is 
            total_travel_time + simulation_duration*number_of_vehicles_that_could_not_complete_their_trip.
        The second term should be zero for reasonable results.

        Parameters
        ----------
        n_routes_per_od : int
            number of routes to enumerate for each OD pair.
            If `initial_solution_World` is given, this arg is ignored and the value in `initial_solution_World` is used.
        pop_size : int
            population size
        max_iter : int
            number of generations
        mutation_occur_rate : float
            mutation rate for individual
        mutation_gene_rate : float
            mutation rate for gene
        n_crossover_points : int
            number of crossover points
        print_progress_detailed : bool
            whether to print the detailed information
        initial_solution_World : World
            initial solution (starting point) for the optimization algorithm. It must have the same structure as the output of func_World, and its simulation has been already executed. Recommended example: `W_init = solve_DUE(func_World); W = solve_DSO(func_World, initial_solution_World=W_init)`
            For unknown reason, this World is not always reproduced extactly. Sometimes small number of vehicles cannot find the routes chosen in DUE solution. TODO: fix

        Returns
        -------
        W : World
            World object with near DSO solution (if properly converged)
        
            
        Notes
        -----
        `self.W_sol` is the final solution. 
        `self.W_intermid_solution` is the latest solution in the iterative process. Can be used when a user terminates the solution algorithm.
        
        GA part is initially written by ChatGPT o1-preview, and reviewed and overwritten by human.
        """
        s.start_time = time.time()
        
        # def initialize_population(pop_size, length, genome_maxs, W_init):
        #     """Initialize a population with random genomes."""
        #     population = []
        #     #default pop is random
        #     for _ in range(pop_size):
        #         genome = [random.randint(0, genome_maxs[i]-1) for i in range(length)]
        #         population.append(genome)
            
        #     #one pop is the initial solution

        #     print(W_init.analyzer.total_travel_time)

        #     for i,veh in enumerate(W_init.VEHICLES.values()):   
        #         route_id = 0
        #         for j,r in enumerate(routes[(veh.orig.name, veh.dest.name)]):
        #             if W_init.defRoute(r) == veh.traveled_route()[0]:
        #                 route_id = j
        #         population[0][i] = route_id
        #     #1/4 of the pop is mutation of the initial solution
        #     for k in range(1, int(pop_size/4)):
        #         population[k] = mutate(population[0], mutation_gene_rate, genome_maxs)
        #     #1/4 of the pop is heavier mutation of the initial solution
        #     for k in range(int(pop_size/4), int(pop_size/2)):
        #         population[k] = mutate(population[0], 0.3, genome_maxs)

        #     return population

        def tournament_selection(population, fitnesses, k=3):
            """Select individuals using tournament selection."""
            selected = []
            for _ in range(len(population)):
                aspirants = random.sample(list(zip(population, fitnesses)), k)
                selected.append(max(aspirants, key=lambda x: x[1])[0])
            return selected

        def crossover(parent1, parent2, n_points=1):
            """Perform n-point crossover between two parents."""
            if n_points >= len(parent1):
                raise ValueError("Number of crossover points must be less than genome length.")
            
            points = sorted(random.sample(range(1, len(parent1)), n_points))
            child1, child2 = [], []
            last_point = 0
            for i, point in enumerate(points + [len(parent1)]):
                if i % 2 == 0:
                    child1.extend(parent1[last_point:point])
                    child2.extend(parent2[last_point:point])
                else:
                    child1.extend(parent2[last_point:point])
                    child2.extend(parent1[last_point:point])
                last_point = point
            return child1, child2

        def mutate(genome, mutation_rate, genome_maxs):
            """Mutate a genome with a given mutation rate."""
            mutated_genome = []
            for i,gene in enumerate(genome):
                if random.random() < mutation_rate:
                    mutated_gene = random.randint(0, genome_maxs[i]-1)
                    mutated_genome.append(mutated_gene)
                else:
                    mutated_genome.append(gene)
            return mutated_genome

            
        ##############################################################
        # initial solution
        if initial_solution_World == None:
            W = s.func_World()
            W.exec_simulation()
            if print_progress:
                print(W.analyzer.basic_to_pandas())
        else:
            W = initial_solution_World
            
        if print_progress:
            W.print_scenario_stats()

        ##############################################################
        # enumerate some routes between each OD pair

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        # routes = {}
        # n_routes_per_od = n_routes_per_od
        # for od_pair in dict_od_to_vehid.keys():
        #     routes[od_pair] = enumerate_k_shortest_routes(W, od_pair[0], od_pair[1], n_routes_per_od)

        if initial_solution_World != None and hasattr(initial_solution_World, "dict_od_to_routes"):
            routes = initial_solution_World.dict_od_to_routes
            print("init sol used")
        else:
            routes = enumerate_k_random_routes(W, k=n_routes_per_od)

        # print("available routes for each OD pair")
        # for key in routes:
        #    for route in routes[key]:
        #        print(key, route)

        ##############################################################
        # Prepare genetic algorithm

        # specify routing based on genome
        def specify_routes(W, genome):
            veh_list = list(W.VEHICLES.values())
            for i, value in enumerate(genome):
                veh = veh_list[i]
                if value < len(routes[(veh.orig.name, veh.dest.name)]):
                    veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][value])
                else:
                    veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][-1])

        pop_size = pop_size     # Population size
        max_iter = max_iter     # Number of generations
        elite_size = elite_size  # Number of elites that survive
        length = len(W.VEHICLES.values())        # Genome length
        mutation_occur_rate = mutation_occur_rate  # Mutation rate for individual
        mutation_gene_rate = mutation_gene_rate  # Mutation rate for gene
        n_points = n_crossover_points    # Number of crossovers

        genome_maxs = []
        for i, veh in enumerate(W.VEHICLES.values()):
            actual_n_route = len(routes[(veh.orig.name, veh.dest.name)])
            genome_maxs.append(actual_n_route)
        
        s.ttts = []
        s.route_log = []
        s.cost_log = []

        print("solving DSO by GA...")

        # Initialize a population with random genomes
        population = []

        #default pop is random
        for _ in range(pop_size):
            genome = [random.randint(0, genome_maxs[i]-1) for i in range(length)]
            population.append(genome)
        
        #one pop is the initial solution
        for i,veh in enumerate(W.VEHICLES.values()):   
            route_id = 0
            for j,r in enumerate(routes[(veh.orig.name, veh.dest.name)]):
                if W.defRoute(r) == veh.traveled_route()[0]:
                    route_id = j
                    break
            population[0][i] = route_id
        #1/4 of the pop is mutation of the initial solution
        for k in range(1, int(pop_size/4)):
            population[k] = mutate(population[0], 0.05, genome_maxs)
        #1/4 of the pop is heavier mutation of the initial solution
        for k in range(int(pop_size/4), int(pop_size/2)):
            population[k] = mutate(population[0], 0.1, genome_maxs)

        for gen in range(max_iter):
            if print_progress:
                print(f"Generation {gen}")
                print(" total travel times: ", end="")
            fitnesses = []
            fitness_best = None
            for genome in population:
                W = s.func_World()
                if gen != max_iter-1:
                    W.vehicle_logging_timestep_interval = -1
                specify_routes(W, genome)
                W.exec_simulation()

                if print_progress:
                    print(W.analyzer.total_travel_time, end=" ")
                fitnesses.append(- W.analyzer.total_travel_time - (W.analyzer.trip_all-W.analyzer.trip_completed)*W.TMAX) #objective function
                if fitness_best == None or fitnesses[-1] > fitness_best:
                    fitness_best = fitnesses[-1]
                    W_best = W
                    
            W_best.dict_od_to_routes = routes
            s.W_intermid_solution = W_best
            
            route_actual = {}
            cost_actual = {}
            for key,veh in W_best.VEHICLES.items():
                if veh.state != "end":
                    continue

                route, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]
                
                route_actual[key] = [rr.name for rr in route]
                cost_actual[key] = travel_time

            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)
            s.ttts.append(int(W_best.analyzer.total_travel_time))

            # Print the best fitness in the current generation
            if print_progress:
                print(f"\n Best fitness = {max(fitnesses)}, TTT = {W_best.analyzer.total_travel_time}, completed trips: {W_best.analyzer.trip_completed}")
            
            s.dfs_link.append(W_best.analyzer.link_to_pandas())

            # Selection
            selected = tournament_selection(population, fitnesses)

            # Generate next generation
            next_generation = []
            # Elite
            for i in sorted(range(len(fitnesses)), key=lambda i: fitnesses[i], reverse=True)[:elite_size]:
                next_generation.append(population[i])
            # Children
            for i in range(int(pop_size/2)):
                parent1 = random.choice(selected)
                parent2 = random.choice(selected)
                # Crossover
                child1, child2 = crossover(parent1, parent2, n_points=n_points)
                # Mutation
                # if an identical pop exists, automatically mutate
                if child1 in next_generation:
                    child1 = mutate(child1, mutation_gene_rate, genome_maxs)
                if child2 in next_generation:
                    child2 = mutate(child2, mutation_gene_rate, genome_maxs)
                # ordinary mutation
                if random.random() < mutation_occur_rate:
                    child1 = mutate(child1, mutation_gene_rate, genome_maxs)
                    child2 = mutate(child2, mutation_gene_rate, genome_maxs)
                next_generation.extend([child1, child2])

            population = next_generation[:pop_size]
        
        s.end_time = time.time()
        print("DSO summary:")
        print(f" total travel time: initial {s.ttts[0]:.1f} -> last {s.ttts[-1]:.1f}")
        print(f" computation time: {s.end_time - s.start_time:.1f} seconds")

        s.W_sol = W
        return s.W_sol
    
    def plot_convergence(s):
        """
        Plot the convergence of total travel time across iterations.
        This method generates a plot showing how the total travel time changes
        throughout the solution iterations, which helps visualize the convergence
        of the traffic assignment algorithm.
        """

        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(s.ttts)
        plt.xlabel("iter")
        plt.show()

    def plot_link_stats(s):
        """
        Plot the traffic volume and average travel time for each link across iterations.
        This function creates two separate figures: one for traffic volume and one for average travel time.
        For each link, it plots the values across all iterations and adds a legend with link identifiers.
        """

        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        """
        Plot vehicle travel time statistics based on route logs.
        This function generates a plot that shows average travel times with standard deviations
        for vehicles, filtered by optional origin and destination locations.

        Parameters
        ----------
        orig : str, optional
            Origin location name to filter vehicles. If None, all origins are included.
        dest : str, optional
            Destination location name to filter vehicles. If None, all destinations are included.
        """

        ave_TT = []
        std_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                length = len(s.route_log)
                ts = [s.cost_log[day][vehid] for day in range(int(length/2), length)]
                ave_TT.append(np.average(ts))
                std_TT.append(np.std(ts))
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.errorbar(x=depature_time, y=ave_TT, yerr=std_TT, 
                fmt='bx', ecolor="#aaaaff", capsize=0, label=r"travel time (mean $\pm$ std)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()


class SolverDSO_ALNS:
    def __init__(s, func_World):
        
        """
        Solve Dynamic System Optimum (DSO) problem using a customized Adaptive Large Neighborhood Search (ADLS).

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
        """
        s.func_World = func_World
        s.W_sol = None  #final solution
        s.W_intermid_solution = None
        s.dfs_link = []
        
        warnings.warn(
            "`SolverDSO_GA` is deprecated. Please consider to use `SolverDSO_D2D`, which is much more efficient.",
            DeprecationWarning
        )
        
    def solve(s, max_iter, n_routes_per_od=10, k_max=None, k_min=None, p0=0.2, trials=50, always_accept=False, print_progress=True, initial_solution_World=None, destroy_set=None, repair_set=None):
        """
        Solve Dynamic System Optimum (DSO) problem using a customized Adaptive Large Neighborhood Search (ADLS).
        The objective function is 
            total_travel_time + simulation_duration*number_of_vehicles_that_could_not_complete_their_trip.
        The second term should be zero for reasonable results.

        Parameters
        ----------
        max_iter : int
            maximum number of iterations
        n_routes_per_od : int
            number of routes to enumerate for each OD pair
            If `initial_solution_World` is given, this arg is ignored and the value in `initial_solution_World` is used.
        k_max : int
            maximum number of vehicles to change the route in each iteration. If None, it is set to 1% of the total number of vehicles.
        k_min : int
            minimum number of vehicles to change the route in each iteration. If None, it is set to 1.
        p0 : float
            initial temperature parameter for ALNS. Default is 0.2.
        trials : int
            number of trials to estimate the temperature parameter. Default is 50.
        always_accept : bool
            whether to always accept a new solution regardless of its value. Default is False.
        print_progress : bool
            whether to print the information
        initial_solution_World : World
            Initial solution (starting point) for the optimization algorithm. If None, it uses the DUO solution as a starting point; this tends to be very slow. Usually, DUE soultion is a good starting point. It must have the same structure as the output of func_World, and its simulation has been already executed.
            Recommended example: `W_init = solve_DUE(func_World); W = solve_DSO(func_World, initial_solution_World=W_init)`.
        destroy_set : list of functions
            list of destroy functions to be used in ALNS. If None, all functions are used.
        repair_set : list of functions
            list of repair functions to be used in ALNS. If None, all functions are used.
        """
        
        s.print_progress = print_progress
        s.n_routes_per_od = n_routes_per_od
        s.max_iter = max_iter

        departure_times = []

        s.start_time = time.time()

        ##############################################################
        # initial solution

        if initial_solution_World == None:
            W = s.func_World()
            W.exec_simulation()
            #if print_progress:
            #    print(W.analyzer.basic_to_pandas())
        else:
            W = initial_solution_World
            dict_od_to_routes = W.dict_od_to_routes
            
        if k_max == None:
            k_max = max([1, int(len(W.VEHICLES)*0.01)])

        if print_progress:
            W.print_scenario_stats()

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)
            departure_times.append(veh.departure_time)

        departure_times = np.array(departure_times)/max(departure_times)

        ##############################################################
        # enumerate some routes between each OD pair

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        if initial_solution_World != None and hasattr(initial_solution_World, "dict_od_to_routes"):
            print("Initial solusion is used")
            routes = initial_solution_World.dict_od_to_routes
            xx0 = [0 for i in range(len(W.VEHICLES))]
            n_route_found = 0
            for i,veh in enumerate(W.VEHICLES.values()):   
                route_id = -1
                route_id_mintt = 0
                route_mintt = float("inf")
                for j,r in enumerate(routes[(veh.orig.name, veh.dest.name)]):
                    route = W.defRoute(r)
                    if route == veh.traveled_route()[0]:
                        route_id = j
                        n_route_found += 1
                        break

                    #経路が経路集合に見当たらない場合，最短経路を探す
                    if route.actual_travel_time(veh.departure_time_in_second) < route_mintt: 
                        route_id_mintt = j
                
                if route_id != -1:
                    xx0[i] = route_id
                else:
                    xx0[i] = route_id_mintt #経路が経路集合に見当たらない場合，最短経路に割付

            #print(f" route recovered: {n_route_found*W.DELTAN}/{len(W.VEHICLES)*W.DELTAN}")

            xx_domain = []
            for i, veh in enumerate(W.VEHICLES.values()):
                actual_n_route = len(routes[(veh.orig.name, veh.dest.name)])
                xx_domain.append([i for i in range(actual_n_route)])

        else:
            print("No initial solution")
            routes = enumerate_k_random_routes(W, k=n_routes_per_od)

            W.dict_od_to_routes = routes
            dict_od_to_routes = routes
            
            xx_domain = []
            for i, veh in enumerate(W.VEHICLES.values()):
                actual_n_route = len(routes[(veh.orig.name, veh.dest.name)])
                xx_domain.append([i for i in range(actual_n_route)])

            xx0 = [random.randint(0, len(xx_domain[i])) for i in range(len(W.VEHICLES))]


        ##############################################################
        # Prepare ALNS

        # specify routing based on x vector
        def specify_routes(W, xx):
            veh_list = list(W.VEHICLES.values())
            for i, value in enumerate(xx):
                veh = veh_list[i]
                if value < len(routes[(veh.orig.name, veh.dest.name)]):
                    veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][value])
                else:
                    veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][-1])
                    
        s.W_intermid_solution = W
        s.best_obj_value = None
        s.iter = 0
        s.ttts = []
        s.ttts_best = []
        s.route_log = []
        s.cost_log = []
        s.dfs_link = []

        s.best_iter = []
        s.best_obj = []

        print("solving DSO by ALNS...")

        def objective_function_by_UXsim(xx):
            W = s.func_World()
            specify_routes(W, xx)
            W.exec_simulation()
            obj_value = W.analyzer.total_travel_time - (W.analyzer.trip_all-W.analyzer.trip_completed)*W.TMAX
            
            route_actual = {}
            cost_actual = {}
            for key,veh in W.VEHICLES.items():
                if veh.state != "end":
                    continue

                route, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]
                
                route_actual[key] = [rr.name for rr in route]
                cost_actual[key] = travel_time
            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)
            s.ttts.append(int(W.analyzer.total_travel_time))
            s.dfs_link.append(W.analyzer.link_to_pandas())


            if s.best_obj_value == None or obj_value < s.best_obj_value:
                s.best_obj_value = obj_value
                s.W_intermid_solution = W
                s.W_intermid_solution.dict_od_to_routes = dict_od_to_routes

                s.ttts_best.append(int(W.analyzer.total_travel_time))

                s.best_iter.append(s.iter)
                s.best_obj.append(s.best_obj_value)

            if s.print_progress and s.iter%20 == 0:
                print(f"iter: {s.iter} - TTT: {W.analyzer.total_travel_time}, trips:{W.analyzer.trip_completed}/{W.analyzer.trip_all}, Obj:{obj_value}, Best Obj:{s.best_obj_value}")

            s.iter += 1
            return obj_value

        state = ALNS.init_alns(
            xx0, objective_function_by_UXsim,
            domains=xx_domain,
            k_max=k_max,
            k_min=k_min,
            destroy_set=destroy_set,
            repair_set=repair_set,
            total_iters=max_iter,
            p0=p0,
            trials=trials,
            always_accept=always_accept,
            seed=0,
            additional_info={"departure_times":departure_times, "W":W}
        )

        ALNS.run_auto(state, n=state.total_iters)

        s.xx_star, s.run = ALNS.finalize(state)

        s.W_sol = s.W_intermid_solution

        s.end_time = time.time()
        print("DSO summary:")
        print(f" total travel time: initial {s.ttts[0]:.1f} -> last {s.ttts_best[-1]:.1f}")
        print(f" computation time: {s.end_time - s.start_time:.1f} seconds")

        return s.W_sol
    
    def print_solution_process(s, full=False):
        df = s.run.to_dataframe()
        df = df.drop(columns=["changed_idx"])
        if full:
            print(df.to_string())
        else:
            print(df[df["accepted"]==True].to_string())
        pprint(s.run.operator_stats)

    def plot_convergence(s):
        plt.figure()
        plt.plot(s.ttts, "b-", lw=0.5, label="search trace")
        plt.plot(s.best_iter, s.best_obj, "r.", label="updated solution")
        obj_init = s.best_obj[0]
        obj_min = s.best_obj[-1]
        plt.ylim([obj_min-(obj_init-obj_min)*0.1, obj_init+(obj_init-obj_min)*0.5])
        plt.xlabel("iteration")
        plt.ylabel("total travel time (s)")
        plt.legend()
        plt.grid()
        plt.show()


    def plot_link_stats(s):
        """
        Plot the traffic volume and average travel time for each link across iterations.
        This function creates two separate figures: one for traffic volume and one for average travel time.
        For each link, it plots the values across all iterations and adds a legend with link identifiers.
        """

        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        """
        Plot vehicle travel time statistics based on route logs.
        This function generates a plot that shows average travel times with standard deviations
        for vehicles, filtered by optional origin and destination locations.

        Parameters
        ----------
        orig : str, optional
            Origin location name to filter vehicles. If None, all origins are included.
        dest : str, optional
            Destination location name to filter vehicles. If None, all destinations are included.
        """

        ave_TT = []
        std_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                length = len(s.route_log)
                ts = [s.cost_log[day][vehid] for day in range(int(length/2), length)]
                ave_TT.append(np.average(ts))
                std_TT.append(np.std(ts))
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.errorbar(x=depature_time, y=ave_TT, yerr=std_TT, 
                fmt='bx', ecolor="#aaaaff", capsize=0, label=r"travel time (mean $\pm$ std)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()


# =============================================================================
# C++ accelerated solvers (dispatched via __new__ in SolverDUE / SolverDSO_D2D)
# =============================================================================

class CppSolverDUE(SolverDUE):
    """C++ accelerated DUE solver. Created via ``SolverDUE(func_World, cpp=True)``."""

    def __init__(s, func_World, **kwargs):
        s.func_World = func_World
        s.W_sol = None
        s.W_intermid_solution = None
        s.dfs_link = []

    def solve(s, max_iter, n_routes_per_od=10, swap_prob=0.05, route_sets=None, print_progress=True):
        s.start_time = time.time()

        func_World = _make_cpp_func_world(s.func_World)

        W_orig = func_World()
        if print_progress:
            W_orig.print_scenario_stats()

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            dict_od_to_vehid[veh.orig.name, veh.dest.name].append(key)

        if W_orig.finalized == False:
            W_orig.finalize_scenario()
        dict_od_to_routes = enumerate_k_random_routes(W_orig, k=n_routes_per_od)

        if route_sets is not None:
            dict_od_to_routes = {}
            for key, routes in route_sets.items():
                o = W_orig.get_node(key[0]).name
                d = W_orig.get_node(key[1]).name
                dict_od_to_routes[o, d] = [[W_orig.get_link(l).name for l in route] for route in routes]

        if print_progress:
            print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.t_gaps = []
        s.route_log = []
        s.cost_log = []

        od_route_sets_ids = None
        link_name_to_id = None
        link_id_to_name = None
        cached_analyzer = None

        print("solving DUE...")
        for i in range(max_iter):
            W = func_World()
            is_last_iter = (i == max_iter - 1)
            if not is_last_iter:
                W.vehicle_logging_timestep_interval = -1

            if od_route_sets_ids is None:
                link_name_to_id, link_id_to_name, node_name_to_id = _build_name_id_maps(W)
                od_route_sets_ids = _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id, link_name_to_id)

            cached_analyzer = _lightweight_finalize(W, cached_analyzer)

            if not is_last_iter:
                W._skip_log_on_terminate = True

            if i != 0:
                enforce_input = _build_enforce_routes_input(W, routes_specified_data, link_name_to_id)
                W._cpp_world.batch_enforce_routes(enforce_input)

            W.exec_simulation()
            W.analyzer.print_simple_stats()

            unfinished_trips = W.analyzer.trip_all - W.analyzer.trip_completed
            if unfinished_trips > 0:
                warnings.warn(f"Warning: {unfinished_trips} / {W.analyzer.trip_all} vehicles have not finished their trips. The DUE solver assumes that all vehicles finish their trips during the simulation duration. Consider increasing the simulation time limit or checking the network configuration.", UserWarning)

            W.dict_od_to_routes = dict_od_to_routes
            s.W_intermid_solution = W
            if is_last_iter:
                s.dfs_link.append(W.analyzer.link_to_pandas())

            rng_seed = random.randint(0, 2**31 - 1)
            cpp_result = W._cpp_world.route_swap_due(
                od_route_sets_ids, swap_prob, route_sets is not None, rng_seed
            )
            result = _convert_cpp_result_to_names(cpp_result, W, link_id_to_name)
            routes_specified_data = result['routes_specified']
            route_actual = result['route_actual']
            cost_actual = result['cost_actual']
            n_swap = result['n_swap']
            potential_n_swap = result['potential_n_swap']
            total_t_gap = result['total_t_gap']

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

    def __init__(s, func_World, **kwargs):
        s.func_World = func_World
        s.W_sol = None
        s.W_intermid_solution = None
        s.dfs_link = []

    def solve(s, max_iter, n_routes_per_od=10, swap_prob=0.05, swap_num=None, route_sets=None, print_progress=True):
        s.start_time = time.time()

        func_World = _make_cpp_func_world(s.func_World)

        W_orig = func_World()
        if print_progress:
            W_orig.print_scenario_stats()

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            dict_od_to_vehid[veh.orig.name, veh.dest.name].append(key)

        if W_orig.finalized == False:
            W_orig.finalize_scenario()
        dict_od_to_routes = enumerate_k_random_routes(W_orig, k=n_routes_per_od)

        if route_sets is not None:
            dict_od_to_routes = {}
            for key, routes in route_sets.items():
                o = W_orig.get_node(key[0]).name
                d = W_orig.get_node(key[1]).name
                dict_od_to_routes[o, d] = [[W_orig.get_link(l).name for l in route] for route in routes]

        if print_progress:
            print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.t_gaps = []
        s.route_log = []
        s.cost_log = []

        od_route_sets_ids = None
        link_name_to_id = None
        link_id_to_name = None
        cached_analyzer = None

        print("solving DSO...")
        for i in range(max_iter):
            W = func_World()
            is_last_iter = (i == max_iter - 1)
            if not is_last_iter:
                W.vehicle_logging_timestep_interval = -1

            if od_route_sets_ids is None:
                link_name_to_id, link_id_to_name, node_name_to_id = _build_name_id_maps(W)
                od_route_sets_ids = _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id, link_name_to_id)

            cached_analyzer = _lightweight_finalize(W, cached_analyzer)

            if not is_last_iter:
                W._skip_log_on_terminate = True

            if i != 0:
                enforce_input = _build_enforce_routes_input(W, routes_specified_data, link_name_to_id)
                W._cpp_world.batch_enforce_routes(enforce_input)

            W.exec_simulation()
            W.analyzer.print_simple_stats()

            unfinished_trips = W.analyzer.trip_all - W.analyzer.trip_completed
            if unfinished_trips > 0:
                warnings.warn(f"Warning: {unfinished_trips} / {W.analyzer.trip_all} vehicles have not finished their trips. The DSO solver assumes that all vehicles finish their trips during the simulation duration. Consider increasing the simulation time limit or checking the network configuration.", UserWarning)

            W.dict_od_to_routes = dict_od_to_routes

            if unfinished_trips == 0:
                if s.W_intermid_solution is None:
                    s.W_intermid_solution = W
                elif W.analyzer.average_travel_time < s.W_intermid_solution.analyzer.average_travel_time:
                    s.W_intermid_solution = W

            if is_last_iter:
                s.dfs_link.append(W.analyzer.link_to_pandas())

            rng_seed = random.randint(0, 2**31 - 1)
            cpp_swap_num = swap_num if swap_num is not None else -1
            cpp_result = W._cpp_world.route_swap_dso(
                od_route_sets_ids, swap_prob, cpp_swap_num, route_sets is not None, rng_seed
            )
            result = _convert_cpp_result_to_names(cpp_result, W, link_id_to_name)
            routes_specified_data = result['routes_specified']
            route_actual = result['route_actual']
            cost_actual = result['cost_actual']
            n_swap = result['n_swap']
            potential_n_swap = result['potential_n_swap']
            total_t_gap = result['total_t_gap']

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
