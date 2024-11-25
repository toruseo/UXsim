"""
Submodule for Dynamic Traffic Assginment solvers
"""
import random
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from ..Utilities import enumerate_k_shortest_routes

def solve_DUE(func_World, max_iter=50, n_routes_per_od=4, swap_prob=0.05, print_info=True, plot_res=False):
    """
    Solve quasi Dynamic User Equilibrium (DUE) problem using day-to-day dynamics

    Parameters
    ----------
    func_World : function
        function that returns a World object with nodes, links, and demand specifications
    max_iter : int
        maximum number of iterations
    n_routes_per_od : int
        number of routes to enumerate for each OD pair
    swap_prob : float
        probability of route swap
    print_info : bool
        whether to print the information
    plot_res : bool
        whether to plot the results

    Returns
    -------
    W : World
        World object with quasi DUE solution (if properly converged)
    
    Notes
    -----
        This function computes a near dynamic user equilibirum state as a steady state of day-to-day dynamical routing game.

        Specifically, on day `i`, vehicles choose their route based on actual travel time on day `i-1` with the same departure time.
        If there are shorter travel time route, they will change with probability `swap_prob`.
        This process is repeated until `max_iter` day.
        It is expected that this process eventually reach a steady state.
        Due to the problem complexity, it does not necessarily reach or converge to Nash equilibrium or any other stationary points.
        However, in the literature, it is argued that the steady state can be considered as a reasonable proxy for Nash equilibrium or dynamic equilibrium state.
        There are some theoretical background for it; but intuitively speaking, the steady state can be considered as a realistic state that people's rational behavior will reach.
    """

    W_orig = func_World()
    if print_info:
        W_orig.print_scenario_stats()

    # enumerate routes for each OD pair
    n_routes_per_od = n_routes_per_od

    dict_od_to_vehid = defaultdict(lambda: [])
    for key, veh in W_orig.VEHICLES.items():
        o = veh.orig.name
        d = veh.dest.name
        dict_od_to_vehid[o,d].append(key)

    dict_od_to_routes = {}
    for o,d in dict_od_to_vehid.keys():
        routes = enumerate_k_shortest_routes(W_orig, o, d, k=n_routes_per_od) #TODO: to be replaced with many-to-many shortest path search
        dict_od_to_routes[o,d] = routes

    if print_info:
        print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

    # day-to-day dynamics
    ttts = []
    n_swaps = []
    potential_swaps = []
    total_t_gaps = []
    swap_prob = swap_prob
    max_iter = max_iter #50 or 100 is better

    if print_info:
        print("solving DUE...")
    for i in range(max_iter):
        W = func_World()

        if i != 0:
            for key in W.VEHICLES:
                if key in routes_specified:
                    W.VEHICLES[key].enforce_route(routes_specified[key])
        
        route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
        for o,d in dict_od_to_routes.keys():
            for r in dict_od_to_routes[o,d]:
                route_set[o,d].append(W.defRoute(r))

        # simulation
        W.exec_simulation()

        # results
        W.analyzer.print_simple_stats()

        # route swap
        routes_specified = {}
        n_swap = 0
        total_t_gap = 0
        potential_n_swap = 0
        for key,veh in W.VEHICLES.items():
            if veh.state != "end":
                continue

            o = veh.orig.name
            d = veh.dest.name
            r, ts = veh.traveled_route()
            travel_time = ts[-1]-ts[0]

            flag_route_changed = False
            route_changed = None
            t_gap = 0
            
            for alt_route in route_set[o,d]:
                if alt_route.actual_travel_time(ts[0]) < r.actual_travel_time(ts[0]):
                    if flag_route_changed == False or (alt_route.actual_travel_time(ts[0]) < route_changed.actual_travel_time(ts[0])):
                        t_gap = r.actual_travel_time(ts[0]) - alt_route.actual_travel_time(ts[0])
                        potential_n_swap += 1 #there may be double count; but not critical issue
                        if random.random() < swap_prob:
                            flag_route_changed = True
                            route_changed = alt_route
            
            total_t_gap += t_gap
            routes_specified[key] = r
            if flag_route_changed:
                n_swap += 1
                routes_specified[key] = route_changed

        if print_info:
            print(f' iter {i}: time gap: {total_t_gap:.1f}, potential route change: {potential_n_swap}, route change: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

        ttts.append(int(W.analyzer.total_travel_time))
        n_swaps.append(n_swap)
        potential_swaps.append(potential_n_swap)
        total_t_gaps.append(total_t_gap)

        # if i == 0:
        #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_0init.gif")
        # if i == int(max_iter/2):
        #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_1mid.gif")
        # if i == max_iter-1:
        #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_2last.gif")

    if plot_res:
        # iteration plot
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(ttts)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of route change")
        plt.plot(n_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of potential route change")
        plt.plot(potential_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("time gap for potential route change")
        plt.plot(total_t_gaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("time gap per potential route change")
        plt.plot(np.array(total_t_gaps)/np.array(potential_swaps))
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.show()
    
    return W
