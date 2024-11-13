"""
This code is to compute a near dynamic user equilibirum state as a steady state of day-to-day dynamical routing game.

Specifically, on day `i`, vehicles choose their route based on actual travel time on day `i-1` with the same departure time.
If there are shorter travel time route, they will change with probability `swap_prob`.
This process is repeated until `max_iter` day.
It is expected that this process eventually reach a steady state.
Due to the problem complexity, it does not necessarily reach or converge to Nash equilibrium or any other stationary points.
However, in the literature, it is argued that the steady state can be considered as a reasonable proxy for Nash equilibrium or dynamic equilibrium state.
There are some theoretical background for it; but intuitively speaking, the steady state can be considered as a realistic state that people's rational behavior will reach.
"""

from pylab import *
import uxsim
from uxsim.Utilities import enumerate_k_shortest_routes
import random
from collections import defaultdict

# scenario definition
seed = None
W_orig = uxsim.World(
    name="",
    deltan=10,
    tmax=4000,
    print_mode=0, save_mode=1, show_mode=1,
    random_seed=seed
)
random.seed(seed)

n_nodes = 4
imax = n_nodes
jmax = n_nodes
nodes = {}
for i in range(imax):
    for j in range(jmax):
        nodes[i,j] = W_orig.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

links = {}
for i in range(imax):
    for j in range(jmax):
        if i != imax-1:
            links[i,j,i+1,j] = W_orig.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
        if i != 0:
            links[i,j,i-1,j] = W_orig.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
        if j != jmax-1:
            links[i,j,i,j+1] = W_orig.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
        if j != 0:
            links[i,j,i,j-1] = W_orig.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)

W_orig.adddemand_nodes2nodes2(nodes.values(), nodes.values(), 0, 3000, volume=20000)

W_orig.print_scenario_stats()

# enumerate routes for each OD pair
n_routes_per_od = 4

dict_od_to_vehid = defaultdict(lambda: [])
for key, veh in W_orig.VEHICLES.items():
    o = veh.orig.name
    d = veh.dest.name
    dict_od_to_vehid[o,d].append(key)

dict_od_to_routes = {}
for o,d in dict_od_to_vehid.keys():
    routes = enumerate_k_shortest_routes(W_orig, o, d, k=n_routes_per_od)
    dict_od_to_routes[o,d] = routes
    #print(o, d, routes)

print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

# day-to-day dynamics
ttts = []
n_swaps = []
potential_swaps = []
total_t_gaps = []
swap_prob = 0.05
max_iter = 100

for i in range(max_iter):
    W = W_orig.copy()

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

    print(f'iter {i}: time gap: {total_t_gap:.1f}, potential swap: {potential_n_swap}, swap: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

    ttts.append(int(W.analyzer.total_travel_time))
    n_swaps.append(n_swap)
    potential_swaps.append(potential_n_swap)
    total_t_gaps.append(total_t_gap)

    if i == 0:
        W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_0init.gif")
    if i == int(max_iter/2):
        W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_1mid.gif")
    if i == max_iter-1:
        W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_2last.gif")

# final result

# iteration plot
figure(figsize=(6,2))
title("total travel time")
plot(ttts)
xlabel("iter")

figure(figsize=(6,2))
title("number of route change")
plot(n_swaps)
ylim(0,None)
xlabel("iter")

figure(figsize=(6,2))
title("number of potential route change")
plot(potential_swaps)
ylim(0,None)
xlabel("iter")

figure(figsize=(6,2))
title("time gap for potential route change")
plot(total_t_gaps)
ylim(0,None)
xlabel("iter")

figure(figsize=(6,2))
title("time gap per potential route change")
plot(array(total_t_gaps)/array(potential_swaps))
ylim(0,None)
xlabel("iter")

show()