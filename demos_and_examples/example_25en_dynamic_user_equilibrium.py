"""
This code is to compute near dynamic user equilibirum state as a steady state (a proxy for Nash equilibrium) of day-to-day dynamical routing game.
Work in progress.
"""

from pylab import *
import uxsim
from uxsim.Utilities import enumerate_k_shortest_routes, generate_grid_network
import random
from collections import defaultdict

# scenario definition
W_orig = uxsim.World(
    name="",
    deltan=10,
    tmax=4000,
    print_mode=0, save_mode=1, show_mode=1,
    random_seed=None
)

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
n_routes_per_od = 3

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
print("starting day-to-day dynamics...")
ttts = []
n_swaps = []
potential_swaps = []
total_t_gaps = []
swap_prob = 0.05
swap_threth = 1.00
max_iter = 100

for i in range(max_iter):
    W = W_orig.copy()

    if i != 0:
        for key in W.VEHICLES:
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
        o = veh.orig.name
        d = veh.dest.name
        r, ts = veh.traveled_route()
        travel_time = ts[-1]-ts[0]

        should_change = False
        should_change_to = None
        t_gap = 0
        
        for alt_route in route_set[o,d]:
            if alt_route.actual_travel_time(ts[0]) < r.actual_travel_time(ts[0])*swap_threth:
                if should_change == False or (alt_route.actual_travel_time(ts[0]) < should_change_to.actual_travel_time(ts[0])*swap_threth):
                    t_gap = r.actual_travel_time(ts[0]) - alt_route.actual_travel_time(ts[0])
                    potential_n_swap += 1
                    if random.random() < swap_prob:
                        should_change = True
                        should_change_to = alt_route
                        n_swap += 1
        
        total_t_gap += t_gap
        routes_specified[key] = r
        if should_change:
            routes_specified[key] = should_change_to

    print(f' iter {i}: time gap: {total_t_gap:.1f}, potential route change: {potential_n_swap}, route change: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

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
pass

# iteration plot
figure(figsize=(6,2))
plot(ttts)
xlabel("iter")
ylabel("total travel time")

figure(figsize=(6,2))
plot(n_swaps)
ylim(0,None)
xlabel("iter")
ylabel("number of route change")

figure(figsize=(6,2))
plot(potential_swaps)
ylim(0,None)
xlabel("iter")
ylabel("number of potential\n route change")

figure(figsize=(6,2))
plot(total_t_gaps)
ylim(0,None)
xlabel("iter")
ylabel("time gap for \n potential route change")

figure(figsize=(6,2))
plot(array(total_t_gaps)/array(potential_swaps))
ylim(0,None)
xlabel("iter")
ylabel("time gap per \n potential route change")

show()