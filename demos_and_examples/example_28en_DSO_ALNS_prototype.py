from collections import defaultdict
import time
from pprint import pprint

from pylab import *

import random

import uxsim
from uxsim.Utilities import enumerate_k_shortest_routes, enumerate_k_random_routes
from uxsim.DTAsolvers import DTAsolvers
from uxsim.DTAsolvers import ALNS

def create_World():
    # simulation world
    W = uxsim.World(
        name="",
        deltan=10,
        tmax=4800,
        duo_update_time=300,
        print_mode=0, save_mode=0, show_mode=1,
        random_seed=42,
    )

    # scenario
    #automated network generation
    #deploy nodes as an imax x jmax grid
    imax = 9
    jmax = 9
    id_center = 4
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    #create links between neighborhood nodes
    links = {}
    for i in range(imax):
        for j in range(jmax):
            free_flow_speed = 10
            if i != imax-1:
                if j == id_center:
                    free_flow_speed = 20
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=free_flow_speed)
            if i != 0:
                if j == id_center:
                    free_flow_speed = 20
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=free_flow_speed)
            if j != jmax-1:
                if i == id_center:
                    free_flow_speed = 20
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=free_flow_speed)
            if j != 0:
                if i == id_center:
                    free_flow_speed = 20
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=free_flow_speed)

    #generate traffic demand between the boundary nodes
    demand_flow = 0.08
    demand_duration = 2400
    outer_ids = 3
    for n1 in [(0,j) for j in range(outer_ids, jmax-outer_ids)]:
        for n2 in [(imax-1,j) for j in range(outer_ids,jmax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(i,jmax-1) for i in range(outer_ids, imax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i,0) for i in range(outer_ids, imax-outer_ids)]:
        for n2 in [(i,jmax-1) for i in range(outer_ids, imax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(imax-1,j) for j in range(outer_ids,jmax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    return W

solver_DUE = DTAsolvers.SolverDUE(create_World)
solver_DUE.solve(max_iter=50, n_routes_per_od=20)

W_DUE = solver_DUE.W_sol
W_DUE.analyzer.print_simple_stats(force_print=True)
W_DUE.analyzer.network_average(network_font_size=0, legend=True)
df_DUE = W_DUE.analyzer.basic_to_pandas()

##############################################################
# DSO

class Self_dummy:
    def __init__(s):
        pass
s = Self_dummy()


##############################################################
# args
s.func_World = create_World
initial_solution_World = W_DUE
print_progress = True
n_routes_per_od = 20
max_iter = 100 #2500
k_max = max([1, int(len(W_DUE.VEHICLES)/100)])

departure_times = []

s.start_time = time.time()

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

# routes = {}
# n_routes_per_od = n_routes_per_od
# for od_pair in dict_od_to_vehid.keys():
#     routes[od_pair] = enumerate_k_shortest_routes(W, od_pair[0], od_pair[1], n_routes_per_od)

if initial_solution_World != None and hasattr(initial_solution_World, "dict_od_to_routes"):
    print("Initial solusion is used")
    routes = initial_solution_World.dict_od_to_routes
    xx0 = [0 for i in range(len(W.VEHICLES))]
    for i,veh in enumerate(W.VEHICLES.values()):   
        route_id = 0
        for j,r in enumerate(routes[(veh.orig.name, veh.dest.name)]):
            if W.defRoute(r) == veh.traveled_route()[0]:
                route_id = j
                break
        xx0[i] = route_id
else:
    print("No initial solution")
    routes = enumerate_k_random_routes(W, k=n_routes_per_od)
    xx0 = [random.randint(0, len(xx_domain[i])) for i in range(len(W.VEHICLES))]

# print("available routes for each OD pair")
# for key in routes:
#    for route in routes[key]:
#        print(key, route)

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
            
xx_domain = []
for i, veh in enumerate(W.VEHICLES.values()):
    actual_n_route = len(routes[(veh.orig.name, veh.dest.name)])
    xx_domain.append([i for i in range(actual_n_route)])

s.W_best = W
s.best_obj_value = None
s.iter = 0
s.ttts = []
s.route_log = []
s.cost_log = []
s.dfs_link = []

s.best_iter = []
s.best_obj = []

print("solving DSO by ALNS...")
print("k_max:", k_max)

def objective_function_by_UXsim(xx):
    W = create_World()
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
        s.W_best = W

        s.best_iter.append(s.iter)
        s.best_obj.append(s.best_obj_value)

    if s.iter%20 == 0:
        print(f"iter: {s.iter} - TTT: {W.analyzer.total_travel_time}, trips:{W.analyzer.trip_completed}/{W.analyzer.trip_all}, Obj.:{obj_value}, Best Obj.:{s.best_obj_value}")

    s.iter += 1
    return obj_value


state = ALNS.init_alns(
    xx0, objective_function_by_UXsim,
    domains=xx_domain,
    move_set=["kflip","swap","shuffle"],
    k_max=k_max,
    total_iters=max_iter,
    auto_shake=True,
    seed=0,
    additional_info={"departure_times":departure_times}
)

converged, reason, done = ALNS.run_auto_until_converged(
    state,
    max_steps=max_iter,
    batch=50,
    check_every=100,
    inject_every=None,
    inject_fn=None, #TODO: `inject_fn`として混雑リンクを使っている車両を選び出す手続きを入れる
    stop_on_converge=True,
)

xx_star, run = ALNS.finalize(state, stop_reason="converged" if converged else "budget")

s.W_sol = s.W_best

s.end_time = time.time()

# try:
#     df = run.to_dataframe()
#     df = df.drop(columns=["changed_idx"])
#     print(df.to_string())
# except Exception:
#     pass

print(f" computation time: {s.end_time - s.start_time:.1f} seconds")
print("converged:", converged, "reason:", reason, "steps:", done)
print("best obj:", run.best_obj)
print("best xx  :", xx_star)
print("op stats:")
pprint(run.operator_stats)

#visualizaion_helper_function(s.W_best)
figure()
plot(s.ttts, "b-", lw=0.5)
plot(s.best_iter, s.best_obj, "ro")
ylim([None,s.ttts[0]*1.1])
xlabel("iteration")
ylabel("total travel time (s)")

#GAで求めた参考値：5104800