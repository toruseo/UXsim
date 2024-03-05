from uxsim import *
import random

# world definition
seed = 0
W = World(
    name="",
    deltan=5,
    tmax=3600,
    print_mode=1, save_mode=0, show_mode=1,
    random_seed=seed,
    duo_update_time=99999
)
random.seed(seed)

# network definition
II = W.addNode("Intersection", 0, 0, signal=[60,60])
EE = W.addNode("E", 1, 0)
WW = W.addNode("W", -1, 0)
SS = W.addNode("S", 0, 1)
NN = W.addNode("N", 0, -1)
W.addLink("EI", EE, II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=0)
W.addLink("WI", WW, II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=0)
W.addLink("SI", SS, II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=1)
W.addLink("NI", NN, II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=1)
W.addLink("IE", II, EE, length=500, free_flow_speed=10, jam_density=0.2)
W.addLink("IW", II, WW, length=500, free_flow_speed=10, jam_density=0.2)
W.addLink("IS", II, SS, length=500, free_flow_speed=10, jam_density=0.2)
W.addLink("IN", II, NN, length=500, free_flow_speed=10, jam_density=0.2)

# random demand definition
dt = 30
for t in range(0, 3600, dt):
    W.adddemand(EE, WW, t, t+dt, random.uniform(0, 0.6))
    W.adddemand(WW, EE, t, t+dt, random.uniform(0, 0.6))
    W.adddemand(SS, NN, t, t+dt, random.uniform(0, 0.6))
    W.adddemand(NN, SS, t, t+dt, random.uniform(0, 0.6))

# simulation
while W.check_simulation_ongoing():
    #run simulation for 30 s
    W.exec_simulation(duration_t=30)

    #count number of vehicles per direction
    vehicles_per_links = {tuple(l.signal_group): 0 for l in II.inlinks.values()}
    for l in II.inlinks.values():
        vehicles_per_links[tuple(l.signal_group)] += l.num_vehicles_queue #l.num_vehicles_queue: the number of vehicles in queue in link l
    max_vehicles_group = max(vehicles_per_links, key=vehicles_per_links.get) #determine the direction with maximum number of vehicles

    #green light for the direction
    II.signal_phase = max_vehicles_group[0]
    II.signal_t = 0

# resutls
W.analyzer.print_simple_stats()
W.analyzer.macroscopic_fundamental_diagram()
W.analyzer.time_space_diagram_traj(["EI", "WI", "SI", "NI"])
W.analyzer.time_space_diagram_traj_links(["EI", "IW"])
for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    W.analyzer.network(t, detailed=1, network_font_size=0, figsize=(2,2))