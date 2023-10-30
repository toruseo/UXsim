from uxsim import *
import random
import itertools


# world definition
seed = None
W = World(
    name="",
    deltan=5,
    tmax=3600,
    print_mode=1, save_mode=0, show_mode=1,
    random_seed=seed,
    duo_update_time=600
)
random.seed(seed)

# network definition
"""
    N1  N2
    |   |
W1--I1--I2--E1
    |   |
W2--I3--I4--E2
    |   |
    S1  S2
"""

I1 = W.addNode("I1", 0, 0, signal=[60,60])
I2 = W.addNode("I2", 1, 0, signal=[60,60])
I3 = W.addNode("I3", 0, -1, signal=[60,60])
I4 = W.addNode("I4", 1, -1, signal=[60,60])
W1 = W.addNode("W1", -1, 0)
W2 = W.addNode("W2", -1, -1)
E1 = W.addNode("E1", 2, 0)
E2 = W.addNode("E2", 2, -1)
N1 = W.addNode("N1", 0, 1)
N2 = W.addNode("N2", 1, 1)
S1 = W.addNode("S1", 0, -2)
S2 = W.addNode("S2", 1, -2)
#E <-> W direction: signal group 0
for n1,n2 in [[W1, I1], [I1, I2], [I2, E1], [W2, I3], [I3, I4], [I4, E2]]:
    W.addLink(n1.name+n2.name, n1, n2, length=500, free_flow_speed=10, jam_density=0.2, signal_group=0)
    W.addLink(n2.name+n1.name, n2, n1, length=500, free_flow_speed=10, jam_density=0.2, signal_group=0)
#N <-> S direction: signal group 1
for n1,n2 in [[N1, I1], [I1, I3], [I3, S1], [N2, I2], [I2, I4], [I4, S2]]:
    W.addLink(n1.name+n2.name, n1, n2, length=500, free_flow_speed=10, jam_density=0.2, signal_group=1)
    W.addLink(n2.name+n1.name, n2, n1, length=500, free_flow_speed=10, jam_density=0.2, signal_group=1)

# random demand definition
dt = 30
demand = 0.20
for n1, n2 in itertools.permutations([W1, W2, E1, E2, N1, N2, S1, S2], 2):
    for t in range(0, 3600, dt):
        W.adddemand(n1, n2, t, t+dt, random.uniform(0, demand))

# simulation
W.exec_simulation()

# resutls
W.analyzer.print_simple_stats()
W.analyzer.macroscopic_fundamental_diagram()
W.analyzer.time_space_diagram_traj_links([["W1I1", "I1I2", "I2E1"], ["N1I1", "I1I3", "I3S1"]])
for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    W.analyzer.network(t, detailed=1, network_font_size=6, figsize=(3,3))