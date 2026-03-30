import time

# Benchmark: C++ mode
from uxsim import World

results = []
for trial in range(5):
    W = World(
        cpp=True,
        name='',
        deltan=5,
        tmax=7200,
        no_cyclic_routing=True,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=0,
    )
    imax = 11
    jmax = 11
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f'n{(i,j)}', i, j)
    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f'l{(i,j,i+1,j)}', nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=20, jam_density=0.2)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f'l{(i,j,i-1,j)}', nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=20, jam_density=0.2)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f'l{(i,j,i,j+1)}', nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=20, jam_density=0.2)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f'l{(i,j,i,j-1)}', nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=20, jam_density=0.2)
    demand_flow = 0.035
    demand_duration = 3600
    for n1 in [(0,j) for j in range(jmax)]:
        for n2 in [(imax-1,j) for j in range(jmax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i,0) for i in range(imax)]:
        for n2 in [(i,jmax-1) for i in range(imax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    t0 = time.time()
    W.exec_simulation()
    elapsed = time.time() - t0
    results.append(elapsed)
    print(f'Trial {trial+1}: {elapsed:.3f}s')

import statistics
print(f'Median: {statistics.median(results):.3f}s, Std: {statistics.stdev(results):.3f}s')
