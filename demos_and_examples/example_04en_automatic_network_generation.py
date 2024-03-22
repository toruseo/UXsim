import pandas as pd
from uxsim import *

if __name__ == "__main__":

    # simulation world
    W = World(
        name="",
        deltan=5,
        tmax=7200,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=0,
    )

    # scenario
    #automated network generation
    #deploy nodes as an imax x jmax grid
    imax = 11
    jmax = 11
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j)

    #create links between neighborhood nodes
    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=20, jam_density=0.2)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=20, jam_density=0.2)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=20, jam_density=0.2)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=20, jam_density=0.2)

    #generate traffic demand between the boundary nodes
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


    # execute simulation
    W.exec_simulation()

    # visualize
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([[links[1,i,1,i+1] for i in range(imax-1)]])
    W.analyzer.macroscopic_fundamental_diagram()
    W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)

    # output results
    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())