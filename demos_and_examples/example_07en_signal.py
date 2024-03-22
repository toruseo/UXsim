import pandas as pd
from uxsim import *

if __name__ == "__main__":

    # simulation world
    W = World(
        name="",
        deltan=1,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # scenario
    #merge network with signal
    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1, signal=[30,60]) #`signal` is a list of [duration of phase 0, duration of phase 1, ...]
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0) #if `signal_group` is 0, the exit of this link will be green at phase 0
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2, capacity_in=1)
    W.adddemand("orig1", "dest", 0, 1000, 0.2)
    W.adddemand("orig2", "dest", 500, 1000, 0.6)

    # execute simulation
    W.exec_simulation()

    # visualize
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["link1", "link3"], ["link2", "link3"]])
    W.analyzer.cumulative_curves()