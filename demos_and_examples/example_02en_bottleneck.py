from uxsim import *
import pandas as pd


if __name__ == "__main__":

    # simulation world
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # scenario
    W.addNode("orig", 0, 0)
    W.addNode("dummy1", 2, 0)
    W.addNode("dummy2", 2.2, 0)
    W.addNode("dest", 3, 0)
    W.addLink("link1", "orig", "dummy1", length=2000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link2", "dummy1", "dummy2", length=200, free_flow_speed=20, jam_density=0.2, capacity_in=0.6) #capacity_in: flow capacity for entering this link（veh/s). Note that the other capacity is 0.8 veh/s. Thus, the entrance to this link works as a bottleneck
    W.addLink("link3", "dummy2", "dest", length=800, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 1800, 0.4)
    W.adddemand("orig", "dest", 300, 600, 0.4)

    # simulation execution
    W.exec_simulation()

    # visualize
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["link1", "link2", "link3"]])
    #for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    #    W.analyzer.network(t, detailed=1, network_font_size=0)
    #W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=1, interval=5, trace_length=5)

    # output pandas.DataFrame
    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())
    print(W.analyzer.link_traffic_state_to_pandas())