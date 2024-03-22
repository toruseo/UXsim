from uxsim import *
import pandas as pd


if __name__ == "__main__":

    print("####"*10)
    print("# gridlock scenario")
    # simulation world
    W = World(
        name="gridlock",
        deltan=5,
        tmax=6000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # scenario
    #circular network for gridlock: load from CSV
    W.load_scenario_from_csv("dat/uroboros_nodes.csv", "dat/uroboros_links.csv", "dat/uroboros_demand.csv")

    # The following control prevents gridlock. Comment out to execute.
    #W.get_link("NE").merge_priority = 2
    #W.get_link("SW").merge_priority = 2

    # execute simulation
    W.exec_simulation()

    # visualize results
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["WN", "NE", "ES", "SW"]])    #trajectory of vehicles on circular roads
    W.analyzer.macroscopic_fundamental_diagram()
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=1, interval=5, trace_length=5)
    # here, you can see that two traffic congestion merges into one (like an uroboros), causing gridlock.

    # output results as pandas.DataFrame
    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())
    print(W.analyzer.vehicles_to_pandas())



    print("####"*10)
    print("# no gridlock scenario")
    # new simulation world
    W = World(
        name="gridlock_prevented",
        deltan=5,
        tmax=6000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # scenario
    #same network and demand as before
    W.load_scenario_from_csv("dat/uroboros_nodes.csv", "dat/uroboros_links.csv", "dat/uroboros_demand.csv")

    # The following control prevents gridlock.
    W.get_link("NE").merge_priority = 2
    W.get_link("SW").merge_priority = 2

    # execute simulation
    W.exec_simulation()

    # visualize results
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["WN", "NE", "ES", "SW"]])
    W.analyzer.macroscopic_fundamental_diagram()
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=1, interval=5, trace_length=5)
    # here, you can see that the gridlock is completely prevented by the merge priority control.


    # output results as pandas.DataFrame
    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())
    print(W.analyzer.vehicles_to_pandas())