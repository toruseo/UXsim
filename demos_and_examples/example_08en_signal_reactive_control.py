import pandas as pd
from uxsim import *

if __name__ == "__main__":

    # Definition of the main simulation
    W = World(
        name="",
        deltan=1,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # Scenario definition
    # Merging network: signal control
    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    node_signal = W.addNode("merge", 1, 1, signal=[30,60]) # The variable 'signal' is a list of [duration of phase 0, duration of phase 1, ...]
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0) # The value of the variable 'signal_group' (say `n`) indicates that the exit of this link will turn green during phase 'n'
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig1", "dest", 0, 1000, 0.2)
    W.adddemand("orig2", "dest", 500, 1000, 0.6)


    # Execute simulation
    # Run the simulation for a specific duration (for cases where intervention is desired midway)
    while W.check_simulation_ongoing():
        # Execute the simulation in increments of 10 seconds
        W.exec_simulation(duration_t=10)

        # Investigate the number of vehicles per direction
        vehicles_per_links = {}
        for l in node_signal.inlinks.values():
            vehicles_per_links[tuple(l.signal_group)] = l.num_vehicles # l.num_vehicles: Number of vehicles on link 'l'
        max_vehicles_group = max(vehicles_per_links, key=vehicles_per_links.get) # Returns the direction with the maximum number of vehicles
        print(vehicles_per_links)

        # Set the signal to green for the direction with the maximum number of vehicles
        node_signal.signal_phase = max_vehicles_group[0]
        node_signal.signal_t = 0


    # Visualize results
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["link1", "link3"], ["link2", "link3"]])
    W.analyzer.cumulative_curves()
    #W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    #W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.3, interval=5, trace_length=5)