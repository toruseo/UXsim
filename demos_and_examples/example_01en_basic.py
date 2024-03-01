from uxsim import *
import pandas as pd

if __name__ == "__main__":

    # Define the main simulation
    # Units are standardized to seconds (s) and meters (m)
    W = World(
        name="",    # Scenario name. Can be blank. Used as the folder name for saving results.
        deltan=5,   # Simulation aggregation unit Δn. Defines how many vehicles are grouped together (i.e., platoon size) for computation. Computation cost is generally inversely proportional to deltan^2.
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=0,    # Various options. print_mode determines whether to print information. Usually set to 1, but recommended 0 when running multiple simulations automatically. save_mode determines if visualization results are saved. show_mode determines if visualization results are displayed. It's good to set show_mode=1 on Jupyter Notebook, otherwise recommended 0.
        random_seed=0    # Set the random seed. Specify if you want repeatable experiments. If not, set to None. On Jupyter Notebook, randomness might not always be consistent (requires a fix).
    )

    # Define the scenario
    # Merge network: Example of hard-coded definition
    W.addNode("orig1", 0, 0) # Create a node. Parameters: node name, visualization x-coordinate, visualization y-coordinate
    node_orig2 = W.addNode("orig2", 0, 2) # W.addNode returns the created node instance, so it can be assigned to a variable
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=0.5) # Create a link. Parameters: link name, start node, end node, length, free_flow_speed, jam_density, merge_priority during merging
    W.addLink("link2", node_orig2, "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2) # Nodes can be specified using the variable name instead of the string name
    link3 = W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2) # W.addLink also returns the created link instance
    W.adddemand("orig1", "dest", 0, 1000, 0.4) # Create OD traffic demand. Parameters: origin node, destination node, start time, end time, demand flow rate
    W.adddemand("orig2", "dest", 500, 1000, 0.6)

    # Execute the simulation
    # Run the simulation to the end
    W.exec_simulation()

    # Run the simulation for a specific time (if you want to intervene during simulation)
    # while W.check_simulation_ongoing():
    #    W.exec_simulation(duration_t=100) # Run the simulation in 100-second chunks

    # Visualization of results: Some methods are very slow. Not necessary for the simulation functionality, so can be omitted.
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_density()
    W.analyzer.time_space_diagram_traj()
    W.analyzer.time_space_diagram_traj_links([["link1", "link3"], ["link2", "link3"]])
    W.analyzer.cumulative_curves()
    W.analyzer.plot_vehicle_log("110")
    W.analyzer.macroscopic_fundamental_diagram()
    for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))
    for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        W.analyzer.network(t, detailed=1, network_font_size=0)
    W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
    W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.3, interval=5, trace_length=5)

    # Convert results to pandas.DataFrame for easier analysis
    print(W.analyzer.basic_to_pandas()) # Basic statistics
    print(W.analyzer.od_to_pandas())    # Information by OD
    print(W.analyzer.mfd_to_pandas())   # MFD (Macroscopic Fundamental Diagram)
    print(W.analyzer.link_to_pandas())  # Information per link
    print(W.analyzer.link_traffic_state_to_pandas())    # Traffic state inside the link
    print(W.analyzer.vehicles_to_pandas())  # Information per vehicle

    # Save the results to CSV
    W.analyzer.output_data()