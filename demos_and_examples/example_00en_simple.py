from uxsim import *

# Define the main simulation
# Units are standardized to seconds (s) and meters (m)
W = World(
    name="",    # Scenario name
    deltan=5,   # Simulation aggregation unit delta n
    tmax=1200,  # Total simulation time (s)
    print_mode=1, save_mode=1, show_mode=0,    # Various options
    random_seed=0    # Set the random seed
)

# Define the scenario
W.addNode("orig1", 0, 0) # Create a node
W.addNode("orig2", 0, 2)
W.addNode("merge", 1, 1)
W.addNode("dest", 2, 1)
W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20) # Create a link
W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20)
W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20)
W.adddemand("orig1", "dest", 0, 1000, 0.45) # Create OD traffic demand
W.adddemand("orig2", "dest", 400, 1000, 0.6)

# Run the simulation to the end
W.exec_simulation()

# Print summary of simulation result
W.analyzer.print_simple_stats()

# Visualize snapshots of network traffic state for several timesteps
W.analyzer.network(100, detailed=1, network_font_size=12)
W.analyzer.network(600, detailed=1, network_font_size=12)
W.analyzer.network(800, detailed=1, network_font_size=12)