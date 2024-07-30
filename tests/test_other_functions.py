"""
This script tests various other functions in UXsim.
"""

import pytest
from uxsim import *
from uxsim.Utilities import generate_grid_network

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

def test_analyzer():
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
    W.analyzer.plot_vehicles_log(["100", "110"])
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
    print(W.analyzer.link_cumulative_to_pandas())
    print(W.analyzer.gps_like_log_to_pandas())
    print(W.analyzer.vehicle_trip_to_pandas())

    # Save the results to CSV
    W.analyzer.output_data()

    assert True

@pytest.mark.filterwarnings("ignore::UserWarning")
def test_osm_import():
    from uxsim.OSMImporter import OSMImporter

    W = World(
        name="",
        deltan=5,
        tmax=7200,
        print_mode=1, save_mode=1, show_mode=0, 
        random_seed=0
    )

    #Tokyo highway
    nodes, links = OSMImporter.import_osm_data(bbox=(35.817, 35.570, 139.881, 139.583), custom_filter='["highway"~"motorway"]')
    nodes, links = OSMImporter.osm_network_postprocessing(nodes, links, node_merge_threshold=0.005, node_merge_iteration=5, enforce_bidirectional=True)  # merge threshold distance: 0.005 degree ~= 500 m. `enforce_bidirectional` makes all links bidirectional, so that network is not fragmented (but the original network topology is not preserved rigorously).
    OSMImporter.osm_network_visualize(nodes, links, show_link_name=0, show_mode=0, save_mode=0)
    OSMImporter.osm_network_to_World(W, nodes, links, default_jam_density=0.2, coef_degree_to_meter=111000)
    W.show_network()
    
    # set demand to central Tokyo from surroundings
    W.adddemand_area2area(139.70, 35.60, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)
    W.adddemand_area2area(139.65, 35.70, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)
    W.adddemand_area2area(139.75, 35.75, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)
    W.adddemand_area2area(139.85, 35.70, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)

    W.exec_simulation()

    assert equal_tolerance(len(W.VEHICLES), 5000*4/5)

def test_readme():
    import re, requests
    url = "https://raw.githubusercontent.com/toruseo/UXsim/main/README.md"

    for _ in range(5):
        try:
            response = requests.get(url)
            content = response.text
        except (requests.exceptions.ConnectionError, requests.exceptions.Timeout, requests.exceptions.RequestException):
            continue
        break
    else:
        assert True
        return True # Skip the test if the content cannot be retrieved

    pattern = re.compile(r'```python\n(.*?)```', re.DOTALL)
    code_blocks = pattern.findall(content)

    print(code_blocks)

    for code in code_blocks:
        print(code)
        exec(code)

    assert True


def test_scenario_write_and_read():
    ##################################
    # Iter 1
    ##################################

    W = World(
        name="",
        deltan=10,
        tmax=3600,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=42
    )

    imax = 4
    jmax = 4
    nodes, links = generate_grid_network(W, imax, jmax, length=1000)

    demand_flow = 0.035
    demand_duration = 1800
    for n1 in [(0,j) for j in range(jmax)]:
        for n2 in [(imax-1,j) for j in range(jmax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i,0) for i in range(imax)]:
        for n2 in [(i,jmax-1) for i in range(imax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    W.adddemand_point2point(0.5, 0.5, 2.5, 2.5, 0, 1800, volume=100)
    W.adddemand_area2area(0.5, 0.5, 2, 2.5, 2.5, 2, 0, 1800, volume=100)

    W.save_scenario("out/test_grid.pkl")

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    df1 = W.analyzer.basic_to_pandas()
    print(df1)

    ##################################
    # Iter 2
    ##################################

    W2 = World(
        name="",
        deltan=10,
        tmax=3600,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=42
    )

    W2.load_scenario("out/test_grid.pkl")

    W2.exec_simulation()

    W2.analyzer.print_simple_stats()

    df2 = W2.analyzer.basic_to_pandas()
    print(df2)
    
    assert df1["total_travel_time"][0] == df2["total_travel_time"][0]
