"""
This script tests various other functions in UXsim.
"""

import pytest
from uxsim import *
from uxsim.Utilities import *

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

def test_k_shortest_path():
    W = World(
        name="",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=1,    # Various options
        random_seed=0    # Set the random seed
    )

    # Define the scenario

    # 2 - D
    # | \ |
    # O - 1
    #free flow travel time: 
    #   O1D = 2000/20 = 100
    #   O2D = 4000/20 = 200
    #   O12D= 1600/20 = 80


    W.addNode(name="O", x=0, y=0)
    W.addNode(name="1", x=1, y=0)
    W.addNode(name="2", x=0, y=1)
    W.addNode(name="D", x=1, y=1)
    W.addLink("O1", "O", "1", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("1D", "1", "D", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("O2", "O", "2", length=3500, free_flow_speed=20, number_of_lanes=1)
    W.addLink("2D", "2", "D", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink("12", "1", "2", length=100, free_flow_speed=20, number_of_lanes=1)
    W.adddemand(orig="O", dest="D", t_start=0, t_end=1000, flow=0.6)

    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()

    df = W.analyzer.link_to_pandas()

    assert df[df["link"]=="O1"]["traffic_volume"].values[0] == 600
    assert df[df["link"]=="O2"]["traffic_volume"].values[0] == 0
    assert df[df["link"]=="12"]["traffic_volume"].values[0] == 600

    assert enumerate_k_shortest_routes(W, "O", "D") == [['O1', '12', '2D']]
    assert enumerate_k_shortest_routes(W, "O", "D", k=3) == [['O1', '12', '2D'], ['O1', '1D'], ['O2', '2D']]
    assert enumerate_k_shortest_routes(W, "O", "D", k=3, return_cost=True) == ([['O1', '12', '2D'], ['O1', '1D'], ['O2', '2D']], [80.0, 100.0, 200.0])

@pytest.mark.flaky(reruns=10)
def test_k_shortest_path_on_t():

    W = World(
        name="",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=1,    # Various options
        random_seed=None    # Set the random seed
    )

    # Define the scenario

    # 2 - D
    # | \ |
    # O - 1
    #free flow travel time: 
    #   O1D = 2000/20 = 100
    #   O2D = 4000/20 = 200
    #   O12D= 1600/20 = 80


    W.addNode(name="O", x=0, y=0)
    W.addNode(name="1", x=1, y=0)
    W.addNode(name="2", x=0, y=1)
    W.addNode(name="D", x=1, y=1)
    W.addLink("O1", "O", "1", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("1D", "1", "D", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("O2", "O", "2", length=3500, free_flow_speed=20, number_of_lanes=1)
    W.addLink("2D", "2", "D", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink("12", "1", "2", length=100, free_flow_speed=20, number_of_lanes=1, capacity_out=0.4)
    W.adddemand(orig="O", dest="D", t_start=0, t_end=1000, flow=0.6)

    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()

    df = W.analyzer.link_to_pandas()
    
    assert equal_tolerance(df[df["link"]=="O1"]["traffic_volume"].values[0], 460, rel_tol=0.2)
    assert equal_tolerance(df[df["link"]=="O2"]["traffic_volume"].values[0], 140, rel_tol=0.2)
    assert equal_tolerance(df[df["link"]=="12"]["traffic_volume"].values[0], 325, rel_tol=0.2)

    t = 0
    assert enumerate_k_shortest_routes_on_t(W, "O", "D", t=t) == [['O1', '12', '2D']]
    assert enumerate_k_shortest_routes_on_t(W, "O", "D", t=t, k=3) == [['O1', '12', '2D'], ['O1', '1D'], ['O2', '2D']]
    routes, costs = enumerate_k_shortest_routes_on_t(W, "O", "D", t=t, k=3, return_cost=True)
    assert routes[0] == ['O1', '12', '2D']
    assert equal_tolerance(costs[0], 80.0)

    t = 200
    assert enumerate_k_shortest_routes_on_t(W, "O", "D", t=t) == [['O1', '1D']]
    assert enumerate_k_shortest_routes_on_t(W, "O", "D", t=t, k=3) == [['O1', '1D'], ['O1', '12', '2D'], ['O2', '2D']]
    routes, costs = enumerate_k_shortest_routes_on_t(W, "O", "D", t=t, k=3, return_cost=True)
    assert routes[0] == ['O1', '1D']
    assert equal_tolerance(costs[0], 131.8181818181818)

    t = 400
    assert enumerate_k_shortest_routes_on_t(W, "O", "D", t=t) == [['O2', '2D']]
    assert enumerate_k_shortest_routes_on_t(W, "O", "D", t=t, k=3) == [['O2', '2D'], ['O1', '1D'], ['O1', '12', '2D']]
    routes, costs = enumerate_k_shortest_routes_on_t(W, "O", "D", t=t, k=3, return_cost=True)
    assert routes[0] == ['O2', '2D']
    assert equal_tolerance(costs[0], 200.0)


def test_util_catch_exceptions_and_warn():
    with pytest.warns(UserWarning, match=r".*network().*"):
        W = World(
            name="",    # Scenario name
            deltan=5,   # Simulation aggregation unit delta n
            tmax=1200,  # Total simulation time (s)
            print_mode=1, save_mode=1, show_mode=0,    # Various options
            random_seed=0    # Set the random seed
        )

        # Define the scenario
        ## Create nodes
        W.addNode(name="orig1", x=0, y=0)
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        ## Create links between nodes
        W.addLink(name="link1", start_node="orig1", end_node="merge",
                length=1000, free_flow_speed=20, number_of_lanes=1)
        W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
        W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
        ## Create OD traffic demand between nodes
        W.adddemand(orig="orig1", dest="dest", t_start=0, t_end=1000, flow=0.45)
        W.adddemand("orig2", "dest", 400, 1000, 0.6)

        # Run the simulation to the end
        W.exec_simulation()

        # Print summary of simulation result
        W.analyzer.print_simple_stats()

        # Visualize snapshots of network traffic state for several timesteps
        W.analyzer.network()

    assert True

def test_util_print_columns():
    W = World(
        name="",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=0,    # Various options
        random_seed=0    # Set the random seed
    )

    # Define the scenario
    ## Create nodes
    W.addNode(name="orig1", x=0, y=0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    ## Create links between nodes
    W.addLink(name="link1", start_node="orig1", end_node="merge",
            length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    ## Create OD traffic demand between nodes
    W.adddemand(orig="orig1", dest="dest", t_start=0, t_end=1000, flow=0.45)
    W.adddemand("orig2", "dest", 400, 1000, 0.6)

    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()

    print_columns(W.VEHICLES["0"].log_t, W.VEHICLES["0"].log_x, W.VEHICLES["0"].log_v)
    print_columns(W.VEHICLES["0"].log_t, W.VEHICLES["0"].log_x, W.VEHICLES["0"].log_v, W.VEHICLES["1"].log_t, W.VEHICLES["1"].log_x, W.VEHICLES["1"].log_v)

    assert True
    
def test_printtry():
    lis = [1,2,3]
    printtry(lambda: (lis[0]))
    printtry(lambda: (lis[10]))
    assert True