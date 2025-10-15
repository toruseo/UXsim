"""
This script tests various other functions in UXsim.
"""

import pytest
from numpy import *
from uxsim import *
from uxsim.Utilities import *
import dill as pickle



def test_analyzer():
    import matplotlib
    matplotlib.use('Agg')
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
    W.analyzer.network_average()
    for t in list(range(0,W.TMAX,int(W.TMAX/3))):
        W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))
        W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4), state_variables="flow_speed", legend=False)
    for t in list(range(0,W.TMAX,int(W.TMAX/3))):
        W.analyzer.network(t, detailed=1, network_font_size=0)
        W.analyzer.network(t, detailed=1, network_font_size=0, state_variables="flow_speed", legend=True)
    W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0, legend=False)
    W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    W.analyzer.network_anim(detailed=0, network_font_size=0, figsize=(12,12), state_variables="flow_speed")
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.3, interval=5, trace_length=5)

    W.analyzer.network_pillow(600, state_variables="density_speed")
    W.analyzer.network_pillow(600, state_variables="density_flow")

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
    nodes, links = OSMImporter.import_osm_data(north=35.817, south=35.570, east=139.881, west=139.583, custom_filter='["highway"~"motorway"]')
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
    import matplotlib
    matplotlib.use('Agg')
    
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

    W.save_scenario("out/test_grid.uxsim_scenario")

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

    W2.load_scenario("out/test_grid.uxsim_scenario")

    W2.exec_simulation()

    W2.analyzer.print_simple_stats()

    df2 = W2.analyzer.basic_to_pandas()
    print(df2)
    
    assert df1["total_travel_time"][0] == df2["total_travel_time"][0]

def test_scenario_write_and_read_areas():
    ##################################
    # Iter 1
    ##################################

    W = World(
        name="",
        deltan=10,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=42,
    )

    n_nodes = 5
    imax = n_nodes
    jmax = n_nodes
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)


    areas = {
        "areaN": [nodes[0,0], nodes[0, n_nodes-1]],
        "areaS": [nodes[n_nodes-1,0], nodes[n_nodes-1, n_nodes-1]],
        "areaNW": [nodes[0,0]],
        "areaSE": [nodes[n_nodes-1, n_nodes-1]]
    }

    W.adddemand_nodes2nodes(areas["areaN"], areas["areaS"], 0, 3000, volume=7000)

    W.save_scenario("out/test_area.uxsim_scenario")

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
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=42,
    )

    W2.load_scenario("out/test_area.uxsim_scenario")

    W2.exec_simulation()
    W2.analyzer.print_simple_stats()

    df2 = W2.analyzer.basic_to_pandas()
    print(df2)

    assert df1["total_travel_time"][0] == df2["total_travel_time"][0]


def test_world_save():
    W = World(
        name="pickle_test",
        deltan=5, 
        tmax=1200,
        print_mode=0, save_mode=0, show_mode=1, 
        random_seed=0
    )

    # Define the scenario

    W.addNode(name="orig1", x=0, y=0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)

    W.addLink(name="link1", start_node="orig1", end_node="merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)

    W.adddemand(orig="orig1", dest="dest", t_start=0, t_end=1000, flow=0.45)
    W.adddemand("orig2", "dest", 400, 1000, 0.6)

    W.show_network()

    W.save("test_pickle")

    W2 = pickle.load(open("test_pickle.pkl", "rb"))

    W2.exec_simulation()

    W2.analyzer.print_simple_stats() #no print

    W2.analyzer.network(t=500)

    assert True

def test_world_copy():
    
    W = World(
        name="pickle_test",
        deltan=5, 
        tmax=1200,
        print_mode=0, save_mode=0, show_mode=1, 
        random_seed=0
    )

    W.addNode(name="orig1", x=0, y=0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)

    W.addLink(name="link1", start_node="orig1", end_node="merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)

    W.adddemand(orig="orig1", dest="dest", t_start=0, t_end=1000, flow=0.45)
    W.adddemand("orig2", "dest", 400, 1000, 0.6)

    W.show_network()

    W2 = W.copy()

    W2.exec_simulation()

    W2.analyzer.print_simple_stats() #no print

    W2.analyzer.network(t=500)


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

@pytest.mark.flaky(reruns=10)
def test_shortest_path_costs():
    W = World(
        name="",
        deltan=5,
        tmax=1200,
        duo_update_time=600,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=None
    )

    orig = W.addNode("orig", 0, 0)
    W.addNode("mid1", 1, 1)
    W.addNode("mid2", 1, -1)
    dest = W.addNode("dest", 2, 0)
    W.addLink("link01", "orig", "mid1", length=1000, free_flow_speed=20, number_of_lanes=2)
    W.addLink("link02", "orig", "mid2", length=1000, free_flow_speed=20, number_of_lanes=2)
    W.addLink("link13", "mid1", "dest", length=2000, free_flow_speed=20, number_of_lanes=2)
    W.addLink("link23", "mid2", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.adddemand("orig", "dest", 0, 1000, 1.2)
    W.show_network(figsize=(3,3))

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.cumulative_curves(figsize=(4,2))

    spd = get_shortest_path_distance_between_all_nodes(W)
    assert spd["orig", "dest"] == 2000
    assert spd["orig", "mid1"] == 1000
    assert spd["orig", "mid2"] == 1000
    assert spd["mid1", "dest"] == 2000
    assert spd["mid2", "dest"] == 1000
    assert spd["dest", "orig"] == np.inf
    assert spd[orig, dest] == 2000
    assert spd[dest, orig] == np.inf

    spd = get_shortest_path_distance_between_all_nodes(W, return_matrix=True)
    assert spd[0, 3] == 2000
    assert spd[3, 0] == np.inf

    spt = get_shortest_path_instantaneous_travel_time_between_all_nodes(W)
    assert equal_tolerance(spt["orig", "dest"], 150)
    assert equal_tolerance(spt["orig", "mid1"], 50, rel_tol=0.2)
    assert equal_tolerance(spt["orig", "mid2"], 150)
    assert equal_tolerance(spt["mid1", "dest"], 100)
    assert equal_tolerance(spt["mid2", "dest"], 50, rel_tol=0.2)
    assert spt["dest", "orig"] == np.inf
    assert equal_tolerance(spt[orig, dest], 150)
    assert spt[dest, orig] == np.inf

    spt = get_shortest_path_instantaneous_travel_time_between_all_nodes(W, return_matrix=True)
    assert equal_tolerance(spt[0, 3], 150)
    assert spt[3, 0] == np.inf

    spt0 = get_shortest_path_instantaneous_travel_time_between_all_nodes_on_t(W,0)
    assert equal_tolerance(spt0["orig", "dest"], 100)
    assert equal_tolerance(spt0["orig", "mid1"], 50, rel_tol=0.2)
    assert equal_tolerance(spt0["orig", "mid2"], 50, rel_tol=0.2)
    assert equal_tolerance(spt0["mid1", "dest"], 100)
    assert equal_tolerance(spt0["mid2", "dest"], 50, rel_tol=0.2)

    spt200, t = get_shortest_path_instantaneous_travel_time_between_all_nodes_on_t(W, 200, return_time=True)
    assert spt0 == spt200
    assert t == 0

    spt600 = get_shortest_path_instantaneous_travel_time_between_all_nodes_on_t(W,600)
    assert equal_tolerance(spt600["orig", "dest"], 150)
    assert equal_tolerance(spt600["orig", "mid1"], 50, rel_tol=0.2)
    assert equal_tolerance(spt600["orig", "mid2"], 150)
    assert equal_tolerance(spt600["mid1", "dest"], 100)
    assert equal_tolerance(spt600["mid2", "dest"], 50, rel_tol=0.2)

    spt1200 = get_shortest_path_instantaneous_travel_time_between_all_nodes_on_t(W,1200)
    assert spt600 == spt1200

    spt600_mat = get_shortest_path_instantaneous_travel_time_between_all_nodes_on_t(W, 600, return_matrix=True)
    assert equal_tolerance(spt600_mat[0, 3], 150)
    assert spt600_mat[3, 0] == np.inf

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

def test_area2area_demand_and_stats():
    W = World(
        name="",
        deltan=10,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=None,
    )

    n_nodes = 5
    imax = n_nodes
    jmax = n_nodes
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)


    areas = {
        "areaN": [nodes[0,0], nodes[0, n_nodes-1]],
        "areaS": [nodes[n_nodes-1,0], nodes[n_nodes-1, n_nodes-1]],
        "areaNW": [nodes[0,0]],
        "areaSE": [nodes[n_nodes-1, n_nodes-1]]
    }

    W.adddemand_nodes2nodes(areas["areaN"], areas["areaS"], 0, 3000, volume=7000)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    df = W.analyzer.areas2areas_to_pandas(areas.values(), list(areas.keys()))
    print(df)

    print(W.analyzer.areas2areas_to_pandas(areas.values()))

    assert W.analyzer.trip_all == 7000
    assert df["total_trips"][(df["origin_area"] == "areaN") & (df["destination_area"] == "areaS")].values[0] == 7000
    assert df["average_free_travel_time"][(df["origin_area"] == "areaN") & (df["destination_area"] == "areaS")].values[0] == 300.0
    assert df["average_shortest_distance"][(df["origin_area"] == "areaN") & (df["destination_area"] == "areaS")].values[0] == 6000.0
    assert df["total_trips"][(df["origin_area"] == "areaNW") & (df["destination_area"] == "areaSE")].values[0] == 1750
    assert df["average_free_travel_time"][(df["origin_area"] == "areaNW") & (df["destination_area"] == "areaSE")].values[0] == 400.0
    assert df["average_shortest_distance"][(df["origin_area"] == "areaNW") & (df["destination_area"] == "areaSE")].values[0] == 8000.0

def test_adddemand_area2area2_nodes2nodes2():
    W = World(
        name="",
        deltan=5,
        tmax=7200,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=0
    )

    # scenario
    #automated network generation
    #deploy nodes as an imax x jmax grid
    imax = 5
    jmax = 5
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

    W.adddemand_nodes2nodes2(nodes.values(), nodes.values(), 0, 3600, volume=5000)
    W.adddemand_area2area2(0, 0, 1.1, 5, 5, 1.1, 0, 3600, volume=5000)

    W.finalize_scenario()

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    assert equal_tolerance(10000, len(W.VEHICLES)*W.DELTAT)

def test_adddemand_area2area2_nodes2nodes2_scenario():
    # simulation world
    W = World(
        name="",
        deltan=5,
        tmax=7200,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=0
    )

    # scenario
    #automated network generation
    #deploy nodes as an imax x jmax grid
    imax = 5
    jmax = 5
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

    W.adddemand_nodes2nodes2(nodes.values(), nodes.values(), 0, 3600, volume=5000)
    W.adddemand_area2area2(0, 0, 1.1, 5, 5, 1.1, 0, 3600, volume=5000)

    W.save_scenario("out/demandtest.uxsim_scenario")

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    #W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.2, interval=5, trace_length=10,  figsize=6, antialiasing=False)
    #display_image_in_notebook("out/anim_network_fancy.gif") 


    ### ITER2

    W2 = World(
        name="",
        deltan=5,
        tmax=7200,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=0
    )

    W2.load_scenario("out/demandtest.uxsim_scenario")

    W2.exec_simulation()
    W2.analyzer.print_simple_stats()

    assert W.analyzer.average_travel_time == W2.analyzer.average_travel_time

@pytest.mark.flaky(reruns=10)
def test_area_stats():

    rec_volume_areaN = []
    rec_volume_areaS = []
    rec_ttt_areaN = []
    rec_delay_areaN = []

    for i in range(10):
        W = World(
            name="",
            deltan=10,
            tmax=3000,
            print_mode=1, save_mode=1, show_mode=0,
            random_seed=None,
        )

        n_nodes = 4
        imax = n_nodes
        jmax = n_nodes
        nodes = {}
        for i in range(imax):
            for j in range(jmax):
                nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

        links = {}
        for i in range(imax):
            for j in range(jmax):
                if i != imax-1:
                    links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
                if i != 0:
                    links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
                if j != jmax-1:
                    links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
                if j != 0:
                    links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)


        area_dict = {
            "areaN": [nodes[0,i] for i in range(n_nodes)],
            "areaS": [nodes[n_nodes-1,i] for i in range(n_nodes)],
            "areaNW": [nodes[0,0]],
            "areaSE": [nodes[n_nodes-1, n_nodes-1]]
        }

        W.adddemand_nodes2nodes(area_dict["areaN"], area_dict["areaS"], 0, 3000, volume=7000)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        df = W.analyzer.area_to_pandas(list(area_dict.values()), list(area_dict.keys()), border_include=True)
        print(df)

        rec_volume_areaN.append(df["traffic_volume"][df["area"] == "areaN"].values[0])
        rec_volume_areaS.append(df["traffic_volume"][df["area"] == "areaS"].values[0])
        rec_ttt_areaN.append(df["total_travel_time"][df["area"] == "areaN"].values[0])
        rec_delay_areaN.append(df["average_delay"][df["area"] == "areaN"].values[0])

    assert equal_tolerance(average(rec_volume_areaN), 6880)
    assert equal_tolerance(average(rec_volume_areaS), 6380)
    assert equal_tolerance(average(rec_ttt_areaN), 840000)
    assert equal_tolerance(average(rec_delay_areaN), 0.77, abs_tol=0.1)

@pytest.mark.flaky(reruns=10)
def test_vehicle_group_stats():
    W = World(
        name="",
        deltan=10,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=None,
    )

    n_nodes = 4
    imax = n_nodes
    jmax = n_nodes
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)


    areas = {
        "areaN": [nodes[0,0], nodes[0, n_nodes-1]],
        "areaS": [nodes[n_nodes-1,0], nodes[n_nodes-1, n_nodes-1]],
        "areaNW": [nodes[0,0]],
        "areaSE": [nodes[n_nodes-1, n_nodes-1]]
    }

    W.adddemand_nodes2nodes(areas["areaN"], areas["areaS"], 0, 3000, volume=7000)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    dt = 500
    group_dict = {}
    for t in range(0, W.TMAX, dt):
        group_dict[f"depart_t={t}"] = [veh for veh in W.VEHICLES.values() if t <= veh.departure_time_in_second < t+dt]

    df = W.analyzer.vehicle_groups_to_pandas(list(group_dict.values()), list(group_dict.keys()))
    print(df)

    assert df["average_travel_time"][df["group"]=="depart_t=0"].values[0] < df["average_travel_time"][df["group"]=="depart_t=1500"].values[0]
    assert df["average_delay_ratio"][df["group"]=="depart_t=0"].values[0] < df["average_delay_ratio"][df["group"]=="depart_t=1500"].values[0]
    assert df["average_traveled_distance"][df["group"]=="depart_t=0"].values[0] < df["average_traveled_distance"][df["group"]=="depart_t=1500"].values[0]
    assert df["average_detour_ratio"][df["group"]=="depart_t=0"].values[0] < df["average_detour_ratio"][df["group"]=="depart_t=1500"].values[0]
    assert df["average_speed"][df["group"]=="depart_t=0"].values[0] > df["average_speed"][df["group"]=="depart_t=1500"].values[0]

def test_change_jam_density():
    W = World(
        name="",
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=0, show_mode=1,
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid1", 0, 2)
    W.addNode("mid2", 1, 1, signal=[240, 240])
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig", "mid1", length=1000, free_flow_speed=20, number_of_lanes=1)
    link2 = W.addLink("link2", "mid1", "mid2", length=400, free_flow_speed=20, jam_density=0.2, number_of_lanes=1, signal_group=0)
    W.addLink("link3", "mid2", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.adddemand("orig", "dest", 0, 1200, 0.4)

    check_flag = 0
    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t2=30)
        print(W.TIME, link2.delta_per_lane, link2.num_vehicles, link2.num_vehicles_queue)
        if W.TIME > 600:
            link2.change_jam_density(0.1)
            
        if W.TIME == 180:
            assert equal_tolerance(link2.num_vehicles, 10)
            assert equal_tolerance(link2.num_vehicles_queue, 0)
            check_flag += 1
        if W.TIME == 450:
            assert equal_tolerance(link2.num_vehicles, 400*0.2)
            assert equal_tolerance(link2.num_vehicles_queue, 400*0.2)
            check_flag += 1
        if W.TIME == 540:
            assert equal_tolerance(link2.num_vehicles, 35)
            assert equal_tolerance(link2.num_vehicles_queue, 25)
            check_flag += 1
        if W.TIME == 900:
            assert equal_tolerance(link2.num_vehicles, 400*0.1)
            assert equal_tolerance(link2.num_vehicles_queue, 400*0.1)
            check_flag += 1

    assert check_flag == 4

    W.analyzer.print_simple_stats()

    #W.analyzer.time_space_diagram_traj_links(["link1","link2","link3"])

def test_user_functions():
    # define custom user_functions

    def world_user_function(W):
        #print the current time and the average speed of traveling vehicles in the World
        if W.T%20 == 0:
            if len(W.VEHICLES_RUNNING):
                print("World state:", W.TIME, np.average([veh.v for veh in W.VEHICLES_RUNNING.values()]))
            else:
                print("World state:", W.TIME)

    def node_user_function(node):
        #print the numnber of incoming vehicles to node
        if node.W.T%20 == 0:
            print(f"Node {node.name} state:", len(node.incoming_vehicles))

    def link_user_function(link):
        #print the numnber of vehicles on link and their average speed, and record the speed
        if link.W.T%20 == 0:
            if len(link.vehicles):
                print(f"Link {link.name} state:", len(link.vehicles), np.average([veh.v for veh in link.vehicles]))
                link.user_attribute["speed_record"].append(np.average([veh.v for veh in link.vehicles]))
            else:
                print(f"Link {link.name} state:", len(link.vehicles))


    def veh_user_function(veh):
        #print when the vehicle stated or ended its trip
        if veh.state != "home" and veh.user_attrubute["state_old"] == "home":
            print(f"Vehicle {veh.name} started trip on time {veh.W.TIME}")        
        if veh.state in ["end", "abort"] and veh.user_attrubute["state_old"] not in ["end", "abort"]:
            print(f"Vehicle {veh.name} ended trip on time {veh.W.TIME}")
        veh.user_attrubute["state_old"] = veh.state

    # define uxsim scenario

    W = World(
        name="",
        deltan=5,
        tmax=1200, 
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=0,
        user_function=world_user_function
    )

    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1, user_function=node_user_function)
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, number_of_lanes=1, user_function=link_user_function, user_attribute={"speed_record":[]})
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.adddemand("orig1", "dest", 0, 1000, 0.45)
    W.adddemand("orig2", "dest", 400, 1000, 0.6)

    W.VEHICLES["10"].user_attrubute = {"state_old":None}
    W.VEHICLES["10"].user_function = veh_user_function
    W.VEHICLES["120"].user_attrubute = {"state_old":None}
    W.VEHICLES["120"].user_function = veh_user_function

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    # prosess data collected by user_function

    link1 = W.get_link("link1")
    print("average speed of link1:", np.nanmean(link1.user_attribute["speed_record"]))

    assert equal_tolerance(np.nanmean(link1.user_attribute["speed_record"]), 15.89)

def test_reduce_memory_delete_vehicle_route_pref():
    W = World(
        name="",
        deltan=10,
        tmax=3000,
        print_mode=1, save_mode=1, show_mode=0,
        reduce_memory_delete_vehicle_route_pref=False,
        random_seed=0,
    )

    n_nodes = 4
    imax = n_nodes
    jmax = n_nodes
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)


    area_dict = {
        "areaN": [nodes[0,i] for i in range(n_nodes)],
        "areaS": [nodes[n_nodes-1,i] for i in range(n_nodes)],
        "areaNW": [nodes[0,0]],
        "areaSE": [nodes[n_nodes-1, n_nodes-1]]
    }

    W.adddemand_nodes2nodes(area_dict["areaN"], area_dict["areaS"], 0, 3000, volume=7000)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    df1 = W.analyzer.area_to_pandas(list(area_dict.values()), list(area_dict.keys()), border_include=True)
    print(df1)

    W = World(
        name="",
        deltan=10,
        tmax=3000,
        print_mode=1, save_mode=1, show_mode=0,
        reduce_memory_delete_vehicle_route_pref=True,
        random_seed=0,
    )

    n_nodes = 4
    imax = n_nodes
    jmax = n_nodes
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000)


    area_dict = {
        "areaN": [nodes[0,i] for i in range(n_nodes)],
        "areaS": [nodes[n_nodes-1,i] for i in range(n_nodes)],
        "areaNW": [nodes[0,0]],
        "areaSE": [nodes[n_nodes-1, n_nodes-1]]
    }

    W.adddemand_nodes2nodes(area_dict["areaN"], area_dict["areaS"], 0, 3000, volume=7000)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    df2 = W.analyzer.area_to_pandas(list(area_dict.values()), list(area_dict.keys()), border_include=True)
    print(df2)

    assert df1["total_travel_time"][0] == df2["total_travel_time"][0]

def test_route_definition():
    
    import matplotlib
    matplotlib.use('Agg')
    
    W = World(
        name="",
        deltan=5,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=42
    )

    # scenario
    W.addNode("0orig", 0, 0)
    W.addNode("1orig", 0, 0)
    W.addNode("2dummy", 2, 0)
    W.addNode("3dummy", 2.2, 0)
    W.addNode("4dest", 3, 0)
    W.addLink("link01", "0orig", "1orig", length=500, free_flow_speed=20, jam_density=0.2)
    W.addLink("link12", "1orig", "2dummy", length=2000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link24", "2dummy", "4dest", length=2000, free_flow_speed=20, jam_density=0.2, capacity_in=0.6)
    W.addLink("link13", "1orig", "3dummy", length=2000, free_flow_speed=15, jam_density=0.2)
    W.addLink("link34", "3dummy", "4dest", length=2000, free_flow_speed=15, jam_density=0.2)
    W.adddemand("0orig", "4dest", 0, 3000, 0.51111111)
    W.adddemand("0orig", "4dest", 1800, 3000, 0.5)


    r1 = W.defRoute(["link01", "link12", "link24"])
    r2 = W.defRoute(["link01", "link13", "link34"])

    # simulation
    W.exec_simulation()

    # results
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links(r1.links)
    W.analyzer.time_space_diagram_traj_links(r2.links)

    assert True

def test_route_actual_travel_time():
    W = World(
        name="",
        deltan=5,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=42
    )

    # scenario
    W.addNode("0orig", 0, 0)
    W.addNode("1orig", 0, 0)
    W.addNode("2dummy", 2, 0)
    W.addNode("3dummy", 2.2, 0)
    W.addNode("4dest", 3, 0)
    W.addLink("link01", "0orig", "1orig", length=500, free_flow_speed=20, jam_density=0.2)
    W.addLink("link12", "1orig", "2dummy", length=2000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link24", "2dummy", "4dest", length=2000, free_flow_speed=20, jam_density=0.2, capacity_in=0.6)
    W.addLink("link13", "1orig", "3dummy", length=2000, free_flow_speed=15, jam_density=0.2)
    W.addLink("link34", "3dummy", "4dest", length=2000, free_flow_speed=15, jam_density=0.2)
    W.adddemand("0orig", "4dest", 0, 3000, 0.51111111)
    W.adddemand("0orig", "4dest", 1800, 3000, 0.5)


    r1 = W.defRoute(["link01", "link12", "link24"])
    r2 = W.defRoute(["link01", "link13", "link34"])

    # simulation
    W.exec_simulation()

    # results
    W.analyzer.print_simple_stats()

    assert equal_tolerance(r1.actual_travel_time(500), (500+2000+2000)/20)
    assert equal_tolerance(r2.actual_travel_time(500), 500/20+(2000+2000)/15)

    assert equal_tolerance(r1.actual_travel_time(2500), 350)
    assert equal_tolerance(r2.actual_travel_time(2500), 300)

    
def test_route_vehicle_methods():
    W = World(
        name="",
        deltan=5,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=42
    )

    # scenario
    W.addNode("0orig", 0, 0)
    W.addNode("1orig", 0, 0)
    W.addNode("2dummy", 2, 0)
    W.addNode("3dummy", 2.2, 0)
    W.addNode("4dest", 3, 0)
    W.addLink("link01", "0orig", "1orig", length=500, free_flow_speed=20, jam_density=0.2)
    W.addLink("link12", "1orig", "2dummy", length=2000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link24", "2dummy", "4dest", length=2000, free_flow_speed=20, jam_density=0.2, capacity_in=0.6)
    W.addLink("link13", "1orig", "3dummy", length=2000, free_flow_speed=15, jam_density=0.2)
    W.addLink("link34", "3dummy", "4dest", length=2000, free_flow_speed=15, jam_density=0.2)
    W.adddemand("0orig", "4dest", 0, 3000, 0.51111111)
    W.adddemand("0orig", "4dest", 1800, 3000, 0.5)


    r1 = W.defRoute(["link01", "link12", "link24"])
    r2 = W.defRoute(["link01", "link13", "link34"])

    # simulation
    W.exec_simulation()

    # results
    W.analyzer.print_simple_stats()

    assert W.VEHICLES["0"].traveled_route()[0] == r1
    assert W.VEHICLES["0"].traveled_route()[0] != r2
    
    assert W.VEHICLES["300"].traveled_route()[0] == r2
    assert W.VEHICLES["300"].traveled_route()[1][0] == 3260
    assert W.VEHICLES["300"].traveled_route()[1][-1] == 3550

    tt_from_vehicle_route = W.VEHICLES["300"].traveled_route()[1][-1]-W.VEHICLES["300"].traveled_route()[1][0]
    tt_from_route_by_departure_time = W.VEHICLES["300"].traveled_route()[0].actual_travel_time(W.VEHICLES["300"].traveled_route()[1][0])
    assert equal_tolerance(tt_from_vehicle_route, tt_from_route_by_departure_time)


def test_route_enforce_route_old():
    W = World(
        name="",
        deltan=5,
        tmax=4000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=42
    )

    # scenario
    W.addNode("0orig", 0, 0)
    W.addNode("1orig", 0, 0)
    W.addNode("2dummy", 2, 0)
    W.addNode("3dummy", 2.2, 0)
    W.addNode("4dest", 3, 0)
    W.addLink("link01", "0orig", "1orig", length=500, free_flow_speed=20, jam_density=0.2)
    W.addLink("link12", "1orig", "2dummy", length=2000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link24", "2dummy", "4dest", length=2000, free_flow_speed=20, jam_density=0.2, capacity_in=0.6)
    W.addLink("link13", "1orig", "3dummy", length=2000, free_flow_speed=15, jam_density=0.2)
    W.addLink("link34", "3dummy", "4dest", length=2000, free_flow_speed=15, jam_density=0.2)
    W.adddemand("0orig", "4dest", 0, 3000, 0.51111111)
    W.adddemand("0orig", "4dest", 1800, 3000, 0.5)


    r1 = W.defRoute(["link01", "link12", "link24"])
    r2 = W.defRoute(["link01", "link13", "link34"])

    for veh in W.VEHICLES.values():
        veh.enforce_route(r2)
    
    # simulation
    W.exec_simulation()

    # results
    W.analyzer.print_simple_stats()

    df = W.analyzer.link_to_pandas()
    for l in r2:
        assert df[df["link"]==l.name]["traffic_volume"].values[0] == 2130

def test_route_enforce_route_by_route_object():

    W = World(
        name="looproute",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=2400,  # Total simulation time (s)
        print_mode=1, save_mode=0, show_mode=1,    # Various options
        random_seed=0    # Set the random seed
    )

    # Define the scenario
    ## Create nodes
    W.addNode(name="O", x=0, y=0)
    W.addNode("D", 2, 0)
    W.addNode("A", 1, 0)
    W.addNode("B", 1.5, 1)
    W.addNode("C", 0.5, 1)
    W.addNode("E", 1, -1)
    ## Create links between nodes
    W.addLink(None, "O", "A", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "A", "D", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "A", "B", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "B", "C", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "C", "A", length=500, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "O", "E", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "E", "D", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink(None, "O", "D", length=500, free_flow_speed=40, number_of_lanes=1)

    ## Create OD traffic demand between nodes
    W.adddemand("O","D", t_start=0, t_end=1000, volume=400)

    #W.show_network()

    route_straight = W.defRoute(["O-A", "A-D"])
    route_detour_loop = W.defRoute(["O-A", "A-B", "B-C", "C-A", "A-D"])
    route_detour_loop_triple = W.defRoute(["O-A", "A-B", "B-C", "C-A", "A-B", "B-C", "C-A", "A-B", "B-C", "C-A", "A-D"])
    route_detour = W.defRoute(["O-E", "E-D"])

    for i,veh in enumerate(W.VEHICLES.values()):
        if i%4 == 0:
            veh.enforce_route(route_straight)
        elif i%4 == 1:
            veh.enforce_route(route_detour)
        elif i%4 == 2:
            veh.enforce_route(route_detour_loop)
        else:
            veh.enforce_route(route_detour_loop_triple)

    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()

    W.analyzer.network_average()
    df = W.analyzer.link_to_pandas()
    print(df)
    
    assert df.query("link == 'O-A'")["traffic_volume"].item() == 300
    assert df.query("link == 'A-D'")["traffic_volume"].item() == 300
    assert df.query("link == 'A-B'")["traffic_volume"].item() == 400
    assert df.query("link == 'B-C'")["traffic_volume"].item() == 400
    assert df.query("link == 'C-A'")["traffic_volume"].item() == 400
    assert df.query("link == 'O-E'")["traffic_volume"].item() == 100
    assert df.query("link == 'E-D'")["traffic_volume"].item() == 100
    assert df.query("link == 'O-D'")["traffic_volume"].item() == 0

    assert W.VEHICLES["0"].traveled_route()[0] == route_straight
    assert W.VEHICLES["1"].traveled_route()[0] == route_detour
    assert W.VEHICLES["2"].traveled_route()[0] == route_detour_loop
    assert W.VEHICLES["3"].traveled_route()[0] == route_detour_loop_triple

def test_construct_time_space_network():
    W = World(
        name="",
        deltan=20,
        tmax=6000,
        print_mode=1, save_mode=1, show_mode=1,
        vehicle_logging_timestep_interval=1, 
        hard_deterministic_mode=False,
        random_seed=42    #fix seed to reproduce random demand 
    )

    W.addNode("1", 0, 1)
    W.addNode("2", 1, 1)
    W.addNode("3", 5, 1)
    W.addNode("4", 0, 0)
    W.addNode("5", 1, 0)
    W.addNode("6", 5, 0)
    W.addNode("7", 6, 0.5)

    W.addLink("highway12", "1", "2", length=1000, number_of_lanes=1, merge_priority=1)
    W.addLink("highway23", "2", "3", length=3000, number_of_lanes=1, merge_priority=1, capacity_out=0.6)
    W.addLink("highway37", "3", "7", length=1000, number_of_lanes=1, merge_priority=1)
    W.addLink("onramp", "5", "2", length=1000, number_of_lanes=1, merge_priority=0.5)
    W.addLink("arterial45", "4", "5", length=1000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)
    W.addLink("arterial56", "5", "6", length=3000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)
    W.addLink("arterial67", "6", "7", length=1000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)

    W.adddemand("1", "7", 0, 3000, 0.3)
    W.adddemand("4", "7", 0, 3000, 0.4*3)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    construct_time_space_network(W)

    assert W.TSN_paths["4", 0]["7", "end"][-2] == ('7', 340)
    assert equal_tolerance(W.TSN_costs["4", 0]["7", "end"], W.TSN_paths["4", 0]["7", "end"][-2][1])

@pytest.mark.flaky(reruns=10)
def test_estimate_congestion_externality_1link():
    def create_world(seed):
        W = World(
            name="aaa",    # Scenario name
            deltan=10,   # Simulation aggregation unit delta n
            tmax=2400,  # Total simulation time (s)
            print_mode=0, save_mode=0, show_mode=0,    # Various options
            random_seed=seed    # Set the random seed
        )

        # Define the scenario
        ## Create nodes
        W.addNode(name="orig1", x=0, y=0)
        W.addNode("mid", 1, 1)
        W.addNode("dest", 2, 1)
        ## Create links between nodes
        link1 = W.addLink(name="link1", start_node="orig1", end_node="mid", length=2000, free_flow_speed=20, number_of_lanes=1, capacity_out=0.8)
        link2 = W.addLink("link3", "mid", "dest", length=1000, free_flow_speed=20, number_of_lanes=1, capacity_in=0.4)

        R1 = uxsim.Route(W, [link1, link2])
        ## Create OD traffic demand between nodes
        W.adddemand(orig="orig1", dest="dest", t_start=0, t_end=1200, flow=0.2)
        W.adddemand(orig="orig1", dest="dest", t_start=300, t_end=600, flow=0.4)

        return W

    dep_ts = []
    acts = []
    ests = []
    for _ in range(50):
        seed = random.randint(0,999999)

        W = create_world(seed)
        W.exec_simulation()
        W.analyzer.print_simple_stats()
        #W.analyzer.time_space_diagram_traj_links(["link1", "link3"])

        key = random.choice(list(W.VEHICLES.keys()))
        veh = W.VEHICLES[key]
        veh_tt = veh.travel_time*W.DELTAN
        route = veh.traveled_route()[0]
        dep_t = veh.log_t_link[1][0]
        ext = estimate_congestion_externality_route(W, route, dep_t)
        ttt1 = W.analyzer.total_travel_time


        W2 = create_world(seed)
        W2.VEHICLES[key].state="abort"
        W2.exec_simulation()
        W2.analyzer.print_simple_stats()
        #W2.analyzer.time_space_diagram_traj_links(["link1", "link3"])
        ttt2 = W2.analyzer.total_travel_time

        #print(f"selected vehicle - dep time {veh.departure_time_in_second}")
        #print("actual ext:", ttt1-ttt2-veh_tt, f"{ttt1}-{ttt2}-{veh_tt}")
        #print("esti. ext: ", ext)

        dep_ts.append(veh.departure_time_in_second)
        acts.append(ttt1-ttt2-veh_tt)
        ests.append(ext)

    # W.analyzer.time_space_diagram_traj_links(["link1", "link3"])

    # figure()
    # plot(dep_ts, acts, "o", label="actual")
    # plot(dep_ts, ests, "x", label="estimated")
    # legend()
    # grid()

    assert uxsim.equal_tolerance(average(acts), average(ests), rel_tol=0.333)