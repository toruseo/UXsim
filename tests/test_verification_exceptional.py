"""
This script verifies whether UXsim outputs plausible solutions for exceptional or uncommon situations.
The behavior of UXsim may be updated in the future.

It also tests other functions that are difficult to categorize to the other scripts.
"""

import pytest
from uxsim import *

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

"""
default FD:
    u = 20
    kappa = 0.2
    tau = 1
    w = 5
    k^* = 0.04
    q^* = 0.8
"""

def test_cannot_reach_destination_no_deadend():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=0, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("A", 0, 0) 
    W.addNode("B", 1, 0)
    W.addNode("dest", -1, 0)
    W.addLink("link1", "A", "B", length=1000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link2", "B", "A", length=1000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link3", "dest", "A", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("A", "dest", 0, 500, volume=20)

    W.exec_simulation()

    df = W.analyzer.basic_to_pandas()
    assert equal_tolerance(df["total_trips"].values[0], 20)
    assert equal_tolerance(df["completed_trips"].values[0], 0)
    assert equal_tolerance(sum([W.DELTAN for veh in W.VEHICLES.values() if veh.state=="run"]), 20)

@pytest.mark.flaky(reruns=5)
def test_cannot_reach_destination_with_deadend():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=0, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("A", 0, 0) 
    W.addNode("B", 1, 0)
    W.addNode("C", 2, 0)
    W.addNode("dest", -1, 0)
    W.addLink("link1", "A", "B", length=1000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link4", "A", "C", length=1000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link2", "B", "A", length=1000, free_flow_speed=20, jam_density=0.2)
    W.addLink("link3", "dest", "A", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("A", "dest", 0, 500, volume=20)

    W.exec_simulation()

    df = W.analyzer.basic_to_pandas()
    assert equal_tolerance(df["total_trips"].values[0], 20)
    assert equal_tolerance(df["completed_trips"].values[0], 0)
    assert equal_tolerance(sum([W.DELTAN for veh in W.VEHICLES.values() if veh.state=="abort"]), 20)

def test_too_many_vehicles_in_vertical_queue():
    W = World(
        name="",
        deltan=5,
        tmax=500,
        print_mode=0, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("A", 0, 0) 
    W.addNode("B", 1, 0)
    W.addLink("link1", "A", "B", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("A", "B", 0, 500, volume=1000)

    W.exec_simulation()

    df = W.analyzer.basic_to_pandas()
    assert equal_tolerance(df["total_trips"].values[0], 1000)
    assert equal_tolerance(df["completed_trips"].values[0], 500*0.8-1000/20)
    assert equal_tolerance(sum([W.DELTAN for veh in W.VEHICLES.values() if veh.state=="run"]), 1000*0.04)
    assert equal_tolerance(sum([W.DELTAN for veh in W.VEHICLES.values() if veh.state=="wait"]), 1000-(500*0.8-1000/20)-(1000*0.04))
    assert equal_tolerance(df["total_travel_time"].values[0], 67050)

def test_random_numbers_are_reproducible_by_fixing_random_seeds():
    ttt = {}
    for itr in range(2):
        print(itr, "========="*3)
        W = World(
            name="",
            deltan=10,
            tmax=3600,
            print_mode=1, save_mode=1, show_mode=0,
            random_seed=42
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

        od_pairs = [
            (f"n(0, 0)", f"n({n_nodes-1}, {n_nodes-1})"),
            (f"n({n_nodes-1}, 0)", f"n(0, {n_nodes-1})"),
            (f"n(0, {n_nodes-1})", f"n({n_nodes-1}, 0)"),
            (f"n({n_nodes-1}, {n_nodes-1})", f"n(0, 0)"),
        ]
        for od_pair in od_pairs:
            W.adddemand(od_pair[0], od_pair[1], 0, 3000, 0.6+W.rng.random()*0.2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        ttt[itr] = W.analyzer.basic_to_pandas()["total_travel_time"][0]

    assert ttt[0] == ttt[1]

def test_hard_deterministic_mode_merge():

    ########################### ITER 1 ###########################

    W = World(
        name="",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=1,    # Various options
        random_seed=None,    # Set the random seed
        hard_deterministic_mode=True
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


    df = W.analyzer.link_to_pandas()

    average_tt_link1 = df["average_travel_time"][df["link"]=="link1"].values[0]
    average_tt_link2 = df["average_travel_time"][df["link"]=="link2"].values[0]

    assert equal_tolerance(average_tt_link1, 50)
    assert equal_tolerance(average_tt_link2, 150)


    ########################### ITER 2 ###########################

    W = World(
        name="",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=1,    # Various options
        random_seed=None,    # Set the random seed
        hard_deterministic_mode=True
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

    assert average_tt_link1 == df["average_travel_time"][df["link"]=="link1"].values[0]
    assert average_tt_link2 == df["average_travel_time"][df["link"]=="link2"].values[0]


def test_hard_deterministic_mode_grid_network():
    ttt = {}
    for itr in range(2):
        print(itr, "========="*3)
        W = World(
            name="",
            deltan=10,
            tmax=3600,
            print_mode=1, save_mode=1, show_mode=0,
            random_seed=None,
            hard_deterministic_mode=True
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

        od_pairs = [
            (f"n(0, 0)", f"n({n_nodes-1}, {n_nodes-1})"),
            (f"n({n_nodes-1}, 0)", f"n(0, {n_nodes-1})"),
            (f"n(0, {n_nodes-1})", f"n({n_nodes-1}, 0)"),
            (f"n({n_nodes-1}, {n_nodes-1})", f"n(0, 0)"),
        ]
        for od_pair in od_pairs:
            W.adddemand(od_pair[0], od_pair[1], 0, 3000, 0.6)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        ttt[itr] = W.analyzer.basic_to_pandas()["total_travel_time"][0]

    assert ttt[0] == ttt[1]