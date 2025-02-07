"""
This script verifies whether UXsim outputs reasonable solutions for a straight road in various configurations.
"""

import pytest
import random
from uxsim import *

"""
default FD:
    u = 20
    kappa = 0.2
    tau = 1
    w = 5
    k^* = 0.04
    q^* = 0.8
"""

def test_1link():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 1, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.5)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 250)
    assert equal_tolerance(W.analyzer.trip_completed, 250)
    assert equal_tolerance(W.analyzer.total_travel_time, 12500)
    assert equal_tolerance(W.analyzer.average_travel_time, 50)
    assert equal_tolerance(W.analyzer.average_delay, 0)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link.q_mat[2, 5], 0.5)
    assert equal_tolerance(link.q_mat[7, 5], 0)
    assert equal_tolerance(link.k_mat[2, 5], 0.025)
    assert equal_tolerance(link.k_mat[7, 5], 0)
    assert equal_tolerance(link.v_mat[2, 5], 20)
    assert equal_tolerance(link.v_mat[7, 5], 20)

def test_1link_demand_by_volume():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 1, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, volume=0.5*500)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 250)
    assert equal_tolerance(W.analyzer.trip_completed, 250)
    assert equal_tolerance(W.analyzer.total_travel_time, 12500)
    assert equal_tolerance(W.analyzer.average_travel_time, 50)
    assert equal_tolerance(W.analyzer.average_delay, 0)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link.q_mat[2, 5], 0.5)
    assert equal_tolerance(link.q_mat[7, 5], 0)
    assert equal_tolerance(link.k_mat[2, 5], 0.025)
    assert equal_tolerance(link.k_mat[7, 5], 0)
    assert equal_tolerance(link.v_mat[2, 5], 20)
    assert equal_tolerance(link.v_mat[7, 5], 20)

def test_1link_iterative_exec():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 1, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.5)

    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t=random.randint(50, 200)) 

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 250)
    assert equal_tolerance(W.analyzer.trip_completed, 250)
    assert equal_tolerance(W.analyzer.total_travel_time, 12500)
    assert equal_tolerance(W.analyzer.average_travel_time, 50)
    assert equal_tolerance(W.analyzer.average_delay, 0)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link.q_mat[2, 5], 0.5)
    assert equal_tolerance(link.q_mat[7, 5], 0)
    assert equal_tolerance(link.k_mat[2, 5], 0.025)
    assert equal_tolerance(link.k_mat[7, 5], 0)
    assert equal_tolerance(link.v_mat[2, 5], 20)
    assert equal_tolerance(link.v_mat[7, 5], 20)

def test_1link_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 1, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.5)

    W.exec_simulation()

    W.analyzer.print_simple_stats()


    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 250)
    assert equal_tolerance(W.analyzer.trip_completed, 250)
    assert equal_tolerance(W.analyzer.total_travel_time, 12500)
    assert equal_tolerance(W.analyzer.average_travel_time, 50)
    assert equal_tolerance(W.analyzer.average_delay, 0)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link.q_mat[2, 5], 0.5)
    assert equal_tolerance(link.q_mat[7, 5], 0)
    assert equal_tolerance(link.k_mat[2, 5], 0.025)
    assert equal_tolerance(link.k_mat[7, 5], 0)
    assert equal_tolerance(link.v_mat[2, 5], 20)
    assert equal_tolerance(link.v_mat[7, 5], 20)

def test_1link_maxflow():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 1, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 2000, 2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link.q_mat[2, 5], 0.8)
    assert equal_tolerance(link.q_mat[7, 5], 0.8)
    assert equal_tolerance(link.k_mat[2, 5], 0.04)
    assert equal_tolerance(link.k_mat[7, 5], 0.04)
    assert equal_tolerance(link.v_mat[2, 5], 20)
    assert equal_tolerance(link.v_mat[7, 5], 20)    

def test_1link_maxflow_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 1, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 2000, 2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link.q_mat[2, 5], 0.8)
    assert equal_tolerance(link.q_mat[7, 5], 0.8)
    assert equal_tolerance(link.k_mat[2, 5], 0.04)
    assert equal_tolerance(link.k_mat[7, 5], 0.04)
    assert equal_tolerance(link.v_mat[2, 5], 20)
    assert equal_tolerance(link.v_mat[7, 5], 20) 

def test_2link():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.5)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 250)
    assert equal_tolerance(W.analyzer.trip_completed, 250)
    assert equal_tolerance(W.analyzer.total_travel_time, 37500)
    assert equal_tolerance(W.analyzer.average_travel_time, 150)
    assert equal_tolerance(W.analyzer.average_delay, 0)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.5)
    assert equal_tolerance(link1.q_mat[7, 5], 0)
    assert equal_tolerance(link1.k_mat[2, 5], 0.025)
    assert equal_tolerance(link1.k_mat[7, 5], 0)
    assert equal_tolerance(link1.v_mat[2, 5], 20)
    assert equal_tolerance(link1.v_mat[7, 5], 20)
    assert equal_tolerance(link2.q_mat[2, 5], 0.5)
    assert equal_tolerance(link2.q_mat[7, 5], 0)
    assert equal_tolerance(link2.k_mat[2, 5], 0.05)
    assert equal_tolerance(link2.k_mat[7, 5], 0)
    assert equal_tolerance(link2.v_mat[2, 5], 10)
    assert equal_tolerance(link2.v_mat[7, 5], 10)

def test_2link_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.5)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 250)
    assert equal_tolerance(W.analyzer.trip_completed, 250)
    assert equal_tolerance(W.analyzer.total_travel_time, 37500)
    assert equal_tolerance(W.analyzer.average_travel_time, 150)
    assert equal_tolerance(W.analyzer.average_delay, 0)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.5)
    assert equal_tolerance(link1.q_mat[7, 5], 0)
    assert equal_tolerance(link1.k_mat[2, 5], 0.025)
    assert equal_tolerance(link1.k_mat[7, 5], 0)
    assert equal_tolerance(link1.v_mat[2, 5], 20)
    assert equal_tolerance(link1.v_mat[7, 5], 20)
    assert equal_tolerance(link2.q_mat[2, 5], 0.5)
    assert equal_tolerance(link2.q_mat[7, 5], 0)
    assert equal_tolerance(link2.k_mat[2, 5], 0.05)
    assert equal_tolerance(link2.k_mat[7, 5], 0)
    assert equal_tolerance(link2.v_mat[2, 5], 10)
    assert equal_tolerance(link2.v_mat[7, 5], 10)

def test_2link_maxflow():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 2000, 2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.8)
    assert equal_tolerance(link1.q_mat[7, 5], 0.8)
    assert equal_tolerance(link1.k_mat[2, 5], 0.04)
    assert equal_tolerance(link1.k_mat[7, 5], 0.04)
    assert equal_tolerance(link1.v_mat[2, 5], 20)
    assert equal_tolerance(link1.v_mat[7, 5], 20)
    assert equal_tolerance(link2.q_mat[2, 5], 0.8)
    assert equal_tolerance(link2.q_mat[7, 5], 0.8)
    assert equal_tolerance(link2.k_mat[2, 5], 0.04)
    assert equal_tolerance(link2.k_mat[7, 5], 0.04)
    assert equal_tolerance(link2.v_mat[2, 5], 20)
    assert equal_tolerance(link2.v_mat[7, 5], 20)

def test_2link_maxflow_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 2000, 2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.8)
    assert equal_tolerance(link1.q_mat[7, 5], 0.8)
    assert equal_tolerance(link1.k_mat[2, 5], 0.04)
    assert equal_tolerance(link1.k_mat[7, 5], 0.04)
    assert equal_tolerance(link1.v_mat[2, 5], 20)
    assert equal_tolerance(link1.v_mat[7, 5], 20)
    assert equal_tolerance(link2.q_mat[2, 5], 0.8)
    assert equal_tolerance(link2.q_mat[7, 5], 0.8)
    assert equal_tolerance(link2.k_mat[2, 5], 0.04)
    assert equal_tolerance(link2.k_mat[7, 5], 0.04)
    assert equal_tolerance(link2.v_mat[2, 5], 20)
    assert equal_tolerance(link2.v_mat[7, 5], 20)

def test_2link_bottleneck_due_to_free_flow_speed():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 146000.0)
    assert equal_tolerance(W.analyzer.average_travel_time, 182.5)
    assert equal_tolerance(W.analyzer.average_delay, 32.5)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.06667) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 10) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.04) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 10) #freeflow

def test_2link_bottleneck_due_to_free_flow_speed_deltan1():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 146000.0)
    assert equal_tolerance(W.analyzer.average_travel_time, 182.5)
    assert equal_tolerance(W.analyzer.average_delay, 32.5)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.06667) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 10) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.04) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 10) #freeflow

def test_2link_bottleneck_due_to_capacity_out():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.66666)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 104775)
    assert equal_tolerance(W.analyzer.average_travel_time, 131)
    assert equal_tolerance(W.analyzer.average_delay, 31)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.03333) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 20) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.02) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 20) #freeflow

def test_2link_bottleneck_due_to_capacity_out_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.66666)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 104775)
    assert equal_tolerance(W.analyzer.average_travel_time, 131)
    assert equal_tolerance(W.analyzer.average_delay, 31)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.03333) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 20) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.02) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 20) #freeflow

def test_2link_bottleneck_due_to_jam_density():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.1)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 106000)
    assert equal_tolerance(W.analyzer.average_travel_time, 132.5)
    assert equal_tolerance(W.analyzer.average_delay, 32.5)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.03333) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 20) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.02) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 20) #freeflow

def test_2link_bottleneck_due_to_node_capacity():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1, flow_capacity=0.666)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 106000)
    assert equal_tolerance(W.analyzer.average_travel_time, 132.5)
    assert equal_tolerance(W.analyzer.average_delay, 32.5)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.03333) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 20) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.02) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 20) #freeflow

def test_2link_bottleneck_due_to_node_capacity_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1, flow_capacity=0.666)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.8)
    W.adddemand("orig", "dest", 500, 1500, 0.4)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 800)
    assert equal_tolerance(W.analyzer.trip_completed, 800)
    assert equal_tolerance(W.analyzer.total_travel_time, 106000)
    assert equal_tolerance(W.analyzer.average_travel_time, 132.5)
    assert equal_tolerance(W.analyzer.average_delay, 32.5)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.06667) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 10) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.4) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.02) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.6667) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.03333) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 20) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.4) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.02) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 20) #freeflow

def test_2link_leave_at_middle():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1)
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "mid", 0, 500, 0.666)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 330)
    assert equal_tolerance(W.analyzer.trip_completed, 330)
    assert equal_tolerance(W.analyzer.total_travel_time, 17300)
    assert equal_tolerance(W.analyzer.average_travel_time, 52.4)
    assert equal_tolerance(W.analyzer.average_delay, 2.4)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[2, 5], 0.6667) #congestion before BN
    assert equal_tolerance(link1.k_mat[2, 5], 0.03333) #congestion before BN
    assert equal_tolerance(link1.v_mat[2, 5], 20) #congestion before BN
    assert equal_tolerance(link1.q_mat[7, 5], 0.0) #freeflow
    assert equal_tolerance(link1.k_mat[7, 5], 0.0) #freeflow
    assert equal_tolerance(link1.v_mat[7, 5], 20) #freeflow
    assert equal_tolerance(link2.q_mat[2, 5], 0.0) #freeflow after BN
    assert equal_tolerance(link2.k_mat[2, 5], 0.0) #freeflow after BN
    assert equal_tolerance(link2.v_mat[2, 5], 20) #freeflow after BN
    assert equal_tolerance(link2.q_mat[8, 5], 0.0) #freeflow
    assert equal_tolerance(link2.k_mat[8, 5], 0.0) #freeflow 
    assert equal_tolerance(link2.v_mat[8, 5], 20) #freeflow

def test_3link_queuespillback():

    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid1", 1, 1)
    W.addNode("mid2", 2, 2)
    W.addNode("dest", 3, 3)
    link1 = W.addLink("link1", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid1", "mid2", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.4)
    link3 = W.addLink("link3", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    t1 = 400
    t2 = 800
    W.adddemand("orig", "dest", 0, t1, 0.8)
    W.adddemand("orig", "dest", t1, t2, 0.4)
    W.adddemand("orig", "dest", t2, 2000, 0.1)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()

    assert equal_tolerance(W.analyzer.trip_all, 600)
    assert equal_tolerance(W.analyzer.trip_completed, 585)
    assert equal_tolerance(W.analyzer.total_travel_time, 221050)
    assert equal_tolerance(W.analyzer.average_travel_time, 378)
    assert equal_tolerance(W.analyzer.average_delay, 228)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[1, 8], 0.8, rel_tol=0.2) #before congestion
    assert equal_tolerance(link1.k_mat[1, 8], 0.04, rel_tol=0.2) #before congestion
    assert equal_tolerance(link1.v_mat[1, 8], 20, rel_tol=0.2) #before congestion
    assert equal_tolerance(link1.q_mat[5, 8], 0.4, rel_tol=0.2) #during congestion
    assert equal_tolerance(link1.k_mat[5, 8], 0.12, rel_tol=0.2) #during congestion
    assert equal_tolerance(link1.v_mat[5, 8], 3.33, rel_tol=0.2) #during congestion
    assert equal_tolerance(link1.q_mat[10, 8], 0.08, rel_tol=0.2) #after congestion
    assert equal_tolerance(link1.k_mat[10, 8], 0.004, rel_tol=0.2) #after congestion
    assert equal_tolerance(link1.v_mat[10, 8], 20, rel_tol=0.2) #after congestion

    assert equal_tolerance(link2.q_mat[5, 8], 0.4, rel_tol=0.2) #during congestion
    assert equal_tolerance(link2.k_mat[5, 8], 0.12, rel_tol=0.2) #during congestion
    assert equal_tolerance(link2.v_mat[5, 8], 3.33, rel_tol=0.2) #during congestion
    assert equal_tolerance(link2.q_mat[13, 8], 0.08, rel_tol=0.2) #after congestion
    assert equal_tolerance(link2.k_mat[13, 8], 0.004, rel_tol=0.2) #after congestion 
    assert equal_tolerance(link2.v_mat[13, 8], 20, rel_tol=0.2) #after congestion

    assert equal_tolerance(link3.q_mat[5, 8], 0.4, rel_tol=0.2) #during congestion
    assert equal_tolerance(link3.k_mat[5, 8], 0.02, rel_tol=0.2) #during congestion
    assert equal_tolerance(link3.v_mat[5, 8], 20, rel_tol=0.2) #during congestion
    assert equal_tolerance(link3.q_mat[15, 8], 0.08, rel_tol=0.2) #after congestion
    assert equal_tolerance(link3.k_mat[15, 8], 0.004, rel_tol=0.2) #after congestion 
    assert equal_tolerance(link3.v_mat[15, 8], 20, rel_tol=0.2) #after congestion

def test_2link_signal():
    W = World(
        name="",
        deltan=5, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1, signal=[60,60])
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 300, 800, 0.4)
    W.adddemand("orig", "dest", 0, 2000, 0.2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 600)
    assert equal_tolerance(W.analyzer.trip_completed, 570)
    assert equal_tolerance(W.analyzer.total_travel_time, 107100)
    assert equal_tolerance(W.analyzer.average_travel_time, 188)
    assert equal_tolerance(W.analyzer.average_delay, 88)

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[:,8].mean() , 0.29947916666666663)
    assert equal_tolerance(link1.k_mat[:,8].mean() , 0.054958767361111105)
    assert equal_tolerance(link1.v_mat[:,8].mean() , 12.86046633191759)
    assert equal_tolerance(link1.q_mat[:,2].mean() , 0.3020833333333333)
    assert equal_tolerance(link1.k_mat[:,2].mean() , 0.026356336805555557)
    assert equal_tolerance(link1.v_mat[:,2].mean() , 17.49905597926791)
    assert equal_tolerance(link1.q_mat[8,:].mean() , 0.309375)
    assert equal_tolerance(link1.k_mat[8,:].mean() , 0.06661458333333334)
    assert equal_tolerance(link1.v_mat[8,:].mean() , 9.633282431298722)
    assert equal_tolerance(link1.q_mat[12,:].mean(),  0.19687499999999997)
    assert equal_tolerance(link1.k_mat[12,:].mean(),  0.011875)
    assert equal_tolerance(link1.v_mat[12,:].mean(),  18.607142857142858)
    assert equal_tolerance(link2.q_mat[:,8].mean() , 0.29427083333333337)
    assert equal_tolerance(link2.k_mat[:,8].mean() , 0.014713541666666666)
    assert equal_tolerance(link2.v_mat[:,8].mean() , 20.0)


def test_2link_signal_deltan1():
    W = World(
        name="",
        deltan=1, 
        tmax=2000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 1, signal=[60,60])
    W.addNode("dest", 2, 2)
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 300, 800, 0.4)
    W.adddemand("orig", "dest", 0, 2000, 0.2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    W.analyzer.basic_analysis()
    assert equal_tolerance(W.analyzer.trip_all, 600)
    assert equal_tolerance(W.analyzer.trip_completed, 568)
    assert equal_tolerance(W.analyzer.total_travel_time, 112348)
    assert equal_tolerance(W.analyzer.average_travel_time, 197)
    assert equal_tolerance(W.analyzer.average_delay, 97)
    
    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[:,8].mean() , 0.2994791666666667)
    assert equal_tolerance(link1.k_mat[:,8].mean() , 0.06033072916666667)
    assert equal_tolerance(link1.v_mat[:,8].mean() , 11.848820125791734)
    assert equal_tolerance(link1.q_mat[:,2].mean() , 0.3026041666666667)
    assert equal_tolerance(link1.k_mat[:,2].mean() , 0.02812717013888889)
    assert equal_tolerance(link1.v_mat[:,2].mean() , 16.587738607378817)
    assert equal_tolerance(link1.q_mat[8,:].mean() , 0.3302083333333333)
    assert equal_tolerance(link1.k_mat[8,:].mean() , 0.07739166666666668)
    assert equal_tolerance(link1.v_mat[8,:].mean() , 7.4833973250262265)
    assert equal_tolerance(link1.q_mat[12,:].mean(),  0.2)
    assert equal_tolerance(link1.k_mat[12,:].mean(),  0.013500000000000002)
    assert equal_tolerance(link1.v_mat[12,:].mean(),  18.444444444444446)
    assert equal_tolerance(link2.q_mat[:,8].mean() , 0.29427083333333337)
    assert equal_tolerance(link2.k_mat[:,8].mean() , 0.014713541666666666)
    assert equal_tolerance(link2.v_mat[:,8].mean() , 20.0)

def test_KW_theory_cumulative_curves_and_travel_time():
    W = World(
        name="",
        deltan=5, 
        tmax=3000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    W.addNode("orig", 0, 0)
    W.addNode("bottleneck", 1, 1)
    W.addNode("dest", 1, 1)
    link1 = W.addLink("link1", "orig", "bottleneck", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.5)
    link2 = W.addLink("link2", "bottleneck", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 500, 0.2)
    W.adddemand("orig", "dest", 500, 1000, 0.5)
    W.adddemand("orig", "dest", 1000, 1500, 0.6)
    W.adddemand("orig", "dest", 1500, 2000, 0.2)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    # W.analyzer.cumulative_curves(link1)

    assert equal_tolerance(link1.arrival_count(700) - link1.departure_count(700), 0.5/link1.u*link1.length)    #free-flow
    assert equal_tolerance(link1.actual_travel_time(700), link1.length/link1.u)    #free-flow
    assert link1.arrival_count(1500) - link1.departure_count(1500) > link1.k_star*link1.length    #congested
    assert link1.actual_travel_time(1500) > link1.length/link1.u    #congested
    assert equal_tolerance(link1.arrival_count(1500), 650)
    assert equal_tolerance(link1.departure_count(1500), 575)
    assert equal_tolerance(link1.arrival_count(1500), link1.departure_count(1500+link1.actual_travel_time(1500))) #congested
    for t in range(0, W.TMAX, 10):
        assert equal_tolerance(link1.arrival_count(t), link1.departure_count(t+link1.actual_travel_time(t)))

def rigorous_verification_of_KW_theory_cumulative_curves_and_travel_time(deltan):
    """
    This function is the most rigorous verification of UXsim, checking that UXsim correctly solves KW theory.
    Specifically, it checks the relation between the cumulative curves and the travel time of vehicles.
    """

    W = World(
        name="",
        deltan=deltan, 
        tmax=3000, 
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("bottleneck", 1, 1)
    W.addNode("dest", 1, 1)
    link1 = W.addLink("link1", "orig", "bottleneck", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.6)
    link2 = W.addLink("link2", "bottleneck", "dest", length=1000, free_flow_speed=20, jam_density=0.2)

    dt = 50
    for t in range(0, 2000, dt):
        W.adddemand("orig", "dest", t, t+dt, random.uniform(0,1.0))

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    #W.analyzer.cumulative_curves(link1)

    for t in range(0, W.TMAX, 10):
        assert equal_tolerance(link1.arrival_count(t), link1.departure_count(t+link1.actual_travel_time(t)), abs_tol=W.DELTAN)

    for vehid in W.VEHICLES:
        depart_t = W.VEHICLES[vehid].log_t_link[0][0]
        depart_timestep = int(depart_t/W.DELTAT)
        traveltime_recorded_by_vehicle = W.VEHICLES[vehid].travel_time
        traveltime_from_log_state = (W.VEHICLES[vehid].log_state.count("run") + W.VEHICLES[vehid].log_state.count("wait") )*W.DELTAT
        traveltime_from_log_t = W.VEHICLES[vehid].log_t[-1] - W.VEHICLES[vehid].log_t[depart_timestep] + W.DELTAT - W.DELTAT

        link_enter_t = W.VEHICLES[vehid].log_t_link[1][0]
        link_enter_timestep = int(link_enter_t/W.DELTAT)
        traveltime_from_cumulative_curves_onlink = int(link1.actual_travel_time(link_enter_t) + link2.actual_travel_time(link1.actual_travel_time(link_enter_t)))
        traveltime_from_log_state_onlink = (W.VEHICLES[vehid].log_state.count("run"))*W.DELTAT
        traveltime_from_log_t_onlink = W.VEHICLES[vehid].log_t[-1] - W.VEHICLES[vehid].log_t[link_enter_timestep] + W.DELTAT

        #trip travel time including waiting at the vertical queue
        assert traveltime_recorded_by_vehicle >= traveltime_from_log_state_onlink
        assert traveltime_recorded_by_vehicle == traveltime_from_log_state
        assert traveltime_recorded_by_vehicle == traveltime_from_log_t

        #within link travel time
        assert traveltime_from_log_state_onlink >= link1.length/link1.u
        assert traveltime_from_log_state_onlink == traveltime_from_log_t_onlink
        assert equal_tolerance(traveltime_from_log_state_onlink, traveltime_from_cumulative_curves_onlink, abs_tol=W.DELTAT)  #cumulative curve is approximation

@pytest.mark.flaky(reruns=2)
def test_KW_theory_cumulative_curves_and_travel_time():
    rigorous_verification_of_KW_theory_cumulative_curves_and_travel_time(5)

@pytest.mark.flaky(reruns=2)
def test_KW_theory_cumulative_curves_and_travel_time_deltan1():
    rigorous_verification_of_KW_theory_cumulative_curves_and_travel_time(1)

def test_iterative_exec_rigorous():
    for _ in range(20):
        W = World(
            name="",
            deltan=10,
            tmax=100,
            duo_update_time=10,
            print_mode=0, save_mode=0, show_mode=0,
            random_seed=42,
        )

        W.addNode("orig", 0, 0)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig", "dest", length=10000, free_flow_speed=20, number_of_lanes=1)
        W.addVehicle("orig", "dest", 0)
        W.addVehicle("orig", "dest", 50)
        W.addVehicle("orig", "dest", 100)

        #print("W.T, start_t, end_t")
        print("W.T", "\t", "W.TIME", "duration_t", "ongoing")
        
        if random.random() < 0.5:
            maxt = 30
        else:
            maxt = 200

        while W.check_simulation_ongoing():
            duration_t = random.randint(0, maxt)
            if hasattr(W, "T"):
                print(W.T, "\t", W.TIME, "\t", duration_t, "\t", W.check_simulation_ongoing())
            else:
                print(0, "\t", 0, "\t", duration_t, "\t", W.check_simulation_ongoing())
            W.exec_simulation(duration_t=duration_t)
        else:
            print(W.T, "\t","\t",  "\t", W.check_simulation_ongoing())


        if  hasattr(W.analyzer, "total_distance_traveled"):
            print("SUCCESS TO TERMINATE")
            W.analyzer.print_simple_stats(force_print=True)
        else:
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            assert False
        print_columns(["log_t"]+W.VEHICLES["0"].log_t, ["log_x"]+W.VEHICLES["0"].log_x, ["log_v"]+W.VEHICLES["0"].log_v)
        print("final_x", W.VEHICLES["0"].x)
        assert W.VEHICLES["0"].log_t[-1] == 90
        assert W.VEHICLES["0"].log_x[-1] == 1600
        assert W.VEHICLES["0"].x == 1800
        """
        This simulation runs until 100 sec with deltat=10 sec.
        Thus, the final time step start from 90 sec and simulate the duration of [90 sec, 100 sec).
        The last recorded vehicle position in `log_x` is 1600 m that is the position on 90 sec ((90 sec - 10 sec) * 20 m/sec) where the departure time was 10 sec.
        The last position `x` is 1800 m that is the "to-be" position on time 100 sec.
        """

        print()


def test_iterative_exec_rigorous_random_size_old():
    for _ in range(100):
        deltan = random.randint(1,10)
        tmax = random.randint(50,150)
        link_u = random.randint(10,30)
        W = World(
            name="",
            deltan=deltan,
            tmax=tmax,
            duo_update_time=10,
            print_mode=1, save_mode=0, show_mode=0,
            random_seed=42,
        )
        if not tmax//W.DELTAT-2 > 1:
            continue

        W.addNode("orig", 0, 0)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig", "dest", length=10000, free_flow_speed=link_u, number_of_lanes=1)
        W.addVehicle("orig", "dest", 0)
        W.addVehicle("orig", "dest", 50)
        W.addVehicle("orig", "dest", 100)

        #print("W.T, start_t, end_t")
        # print("W.T", "\t", "W.TIME", "duration_t", "ongoing")
        
        if random.random() < 0.5:
            maxt = 30
        else:
            maxt = 200

        while W.check_simulation_ongoing():
            duration_t = random.randint(0, maxt)
            # if hasattr(W, "T"):
            #     print(W.T, "\t", W.TIME, "\t", duration_t, "\t", W.check_simulation_ongoing())
            # else:
            #     print(0, "\t", 0, "\t", duration_t, "\t", W.check_simulation_ongoing())
            W.exec_simulation(duration_t=duration_t)
        # else:
        #     print(W.T, "\t","\t",  "\t", W.check_simulation_ongoing())

        if  hasattr(W.analyzer, "total_distance_traveled"):
            print("SUCCESS TO TERMINATE")
            W.analyzer.print_simple_stats(force_print=True)
        else:
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            assert False
        print_columns(["log_t"]+W.VEHICLES["0"].log_t, ["log_x"]+W.VEHICLES["0"].log_x, ["log_v"]+W.VEHICLES["0"].log_v)
        print("final_x", W.VEHICLES["0"].x)
        assert W.VEHICLES["0"].log_t[-1] == (tmax//W.DELTAT-1)*W.DELTAT
        assert W.VEHICLES["0"].log_x[-1] == (tmax//W.DELTAT-2)*W.DELTAT*link_u
        assert W.VEHICLES["0"].x == (tmax//W.DELTAT-1)*W.DELTAT*link_u

        print()


def test_iterative_exec_rigorous_random_size_duration_t2():
    for _ in range(100):
        deltan = random.randint(1,10)
        tmax = random.randint(50,150)
        link_u = random.randint(10,30)
        W = World(
            name="",
            deltan=deltan,
            tmax=tmax,
            duo_update_time=10,
            print_mode=1, save_mode=0, show_mode=0,
            random_seed=42,
        )
        if not tmax//W.DELTAT-2 > 1:
            continue

        W.addNode("orig", 0, 0)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig", "dest", length=10000, free_flow_speed=link_u, number_of_lanes=1)
        W.addVehicle("orig", "dest", 0)
        W.addVehicle("orig", "dest", 50)
        W.addVehicle("orig", "dest", 100)

        #print("W.T, start_t, end_t")
        # print("W.T", "\t", "W.TIME", "duration_t2", "ongoing")
        
        if random.random() < 0.5:
            maxt = 30
        else:
            maxt = 200

        while W.check_simulation_ongoing():
            duration_t2 = random.randint(deltan, maxt)
            # if hasattr(W, "T"):
            #     print(W.T, "\t", W.TIME, "\t", duration_t2, "\t", W.check_simulation_ongoing())
            # else:
            #     print(0, "\t", 0, "\t", duration_t2, "\t", W.check_simulation_ongoing())
            W.exec_simulation(duration_t2=duration_t2)
        # else:
        #     print(W.T, "\t","\t",  "\t", W.check_simulation_ongoing())

        if  hasattr(W.analyzer, "total_distance_traveled"):
            print("SUCCESS TO TERMINATE")
            W.analyzer.print_simple_stats(force_print=True)
        else:
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("FAILED TO TERMINATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            assert False
        print_columns(["log_t"]+W.VEHICLES["0"].log_t, ["log_x"]+W.VEHICLES["0"].log_x, ["log_v"]+W.VEHICLES["0"].log_v)
        print("final_x", W.VEHICLES["0"].x)
        assert W.VEHICLES["0"].log_t[-1] == (tmax//W.DELTAT-1)*W.DELTAT
        assert W.VEHICLES["0"].log_x[-1] == (tmax//W.DELTAT-2)*W.DELTAT*link_u
        assert W.VEHICLES["0"].x == (tmax//W.DELTAT-1)*W.DELTAT*link_u

        print()

def test_vehicle_speed_at_node_transfer():
    for i in range(10):    
        if i == 0:
            deltan = 5
            free_flow_speed = 10
            length1 = 1000
            length2 = random.randint(500,1000)
            length3 = 1000
            lanes1 = 1
            lanes2 = 1
            lanes3 = 1
        else:
            deltan = random.randint(1,10)
            free_flow_speed = random.randint(5,25)
            length1 = random.randint(500,1000)
            length2 = random.randint(500,1000)
            length3 = random.randint(500,1000)
            lanes1 = random.randint(1,5)
            lanes2 = random.randint(1,5)
            lanes3 = random.randint(1,5)
            
        W = World(
            name="",
            deltan=deltan,
            tmax=3600,
            print_mode=1, save_mode=0, show_mode=0,
            random_seed=0
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", 1, 1)
        W.addNode("dest", 2, 1)
        W.addLink(f"link1-{length1}", "orig", "mid1", length=length1, free_flow_speed=free_flow_speed, number_of_lanes=lanes1)
        W.addLink(f"link2-{length2}", "mid1", "mid2", length=length2, free_flow_speed=free_flow_speed, number_of_lanes=lanes2)
        W.addLink(f"link3-{length3}", "mid2", "dest", length=length3, free_flow_speed=free_flow_speed, number_of_lanes=lanes3)
        W.adddemand("orig", "dest", 0, 1000, 0.1)
        W.exec_simulation()

        W.analyzer.print_simple_stats()
        df = W.analyzer.vehicles_to_pandas()
        df = df[(df["name"]=="0") & (df["x"]!=-1)]
        print(df)
        vs = df["v"]
        for v in list(vs):
            assert equal_tolerance(v, free_flow_speed, rel_tol=0, abs_tol=0.01)
                    
