"""
This script verifies whether UXsim outputs reasonable solutions for a straight road in various configurations.
"""

import pytest
from uxsim import *

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol==0.0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=10, jam_density=0.2)
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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.66666)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.66666)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
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
    link1 = W.addLink("link", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2)
    link2 = W.addLink("link", "mid", "dest", length=1000, free_flow_speed=20, jam_density=0.1)
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