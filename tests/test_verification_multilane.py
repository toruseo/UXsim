"""
This script verifies whether UXsim outputs plausible solutions for multilane scenarios.
Some of the tests are probablistic.
"""

import pytest
from uxsim import *
import numpy as np

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


@pytest.mark.flaky(reruns=5)
def test_straight_1link_2lane_low_demand():
    W = World(
        name="",
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 2, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    t = 600
    flow = 1
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    W.analyzer.compute_edie_state()
    
    assert equal_tolerance(link.capacity, 0.8*2)
    assert equal_tolerance(link.u, 20)
    assert equal_tolerance(link.k_star, 0.04*2)
    assert equal_tolerance(link.q_mat[2:4,3:5].mean(), 1)
    assert equal_tolerance(link.v_mat[2:4,3:5].mean(), link.u)
    assert equal_tolerance(link.k_mat[2:4,3:5].mean(), 1/link.u)
    assert equal_tolerance(link.q_mat[6:,3:5].mean(), 0)
    assert equal_tolerance(link.v_mat[6:,3:5].mean(), link.u)
    assert equal_tolerance(link.k_mat[6:,3:5].mean(), 0)
    

@pytest.mark.flaky(reruns=5)
def test_straight_1link_2lane_high_demand():
    W = World(
        name="",
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("dest", 2, 1)
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    t = 1200
    flow = 2
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    W.analyzer.compute_edie_state()
    
    assert equal_tolerance(link.capacity, 0.8*2)
    assert equal_tolerance(link.u, 20)
    assert equal_tolerance(link.k_star, 0.04*2)
    assert equal_tolerance(link.q_mat[2:6,3:5].mean(), link.capacity)
    assert equal_tolerance(link.v_mat[2:6,3:5].mean(), link.u)
    assert equal_tolerance(link.k_mat[2:6,3:5].mean(), link.k_star)
    

@pytest.mark.flaky(reruns=5)
def test_straight_2link_2lane_low_demand():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    t = 600
    flow = 1
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
        
    assert equal_tolerance(link1.q_mat[2:4,3:5].mean(), 1)
    assert equal_tolerance(link1.v_mat[2:4,3:5].mean(), link1.u)
    assert equal_tolerance(link1.k_mat[2:4,3:5].mean(), 1/link1.u)
    assert equal_tolerance(link1.q_mat[7:,3:5].mean(), 0)
    assert equal_tolerance(link1.v_mat[7:,3:5].mean(), link1.u)
    assert equal_tolerance(link1.k_mat[7:,3:5].mean(), 0)
    assert equal_tolerance(link2.q_mat[2:4,3:5].mean(), 1)
    assert equal_tolerance(link2.v_mat[2:4,3:5].mean(), link2.u)
    assert equal_tolerance(link2.k_mat[2:4,3:5].mean(), 1/link2.u)
    assert equal_tolerance(link2.q_mat[7:,3:5].mean(), 0)
    assert equal_tolerance(link2.v_mat[7:,3:5].mean(), link2.u)
    assert equal_tolerance(link2.k_mat[7:,3:5].mean(), 0)
    

@pytest.mark.flaky(reruns=5)
def test_straight_2link_2lane_high_demand():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    t = 400
    flow = 2
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
        
    assert equal_tolerance(link1.q_mat[2:4,3:5].mean(), 0.8*2)
    assert equal_tolerance(link1.v_mat[2:4,3:5].mean(), 20)
    assert equal_tolerance(link1.k_mat[2:4,3:5].mean(), 0.04*2)
    assert equal_tolerance(link1.q_mat[7:,3:5].mean(), 0)
    assert equal_tolerance(link1.v_mat[7:,3:5].mean(), link1.u)
    assert equal_tolerance(link1.k_mat[7:,3:5].mean(), 0)
    assert equal_tolerance(link2.q_mat[2:4,3:5].mean(), 0.8*2)
    assert equal_tolerance(link2.v_mat[2:4,3:5].mean(), 20)
    assert equal_tolerance(link2.k_mat[2:4,3:5].mean(), 0.04*2)
    assert equal_tolerance(link2.q_mat[7:,3:5].mean(), 0)
    assert equal_tolerance(link2.v_mat[7:,3:5].mean(), link2.u)
    assert equal_tolerance(link2.k_mat[7:,3:5].mean(), 0)
    


@pytest.mark.flaky(reruns=5)
def test_straight_2link_2lane_to_1lane_congestion():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
    t = 1000
    flow = 2
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
        
    assert equal_tolerance(link1.q_mat[2:4,3:5].mean(), 0.8*2)  #before congestion
    assert equal_tolerance(link1.v_mat[2:4,3:5].mean(), 20)
    assert equal_tolerance(link1.k_mat[2:4,3:5].mean(), 0.04*2)
    assert equal_tolerance(link1.q_mat[7:,3:5].mean(), 0.8)  #after congestion
    assert equal_tolerance(link1.v_mat[7:,3:5].mean(), 3.33)
    assert equal_tolerance(link1.k_mat[7:,3:5].mean(), 0.24)
    assert equal_tolerance(link2.q_mat[2:,3:5].mean(), 0.8)  #downstream
    assert equal_tolerance(link2.v_mat[2:,3:5].mean(), 20)
    assert equal_tolerance(link2.k_mat[2:,3:5].mean(), 0.04)
    

@pytest.mark.flaky(reruns=5)
def test_straight_2link_2lane_to_1lane_congestion_deltan1():
    W = World(
        name="",
        deltan=1,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
    t = 1000
    flow = 2
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
        
    assert equal_tolerance(link1.q_mat[2:4,3:5].mean(), 0.8*2)  #before congestion
    assert equal_tolerance(link1.v_mat[2:4,3:5].mean(), 20)
    assert equal_tolerance(link1.k_mat[2:4,3:5].mean(), 0.04*2)
    assert equal_tolerance(link1.q_mat[7:,3:5].mean(), 0.8)  #after congestion
    assert equal_tolerance(link1.v_mat[7:,3:5].mean(), 3.33)
    assert equal_tolerance(link1.k_mat[7:,3:5].mean(), 0.24)
    assert equal_tolerance(link2.q_mat[2:,3:5].mean(), 0.8)  #downstream
    assert equal_tolerance(link2.v_mat[2:,3:5].mean(), 20)
    assert equal_tolerance(link2.k_mat[2:,3:5].mean(), 0.04)
    
    

@pytest.mark.flaky(reruns=5)
def test_straight_2link_2lane_to_3lane():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density=0.2, number_of_lanes=3)
    t = 2000
    flow = 2
    W.adddemand("orig", "dest", 0, t, flow)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
        
    assert equal_tolerance(link1.q_mat[2:4,3:5].mean(), 0.8*2)
    assert equal_tolerance(link1.v_mat[2:4,3:5].mean(), 20)
    assert equal_tolerance(link1.k_mat[2:4,3:5].mean(), 0.04*2)
    assert equal_tolerance(link1.q_mat[7:,3:5].mean(), 0.8*2)
    assert equal_tolerance(link1.v_mat[7:,3:5].mean(), 20)
    assert equal_tolerance(link1.k_mat[7:,3:5].mean(), 0.04*2)
    assert equal_tolerance(link2.q_mat[2:,3:5].mean(), 0.8*2)
    assert equal_tolerance(link2.v_mat[2:,3:5].mean(), 20)
    assert equal_tolerance(link2.k_mat[2:,3:5].mean(), 0.04*2)
    

@pytest.mark.flaky(reruns=5)
def test_merge_saturated():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, number_of_lanes=1)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, number_of_lanes=2)
    link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, number_of_lanes=3)
    t1 = 1000
    flow1 = 0.8
    t2 = 1000
    flow2 = 1.6
    W.adddemand("orig1", "dest", 0, t1, flow1)
    W.adddemand("orig2", "dest", 0, t2, flow2)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[3:6,3].mean(), 0.8) 
    assert equal_tolerance(link1.k_mat[3:6,3].mean(), 0.04)
    assert equal_tolerance(link1.v_mat[3:6,3].mean(), 20)
    assert equal_tolerance(link2.q_mat[3:6,3].mean(), 1.6) 
    assert equal_tolerance(link2.k_mat[3:6,3].mean(), 0.08)
    assert equal_tolerance(link2.v_mat[3:6,3].mean(), 20)
    assert equal_tolerance(link3.q_mat[3:6,3].mean(), 2.4) 
    assert equal_tolerance(link3.k_mat[3:6,3].mean(), 0.12)
    assert equal_tolerance(link3.v_mat[3:6,3].mean(), 20)
    


@pytest.mark.flaky(reruns=5)
def test_merge_small_demand():
    W = World(
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 0)
    W.addNode("mid", 0, 0)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, number_of_lanes=1)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, number_of_lanes=2)
    link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, number_of_lanes=3)
    t1 = 1000
    flow1 = 0.8
    t2 = 1000
    flow2 = 0.8
    W.adddemand("orig1", "dest", 0, t1, flow1)
    W.adddemand("orig2", "dest", 0, t2, flow2)

    W.exec_simulation()
    W.analyzer.print_simple_stats()

    W.analyzer.compute_edie_state()
    assert equal_tolerance(link1.q_mat[3:6,3].mean(), 0.8) 
    assert equal_tolerance(link1.k_mat[3:6,3].mean(), 0.04)
    assert equal_tolerance(link1.v_mat[3:6,3].mean(), 20)
    assert equal_tolerance(link2.q_mat[3:6,3].mean(), 0.8) 
    assert equal_tolerance(link2.k_mat[3:6,3].mean(), 0.04)
    assert equal_tolerance(link2.v_mat[3:6,3].mean(), 20)
    assert equal_tolerance(link3.q_mat[3:6,3].mean(), 1.6) 
    assert equal_tolerance(link3.k_mat[3:6,3].mean(), 0.08)
    assert equal_tolerance(link3.v_mat[3:6,3].mean(), 20)




#TODO: add more tests, merging logic should be checked