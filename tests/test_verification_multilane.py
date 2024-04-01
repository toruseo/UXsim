"""
This script verifies whether UXsim outputs reasonable solutions for multilane scenarios.
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

def q2k_cong(q, link):
    return -(q-link.kappa*link.w)/link.w



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
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
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
    link = W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
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
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
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
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
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
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
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
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
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
    link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=3)
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
def test_straight_2link_capacity_out():
        
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2, capacity_out=1.0)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())

        
    assert equal_tolerance(np.average(q1s), 1)
    assert equal_tolerance(np.average(k1s), q2k_cong(1, link1))
    assert equal_tolerance(np.average(v1s), 1/q2k_cong(1, link1))
    assert equal_tolerance(np.average(q2s), 1)
    assert equal_tolerance(np.average(k2s), 1/20)
    assert equal_tolerance(np.average(v2s), 20)

@pytest.mark.flaky(reruns=5)
def test_straight_2link_capacity_in():
        
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2, capacity_in=1.0)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())

        
    assert equal_tolerance(np.average(q1s), 1)
    assert equal_tolerance(np.average(k1s), q2k_cong(1, link1))
    assert equal_tolerance(np.average(v1s), 1/q2k_cong(1, link1))
    assert equal_tolerance(np.average(q2s), 1)
    assert equal_tolerance(np.average(k2s), 1/20)
    assert equal_tolerance(np.average(v2s), 20)


@pytest.mark.flaky(reruns=5)
def test_straight_2link_node_capacity():
        
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0, flow_capacity=1.0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())

        
    assert equal_tolerance(np.average(q1s), 1)
    assert equal_tolerance(np.average(k1s), q2k_cong(1, link1))
    assert equal_tolerance(np.average(v1s), 1/q2k_cong(1, link1))
    assert equal_tolerance(np.average(q2s), 1)
    assert equal_tolerance(np.average(k2s), 1/20)
    assert equal_tolerance(np.average(v2s), 20)

@pytest.mark.flaky(reruns=5)
def test_straight_trip_end_in_middle_freeflow():
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
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=3)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)

    W.adddemand("orig", "dest", 0, 2000, 1.6)
    W.adddemand("orig", "mid", 1000, 2000, 0.8)

    W.exec_simulation()
    W.analyzer.compute_edie_state()
    # df = W.analyzer.od_to_pandas()
    # W.analyzer.print_simple_stats()
    # W.analyzer.time_space_diagram_traj_links([link1, link2])
    # W.analyzer.cumulative_curves()
    assert equal_tolerance(link1.q_mat[1:7,3].mean(), 1.6)
    assert equal_tolerance(link1.q_mat[10:,3].mean(), 1.6+0.8)
    assert equal_tolerance(link2.q_mat[1:7,3].mean(), 1.6)
    assert equal_tolerance(link2.q_mat[10:,3].mean(), 1.6)

@pytest.mark.flaky(reruns=5)
def test_straight_trip_end_in_middle():
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
    link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=3)
    link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
    t = 600
    flow = 1.6
    W.adddemand("orig", "dest", 0, t, flow)
    W.adddemand("orig", "mid", 500, t, flow)

    W.exec_simulation()
    df = W.analyzer.od_to_pandas()
    # W.analyzer.print_simple_stats()
    # W.analyzer.compute_edie_state()
    # W.analyzer.time_space_diagram_traj_links([link1, link2])
    # W.analyzer.cumulative_curves()
    assert equal_tolerance(df["completed_trips"][0], 960)
    assert equal_tolerance(df["completed_trips"][1], 160)
    assert equal_tolerance(df["average_travel_time"][0], 400)
    assert equal_tolerance(df["average_travel_time"][1], 600)

@pytest.mark.flaky(reruns=5)
def test_straight_different_fd():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=5, jam_density_per_lane=0.1, number_of_lanes=2)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())

    assert equal_tolerance(np.average(q1s), 0.666)
    assert equal_tolerance(np.average(k1s), q2k_cong(0.666, link1))
    assert equal_tolerance(np.average(v1s), 0.666/q2k_cong(0.666, link1))
    assert equal_tolerance(np.average(q2s), 0.666)
    assert equal_tolerance(np.average(k2s), 0.666/5)
    assert equal_tolerance(np.average(v2s), 5)

@pytest.mark.flaky(reruns=5)
def test_straight_different_fd_different_arg():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.4, number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=5, jam_density=0.2, number_of_lanes=2)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())

    assert equal_tolerance(np.average(q1s), 0.666)
    assert equal_tolerance(np.average(k1s), q2k_cong(0.666, link1))
    assert equal_tolerance(np.average(v1s), 0.666/q2k_cong(0.666, link1))
    assert equal_tolerance(np.average(q2s), 0.666)
    assert equal_tolerance(np.average(k2s), 0.666/5)
    assert equal_tolerance(np.average(v2s), 5)


@pytest.mark.flaky(reruns=5)
def test_straight_different_fd_different_lanes():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=5, jam_density_per_lane=0.1, number_of_lanes=3)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())
        
    assert equal_tolerance(np.average(q1s), 0.666*1.5)
    assert equal_tolerance(np.average(k1s), q2k_cong(0.666*1.5, link1))
    assert equal_tolerance(np.average(v1s), 0.666*1.5/q2k_cong(0.666*1.5, link1))
    assert equal_tolerance(np.average(q2s), 0.666*1.5)
    assert equal_tolerance(np.average(k2s), 0.666*1.5/5)
    assert equal_tolerance(np.average(v2s), 5)

@pytest.mark.flaky(reruns=5)
def test_straight_different_fd_different_lanes_different_arg():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=3000, free_flow_speed=20, jam_density=0.4, number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest", length=3000, free_flow_speed=5, jam_density=0.3, number_of_lanes=3)
        t = 2000
        flow = 1.4
        W.adddemand("orig", "dest", 0, t, flow)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,2:4].mean())
        k1s.append(link1.k_mat[10:,2:4].mean())
        v1s.append(link1.v_mat[10:,2:4].mean())
        q2s.append(link2.q_mat[10:,2:4].mean())
        k2s.append(link2.k_mat[10:,2:4].mean())
        v2s.append(link2.v_mat[10:,2:4].mean())
        
    assert equal_tolerance(np.average(q1s), 0.666*1.5)
    assert equal_tolerance(np.average(k1s), q2k_cong(0.666*1.5, link1))
    assert equal_tolerance(np.average(v1s), 0.666*1.5/q2k_cong(0.666*1.5, link1))
    assert equal_tolerance(np.average(q2s), 0.666*1.5)
    assert equal_tolerance(np.average(k2s), 0.666*1.5/5)
    assert equal_tolerance(np.average(v2s), 5)

@pytest.mark.flaky(reruns=10)
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
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=1)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
    link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=3)
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
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=1)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
    link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=3)
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

@pytest.mark.flaky(reruns=5)
def test_merge_congested_fair():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 1.4
        t2 = 2000
        flow2 = 1.4
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,3].mean())
        k1s.append(link1.k_mat[5:,3].mean())
        q2s.append(link2.q_mat[5:,3].mean())
        k2s.append(link2.k_mat[5:,3].mean())
        q3s.append(link3.q_mat[5:,3].mean())
        k3s.append(link3.k_mat[5:,3].mean())
        v3s.append(link3.v_mat[5:,3].mean())

    assert equal_tolerance(np.average(q1s), 0.8)
    assert equal_tolerance(np.average(k1s), q2k_cong(0.8, link1))
    assert equal_tolerance(np.average(q2s), 0.8)
    assert equal_tolerance(np.average(k2s), q2k_cong(0.8, link2))
    assert equal_tolerance(np.average(q3s), 1.6)
    assert equal_tolerance(np.average(k3s), 0.08)

@pytest.mark.flaky(reruns=5)
def test_merge_congested_unfair():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=2, number_of_lanes=2)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 1.4
        t2 = 2000
        flow2 = 1.4
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,3].mean())
        k1s.append(link1.k_mat[5:,3].mean())
        q2s.append(link2.q_mat[5:,3].mean())
        k2s.append(link2.k_mat[5:,3].mean())
        q3s.append(link3.q_mat[5:,3].mean())
        k3s.append(link3.k_mat[5:,3].mean())
        v3s.append(link3.v_mat[5:,3].mean())

    assert equal_tolerance(np.average(q1s), 1.6*0.666) 
    assert equal_tolerance(np.average(k1s), q2k_cong(1.6*0.666, link1))
    assert equal_tolerance(np.average(q2s), 1.6*0.333)
    assert equal_tolerance(np.average(k2s), q2k_cong(1.6*0.333, link2))
    assert equal_tolerance(np.average(q3s), 1.6) 
    assert equal_tolerance(np.average(k3s), 0.08)
    assert equal_tolerance(np.average(v3s), 20)


@pytest.mark.flaky(reruns=5)
def test_merge_congested_unfair_deltan1():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=1,
            tmax=1000,
            print_mode=0, save_mode=0, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=2, number_of_lanes=2)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 1000
        flow1 = 1.4
        t2 = 1000
        flow2 = 1.4
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,3].mean())
        k1s.append(link1.k_mat[5:,3].mean())
        q2s.append(link2.q_mat[5:,3].mean())
        k2s.append(link2.k_mat[5:,3].mean())
        q3s.append(link3.q_mat[5:,3].mean())
        k3s.append(link3.k_mat[5:,3].mean())
        v3s.append(link3.v_mat[5:,3].mean())

    assert equal_tolerance(np.average(q1s), 1.6*0.666) 
    assert equal_tolerance(np.average(k1s), q2k_cong(1.6*0.666, link1))
    assert equal_tolerance(np.average(q2s), 1.6*0.333)
    assert equal_tolerance(np.average(k2s), q2k_cong(1.6*0.333, link2))
    assert equal_tolerance(np.average(q3s), 1.6) 
    assert equal_tolerance(np.average(k3s), 0.08)
    assert equal_tolerance(np.average(v3s), 20)


@pytest.mark.flaky(reruns=5)
def test_merge_congested_veryunfair():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=10, number_of_lanes=2)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 1.4
        t2 = 2000
        flow2 = 1.4
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,3].mean())
        k1s.append(link1.k_mat[5:,3].mean())
        q2s.append(link2.q_mat[5:,3].mean())
        k2s.append(link2.k_mat[5:,3].mean())
        q3s.append(link3.q_mat[5:,3].mean())
        k3s.append(link3.k_mat[5:,3].mean())
        v3s.append(link3.v_mat[5:,3].mean())

    assert equal_tolerance(np.average(q1s), 1.4) 
    assert equal_tolerance(np.average(k1s), 1.4/link1.u)
    assert equal_tolerance(np.average(q2s), 0.2)
    assert equal_tolerance(np.average(k2s), q2k_cong(0.2, link2))
    assert equal_tolerance(np.average(q3s), 1.6) 
    assert equal_tolerance(np.average(k3s), 0.08)
    assert equal_tolerance(np.average(v3s), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_congested_different_lanes():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=1)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 0.8
        t2 = 2000
        flow2 = 1.6
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,6:8].mean())
        k1s.append(link1.k_mat[10:,6:8].mean())
        q2s.append(link2.q_mat[10:,6:8].mean())
        k2s.append(link2.k_mat[10:,6:8].mean())
        q3s.append(link3.q_mat[10:,6:8].mean())
        k3s.append(link3.k_mat[10:,6:8].mean())
        v3s.append(link3.v_mat[10:,6:8].mean())

    #車線の少ないリンクは，車線の多いリンクの試行回数の恩恵を受けて少し有利になる．大きな差はでないので許容する
    assert equal_tolerance(np.average(q1s), 1.6*0.333, rel_tol=0.3) 
    assert equal_tolerance(np.average(k1s), q2k_cong(1.6*0.333, link1), rel_tol=0.3)
    assert equal_tolerance(np.average(q2s), 1.6*0.666, rel_tol=0.3)
    assert equal_tolerance(np.average(k2s), q2k_cong(1.6*0.666, link2), rel_tol=0.3)
    assert equal_tolerance(np.average(q3s), 1.6) 
    assert equal_tolerance(np.average(k3s), 0.08)
    assert equal_tolerance(np.average(v3s), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_congested_signal():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0, signal=[60,60])
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2, signal_group=0)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2, signal_group=1)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 1.6
        t2 = 2000
        flow2 = 1.6
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,6:8].mean())
        k1s.append(link1.k_mat[10:,6:8].mean())
        q2s.append(link2.q_mat[10:,6:8].mean())
        k2s.append(link2.k_mat[10:,6:8].mean())
        q3s.append(link3.q_mat[10:,6:8].mean())
        k3s.append(link3.k_mat[10:,6:8].mean())
        v3s.append(link3.v_mat[10:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 0.8) 
    assert equal_tolerance(np.average(k1s), q2k_cong(0.8, link1))
    assert equal_tolerance(np.average(q2s), 0.8)
    assert equal_tolerance(np.average(k2s), q2k_cong(0.8, link2))
    assert equal_tolerance(np.average(q3s), 1.6) 
    assert equal_tolerance(np.average(k3s), 0.08)
    assert equal_tolerance(np.average(v3s), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_saturated_signal():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0, signal=[60,60])
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2, signal_group=0)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2, signal_group=1)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 0.8
        t2 = 2000
        flow2 = 0.8
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,6:8].mean())
        k1s.append(link1.k_mat[10:,6:8].mean())
        v1s.append(link1.v_mat[10:,9:].mean())
        q2s.append(link2.q_mat[10:,6:8].mean())
        k2s.append(link2.k_mat[10:,6:8].mean())
        q3s.append(link3.q_mat[10:,6:8].mean())
        k3s.append(link3.k_mat[10:,6:8].mean())
        v3s.append(link3.v_mat[10:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 0.8) 
    assert equal_tolerance(np.average(k1s), 0.8/20)
    assert equal_tolerance(np.average(v1s), 5)
    assert equal_tolerance(np.average(q2s), 0.8)
    assert equal_tolerance(np.average(k2s), 0.8/20)
    assert equal_tolerance(np.average(q3s), 1.6) 
    assert equal_tolerance(np.average(k3s), 0.08)
    assert equal_tolerance(np.average(v3s), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_congested_node_capacity():
    
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig1", 0, 0)
        W.addNode("orig2", 0, 0)
        W.addNode("mid", 0, 0, flow_capacity=1)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=2, number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, merge_priority=1, number_of_lanes=2)
        t1 = 2000
        flow1 = 1.6
        t2 = 2000
        flow2 = 1.6
        W.adddemand("orig1", "dest", 0, t1, flow1)
        W.adddemand("orig2", "dest", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[10:,6:8].mean())
        k1s.append(link1.k_mat[10:,6:8].mean())
        v1s.append(link1.v_mat[10:,9:].mean())
        q2s.append(link2.q_mat[10:,6:8].mean())
        k2s.append(link2.k_mat[10:,6:8].mean())
        q3s.append(link3.q_mat[10:,6:8].mean())
        k3s.append(link3.k_mat[10:,6:8].mean())
        v3s.append(link3.v_mat[10:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 0.333) 
    assert equal_tolerance(np.average(k1s), q2k_cong(0.333, link1))
    assert equal_tolerance(np.average(q2s), 0.666)
    assert equal_tolerance(np.average(k2s), q2k_cong(0.666, link2))
    assert equal_tolerance(np.average(q3s), 1.0) 
    assert equal_tolerance(np.average(k3s), 1/20)
    assert equal_tolerance(np.average(v3s), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_disappear_freeflow():
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
    W.addNode("dest1", 2, 1)
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
    link3 = W.addLink("link3", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)

    W.adddemand("orig1", "dest1", 0, 2000, 0.8)
    W.adddemand("orig1", "mid",   0, 2000, 0.4)
    W.adddemand("orig2", "dest1", 0, 2000, 0.8)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    W.analyzer.compute_edie_state()
    #W.analyzer.time_space_diagram_traj()

    assert equal_tolerance(link1.q_mat[3:8,3:5].mean(), 1.2)
    assert equal_tolerance(link2.q_mat[3:8,3:5].mean(), 0.8)
    assert equal_tolerance(link3.q_mat[3:8,3:5].mean(), 1.6)
    assert equal_tolerance(link1.v_mat[3:8,3:5].mean(), 20)
    assert equal_tolerance(link2.v_mat[3:8,3:5].mean(), 20)
    assert equal_tolerance(link3.v_mat[3:8,3:5].mean(), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_disappear_saturated():
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
    W.addNode("dest1", 2, 1)
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
    link3 = W.addLink("link3", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)

    W.adddemand("orig1", "dest1", 0, 2000, 0.8)
    W.adddemand("orig1", "mid",   0, 2000, 0.8)
    W.adddemand("orig2", "dest1", 0, 2000, 0.8)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    W.analyzer.compute_edie_state()

    assert equal_tolerance(link1.q_mat[3:8,3:5].mean(), 1.6)
    assert equal_tolerance(link2.q_mat[3:8,3:5].mean(), 0.8)
    assert equal_tolerance(link3.q_mat[3:8,3:5].mean(), 1.6)
    assert equal_tolerance(link1.v_mat[3:8,3:5].mean(), 20)
    assert equal_tolerance(link2.v_mat[3:8,3:5].mean(), 20)
    assert equal_tolerance(link3.v_mat[3:8,3:5].mean(), 20)

@pytest.mark.flaky(reruns=10)
def test_merge_disappear_congested():
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
    W.addNode("dest1", 2, 1)
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
    link3 = W.addLink("link3", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)

    W.adddemand("orig1", "dest1", 0, 2000, 1.2)
    W.adddemand("orig1", "mid",   0, 2000, 0.4)
    W.adddemand("orig2", "dest1", 0, 2000, 0.8)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    W.analyzer.compute_edie_state()

    assert equal_tolerance(link1.q_mat[3:8,3:5].mean(), 1.3, rel_tol=0.5)
    assert equal_tolerance(link2.q_mat[3:8,3:5].mean(), 0.6, rel_tol=0.5)
    assert equal_tolerance(link3.q_mat[3:8,3:5].mean(), 1.6)
    assert equal_tolerance(link1.v_mat[3:8,3:5].mean(), 9, rel_tol=0.5)
    assert equal_tolerance(link2.v_mat[3:8,3:5].mean(), 8, rel_tol=0.5)
    assert equal_tolerance(link3.v_mat[3:8,3:5].mean(), 20)

@pytest.mark.flaky(reruns=5)
def test_merge_diverge_disappear_freeflow():
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
    W.addNode("dest1", 2, 1)
    W.addNode("dest2", 0, 0)
    link1 = W.addLink("link1", "orig1", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=3)
    link2 = W.addLink("link2", "orig2", "mid", length=1000, free_flow_speed=30, jam_density_per_lane=0.2, number_of_lanes=2)
    link3 = W.addLink("link3", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    link4 = W.addLink("link4", "mid", "dest2", length=1000, free_flow_speed=30, jam_density_per_lane=0.2, number_of_lanes=1)

    W.adddemand("orig1", "dest1", 0, 2000, 0.8)
    W.adddemand("orig1", "dest2", 0, 2000, 0.4)
    W.adddemand("orig1", "mid",   0, 2000, 0.8)
    W.adddemand("orig2", "dest1", 0, 2000, 0.8)
    W.adddemand("orig2", "dest2", 0, 2000, 0.4)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    W.analyzer.compute_edie_state()
    # W.analyzer.time_space_diagram_traj()

    assert equal_tolerance(link1.q_mat[3:8,3:5].mean(), 2.0)
    assert equal_tolerance(link2.q_mat[3:8,3:5].mean(), 1.2)
    assert equal_tolerance(link3.q_mat[3:8,3:5].mean(), 0.8*2)
    assert equal_tolerance(link4.q_mat[3:8,3:5].mean(), 0.8*1)
    assert equal_tolerance(link1.v_mat[3:8,3:5].mean(), 20)
    assert equal_tolerance(link2.v_mat[3:8,3:5].mean(), 30)
    assert equal_tolerance(link3.v_mat[3:8,3:5].mean(), 20)
    assert equal_tolerance(link4.v_mat[3:8,3:5].mean(), 30)

@pytest.mark.flaky(reruns=5)
def test_diverge_saturated_2_2_2():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=1000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,   number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,  number_of_lanes=2)
        link3 = W.addLink("link3", "mid", "dest2",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
        t1 = 1000
        flow1 = 0.8
        t2 = 1000
        flow2 = 0.8
        W.adddemand("orig", "dest1", 0, t1, flow1)
        W.adddemand("orig", "dest2", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,6:8].mean())
        k1s.append(link1.k_mat[5:,6:8].mean())
        v1s.append(link1.v_mat[5:,6:8].mean())
        q2s.append(link2.q_mat[5:,6:8].mean())
        k2s.append(link2.k_mat[5:,6:8].mean())
        q3s.append(link3.q_mat[5:,6:8].mean())
        k3s.append(link3.k_mat[5:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 1.6) 
    assert equal_tolerance(np.average(k1s), 0.08)
    assert equal_tolerance(np.average(v1s), 20)
    assert equal_tolerance(np.average(q2s), 0.8)
    assert equal_tolerance(np.average(k2s), 0.04)
    assert equal_tolerance(np.average(q3s), 0.8) 
    assert equal_tolerance(np.average(k3s), 0.04)

@pytest.mark.flaky(reruns=5)
def test_diverge_saturated_2_1_1_congestion_due_to_friction(): #TODO: frictionは要精査
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=1000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,   number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,  number_of_lanes=1)
        link3 = W.addLink("link3", "mid", "dest2",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
        t1 = 1000
        flow1 = 0.8
        t2 = 1000
        flow2 = 0.8
        W.adddemand("orig", "dest1", 0, t1, flow1)
        W.adddemand("orig", "dest2", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,6:8].mean())
        k1s.append(link1.k_mat[5:,6:8].mean())
        v1s.append(link1.v_mat[5:,6:8].mean())
        q2s.append(link2.q_mat[5:,6:8].mean())
        k2s.append(link2.k_mat[5:,6:8].mean())
        q3s.append(link3.q_mat[5:,6:8].mean())
        k3s.append(link3.k_mat[5:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 1.44) 
    assert equal_tolerance(np.average(k1s), 0.11)
    assert equal_tolerance(np.average(v1s), 13)
    assert equal_tolerance(np.average(q2s), 0.72)
    assert equal_tolerance(np.average(k2s), 0.036)
    assert equal_tolerance(np.average(q3s), 0.72) 
    assert equal_tolerance(np.average(k3s), 0.036)

@pytest.mark.flaky(reruns=10)
def test_diverge_lesssaturated_2_1_1():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=1000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,   number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,  number_of_lanes=1)
        link3 = W.addLink("link3", "mid", "dest2",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
        t1 = 1000
        flow1 = 0.7
        t2 = 1000
        flow2 = 0.7
        W.adddemand("orig", "dest1", 0, t1, flow1)
        W.adddemand("orig", "dest2", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,6:8].mean())
        k1s.append(link1.k_mat[5:,6:8].mean())
        v1s.append(link1.v_mat[5:,6:8].mean())
        q2s.append(link2.q_mat[5:,6:8].mean())
        k2s.append(link2.k_mat[5:,6:8].mean())
        q3s.append(link3.q_mat[5:,6:8].mean())
        k3s.append(link3.k_mat[5:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 1.4) 
    assert equal_tolerance(np.average(k1s), 0.07)
    assert equal_tolerance(np.average(v1s), 20)
    assert equal_tolerance(np.average(q2s), 0.7)
    assert equal_tolerance(np.average(k2s), 0.035)
    assert equal_tolerance(np.average(q3s), 0.7) 
    assert equal_tolerance(np.average(k3s), 0.035)

@pytest.mark.flaky(reruns=5)
def test_diverge_congested_2_1_1():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(
            name="",
            deltan=5,
            tmax=1000,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid", 0, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, 1)
        link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,   number_of_lanes=2)
        link2 = W.addLink("link2", "mid", "dest1", length=1000, free_flow_speed=20, jam_density_per_lane=0.2,  number_of_lanes=1)
        link3 = W.addLink("link3", "mid", "dest2",  length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
        t1 = 1000
        flow1 = 1.6*2/3
        t2 = 1000
        flow2 = 1.6*1/3
        W.adddemand("orig", "dest1", 0, t1, flow1)
        W.adddemand("orig", "dest2", 0, t2, flow2)

        W.exec_simulation()
        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1s.append(link1.q_mat[5:,6:8].mean())
        k1s.append(link1.k_mat[5:,6:8].mean())
        v1s.append(link1.v_mat[5:,6:8].mean())
        q2s.append(link2.q_mat[5:,6:8].mean())
        k2s.append(link2.k_mat[5:,6:8].mean())
        q3s.append(link3.q_mat[5:,6:8].mean())
        k3s.append(link3.k_mat[5:,6:8].mean())

    assert equal_tolerance(np.average(q1s), 1.2) 
    assert equal_tolerance(np.average(k1s), q2k_cong(1.2, link1))
    assert equal_tolerance(np.average(v1s), 1.2/q2k_cong(1.2, link1))
    assert equal_tolerance(np.average(q2s), 0.8)
    assert equal_tolerance(np.average(k2s), 0.04)
    assert equal_tolerance(np.average(q3s), 0.4) 
    assert equal_tolerance(np.average(k3s), 0.4/20)

@pytest.mark.flaky(reruns=10)
def test_4phase_signal_jpstyle_straight_2lane():
    W = World(
        name="",
        deltan=1,
        tmax=1500,
        print_mode=1, save_mode=0, show_mode=0,
        random_seed=None
    )

    # シナリオ定義
    #4現示信号:
    #    現示0: 南北直左のみ50秒
    #    現示1: 南北右折のみ10秒
    #    現示2: 東西直左のみ40秒
    #    現示3: 東西右折のみ5秒
    W.addNode("S_orig", 0, -3)    #起点ノード
    W.addNode("N_orig", 0, 3)
    W.addNode("W_orig", -3, 0)
    W.addNode("E_orig", 3, 0)
    W.addNode("S_dest", 0, -3)    #終点ノード
    W.addNode("N_dest", 0, 3)
    W.addNode("W_dest", -3, 0)
    W.addNode("E_dest", 3, 0)

    signal_phase_setting = [50,10,40,5]
    W.addNode("S_i", 0, -1, signal=signal_phase_setting)  #信号交差点の南側入口ノード
    W.addNode("N_i", 0.2, 1, signal=signal_phase_setting)
    W.addNode("W_i", -1, 0.2, signal=signal_phase_setting)
    W.addNode("E_i", 1, 0, signal=signal_phase_setting)
    W.addNode("S_o", 0.2, -1, signal=signal_phase_setting)  #信号交差点の南側出口ノード
    W.addNode("N_o", 0, 1, signal=signal_phase_setting)
    W.addNode("W_o", -1, 0, signal=signal_phase_setting)
    W.addNode("E_o", 1, 0.2, signal=signal_phase_setting)

    W.addLink("linkSin", "S_orig", "S_i", length=200, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=[0,1], number_of_lanes=2)   #交差点への流入リンク
    W.addLink("linkNin", "N_orig", "N_i", length=200, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=[0,1], number_of_lanes=2)
    W.addLink("linkWin", "W_orig", "W_i", length=200, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=[2,3], number_of_lanes=2)
    W.addLink("linkEin", "E_orig", "E_i", length=200, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=[2,3], number_of_lanes=2)
    W.addLink("linkSout", "S_o", "S_dest", length=200, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)   #交差点からの流出リンク
    W.addLink("linkNout", "N_o", "N_dest", length=200, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    W.addLink("linkWout", "W_o", "W_dest", length=200, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
    W.addLink("linkEout", "E_o", "E_dest", length=200, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)

    #交差点内部ダミーリンク
    W.addLink("signal_SN_s", "S_i", "N_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=0, number_of_lanes=2)  #直進．lengthは仮のもの．あまりに短すぎると丸め誤差で変なことがおきるかも
    W.addLink("signal_NS_s", "N_i", "S_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=0, number_of_lanes=2)
    W.addLink("signal_SW_l", "S_i", "W_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=0)  #左折
    W.addLink("signal_NE_l", "N_i", "E_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=0)
    W.addLink("signal_SE_r", "S_i", "E_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=1)  #右折．lengthは右折待ち行列の最大長さに相当
    W.addLink("signal_NW_r", "N_i", "W_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=1)
    W.addLink("signal_WE_s", "W_i", "E_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=2, number_of_lanes=2)
    W.addLink("signal_EW_s", "E_i", "W_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=2, number_of_lanes=2)
    W.addLink("signal_WN_l", "W_i", "N_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=2)
    W.addLink("signal_ES_l", "E_i", "S_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=2)
    W.addLink("signal_WS_r", "W_i", "S_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=3)
    W.addLink("signal_EN_r", "E_i", "N_o", length=20, free_flow_speed=20, jam_density_per_lane=0.2, signal_group=3)

    demand_s = 0.4
    demand_l = 0.03
    demand_r = 0.03

    duration = 1000
    dt = 50
    randomness = (0.7, 1.3)
    for t in range(0, duration+1, dt):
        W.adddemand("S_orig", "N_dest", t, t+dt, demand_s*random.uniform(*randomness))
        W.adddemand("S_orig", "E_dest", t, t+dt, demand_l*random.uniform(*randomness))
        W.adddemand("S_orig", "W_dest", t, t+dt, demand_r*random.uniform(*randomness))
        W.adddemand("N_orig", "S_dest", t, t+dt, demand_s*random.uniform(*randomness))
        W.adddemand("N_orig", "E_dest", t, t+dt, demand_l*random.uniform(*randomness))
        W.adddemand("N_orig", "W_dest", t, t+dt, demand_r*random.uniform(*randomness))
        W.adddemand("W_orig", "E_dest", t, t+dt, demand_s*random.uniform(*randomness))
        W.adddemand("W_orig", "N_dest", t, t+dt, demand_l*random.uniform(*randomness))
        W.adddemand("W_orig", "S_dest", t, t+dt, demand_r*random.uniform(*randomness))
        W.adddemand("E_orig", "W_dest", t, t+dt, demand_s*random.uniform(*randomness))
        W.adddemand("E_orig", "N_dest", t, t+dt, demand_l*random.uniform(*randomness))
        W.adddemand("E_orig", "S_dest", t, t+dt, demand_r*random.uniform(*randomness))

    # シミュレーション実行
    #最後までシミュを回す
    W.exec_simulation()

    df = W.analyzer.od_to_pandas()

    # 結果可視化
    W.analyzer.print_simple_stats()

    print("南から北へ直進する経路")
    W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SN_s", "linkNout"], xlim=[0,300])
    W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SN_s", "linkNout"])
    print("南から西へ左折する経路")
    W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SW_l", "linkWout"], xlim=[0,300])
    W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SW_l", "linkWout"])
    print("南から東へ右折する経路")
    W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SE_r", "linkEout"], xlim=[0,300])
    W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SE_r", "linkEout"])

    avt = df["average_travel_time"].values
    print(avt)
    
    referemce_avt = [41.40449658, 65.77142857, 41.56507937, 41.41644018, 41.30793651, 65.97619048, 49.7849498, 49.15555556, 69.54444444, 49.11051701, 70.71428571, 48.62857143]
    for i in range(len(avt)):
        assert equal_tolerance(avt[i], referemce_avt[i])

@pytest.mark.flaky(reruns=5)
def test_route_choice_one_is_too_long_and_another_has_bottleneck():
    vol1s = []
    vol2s = []
    atts1 = []
    atts2 = []

    for i in range(10):
        W = World(
            name="",
            deltan=5, 
            tmax=3000, 
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
        link22 = W.addLink("link22", "mid2", "dest", length=2000, free_flow_speed=20, jam_density=0.2, number_of_lanes=2)
        W.adddemand("orig", "dest", 0, 3000, 1.6)

        W.exec_simulation()
        

        W.analyzer.print_simple_stats()

        W.analyzer.basic_analysis()

        W.analyzer.compute_edie_state()
        # W.analyzer.time_space_diagram_traj_links([link11, link12])
        # W.analyzer.time_space_diagram_traj_links([link21, link22])

        vol1s.append(link11.q_mat[5:,5].sum()*link11.edie_dt)
        vol2s.append(link21.q_mat[5:,5].sum()*link21.edie_dt)
        for veh in W.VEHICLES.values():
            if link12 in veh.log_link:
                atts1.append(veh.travel_time)
            elif link22 in veh.log_link:
                atts2.append(veh.travel_time)
        
    # plt.hist(atts1, bins=100, color="red", alpha=0.5)
    # plt.hist(atts2, bins=100, color="blue", alpha=0.5)
    print(f"{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(atts1) = }\n{np.average(atts2) = }")
    assert equal_tolerance(np.average(vol1s), 1915)
    assert equal_tolerance(np.average(vol1s), 1915)
    assert equal_tolerance(np.average(atts1), 200)
    assert equal_tolerance(np.average(atts2), 200)

def test_route_choice_4route_congestion_avoidance():
    tt1s = []
    tt2s = []
    tt3s = []
    tt4s = []
    vol1s = []
    vol2s = []
    vol3s = []
    vol4s = []
    ttas = []

    for i in range(20):
        W = World(
            name="",
            deltan=5, 
            tmax=3000, 
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100,
            duo_update_weight=0.3
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", 2, 1)
        W.addNode("mid3", 3, 1)
        W.addNode("mid4", 4, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.4, number_of_lanes=2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20, jam_density=0.4, number_of_lanes=2)
        link22 = W.addLink("link22", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
        link31 = W.addLink("link31", "orig", "mid3", length=1000, free_flow_speed=20, jam_density=0.4, number_of_lanes=2)
        link32 = W.addLink("link32", "mid3", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
        link41 = W.addLink("link41", "orig", "mid4", length=1000, free_flow_speed=20, jam_density=0.4, number_of_lanes=2)
        link42 = W.addLink("link42", "mid4", "dest", length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1)
        W.adddemand("orig", "dest", 0, 2000, 0.8*4)

        W.exec_simulation() 

        W.analyzer.print_simple_stats()

        # W.analyzer.time_space_diagram_traj_links([link11, link12])
        # W.analyzer.time_space_diagram_traj_links([link21, link22])
        # W.analyzer.time_space_diagram_traj_links([link31, link32])
        # W.analyzer.time_space_diagram_traj_links([link41, link42])
        W.analyzer.basic_analysis()

        df = W.analyzer.link_to_pandas()

        tt1 = df[df["link"].isin(("link11", "link12"))]["average_travel_time"].sum()
        tt2 = df[df["link"].isin(("link21", "link22"))]["average_travel_time"].sum()
        tt3 = df[df["link"].isin(("link31", "link32"))]["average_travel_time"].sum()
        tt4 = df[df["link"].isin(("link41", "link42"))]["average_travel_time"].sum()
        vol1 = df[df["link"].isin(["link11"])]["traffic_volume"].values[0]
        vol2 = df[df["link"].isin(["link21"])]["traffic_volume"].values[0]
        vol3 = df[df["link"].isin(["link31"])]["traffic_volume"].values[0]
        vol4 = df[df["link"].isin(["link41"])]["traffic_volume"].values[0]

        df2 = W.analyzer.od_to_pandas()
        tta = df2["average_travel_time"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        tt4s.append(tt4)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)
        vol4s.append(vol4)
            
        ttas.append(tta)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(tt4s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }\n{np.average(vol4s) = }\n{np.average(np.concatenate((tt1s, tt2s, tt3s, tt4s))) = }\n{np.average(np.concatenate((vol1s, vol2s, vol3s, vol4s))) = }")
    ttave = np.average(np.concatenate((tt1s, tt2s, tt3s, tt4s)))
    volave = np.average(np.concatenate((vol1s, vol2s, vol3s, vol4s)))

    assert equal_tolerance(np.average(tt1s), ttave, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), ttave, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), ttave, rel_tol=0.2)
    assert equal_tolerance(np.average(tt4s), ttave, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), volave, rel_tol=0.2)
    assert equal_tolerance(np.average(vol2s), volave, rel_tol=0.2)
    assert equal_tolerance(np.average(vol3s), volave, rel_tol=0.2)
    assert equal_tolerance(np.average(vol4s), volave, rel_tol=0.2)
    assert equal_tolerance(ttave, 225, rel_tol=0.2)
    assert equal_tolerance(volave*4, 2000*0.8*4, rel_tol=0.2)
