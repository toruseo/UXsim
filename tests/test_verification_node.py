"""
This script verifies whether UXsim outputs reasonable solutions for merging/diverging/intersection nodes in various configurations.
Note that it uses random numbers for node traffic behavior, so the results may vary slightly between runs.
"""

import pytest
from uxsim import *
import pandas as pd

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

def q2k_cong(q, link):
    return -(q-link.kappa*link.w)/link.w

@pytest.mark.flaky(reruns=5)
def test_merge_fair_nocongestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=1200,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 1000, 0.3)
        W.adddemand("orig2", "dest", 0, 1000, 0.3)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    assert equal_tolerance(np.average(tt1s), 50)
    assert equal_tolerance(np.average(tt2s), 50)
    assert equal_tolerance(np.average(tt3s), 50)
    assert equal_tolerance(np.average(vol1s), 300)
    assert equal_tolerance(np.average(vol2s), 300)
    assert equal_tolerance(np.average(vol3s), 600)

@pytest.mark.flaky(reruns=5)
def test_merge_fair_congestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 1000, 0.5)
        W.adddemand("orig2", "dest", 0, 1000, 0.5)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }")

    assert equal_tolerance(np.average(tt1s), np.average(tt2s))
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 500)
    assert equal_tolerance(np.average(vol2s), 500)
    assert equal_tolerance(np.average(vol3s), 1000)

@pytest.mark.flaky(reruns=5)
def test_merge_unfair():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1.5)
        W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 1000, 0.5)
        W.adddemand("orig2", "dest", 0, 1000, 0.5)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }")

    assert np.average(tt1s) > np.average(tt2s)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 500)
    assert equal_tolerance(np.average(vol2s), 500)
    assert equal_tolerance(np.average(vol3s), 1000)

@pytest.mark.flaky(reruns=5)
def test_merge_veryunfair():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=100)
        W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 1000, 0.5)
        W.adddemand("orig2", "dest", 0, 1000, 0.5)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }")

    assert np.average(tt1s) > np.average(tt2s)
    assert equal_tolerance(np.average(tt2s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 500)
    assert equal_tolerance(np.average(vol2s), 500)
    assert equal_tolerance(np.average(vol3s), 1000)

@pytest.mark.flaky(reruns=5)
def test_diverge_nocongestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(1):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig", 0, 0) 
        W.addNode("diverge", 1, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, -1)
        W.addLink("link1", "orig", "diverge", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link2", "diverge", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link3", "diverge", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest1", 0, 1000, 0.3)
        W.adddemand("orig", "dest2", 0, 1000, 0.3)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }")

    assert equal_tolerance(np.average(tt1s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 600)
    assert equal_tolerance(np.average(vol2s), 300)
    assert equal_tolerance(np.average(vol3s), 300)

@pytest.mark.flaky(reruns=5)
def test_diverge_lesscapacity_nocongestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0) 
        W.addNode("diverge", 1, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, -1)
        W.addLink("link1", "orig", "diverge", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link2", "diverge", "dest1", length=1000, free_flow_speed=20, jam_density=0.2, capacity_in=0.2)
        W.addLink("link3", "diverge", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest1", 0, 1000, 0.2)
        W.adddemand("orig", "dest2", 0, 1000, 0.3)

        W.exec_simulation()

        W.analyzer.print_simple_stats()
        # W.analyzer.time_space_diagram_traj_links(["link1", "link3"])
        # W.analyzer.time_space_diagram_traj_links(["link1", "link2"])

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }")

    assert equal_tolerance(np.average(tt1s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 500)
    assert equal_tolerance(np.average(vol2s), 200)
    assert equal_tolerance(np.average(vol3s), 300)

@pytest.mark.flaky(reruns=5)
def test_diverge_lesscapacity_congestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None
        )

        W.addNode("orig", 0, 0) 
        W.addNode("diverge", 1, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, -1)
        W.addLink("link1", "orig", "diverge", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link2", "diverge", "dest1", length=1000, free_flow_speed=20, jam_density=0.2, capacity_in=0.2)
        W.addLink("link3", "diverge", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest1", 0, 1000, 0.3)
        W.adddemand("orig", "dest2", 0, 1000, 0.2)

        W.exec_simulation()

        W.analyzer.print_simple_stats()
        # W.analyzer.time_space_diagram_traj_links(["link1", "link3"])
        # W.analyzer.time_space_diagram_traj_links(["link1", "link2"])

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }")

    assert equal_tolerance(np.average(tt1s), 333, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 500)
    assert equal_tolerance(np.average(vol2s), 300)
    assert equal_tolerance(np.average(vol3s), 200)

@pytest.mark.flaky(reruns=5)
def test_merge_flowcheck():
    q1s = []
    q2s = []
    q3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        link2 = W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2)
        link3 = W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 2000, 0.6)
        W.adddemand("orig2", "dest", 0, 2000, 0.6)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1 = link1.q_mat[8:,5].mean()
        q2 = link2.q_mat[8:,5].mean()
        q3 = link3.q_mat[8:,5].mean()

        q1s.append(q1)
        q2s.append(q2)
        q3s.append(q3)

    q1 = np.average(q1s)
    q2 = np.average(q2s)
    q3 = np.average(q3s)
    print(f"{q1 = }\n{q2 = }\n{q3 = }")

    assert equal_tolerance(q1*2, q2)
    assert equal_tolerance(q1 + q2, q3)
    assert equal_tolerance(q3, 0.8)

@pytest.mark.flaky(reruns=5)
def test_merge_flowcheck_deltan1():
    q1s = []
    q2s = []
    q3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=1,
            tmax=500,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "merge", length=500, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        link2 = W.addLink("link2", "orig2", "merge", length=500, free_flow_speed=20, jam_density=0.2, merge_priority=2)
        link3 = W.addLink("link3", "merge", "dest", length=500, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 500, 0.6)
        W.adddemand("orig2", "dest", 0, 500, 0.6)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1 = link1.q_mat[2:,2].mean()
        q2 = link2.q_mat[2:,2].mean()
        q3 = link3.q_mat[2:,2].mean()

        q1s.append(q1)
        q2s.append(q2)
        q3s.append(q3)

    q1 = np.average(q1s)
    q2 = np.average(q2s)
    q3 = np.average(q3s)
    print(f"{q1 = }\n{q2 = }\n{q3 = }")

    assert equal_tolerance(q1*2, q2)
    assert equal_tolerance(q1 + q2, q3)
    assert equal_tolerance(q3, 0.8)

@pytest.mark.flaky(reruns=5)
def test_merge_flowcheck_veryunfair():
    q1s = []
    q2s = []
    v2s = []
    q3s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("merge", 1, 1)
        W.addNode("dest", 2, 1)
        link1 = W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1)
        link2 = W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=5)
        link3 = W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest", 0, 2000, 0.6)
        W.adddemand("orig2", "dest", 0, 2000, 0.6)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        W.analyzer.compute_edie_state()
        q1 = link1.q_mat[8:,5].mean()
        q2 = link2.q_mat[8:,5].mean()
        q3 = link3.q_mat[8:,5].mean()
        v2 = link2.v_mat[8:,:].mean()

        q1s.append(q1)
        q2s.append(q2)
        q3s.append(q3)
        v2s.append(v2)

    q1 = np.average(q1s)
    q2 = np.average(q2s)
    q3 = np.average(q3s)
    v2 = np.average(v2s)
    print(f"{q1 = }\n{q2 = }\n{q3 = }\n{v2 = }")

    assert equal_tolerance(q1, 0.2)
    assert equal_tolerance(q2, 0.6)
    assert equal_tolerance(q3, 0.8)
    assert equal_tolerance(v2, 20, rel_tol=0.2)


@pytest.mark.flaky(reruns=5)
def test_diverge_flowcheck():
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
        link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density=0.2,   number_of_lanes=1)
        link2 = W.addLink("link2", "mid", "dest1", length=1000, free_flow_speed=20, jam_density=0.2,  number_of_lanes=1, capacity_in=0.4)
        link3 = W.addLink("link3", "mid", "dest2",  length=1000, free_flow_speed=20, jam_density=0.2, number_of_lanes=1, capacity_in=0.4)
        t1 = 1000
        flow1 = 0.8*2/3
        t2 = 1000
        flow2 = 0.8*1/3
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

    print(np.average(q1s), 0.6) 
    print(np.average(k1s), q2k_cong(0.6, link1))
    print(np.average(v1s), 0.6/q2k_cong(0.6, link1))
    print(np.average(q2s), 0.4)
    print(np.average(k2s), 0.02)
    print(np.average(q3s), 0.2) 
    print(np.average(k3s), 0.01)

@pytest.mark.flaky(reruns=5)
def test_2to2_no_flow_to_one_dest():
    tt1s = []
    tt2s = []
    tt3s = []
    tt4s = []
    vol1s = []
    vol2s = []
    vol3s = []
    vol4s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("inter", 1, 1)
        W.addNode("dest1", 2, 0)
        W.addNode("dest2", 2, 2)
        link1 = W.addLink("link1", "orig1", "inter", length=1000, free_flow_speed=20, jam_density=0.2)
        link2 = W.addLink("link2", "orig2", "inter", length=1000, free_flow_speed=20, jam_density=0.2)
        link3 = W.addLink("link3", "inter", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        link4 = W.addLink("link4", "inter", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest1", 0, 1000, 0.6)
        W.adddemand("orig2", "dest1", 0, 1000, 0.6)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]
        tt4 = df[df["link"]=="link4"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]
        vol4 = df[df["link"]=="link4"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        tt4s.append(tt4)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)
        vol4s.append(vol4)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(tt4s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }\n{np.average(vol4s) = }")

    assert equal_tolerance(np.average(tt1s), 250, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 250, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt4s), -1, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 600)
    assert equal_tolerance(np.average(vol2s), 600)
    assert equal_tolerance(np.average(vol3s), 1200)
    assert equal_tolerance(np.average(vol4s), 0)

@pytest.mark.flaky(reruns=5)
def test_2to2_noconstraint():
    tt1s = []
    tt2s = []
    tt3s = []
    tt4s = []
    vol1s = []
    vol2s = []
    vol3s = []
    vol4s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("inter", 1, 1)
        W.addNode("dest1", 2, 0)
        W.addNode("dest2", 2, 2)
        link1 = W.addLink("link1", "orig1", "inter", length=1000, free_flow_speed=20, jam_density=0.2)
        link2 = W.addLink("link2", "orig2", "inter", length=1000, free_flow_speed=20, jam_density=0.2)
        link3 = W.addLink("link3", "inter", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        link4 = W.addLink("link4", "inter", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest1", 0, 1000, 0.3)
        W.adddemand("orig2", "dest1", 0, 1000, 0.3)
        W.adddemand("orig1", "dest2", 0, 1000, 0.3)
        W.adddemand("orig2", "dest2", 0, 1000, 0.3)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]
        tt4 = df[df["link"]=="link4"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]
        vol4 = df[df["link"]=="link4"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        tt4s.append(tt4)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)
        vol4s.append(vol4)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(tt4s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }\n{np.average(vol4s) = }")

    assert equal_tolerance(np.average(tt1s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt4s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 600)
    assert equal_tolerance(np.average(vol2s), 600)
    assert equal_tolerance(np.average(vol3s), 600)
    assert equal_tolerance(np.average(vol4s), 600)

@pytest.mark.flaky(reruns=5)
def test_2to2_signal_oversaturated():
    tt1s = []
    tt2s = []
    tt3s = []
    tt4s = []
    vol1s = []
    vol2s = []
    vol3s = []
    vol4s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("inter", 1, 1, signal=[60,60])
        W.addNode("dest1", 2, 0)
        W.addNode("dest2", 2, 2)
        link1 = W.addLink("link1", "orig1", "inter", length=1000, free_flow_speed=20, jam_density=0.2, signal_group=0)
        link2 = W.addLink("link2", "orig2", "inter", length=1000, free_flow_speed=20, jam_density=0.2, signal_group=1)
        link3 = W.addLink("link3", "inter", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        link4 = W.addLink("link4", "inter", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest1", 0, 1000, 0.3)
        W.adddemand("orig2", "dest1", 0, 1000, 0.3)
        W.adddemand("orig1", "dest2", 0, 1000, 0.3)
        W.adddemand("orig2", "dest2", 0, 1000, 0.3)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]
        tt4 = df[df["link"]=="link4"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]
        vol4 = df[df["link"]=="link4"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        tt4s.append(tt4)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)
        vol4s.append(vol4)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(tt4s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }\n{np.average(vol4s) = }")

    assert equal_tolerance(np.average(tt1s), 250, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 250, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt4s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 600)
    assert equal_tolerance(np.average(vol2s), 600)
    assert equal_tolerance(np.average(vol3s), 600)
    assert equal_tolerance(np.average(vol4s), 600)

@pytest.mark.flaky(reruns=5)
def test_2to2_signal_undersaturated():
    tt1s = []
    tt2s = []
    tt3s = []
    tt4s = []
    vol1s = []
    vol2s = []
    vol3s = []
    vol4s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("inter", 1, 1, signal=[60,60])
        W.addNode("dest1", 2, 0)
        W.addNode("dest2", 2, 2)
        link1 = W.addLink("link1", "orig1", "inter", length=1000, free_flow_speed=20, jam_density=0.2, signal_group=0)
        link2 = W.addLink("link2", "orig2", "inter", length=1000, free_flow_speed=20, jam_density=0.2, signal_group=1)
        link3 = W.addLink("link3", "inter", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        link4 = W.addLink("link4", "inter", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest1", 0, 1000, 0.15)
        W.adddemand("orig2", "dest1", 0, 1000, 0.15)
        W.adddemand("orig1", "dest2", 0, 1000, 0.15)
        W.adddemand("orig2", "dest2", 0, 1000, 0.15)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]
        tt4 = df[df["link"]=="link4"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]
        vol4 = df[df["link"]=="link4"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        tt4s.append(tt4)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)
        vol4s.append(vol4)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(tt4s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }\n{np.average(vol4s) = }")

    assert equal_tolerance(np.average(tt1s), 70, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 90, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt4s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 300)
    assert equal_tolerance(np.average(vol2s), 300)
    assert equal_tolerance(np.average(vol3s), 300)
    assert equal_tolerance(np.average(vol4s), 300)

@pytest.mark.flaky(reruns=5)
def test_2to2_signal_macroscopic_signal():
    tt1s = []
    tt2s = []
    tt3s = []
    tt4s = []
    vol1s = []
    vol2s = []
    vol3s = []
    vol4s = []
    for i in range(10):
        W = World(
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=0,
            random_seed=None
        )

        W.addNode("orig1", 0, 0) 
        W.addNode("orig2", 0, 2)
        W.addNode("inter", 1, 1, flow_capacity=0.8)
        W.addNode("dest1", 2, 0)
        W.addNode("dest2", 2, 2)
        link1 = W.addLink("link1", "orig1", "inter", length=1000, free_flow_speed=20, jam_density=0.2)
        link2 = W.addLink("link2", "orig2", "inter", length=1000, free_flow_speed=20, jam_density=0.2)
        link3 = W.addLink("link3", "inter", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        link4 = W.addLink("link4", "inter", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig1", "dest1", 0, 1000, 0.3)
        W.adddemand("orig2", "dest1", 0, 1000, 0.3)
        W.adddemand("orig1", "dest2", 0, 1000, 0.3)
        W.adddemand("orig2", "dest2", 0, 1000, 0.3)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"]=="link1"]["average_travel_time"].values[0]
        tt2 = df[df["link"]=="link2"]["average_travel_time"].values[0]
        tt3 = df[df["link"]=="link3"]["average_travel_time"].values[0]
        tt4 = df[df["link"]=="link4"]["average_travel_time"].values[0]

        vol1 = df[df["link"]=="link1"]["traffic_volume"].values[0]
        vol2 = df[df["link"]=="link2"]["traffic_volume"].values[0]
        vol3 = df[df["link"]=="link3"]["traffic_volume"].values[0]
        vol4 = df[df["link"]=="link4"]["traffic_volume"].values[0]

        tt1s.append(tt1)
        tt2s.append(tt2)
        tt3s.append(tt3)
        tt4s.append(tt4)
        vol1s.append(vol1)
        vol2s.append(vol2)
        vol3s.append(vol3)
        vol4s.append(vol4)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(tt3s) = }\n{np.average(tt4s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(vol3s) = }\n{np.average(vol4s) = }")

    assert equal_tolerance(np.average(tt1s), 250, rel_tol=0.2)
    assert equal_tolerance(np.average(tt2s), 250, rel_tol=0.2)
    assert equal_tolerance(np.average(tt3s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(tt4s), 50, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), 600)
    assert equal_tolerance(np.average(vol2s), 600)
    assert equal_tolerance(np.average(vol3s), 600)
    assert equal_tolerance(np.average(vol4s), 600)


@pytest.mark.flaky(reruns=5)
def test_4phase_signal_jpstyle():
    dfs = []
    for i in range(1):
        W = World(
            name="",
            deltan=1,
            tmax=1500,
            print_mode=0, save_mode=0, show_mode=1,
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

        W.addLink("linkSin", "S_orig", "S_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[0,1])   #交差点への流入リンク
        W.addLink("linkNin", "N_orig", "N_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[0,1])
        W.addLink("linkWin", "W_orig", "W_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[2,3])
        W.addLink("linkEin", "E_orig", "E_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[2,3])
        W.addLink("linkSout", "S_o", "S_dest", length=200, free_flow_speed=20, jam_density=0.2)   #交差点からの流出リンク
        W.addLink("linkNout", "N_o", "N_dest", length=200, free_flow_speed=20, jam_density=0.2)
        W.addLink("linkWout", "W_o", "W_dest", length=200, free_flow_speed=20, jam_density=0.2)
        W.addLink("linkEout", "E_o", "E_dest", length=200, free_flow_speed=20, jam_density=0.2)

        #交差点内部ダミーリンク
        W.addLink("signal_SN_s", "S_i", "N_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)  #直進．lengthは仮のもの．あまりに短すぎると丸め誤差で変なことがおきるかも
        W.addLink("signal_NS_s", "N_i", "S_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)
        W.addLink("signal_SW_l", "S_i", "W_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)  #左折
        W.addLink("signal_NE_l", "N_i", "E_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)
        W.addLink("signal_SE_r", "S_i", "E_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=1)  #右折．lengthは右折待ち行列の最大長さに相当
        W.addLink("signal_NW_r", "N_i", "W_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=1)
        W.addLink("signal_WE_s", "W_i", "E_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
        W.addLink("signal_EW_s", "E_i", "W_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
        W.addLink("signal_WN_l", "W_i", "N_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
        W.addLink("signal_ES_l", "E_i", "S_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
        W.addLink("signal_WS_r", "W_i", "S_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=3)
        W.addLink("signal_EN_r", "E_i", "N_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=3)

        demand_s = 0.2
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
        dfs.append(df)
        
        # # 結果可視化
        # W.analyzer.print_simple_stats()

        # print("南から北へ直進する経路")
        # W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SN_s", "linkNout"], xlim=[0,300])
        # W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SN_s", "linkNout"])
        # print("南から西へ左折する経路")
        # W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SW_l", "linkWout"], xlim=[0,300])
        # W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SW_l", "linkWout"])
        # print("南から東へ右折する経路")
        # W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SE_r", "linkEout"], xlim=[0,300])
        # W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SE_r", "linkEout"])

    # df_concat = pd.concat(dfs)
    # df_mean = df_concat.groupby(df_concat.index).mean()
    # avt = df_mean["average_travel_time"].values

    avt = df["average_travel_time"].values
    print(avt)
    referemce_avt = [41.40449658, 65.77142857, 41.56507937, 41.41644018, 41.30793651, 65.97619048, 49.7849498, 49.15555556, 69.54444444, 49.11051701, 70.71428571, 48.62857143]
    for i in range(len(avt)):
        assert equal_tolerance(avt[i], referemce_avt[i])
