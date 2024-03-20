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
        W.addNode("merge", 1, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, -1)
        W.addLink("link1", "orig", "merge", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link2", "merge", "dest1", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link3", "merge", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
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
        W.addNode("merge", 1, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, -1)
        W.addLink("link1", "orig", "merge", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link2", "merge", "dest1", length=1000, free_flow_speed=20, jam_density=0.2, capacity_in=0.2)
        W.addLink("link3", "merge", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
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
        W.addNode("merge", 1, 0)
        W.addNode("dest1", 2, 1)
        W.addNode("dest2", 2, -1)
        W.addLink("link1", "orig", "merge", length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("link2", "merge", "dest1", length=1000, free_flow_speed=20, jam_density=0.2, capacity_in=0.2)
        W.addLink("link3", "merge", "dest2", length=1000, free_flow_speed=20, jam_density=0.2)
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

#TODO: add tests later, check flow values