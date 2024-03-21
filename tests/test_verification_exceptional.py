"""
This script verifies whether UXsim outputs plausible solutions for exceptional or uncommon situations.
The behavior of UXsim may be updated in the future.
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