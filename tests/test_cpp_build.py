"""
Tests to verify C++ extension module builds and works correctly.
Based on UXsimpp-main/tests/test_for_build.py, adapted for integrated UXsim.

These tests use the C++ module (uxsim_cpp) directly via its raw API,
without depending on the Python wrapper layer (uxsim.py integration).
Once the Python integration is complete (by Bob), higher-level tests
using the unified API should also be added.
"""

import pytest
import random
import warnings

import numpy as np

warnings.filterwarnings("ignore", message=".*cannot collect 'test' because it is not a function.*")


# -----------------------------------------------------------------------
# Helpers (equivalent to UXsimpp's Python wrapper functions)
# -----------------------------------------------------------------------

def eq_tol(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    print(f"  eq_tol: {val} vs {check}")
    return abs(val - check) <= abs(check * rel_tol) + abs_tol


def make_world(name="test", tmax=3000.0, deltan=5.0, tau=1.0,
               duo_update_time=300.0, duo_update_weight=0.25,
               print_mode=1, random_seed=42):
    from uxsim import uxsim_cpp
    return uxsim_cpp.create_world(
        name, tmax, deltan, tau,
        duo_update_time, duo_update_weight,
        0.0,            # route_choice_uncertainty
        print_mode,
        random_seed,
        True,           # vehicle_log_mode
    )


def inflow(link, t1, t2, delta_t):
    return (link.arrival_curve[int(t2/delta_t)] - link.arrival_curve[int(t1/delta_t)]) / (t2 - t1)


def outflow(link, t1, t2, delta_t):
    return (link.departure_curve[int(t2/delta_t)] - link.departure_curve[int(t1/delta_t)]) / (t2 - t1)


# -----------------------------------------------------------------------
# MARK: Import test
# -----------------------------------------------------------------------

def test_cpp_module_import():
    """Verify that the C++ extension module is importable."""
    from uxsim import uxsim_cpp
    assert hasattr(uxsim_cpp, "create_world")
    assert hasattr(uxsim_cpp, "add_node")
    assert hasattr(uxsim_cpp, "add_link")
    assert hasattr(uxsim_cpp, "add_demand")
    assert hasattr(uxsim_cpp, "World")
    assert hasattr(uxsim_cpp, "Node")
    assert hasattr(uxsim_cpp, "Link")
    assert hasattr(uxsim_cpp, "Vehicle")
    assert hasattr(uxsim_cpp, "get_compile_datetime")


# -----------------------------------------------------------------------
# MARK: Straight road
# -----------------------------------------------------------------------

def test_cpp_1link_freeflow():
    """Basic single-link free flow test using C++ backend."""
    from uxsim import uxsim_cpp

    W = make_world(random_seed=42)

    uxsim_cpp.add_node(W, "orig", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "dest", 1, 0, [0], 0)

    uxsim_cpp.add_link(W, "link", "orig", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])

    uxsim_cpp.add_demand(W, "orig", "dest", 0, 1000, 0.5, [])

    W.initialize_adj_matrix()
    W.print_scenario_stats()
    W.main_loop(-1, -1)
    W.print_simple_results()

    link = W.get_link("link")
    dt = W.delta_t

    assert eq_tol(inflow(link, 0, 1000, dt), 0.5)
    assert eq_tol(inflow(link, 1000, 2000, dt), 0)

    assert eq_tol(outflow(link, 0, 500, dt), 0)
    assert eq_tol(outflow(link, 500, 1500, dt), 0.5)
    assert eq_tol(outflow(link, 1500, 2500, dt), 0)

    assert eq_tol(W.VEHICLES[0].travel_time, 500)
    assert eq_tol(W.VEHICLES[-1].travel_time, 500)


# -----------------------------------------------------------------------
# MARK: Merge
# -----------------------------------------------------------------------

def test_cpp_merge_free():
    """Merge junction free flow test using C++ backend."""
    from uxsim import uxsim_cpp

    W = make_world(random_seed=42)

    uxsim_cpp.add_node(W, "orig1", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "orig2", 0, 2, [0], 0)
    uxsim_cpp.add_node(W, "merge", 1, 1, [0], 0)
    uxsim_cpp.add_node(W, "dest", 2, 1, [0], 0)

    uxsim_cpp.add_link(W, "link1", "orig1", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link2", "orig2", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link3", "merge", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])

    uxsim_cpp.add_demand(W, "orig1", "dest", 0, 1000, 0.3, [])
    uxsim_cpp.add_demand(W, "orig2", "dest", 0, 1000, 0.3, [])

    W.initialize_adj_matrix()
    W.print_scenario_stats()
    W.main_loop(-1, -1)
    W.print_simple_results()

    l1 = W.get_link("link1")
    l2 = W.get_link("link2")
    l3 = W.get_link("link3")
    dt = W.delta_t

    assert eq_tol(inflow(l1, 0, 1000, dt), 0.3)
    assert eq_tol(outflow(l1, 500, 1500, dt), 0.3)
    assert eq_tol(outflow(l1, 1500, 2000, dt), 0)

    assert eq_tol(inflow(l2, 0, 1000, dt), 0.3)
    assert eq_tol(outflow(l2, 500, 1500, dt), 0.3)
    assert eq_tol(outflow(l2, 1500, 2000, dt), 0)

    assert eq_tol(inflow(l3, 0, 500, dt), 0.0)
    assert eq_tol(inflow(l3, 500, 1500, dt), 0.6)
    assert eq_tol(inflow(l3, 1500, 2000, dt), 0.0)

    assert eq_tol(outflow(l3, 0, 1000, dt), 0.0)
    assert eq_tol(outflow(l3, 1000, 2000, dt), 0.6)
    assert eq_tol(outflow(l3, 2000, 2900, dt), 0.0)


# -----------------------------------------------------------------------
# MARK: Route choice
# -----------------------------------------------------------------------

def test_cpp_routechoice_faster_1route():
    """Route choice test: all traffic should take the faster route."""
    from uxsim import uxsim_cpp

    W = make_world(random_seed=random.randint(0, 999999))

    uxsim_cpp.add_node(W, "orig", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "mid1", 1, 1, [0], 0)
    uxsim_cpp.add_node(W, "mid2", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "dest", 2, 1, [0], 0)

    uxsim_cpp.add_link(W, "link1a", "orig", "mid1", 20, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link1b", "mid1", "dest", 20, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link2a", "orig", "mid2", 10, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link2b", "mid2", "dest", 10, 0.2, 1000, 1, 1, -1, -1, [0])

    uxsim_cpp.add_demand(W, "orig", "dest", 0, 1000, 0.6, [])

    W.initialize_adj_matrix()
    W.print_scenario_stats()
    W.main_loop(-1, -1)
    W.print_simple_results()

    l1a = W.get_link("link1a")
    l2a = W.get_link("link2a")
    dt = W.delta_t

    assert eq_tol(inflow(l1a, 0, 1000, dt), 0.6)
    assert eq_tol(inflow(l2a, 0, 1000, dt), 0)


# -----------------------------------------------------------------------
# MARK: Preferred route
# -----------------------------------------------------------------------

def test_cpp_route_specified():
    """Route choice test: vehicles should follow preferred links."""
    from uxsim import uxsim_cpp

    W = make_world(random_seed=random.randint(0, 999999))

    uxsim_cpp.add_node(W, "orig0", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "orig", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "mid1", 1, 1, [0], 0)
    uxsim_cpp.add_node(W, "mid2", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "dest", 2, 1, [0], 0)

    uxsim_cpp.add_link(W, "link0", "orig0", "orig", 20, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link1a", "orig", "mid1", 20, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link1b", "mid1", "dest", 20, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link2a", "orig", "mid2", 10, 0.2, 1000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link2b", "mid2", "dest", 10, 0.2, 1000, 1, 1, -1, -1, [0])

    uxsim_cpp.add_demand(W, "orig0", "dest", 0, 1000, 0.6, ["link2a", "link2b"])

    W.initialize_adj_matrix()
    W.print_scenario_stats()
    W.main_loop(-1, -1)
    W.print_simple_results()

    l1a = W.get_link("link1a")
    l2a = W.get_link("link2a")
    dt = W.delta_t

    assert eq_tol(inflow(l1a, 0, 1000, dt), 0)
    assert eq_tol(inflow(l2a, 50, 1050, dt), 0.6)


# -----------------------------------------------------------------------
# MARK: Signal
# -----------------------------------------------------------------------

def test_cpp_signal_straight():
    """Signal test on a straight road."""
    from uxsim import uxsim_cpp

    W = make_world(random_seed=42)

    uxsim_cpp.add_node(W, "orig", 0, 0, [0], 0)
    uxsim_cpp.add_node(W, "mid", 0.5, 0, [60, 60], 0)
    uxsim_cpp.add_node(W, "dest", 1, 0, [0], 0)

    uxsim_cpp.add_link(W, "link1", "orig", "mid", 20, 0.2, 10000, 1, 1, -1, -1, [0])
    uxsim_cpp.add_link(W, "link2", "mid", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])

    uxsim_cpp.add_demand(W, "orig", "dest", 0, 1000, 0.4, [])
    uxsim_cpp.add_demand(W, "orig", "dest", 1000, 2000, 0.8, [])

    W.initialize_adj_matrix()
    W.print_scenario_stats()
    W.main_loop(-1, -1)
    W.print_simple_results()

    link1 = W.get_link("link1")
    dt = W.delta_t

    assert eq_tol(outflow(link1, 500, 2500, dt), 0.4)
    assert eq_tol(outflow(link1, 500, 1500, dt), 0.4)
    assert eq_tol(outflow(link1, 1500, 2500, dt), 0.4)


# -----------------------------------------------------------------------
# MARK: Iterative execution
# -----------------------------------------------------------------------

def test_cpp_iterative_execution():
    """Verify iterative execution produces same results as full execution."""
    from uxsim import uxsim_cpp

    random.seed(None)
    for i in range(3):
        seed = random.randint(0, 300)

        # Full execution
        W = make_world(random_seed=seed)
        uxsim_cpp.add_node(W, "orig1", 0, 0, [0], 0)
        uxsim_cpp.add_node(W, "orig2", 0, 2, [0], 0)
        uxsim_cpp.add_node(W, "merge", 1, 1, [0], 0)
        uxsim_cpp.add_node(W, "dest", 2, 1, [0], 0)
        uxsim_cpp.add_link(W, "link1", "orig1", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W, "link2", "orig2", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W, "link3", "merge", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_demand(W, "orig1", "dest", 0, 1000, 0.5, [])
        uxsim_cpp.add_demand(W, "orig2", "dest", 0, 1000, 0.5, [])

        W.initialize_adj_matrix()
        W.main_loop(-1, -1)

        vehicle_log_x = list(W.VEHICLES[160].log_x)
        vehicle_log_v = list(W.VEHICLES[160].log_v)

        # Iterative execution with same seed
        W2 = make_world(random_seed=seed)
        uxsim_cpp.add_node(W2, "orig1", 0, 0, [0], 0)
        uxsim_cpp.add_node(W2, "orig2", 0, 2, [0], 0)
        uxsim_cpp.add_node(W2, "merge", 1, 1, [0], 0)
        uxsim_cpp.add_node(W2, "dest", 2, 1, [0], 0)
        uxsim_cpp.add_link(W2, "link1", "orig1", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W2, "link2", "orig2", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W2, "link3", "merge", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_demand(W2, "orig1", "dest", 0, 1000, 0.5, [])
        uxsim_cpp.add_demand(W2, "orig2", "dest", 0, 1000, 0.5, [])

        W2.initialize_adj_matrix()
        while W2.check_simulation_ongoing():
            if W2.time < 50:
                t = random.randint(0, 20)
            else:
                t = random.randint(0, 300)
            W2.main_loop(t, -1)

        vehicle_log_x2 = list(W2.VEHICLES[160].log_x)
        vehicle_log_v2 = list(W2.VEHICLES[160].log_v)

        assert sum(vehicle_log_x) == sum(vehicle_log_x2)
        assert np.average(vehicle_log_v) == np.average(vehicle_log_v2)


def test_cpp_iterative_execution_until_t():
    """Verify iterative execution with until_t produces same results."""
    from uxsim import uxsim_cpp

    random.seed(None)
    for i in range(3):
        seed = random.randint(0, 300)

        # Full execution
        W = make_world(random_seed=seed)
        uxsim_cpp.add_node(W, "orig1", 0, 0, [0], 0)
        uxsim_cpp.add_node(W, "orig2", 0, 2, [0], 0)
        uxsim_cpp.add_node(W, "merge", 1, 1, [0], 0)
        uxsim_cpp.add_node(W, "dest", 2, 1, [0], 0)
        uxsim_cpp.add_link(W, "link1", "orig1", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W, "link2", "orig2", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W, "link3", "merge", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_demand(W, "orig1", "dest", 0, 1000, 0.5, [])
        uxsim_cpp.add_demand(W, "orig2", "dest", 0, 1000, 0.5, [])

        W.initialize_adj_matrix()
        W.main_loop(-1, -1)

        vehicle_log_x = list(W.VEHICLES[160].log_x)
        vehicle_log_v = list(W.VEHICLES[160].log_v)

        # Iterative execution with until_t
        W2 = make_world(random_seed=seed)
        uxsim_cpp.add_node(W2, "orig1", 0, 0, [0], 0)
        uxsim_cpp.add_node(W2, "orig2", 0, 2, [0], 0)
        uxsim_cpp.add_node(W2, "merge", 1, 1, [0], 0)
        uxsim_cpp.add_node(W2, "dest", 2, 1, [0], 0)
        uxsim_cpp.add_link(W2, "link1", "orig1", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W2, "link2", "orig2", "merge", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_link(W2, "link3", "merge", "dest", 20, 0.2, 10000, 1, 1, -1, -1, [0])
        uxsim_cpp.add_demand(W2, "orig1", "dest", 0, 1000, 0.5, [])
        uxsim_cpp.add_demand(W2, "orig2", "dest", 0, 1000, 0.5, [])

        W2.initialize_adj_matrix()
        while W2.check_simulation_ongoing():
            if W2.time < 50:
                t = random.randint(0, 20)
            else:
                t = random.randint(0, 300)
            W2.main_loop(-1, W2.time + t)

        vehicle_log_x2 = list(W2.VEHICLES[160].log_x)
        vehicle_log_v2 = list(W2.VEHICLES[160].log_v)

        assert sum(vehicle_log_x) == sum(vehicle_log_x2)
        assert np.average(vehicle_log_v) == np.average(vehicle_log_v2)
