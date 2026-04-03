"""
C++ mode integration tests.
All tests from Python mode are fully inlined here with cpp=True.

Excluded (by design):
  - pickle/save/copy: C++ objects are not serializable
  - user_function: Python callbacks not supported in C++ mode
  - osm_import / readme: environment-dependent (optional packages)
"""

import pytest
import random
import inspect
import numpy as np
from numpy import *
import pandas as pd
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')

import uxsim
from uxsim import *
from uxsim.utils import eq_tol
from uxsim.Utilities import *
from uxsim.DTAsolvers import *


# ======================================================================
# From test_verification_straight_road.py
# ======================================================================

def rigorous_verification_of_KW_theory_cumulative_curves_and_travel_time(deltan):
    """
    This function is the most rigorous verification of UXsim, checking that UXsim correctly solves KW theory.
    Specifically, it checks the relation between the cumulative curves and the travel time of vehicles.
    """

    W = World(cpp=True, 
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


def test_1link():
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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

    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_KW_theory_cumulative_curves_and_travel_time():
    rigorous_verification_of_KW_theory_cumulative_curves_and_travel_time(5)


@pytest.mark.flaky(reruns=10)
def test_KW_theory_cumulative_curves_and_travel_time_deltan1():
    rigorous_verification_of_KW_theory_cumulative_curves_and_travel_time(1)


def test_iterative_exec_rigorous():
    for _ in range(20):
        W = World(cpp=True, 
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
        print_columns(["log_t"]+list(W.VEHICLES["0"].log_t), ["log_x"]+list(W.VEHICLES["0"].log_x), ["log_v"]+list(W.VEHICLES["0"].log_v))
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
        W = World(cpp=True, 
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
        print_columns(["log_t"]+list(W.VEHICLES["0"].log_t), ["log_x"]+list(W.VEHICLES["0"].log_x), ["log_v"]+list(W.VEHICLES["0"].log_v))
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
        W = World(cpp=True, 
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
        print_columns(["log_t"]+list(W.VEHICLES["0"].log_t), ["log_x"]+list(W.VEHICLES["0"].log_x), ["log_v"]+list(W.VEHICLES["0"].log_v))
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
            
        W = World(cpp=True, 
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
                    



# ======================================================================
# From test_verification_exceptional.py
# ======================================================================

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol


def test_cannot_reach_destination_no_deadend():
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
        W = World(cpp=True, 
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

    W = World(cpp=True, 
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

    W = World(cpp=True, 
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
        W = World(cpp=True, 
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


# ======================================================================
# From test_verification_multilane.py
# ======================================================================

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol


def q2k_cong(q, link):
    return -(q-link.kappa*link.w)/link.w




@pytest.mark.flaky(reruns=10)
def test_straight_1link_2lane_low_demand():
    W = World(cpp=True, 
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
    

@pytest.mark.flaky(reruns=10)
def test_straight_1link_2lane_high_demand():
    W = World(cpp=True, 
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
    


@pytest.mark.flaky(reruns=10)
def test_straight_2link_2lane_low_demand():
    W = World(cpp=True, 
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
    


@pytest.mark.flaky(reruns=10)
def test_straight_2link_2lane_high_demand():
    W = World(cpp=True, 
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
    



@pytest.mark.flaky(reruns=10)
def test_straight_2link_2lane_to_1lane_congestion():
    W = World(cpp=True, 
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
    


@pytest.mark.flaky(reruns=10)
def test_straight_2link_2lane_to_1lane_congestion_deltan1():
    W = World(cpp=True, 
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
    
    


@pytest.mark.flaky(reruns=10)
def test_straight_2link_2lane_to_3lane():
    W = World(cpp=True, 
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
    

@pytest.mark.flaky(reruns=10)
def test_straight_2link_capacity_out():
        
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_straight_2link_capacity_in():
        
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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



@pytest.mark.flaky(reruns=10)
def test_straight_2link_node_capacity():
        
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_straight_trip_end_in_middle_freeflow():
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_straight_trip_end_in_middle():
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_straight_different_fd():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_straight_different_fd_different_arg():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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



@pytest.mark.flaky(reruns=10)
def test_straight_different_fd_different_lanes():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_straight_different_fd_different_lanes_different_arg():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    v2s = []

    for i in range(1):
        W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    


@pytest.mark.flaky(reruns=10)
def test_merge_small_demand():
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_congested_fair():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_congested_unfair():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(cpp=True, 
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



@pytest.mark.flaky(reruns=10)
def test_merge_congested_unfair_deltan1():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(cpp=True, 
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



@pytest.mark.flaky(reruns=10)
def test_merge_congested_veryunfair():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_congested_different_lanes():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_congested_signal():
    q1s = []
    k1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    v3s = []
    for i in range(5):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_disappear_freeflow():
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_disappear_saturated():
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_diverge_disappear_freeflow():
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_diverge_saturated_2_2_2():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_diverge_saturated_2_1_1_congestion_due_to_friction(): #TODO: frictionは要精査
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(cpp=True, 
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_diverge_congested_2_1_1():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(cpp=True, 
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
    W = World(cpp=True, 
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
        assert equal_tolerance(avt[i], referemce_avt[i], rel_tol=0.2)


@pytest.mark.flaky(reruns=10)
def test_route_choice_one_is_too_long_and_another_has_bottleneck():
    vol1s = []
    vol2s = []
    atts1 = []
    atts2 = []

    for i in range(10):
        W = World(cpp=True, 
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
        W = World(cpp=True, 
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



# ======================================================================
# From test_verification_node.py
# ======================================================================

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol


def q2k_cong(q, link):
    return -(q-link.kappa*link.w)/link.w


@pytest.mark.flaky(reruns=10)
def test_merge_fair_nocongestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(1):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_fair_congestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_unfair():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_veryunfair():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_diverge_nocongestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(1):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_diverge_lesscapacity_nocongestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_diverge_lesscapacity_congestion():
    tt1s = []
    tt2s = []
    tt3s = []
    vol1s = []
    vol2s = []
    vol3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_flowcheck():
    q1s = []
    q2s = []
    q3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_flowcheck_deltan1():
    q1s = []
    q2s = []
    q3s = []
    for i in range(10):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_merge_flowcheck_veryunfair():
    q1s = []
    q2s = []
    v2s = []
    q3s = []
    for i in range(10):
        W = World(cpp=True, 
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



@pytest.mark.flaky(reruns=10)
def test_diverge_flowcheck():
    q1s = []
    k1s = []
    v1s = []
    q2s = []
    k2s = []
    q3s = []
    k3s = []
    for i in range(5):
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
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
        W = World(cpp=True, 
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



@pytest.mark.flaky(reruns=10)
def test_4phase_signal_jpstyle():
    dfs = []
    for i in range(1):
        W = World(cpp=True, 
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



# ======================================================================
# From test_wrapper_functions.py
# ======================================================================

def arg_compare_wrapper(orig, wrapper):
    sig_original = inspect.signature(orig)
    sig_wrapper = inspect.signature(wrapper)

    params_original = list(sig_original.parameters.values())[2:]
    params_wrapper = list(sig_wrapper.parameters.values())[1:]
    
    assert len(params_original) == len(params_wrapper), "Number of args differ"

    for po, pw in zip(params_original, params_wrapper):
        print("checking:", po, "and", pw, end=" ... ")
        assert po.name == pw.name, f"Arg name mismatch: {po.name} != {pw.name}"
        assert po.default == pw.default, f"Default mismatch for {po.name}"
        assert po.kind == pw.kind, f"Kind mismatch for {po.name}"
        print("OK")
    
    return True


def test_addNode():
    assert arg_compare_wrapper(uxsim.Node.__init__, uxsim.World.addNode)


def test_addLink():
    assert arg_compare_wrapper(uxsim.Link.__init__, uxsim.World.addLink)


def test_defRoute():
    assert arg_compare_wrapper(uxsim.Route.__init__, uxsim.World.defRoute)


# ======================================================================
# From test_verification_sioux_falls.py
# ======================================================================

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol


@pytest.mark.flaky(reruns=10)
def test_sioux_falls():
    total_trips_list = []
    completed_trips_list = []
    total_travel_time_list = []
    average_travel_time_list = []
    average_delay_list = []
    link_traffic_volume_mean_list = []
    link_traffic_volume_std_list = []

    for i in range(1):
        W = World(cpp=True, 
            name="",
            deltan=5,
            tmax=7200,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=None
        )

        W.load_scenario_from_csv("dat/siouxfalls_nodes.csv", "dat/siouxfalls_links.csv", "dat/siouxfalls_demand.csv")

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.basic_to_pandas()
        total_trips_list.append(df["total_trips"].values[0])
        completed_trips_list.append(df["completed_trips"].values[0])
        total_travel_time_list.append(df["total_travel_time"].values[0])
        average_travel_time_list.append(df["average_travel_time"].values[0])
        average_delay_list.append(df["average_delay"].values[0])

        df2 = W.analyzer.link_to_pandas()
        link_traffic_volume_mean_list.append(df2["traffic_volume"].mean())
        link_traffic_volume_std_list.append(df2["traffic_volume"].std())
        
        # print(i)
        # for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        #     W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))

    print(f"{np.average(total_trips_list) = }\n{np.average(completed_trips_list) = }\n{np.average(total_travel_time_list) = }\n{np.average(average_travel_time_list) = }\n{np.average(average_delay_list) = }\n{np.average(link_traffic_volume_mean_list) = }\n{np.average(link_traffic_volume_std_list) = }")
    print()
    print(f"{np.std(total_trips_list) = }\n{np.std(completed_trips_list) = }\n{np.std(total_travel_time_list) = }\n{np.std(average_travel_time_list) = }\n{np.std(average_delay_list) = }\n{np.std(link_traffic_volume_mean_list) = }\n{np.std(link_traffic_volume_std_list) = }")

    # Below are stats from 100 iterations
    assert equal_tolerance(np.average(total_trips_list), 34690.0)
    assert equal_tolerance(np.average(completed_trips_list), 33287.75)
    assert equal_tolerance(np.average(total_travel_time_list), 58050854.25)
    assert equal_tolerance(np.average(average_travel_time_list), 1743.9327674417486)
    assert equal_tolerance(np.average(average_delay_list), 372.53428525704396)
    assert equal_tolerance(np.average(link_traffic_volume_mean_list), 1182.0697368421054)
    assert equal_tolerance(np.average(link_traffic_volume_std_list), 748.8503718976154)

    # assert equal_tolerance(np.std(total_trips_list), 0.0, rel_tol=1)
    # assert equal_tolerance(np.std(completed_trips_list), 92.4834444644013, rel_tol=1)
    # assert equal_tolerance(np.std(total_travel_time_list), 419384.3411933587, rel_tol=1)
    # assert equal_tolerance(np.std(average_travel_time_list), 14.604707280467585, rel_tol=1)
    # assert equal_tolerance(np.std(average_delay_list), 15.823545524709168, rel_tol=1)
    # assert equal_tolerance(np.std(link_traffic_volume_mean_list), 5.399030864866247, rel_tol=1)
    # assert equal_tolerance(np.std(link_traffic_volume_std_list), 8.919718941538516, rel_tol=1)



@pytest.mark.flaky(reruns=10)
def test_sioux_falls_gradual():
    total_trips_list = []
    completed_trips_list = []
    total_travel_time_list = []
    average_travel_time_list = []
    average_delay_list = []
    link_traffic_volume_mean_list = []
    link_traffic_volume_std_list = []

    for i in range(1):
        W = World(cpp=True, 
            name="",
            deltan=5,
            tmax=7200,
            print_mode=1, save_mode=0, show_mode=1,
            random_seed=None,
            duo_update_weight=0.8,
            route_choice_update_gradual=True
        )

        W.load_scenario_from_csv("dat/siouxfalls_nodes.csv", "dat/siouxfalls_links.csv", "dat/siouxfalls_demand.csv")

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.basic_to_pandas()
        total_trips_list.append(df["total_trips"].values[0])
        completed_trips_list.append(df["completed_trips"].values[0])
        total_travel_time_list.append(df["total_travel_time"].values[0])
        average_travel_time_list.append(df["average_travel_time"].values[0])
        average_delay_list.append(df["average_delay"].values[0])

        df2 = W.analyzer.link_to_pandas()
        link_traffic_volume_mean_list.append(df2["traffic_volume"].mean())
        link_traffic_volume_std_list.append(df2["traffic_volume"].std())
        
        # print(i)
        # for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        #     W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))

    print(f"{np.average(total_trips_list) = }\n{np.average(completed_trips_list) = }\n{np.average(total_travel_time_list) = }\n{np.average(average_travel_time_list) = }\n{np.average(average_delay_list) = }\n{np.average(link_traffic_volume_mean_list) = }\n{np.average(link_traffic_volume_std_list) = }")
    print()
    print(f"{np.std(total_trips_list) = }\n{np.std(completed_trips_list) = }\n{np.std(total_travel_time_list) = }\n{np.std(average_travel_time_list) = }\n{np.std(average_delay_list) = }\n{np.std(link_traffic_volume_mean_list) = }\n{np.std(link_traffic_volume_std_list) = }")

    # Below are stats from 10 iterations
    assert equal_tolerance(np.average(total_trips_list), 34690.0)
    assert equal_tolerance(np.average(completed_trips_list), 33115.5)
    assert equal_tolerance(np.average(total_travel_time_list), 58410357.5)
    assert equal_tolerance(np.average(average_travel_time_list), 1763.8926467691217)
    assert equal_tolerance(np.average(average_delay_list), 398.1333883284687)
    assert equal_tolerance(np.average(link_traffic_volume_mean_list), 1186.1907894736844)
    assert equal_tolerance(np.average(link_traffic_volume_std_list), 723.8894185892955)


# ======================================================================
# From test_verification_route_choice.py
# ======================================================================

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol


@pytest.mark.flaky(reruns=5)
def test_2route_equal():
    tt1s = []
    tt2s = []
    vol1s = []
    vol2s = []
    ttas = []

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        #W.analyzer.time_space_diagram_traj_links([link11, link12])
        #W.analyzer.time_space_diagram_traj_links([link21, link22])
        W.analyzer.basic_analysis()

        df = W.analyzer.link_to_pandas()

        tt1 = df[df["link"].isin(("link11", "link12"))]["average_travel_time"].sum()
        tt2 = df[df["link"].isin(("link21", "link22"))]["average_travel_time"].sum()
        vol1 = df[df["link"].isin(("link11", "link12"))]["traffic_volume"].sum()/2
        vol2 = df[df["link"].isin(("link21", "link22"))]["traffic_volume"].sum()/2

        df2 = W.analyzer.od_to_pandas()
        tta = df2["average_travel_time"]

        tt1s.append(tt1)
        tt2s.append(tt2)   
        vol1s.append(vol1)
        vol2s.append(vol2)
        ttas.append(tta)
    
    print(f"{np.average(tt1s) = }, {np.average(tt2s) = }, {np.average(vol1s) = }, {np.average(vol2s) = }, {np.average(tt1s) = }, {np.average(ttas) = }")

    assert equal_tolerance(np.average(tt1s), np.average(tt2s))
    assert equal_tolerance(np.average(vol1s), np.average(vol2s))
    assert equal_tolerance(np.average(tt1s), 100)
    assert equal_tolerance(np.average(ttas), 100)
    assert equal_tolerance(np.average(vol1s)+np.average(vol2s), 1200)


@pytest.mark.flaky(reruns=5)
def test_2route_equal_deltan1():
    tt1s = []
    tt2s = []
    vol1s = []
    vol2s = []
    ttas = []

    for i in range(5):
        W = World(cpp=True, 
            name="",
            deltan=1, 
            tmax=2000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        #W.analyzer.time_space_diagram_traj_links([link11, link12])
        #W.analyzer.time_space_diagram_traj_links([link21, link22])
        W.analyzer.basic_analysis()

        df = W.analyzer.link_to_pandas()

        tt1 = df[df["link"].isin(("link11", "link12"))]["average_travel_time"].sum()
        tt2 = df[df["link"].isin(("link21", "link22"))]["average_travel_time"].sum()
        vol1 = df[df["link"].isin(("link11", "link12"))]["traffic_volume"].sum()/2
        vol2 = df[df["link"].isin(("link21", "link22"))]["traffic_volume"].sum()/2

        df2 = W.analyzer.od_to_pandas()
        tta = df2["average_travel_time"]

        tt1s.append(tt1)
        tt2s.append(tt2)   
        vol1s.append(vol1)
        vol2s.append(vol2)
        ttas.append(tta)
    
    print(f"{np.average(tt1s) = }, {np.average(tt2s) = }, {np.average(vol1s) = }, {np.average(vol2s) = }, {np.average(tt1s) = }, {np.average(ttas) = }")

    assert equal_tolerance(np.average(tt1s), np.average(tt2s), rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s), np.average(vol2s), rel_tol=0.2)
    assert equal_tolerance(np.average(tt1s), 100, rel_tol=0.2)
    assert equal_tolerance(np.average(ttas), 100, rel_tol=0.2)
    assert equal_tolerance(np.average(vol1s)+np.average(vol2s), 1200)


@pytest.mark.flaky(reruns=5)
def test_2route_equal_iterative_exec():
    tt1s = []
    tt2s = []
    vol1s = []
    vol2s = []
    ttas = []

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        while W.check_simulation_ongoing():
            W.exec_simulation(duration_t=random.randint(50, 200)) 

        W.analyzer.print_simple_stats()

        #W.analyzer.time_space_diagram_traj_links([link11, link12])
        #W.analyzer.time_space_diagram_traj_links([link21, link22])
        W.analyzer.basic_analysis()

        df = W.analyzer.link_to_pandas()

        tt1 = df[df["link"].isin(("link11", "link12"))]["average_travel_time"].sum()
        tt2 = df[df["link"].isin(("link21", "link22"))]["average_travel_time"].sum()
        vol1 = df[df["link"].isin(("link11", "link12"))]["traffic_volume"].sum()/2
        vol2 = df[df["link"].isin(("link21", "link22"))]["traffic_volume"].sum()/2

        df2 = W.analyzer.od_to_pandas()
        tta = df2["average_travel_time"]

        tt1s.append(tt1)
        tt2s.append(tt2)   
        vol1s.append(vol1)
        vol2s.append(vol2)
        ttas.append(tta)
    
    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(tt1s) = }\n{np.average(ttas) = }")

    assert equal_tolerance(np.average(tt1s), np.average(tt2s))
    assert equal_tolerance(np.average(vol1s), np.average(vol2s))
    assert equal_tolerance(np.average(tt1s), 100)
    assert equal_tolerance(np.average(ttas), 100)
    assert equal_tolerance(np.average(vol1s)+np.average(vol2s), 1200)


@pytest.mark.flaky(reruns=5)
def test_2route_equal_but_different_structure():
    tt1s = []
    tt2s = []
    vol1s = []
    vol2s = []
    ttas = []

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid21", -1, 1.5)
        W.addNode("mid22", -1, 0.5)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=2000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=2000, free_flow_speed=10, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid21", length=1000, free_flow_speed=10, jam_density=0.2)
        link22 = W.addLink("link22", "mid21", "mid22", length=1000, free_flow_speed=10, jam_density=0.2)
        link23 = W.addLink("link23", "mid22", "dest", length=2000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        W.analyzer.basic_analysis()

        df = W.analyzer.link_to_pandas()
        tt1 = df[df["link"].isin(("link11", "link12"))]["average_travel_time"].sum()
        tt2 = df[df["link"].isin(("link21", "link22", "link23"))]["average_travel_time"].sum()
        vol1 = df[df["link"].isin(("link11", "link12"))]["traffic_volume"].sum()/2
        vol2 = df[df["link"].isin(("link21", "link22", "link23"))]["traffic_volume"].sum()/3

        df2 = W.analyzer.od_to_pandas()
        tta = df2["average_travel_time"]

        tt1s.append(tt1)
        tt2s.append(tt2)   
        vol1s.append(vol1)
        vol2s.append(vol2)
        ttas.append(tta)
        
    print(f"{np.average(tt1s) = }, {np.average(tt2s) = }, {np.average(vol1s) = }, {np.average(vol2s) = }, {np.average(tt1s) = }, {np.average(ttas) = }")

    assert equal_tolerance(np.average(tt1s), np.average(tt2s))
    assert equal_tolerance(np.average(vol1s), np.average(vol2s))
    assert equal_tolerance(np.average(tt1s), 300)
    assert equal_tolerance(np.average(ttas), 300)
    assert equal_tolerance(np.average(vol1s)+np.average(vol2s), 1200)


@pytest.mark.flaky(reruns=5)
def test_2route_one_is_too_long():
    tt1s = []
    tt2s = []
    vol1s = []
    vol2s = []
    ttas = []

    for i in range(3):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=2000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        #W.analyzer.time_space_diagram_traj_links([link11, link12])
        #W.analyzer.time_space_diagram_traj_links([link21, link22])
        W.analyzer.basic_analysis()

        df = W.analyzer.link_to_pandas()

        tt1 = df[df["link"].isin(("link11", "link12"))]["average_travel_time"].sum()
        tt2 = df[df["link"].isin(("link21", "link22"))]["average_travel_time"].sum()
        vol1 = df[df["link"].isin(("link11", "link12"))]["traffic_volume"].sum()/2
        vol2 = df[df["link"].isin(("link21", "link22"))]["traffic_volume"].sum()/2

        df2 = W.analyzer.od_to_pandas()
        tta = df2["average_travel_time"]

        tt1s.append(tt1)
        tt2s.append(tt2)   
        vol1s.append(vol1)
        vol2s.append(vol2)
        ttas.append(tta)

    print(f"{np.average(tt1s) = }\n{np.average(tt2s) = }\n{np.average(vol1s) = }\n{np.average(vol2s) = }\n{np.average(tt1s) = }\n{np.average(ttas) = }")

    assert equal_tolerance(np.average(tt1s), 100)
    assert equal_tolerance(np.average(vol1s), 1200)
    assert equal_tolerance(np.average(vol2s), 0, abs_tol=100)
    assert equal_tolerance(np.average(ttas), 100)
    assert equal_tolerance(np.average(vol1s)+np.average(vol2s), 1200)


@pytest.mark.flaky(reruns=5)
def test_2route_one_is_too_long_but_u_changes_during_simulation():
    vol1_es = []
    vol2_es = []
    vol1_ls = []
    vol2_ls = []

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=2000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation(duration_t=500)
        link11.change_free_flow_speed(5)
        link12.change_free_flow_speed(5)
        W.exec_simulation()
        

        W.analyzer.print_simple_stats()

        W.analyzer.basic_analysis()

        W.analyzer.compute_edie_state()

        vol1_es.append(link11.q_mat[:8,5].sum()*link11.edie_dt)
        vol1_ls.append(link11.q_mat[8:,5].sum()*link11.edie_dt)
        vol2_es.append(link21.q_mat[:8,5].sum()*link21.edie_dt)
        vol2_ls.append(link21.q_mat[8:,5].sum()*link21.edie_dt)

    print(f"{np.average(vol1_es) = }\n{np.average(vol1_ls) = }\n{np.average(vol2_es) = }\n{np.average(vol2_ls) = }")
    assert equal_tolerance(np.average(vol1_es), 500)
    assert equal_tolerance(np.average(vol1_ls), 20, abs_tol=100)
    assert equal_tolerance(np.average(vol2_es), 200)
    assert equal_tolerance(np.average(vol2_ls), 470)


@pytest.mark.flaky(reruns=5)
def test_2route_change_too_small_pricing():
    vol1_es = []
    vol2_es = []
    vol1_ls = []
    vol2_ls = []

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=4000, 
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100,
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=2000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 3000, 0.6)

        W.exec_simulation(duration_t=500)
        link11.route_choice_penalty = 90
        W.exec_simulation()
        
        W.analyzer.print_simple_stats()

        # W.analyzer.time_space_diagram_traj_links([link11, link12])
        # W.analyzer.time_space_diagram_traj_links([link21, link22])

        W.analyzer.basic_analysis()

        W.analyzer.compute_edie_state()

        vol1_es.append(link11.q_mat[:5,5].sum()*link11.edie_dt)
        vol1_ls.append(link11.q_mat[15:,5].sum()*link11.edie_dt)
        vol2_es.append(link21.q_mat[:5,5].sum()*link21.edie_dt)
        vol2_ls.append(link21.q_mat[15:,5].sum()*link21.edie_dt)

    print(f"{np.average(vol1_es) = }\n{np.average(vol1_ls) = }\n{np.average(vol2_es) = }\n{np.average(vol2_ls) = }")
    assert equal_tolerance(np.average(vol1_es), 340, abs_tol=100)
    assert equal_tolerance(np.average(vol1_ls), 750)
    assert equal_tolerance(np.average(vol2_es), 0, abs_tol=100)
    assert equal_tolerance(np.average(vol2_ls), 0)


@pytest.mark.flaky(reruns=5)
def test_2route_change_too_large_pricing_iterative():
    vol1_es = []
    vol2_es = []
    vol1_ls = []
    vol2_ls = []

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=4000, 
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100,
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=2000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 3000, 0.6)

        W.exec_simulation(duration_t=500)
        link11.route_choice_penalty = 110
        W.exec_simulation()
        
        W.analyzer.print_simple_stats()

        # W.analyzer.time_space_diagram_traj_links([link11, link12])
        # W.analyzer.time_space_diagram_traj_links([link21, link22])

        W.analyzer.basic_analysis()

        W.analyzer.compute_edie_state()

        vol1_es.append(link11.q_mat[:5,5].sum()*link11.edie_dt)
        vol1_ls.append(link11.q_mat[15:,5].sum()*link11.edie_dt)
        vol2_es.append(link21.q_mat[:5,5].sum()*link21.edie_dt)
        vol2_ls.append(link21.q_mat[15:,5].sum()*link21.edie_dt)

    print(f"{np.average(vol1_es) = }\n{np.average(vol1_ls) = }\n{np.average(vol2_es) = }\n{np.average(vol2_ls) = }")
    assert equal_tolerance(np.average(vol1_es), 340, abs_tol=100)
    assert equal_tolerance(np.average(vol1_ls), 0)
    assert equal_tolerance(np.average(vol2_es), 0, abs_tol=100)
    assert equal_tolerance(np.average(vol2_ls), 750)



@pytest.mark.flaky(reruns=5)
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
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=3000, 
            print_mode=1, save_mode=1, show_mode=1,
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
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link22 = W.addLink("link22", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link31 = W.addLink("link31", "orig", "mid3", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link32 = W.addLink("link32", "mid3", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link41 = W.addLink("link41", "orig", "mid4", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link42 = W.addLink("link42", "mid4", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 2000, 0.8)

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
    assert equal_tolerance(volave*4, 2000*0.8)


def test_route_links_prefer():     
    W = World(cpp=True, 
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid1",0,0)
    W.addNode("mid2",0,0)
    W.addNode("dest", 0, 0)
    link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20)
    link12 = W.addLink("link12", "orig", "mid1", length=1000, free_flow_speed=10)
    link21 = W.addLink("link21", "mid1", "mid2", length=1000, free_flow_speed=20)
    link22 = W.addLink("link22", "mid1", "mid2", length=1000, free_flow_speed=10)
    link31 = W.addLink("link31", "mid2", "dest", length=1000, free_flow_speed=20)
    link32 = W.addLink("link32", "mid2", "dest", length=1000, free_flow_speed=10)

    for t in range(0,1000, 10):
        W.addVehicle("orig", "dest", links_prefer=[link12, link21, link32], departure_time=t)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    df = W.analyzer.link_to_pandas()

    assert equal_tolerance(df[df["link"]=="link12"]["traffic_volume"].values[0], 500)
    assert equal_tolerance(df[df["link"]=="link21"]["traffic_volume"].values[0], 500)
    assert equal_tolerance(df[df["link"]=="link32"]["traffic_volume"].values[0], 500)


def test_route_links_avoid():
    W = World(cpp=True, 
        name="",
        deltan=5,
        tmax=2000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None
    )

    W.addNode("orig", 0, 0)
    W.addNode("mid1",0,0)
    W.addNode("mid2",0,0)
    W.addNode("dest", 0, 0)
    link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20)
    link12 = W.addLink("link12", "orig", "mid1", length=1000, free_flow_speed=10)
    link21 = W.addLink("link21", "mid1", "mid2", length=1000, free_flow_speed=20)
    link22 = W.addLink("link22", "mid1", "mid2", length=1000, free_flow_speed=10)
    link31 = W.addLink("link31", "mid2", "dest", length=1000, free_flow_speed=20)
    link32 = W.addLink("link32", "mid2", "dest", length=1000, free_flow_speed=10)

    for t in range(0,1000, 10):
        W.addVehicle("orig", "dest", links_avoid=[link12, link21, link32], departure_time=t)

    W.exec_simulation()

    W.analyzer.print_simple_stats()

    df = W.analyzer.link_to_pandas()

    assert equal_tolerance(df[df["link"]=="link11"]["traffic_volume"].values[0], 500)
    assert equal_tolerance(df[df["link"]=="link22"]["traffic_volume"].values[0], 500)
    assert equal_tolerance(df[df["link"]=="link31"]["traffic_volume"].values[0], 500)


@pytest.mark.flaky(reruns=5)
def test_route_multiple_links_between_same_nodes():    
    vol11s = []
    vol12s = []
    vol21s = []
    vol22s = []

    for i in range(10):

        W = World(cpp=True, 
            name="",
            deltan=5,
            tmax=2000,
            print_mode=0, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1",0,0)
        W.addNode("mid2",0,0)
        W.addNode("dest", 0, 0)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20)
        link12 = W.addLink("link12", "orig", "mid1", length=1000, free_flow_speed=10)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20)
        link22 = W.addLink("link22", "orig", "mid2", length=1000, free_flow_speed=10)
        link31 = W.addLink("link31", "mid1", "dest", length=1000, free_flow_speed=20)
        link32 = W.addLink("link32", "mid2", "dest", length=1000, free_flow_speed=20)

        W.adddemand("orig", "dest", 0, 1000, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        df = W.analyzer.link_to_pandas()
        #display(df)

        vol11s.append(df[df["link"]=="link11"]["traffic_volume"].values[0])
        vol12s.append(df[df["link"]=="link12"]["traffic_volume"].values[0])
        vol21s.append(df[df["link"]=="link21"]["traffic_volume"].values[0])
        vol22s.append(df[df["link"]=="link22"]["traffic_volume"].values[0])

    print(f"{np.average(vol11s) = }\n{np.average(vol12s) = }\n{np.average(vol21s) = }\n{np.average(vol22s) = }")
    assert equal_tolerance(np.average(vol11s), np.average(vol12s), rel_tol=0.2)
    assert equal_tolerance(np.average(vol21s), np.average(vol22s), rel_tol=0.2)
    assert equal_tolerance(np.average(vol11s)+np.average(vol12s), np.average(vol21s)+np.average(vol22s), rel_tol=0.2)



@pytest.mark.flaky(reruns=5)
def test_route_choice_update_gradual():
    res = defaultdict(list)

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=0, show_mode=1,
            random_seed=None,
            duo_update_time=300,
            duo_update_weight=1,
            route_choice_update_gradual=True
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=2000, free_flow_speed=30, jam_density=0.2, capacity_out=0.4)
        link12 = W.addLink("link12", "mid1", "dest", length=500, free_flow_speed=30, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=500, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        W.analyzer.basic_analysis()

        df = W.analyzer.link_cumulative_to_pandas()

        for link, t in [("link11", 250), ("link11", 500), ("link11", 750), ("link11", 1000), ("link21", 250), ("link21", 500), ("link21", 750), ("link21", 1000)]:
            res[link, t].append(df.loc[(df["link"] == link) & (df["t"] == t), "arrival_count"].item())

    for key in res:
        res[key] = np.mean(res[key])

    assert equal_tolerance(res["link11", 250], 200, rel_tol=0.1)
    assert equal_tolerance(res["link11", 500], 360, rel_tol=0.1)
    assert equal_tolerance(res["link11", 750], 430, rel_tol=0.1)
    assert equal_tolerance(res["link11",1000], 460, rel_tol=0.1)
    assert equal_tolerance(res["link21", 250], 0, abs_tol=20)
    assert equal_tolerance(res["link21", 500], 40, abs_tol=20, rel_tol=0.2)
    assert equal_tolerance(res["link21", 750], 170, rel_tol=0.1)
    assert equal_tolerance(res["link21",1000], 340, rel_tol=0.1)

    

@pytest.mark.flaky(reruns=5)
def test_route_choice_update_instant():
    res = defaultdict(list)

    for i in range(10):
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=2000, 
            print_mode=1, save_mode=0, show_mode=1,
            random_seed=None,
            duo_update_time=300,
            duo_update_weight=1,
            route_choice_update_gradual=False
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", -1, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=2000, free_flow_speed=30, jam_density=0.2, capacity_out=0.4)
        link12 = W.addLink("link12", "mid1", "dest", length=500, free_flow_speed=30, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=2000, free_flow_speed=20, jam_density=0.2)
        link22 = W.addLink("link22", "mid2", "dest", length=500, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 1500, 0.8)

        W.exec_simulation()

        W.analyzer.print_simple_stats()

        W.analyzer.basic_analysis()

        df = W.analyzer.link_cumulative_to_pandas()

        for link, t in [("link11", 250), ("link11", 500), ("link11", 750), ("link11", 1000), ("link21", 250), ("link21", 500), ("link21", 750), ("link21", 1000)]:
            res[link, t].append(df.loc[(df["link"] == link) & (df["t"] == t), "arrival_count"].item())

    for key in res:
        res[key] = np.mean(res[key])

    assert equal_tolerance(res["link11", 250], 200, rel_tol=0.1)
    assert equal_tolerance(res["link11", 500], 240, rel_tol=0.1)
    assert equal_tolerance(res["link11", 750], 240, rel_tol=0.1)
    assert equal_tolerance(res["link11",1000], 320, rel_tol=0.1)
    assert equal_tolerance(res["link21", 250], 0, abs_tol=20)
    assert equal_tolerance(res["link21", 500], 160, abs_tol=20, rel_tol=0.2)
    assert equal_tolerance(res["link21", 750], 360, rel_tol=0.1)
    assert equal_tolerance(res["link21",1000], 480, rel_tol=0.1)



@pytest.mark.flaky(reruns=5)
def test_route_choice_4route_congestion_avoidance_gradual():
    
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
        W = World(cpp=True, 
            name="",
            deltan=5, 
            tmax=3000, 
            print_mode=1, save_mode=1, show_mode=1,
            random_seed=None,
            duo_update_time=100,
            duo_update_weight=0.5,
            route_choice_update_gradual=True
        )

        W.addNode("orig", 0, 0)
        W.addNode("mid1", 1, 1)
        W.addNode("mid2", 2, 1)
        W.addNode("mid3", 3, 1)
        W.addNode("mid4", 4, 1)
        W.addNode("dest", 0, 2)
        link11 = W.addLink("link11", "orig", "mid1", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link12 = W.addLink("link12", "mid1", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link21 = W.addLink("link21", "orig", "mid2", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link22 = W.addLink("link22", "mid2", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link31 = W.addLink("link31", "orig", "mid3", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link32 = W.addLink("link32", "mid3", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        link41 = W.addLink("link41", "orig", "mid4", length=1000, free_flow_speed=20, jam_density=0.2, capacity_out=0.3)
        link42 = W.addLink("link42", "mid4", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand("orig", "dest", 0, 2000, 0.8)

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

    assert equal_tolerance(np.average(tt1s), ttave, rel_tol=0.1)
    assert equal_tolerance(np.average(tt2s), ttave, rel_tol=0.1)
    assert equal_tolerance(np.average(tt3s), ttave, rel_tol=0.1)
    assert equal_tolerance(np.average(tt4s), ttave, rel_tol=0.1)
    assert equal_tolerance(np.average(vol1s), volave, rel_tol=0.1)
    assert equal_tolerance(np.average(vol2s), volave, rel_tol=0.1)
    assert equal_tolerance(np.average(vol3s), volave, rel_tol=0.1)
    assert equal_tolerance(np.average(vol4s), volave, rel_tol=0.1)
    assert equal_tolerance(volave*4, 2000*0.8)

    

@pytest.mark.flaky(reruns=5)
def test_route_choice_dynamic_congestion_pricing():
    W = World(cpp=True, 
        name="aaa",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=5000,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=1,    # Various options
        duo_update_time=120,
        duo_update_weight=0.8,
    )

    def congestion_pricing(t):
        coef = 0.01
        change_t = 2500
        p = t*coef
        if t > change_t:
            p = change_t*coef - (t-change_t)*coef
        return p

    # Define the scenario
    ## Create nodes
    W.addNode("orig", 0, 1)
    W.addNode("diverge", 1, 1)
    W.addNode("nodeA", 2, 2)
    W.addNode("nodeB", 2, 0)
    W.addNode("merge", 3, 1)
    W.addNode("dest", 4, 1)
    ## Create links between nodes
    W.addLink("link1", "orig", "diverge", length=1000, free_flow_speed=20, number_of_lanes=1)
    linkA = W.addLink("linkA1", "diverge", "nodeA", length=1000, free_flow_speed=30, number_of_lanes=1, congestion_pricing=congestion_pricing)
    W.addLink("linkA2", "nodeA", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    linkB = W.addLink("linkB1", "diverge", "nodeB", length=1000, free_flow_speed=20, number_of_lanes=1) #time difference: 1000/20-1000/30 = 16 s
    W.addLink("linkB2", "nodeB", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    ## Create OD traffic demand between nodes
    W.adddemand(orig="orig", dest="dest", t_start=0, t_end=5000, flow=0.6)

    W.show_network()

    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()

    # W.analyzer.network_average()

    # # Plot
    # fig, ax1 = np.subplots()
    # tt = np.linspace(0, 5000, 100)
    # # Plot cumulative arrivals on the left y-axis
    # ax1.plot(tt, [linkA.arrival_count(ti) for ti in tt], label='linkA', color='tab:blue')
    # ax1.plot(tt, [linkB.arrival_count(ti) for ti in tt], label='linkB', color='tab:orange')
    # ax1.set_xlabel('Time (s)')
    # ax1.set_ylabel('Cumulative arrivals', color='k')
    # ax1.tick_params(axis='y', labelcolor='k')
    # ax1.legend(loc='upper left')
    # # Create a second y-axis for congestion pricing
    # ax2 = ax1.twinx()
    # ax2.plot(tt, [congestion_pricing(ti) for ti in tt], color='tab:green', linestyle='--', label='Congestion Pricing')
    # ax2.set_ylabel('Toll', color='tab:green')
    # ax2.tick_params(axis='y', labelcolor='tab:green')
    # ax2.legend(loc='upper right')
    # np.tight_layout()

    assert eq_tol(linkA.average_flow(1000), 0.6)
    assert eq_tol(linkA.average_flow(2000), 0.0)
    assert eq_tol(linkA.average_flow(3000), 0.0)
    assert eq_tol(linkA.average_flow(4500), 0.6)
    assert eq_tol(linkB.average_flow(1000), 0.0)
    assert eq_tol(linkB.average_flow(2000), 0.6)
    assert eq_tol(linkB.average_flow(3000), 0.6)
    assert eq_tol(linkB.average_flow(4500), 0.0)



# ======================================================================
# From test_verification_dta_solvers.py
# ======================================================================

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol


@pytest.mark.flaky(reruns=10)
def test_DTA_total_travel_time_comparison():
    # scenario definition
    def create_World(cpp=True):
        """
        A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.
        """
        W = uxsim.World(cpp=True, 
            name="",
            deltan=20,
            tmax=6000,
            print_mode=0, save_mode=1, show_mode=1,
            vehicle_logging_timestep_interval=1, 
            hard_deterministic_mode=False,
            random_seed=42
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

        return W

    #################################
    # DUO (default)
    W_DUO = create_World(cpp=True)
    W_DUO.exec_simulation()
    W_DUO.analyzer.print_simple_stats(force_print=True)
    df_DUO = W_DUO.analyzer.basic_to_pandas()

    #################################
    # DUE
    solver_DUE = DTAsolvers.SolverDUE(create_World)
    solver_DUE.solve(max_iter=20)   # max_iter should be larger (e.g., 100). this is just for demonstration
    W_DUE = solver_DUE.W_sol
    W_DUE.analyzer.print_simple_stats(force_print=True)
    df_DUE = W_DUE.analyzer.basic_to_pandas()

    #################################
    # DSO by GA: this is obsolete
    solver_DSO_GA = DTAsolvers.SolverDSO_GA(create_World)
    solver_DSO_GA.solve(max_iter=5, pop_size=5)   #deprecated; just run
    W_DSO_GA = solver_DSO_GA.W_sol
    W_DSO_GA.analyzer.print_simple_stats(force_print=True)
    df_DSO_GA = W_DSO_GA.analyzer.basic_to_pandas()

    #################################
    # DSO by ALNS: this is obsolete
    solver_DSO_ALNS = DTAsolvers.SolverDSO_ALNS(create_World)
    solver_DSO_ALNS.solve(max_iter=10)  #deprecated; just run
    W_DSO_ALNS = solver_DSO_ALNS.W_sol
    W_DSO_ALNS.analyzer.print_simple_stats(force_print=True)
    df_DSO_ALNS = W_DSO_ALNS.analyzer.basic_to_pandas()

    #################################
    # DSO by day-to-day: this is recommended
    solver_DSO_D2D= DTAsolvers.SolverDSO_D2D(create_World)
    solver_DSO_D2D.solve(max_iter=20)   # max_iter should be larger (e.g., 100). this is just for demonstration
    W_DSO_D2D = solver_DSO_D2D.W_sol
    W_DSO_D2D.analyzer.print_simple_stats(force_print=True)
    df_DSO_D2D = W_DSO_D2D.analyzer.basic_to_pandas()

    # stats
    print("DUO")
    print(df_DUO)
    print("DUE")
    print(df_DUE)
    print("DSO_GA")
    print(df_DSO_GA)
    print("DSO_ALNS")
    print(df_DSO_ALNS)
    print("DSO_D2D")
    print(df_DSO_D2D)

    # visualizations
    solver_DUE.plot_convergence()
    solver_DUE.plot_link_stats()
    solver_DUE.plot_vehicle_stats(orig="4", dest="7")

    solver_DSO_GA.plot_convergence()
    solver_DSO_GA.plot_link_stats()
    solver_DSO_GA.plot_vehicle_stats(orig="4", dest="7")

    solver_DSO_ALNS.plot_convergence()
    solver_DSO_ALNS.plot_link_stats()
    solver_DSO_ALNS.plot_vehicle_stats(orig="4", dest="7")

    solver_DSO_D2D.plot_convergence()
    solver_DSO_D2D.plot_link_stats()
    solver_DSO_D2D.plot_vehicle_stats(orig="4", dest="7")
    
    assert W_DUO.analyzer.total_travel_time > W_DUE.analyzer.total_travel_time
    #assert W_DUE.analyzer.total_travel_time > W_DSO_GA.analyzer.total_travel_time
    #assert W_DUE.analyzer.total_travel_time > W_DSO_ALNS.analyzer.total_travel_time
    assert W_DUE.analyzer.total_travel_time > W_DSO_D2D.analyzer.total_travel_time



@pytest.mark.flaky(reruns=10)
def test_DTA_total_travel_time_comparison_with_initial_solutions_for_DSO():
    # scenario definition
    def create_World(cpp=True):
        """
        A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.
        """
        W = uxsim.World(cpp=True, 
            name="",
            deltan=20,
            tmax=6000,
            print_mode=0, save_mode=1, show_mode=1,
            vehicle_logging_timestep_interval=1, 
            hard_deterministic_mode=False,
            random_seed=42
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

        return W

    #################################
    # DUO (default)
    W_DUO = create_World(cpp=True)
    W_DUO.exec_simulation()
    W_DUO.analyzer.print_simple_stats(force_print=True)
    df_DUO = W_DUO.analyzer.basic_to_pandas()

    #################################
    # DUE
    solver_DUE = SolverDUE(create_World)
    solver_DUE.solve(max_iter=20)   # max_iter should be larger (e.g., 100). this is just for demonstration
    W_DUE = solver_DUE.W_sol
    W_DUE.analyzer.print_simple_stats(force_print=True)
    df_DUE = W_DUE.analyzer.basic_to_pandas()

    #################################
    # DSO by GA
    solver_DSO_GA = SolverDSO_GA(create_World)
    solver_DSO_GA.solve(max_iter=5, pop_size=5, initial_solution_World=W_DUE)    #deprecated; just run
    W_DSO_GA = solver_DSO_GA.W_sol
    W_DSO_GA.analyzer.print_simple_stats(force_print=True)
    df_DSO_GA = W_DSO_GA.analyzer.basic_to_pandas()

    #################################
    # DSO by ALNS
    solver_DSO_ALNS = SolverDSO_ALNS(create_World)
    solver_DSO_ALNS.solve(max_iter=10, initial_solution_World=W_DUE)    #deprecated; just run
    W_DSO_ALNS = solver_DSO_ALNS.W_sol
    W_DSO_ALNS.analyzer.print_simple_stats(force_print=True)
    df_DSO_ALNS = W_DSO_ALNS.analyzer.basic_to_pandas()

    # stats
    print("DUO")
    print(df_DUO)
    print("DUE")
    print(df_DUE)
    print("DSO_GA")
    print(df_DSO_GA)
    print("DSO_ALNS")
    print(df_DSO_ALNS)

    # visualizations
    solver_DUE.plot_convergence()
    solver_DUE.plot_link_stats()
    solver_DUE.plot_vehicle_stats(orig="4", dest="7")

    solver_DSO_GA.plot_convergence()
    solver_DSO_GA.plot_link_stats()
    solver_DSO_GA.plot_vehicle_stats(orig="4", dest="7")

    solver_DSO_ALNS.plot_convergence()

    assert W_DUO.analyzer.total_travel_time > W_DUE.analyzer.total_travel_time
    #assert W_DUE.analyzer.total_travel_time > W_DSO_GA.analyzer.total_travel_time
    #assert W_DUE.analyzer.total_travel_time > W_DSO_ALNS.analyzer.total_travel_time


@pytest.mark.flaky(reruns=10)
def test_DTA_with_given_route_sets():
        
    def create_World(cpp=True):
        """
        A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.
        """
        W = uxsim.World(cpp=True, 
            name="",
            deltan=20,
            tmax=6000,
            print_mode=0, save_mode=1, show_mode=1,
            vehicle_logging_timestep_interval=1, 
            hard_deterministic_mode=False,
            random_seed=42
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
        W.addLink("offramp", "2", "5", length=1000, number_of_lanes=1, merge_priority=0.5)
        W.addLink("arterial45", "4", "5", length=1000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)
        W.addLink("arterial56", "5", "6", length=3000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)
        W.addLink("arterial67", "6", "7", length=1000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)

        W.adddemand("1", "7", 0, 3000, 0.3) #900
        W.adddemand("4", "7", 0, 3000, 0.4*3) #3600

        return W

    #################################
    # DUO (default)
    W_DUO = create_World(cpp=True)
    W_DUO.exec_simulation()
    W_DUO.analyzer.print_simple_stats(force_print=True)
    df_DUO = W_DUO.analyzer.basic_to_pandas()

    #################################
    # DUE

    route_sets = {
        ('1', '7'): [
            ['highway12', 'highway23', 'highway37'],
            #['highway12', 'offramp', 'arterial56', 'arterial67']
        ],
        ('4', '7'): [
            ['arterial45', 'arterial56', 'arterial67'],
            #['arterial45', 'onramp', 'highway23', 'highway37']
        ]
    }

    solver_DUE = DTAsolvers.SolverDUE(create_World)
    solver_DUE.solve(max_iter=20, route_sets=route_sets)
    W_DUE = solver_DUE.W_sol
    W_DUE.analyzer.print_simple_stats(force_print=True)
    df_DUE = W_DUE.analyzer.basic_to_pandas()

    df_DUE_link = W_DUE.analyzer.link_to_pandas()
    assert df_DUE_link["traffic_volume"][df_DUE_link["link"]=="onramp"].item() == 0
    assert df_DUE_link["traffic_volume"][df_DUE_link["link"]=="offramp"].item() == 0


    solver_DSO_D2D= DTAsolvers.SolverDSO_D2D(create_World)
    solver_DSO_D2D.solve(max_iter=20, route_sets=route_sets)
    W_DSO_D2D = solver_DSO_D2D.W_sol
    W_DSO_D2D.analyzer.print_simple_stats(force_print=True)
    df_DSO_D2D = W_DSO_D2D.analyzer.basic_to_pandas()

    df_DSO_link = W_DUE.analyzer.link_to_pandas()
    assert df_DSO_link["traffic_volume"][df_DSO_link["link"]=="onramp"].item() == 0
    assert df_DSO_link["traffic_volume"][df_DSO_link["link"]=="offramp"].item() == 0


@pytest.mark.flaky(reruns=20)
def test_DTA_dynamic_congestion_pricing_on_highway_bottleneck():
    # scenario definition
    def create_World(cpp=True):
        """
        A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.
        """
        W = uxsim.World(cpp=True, 
            name="",
            deltan=20,
            tmax=9000,
            print_mode=0, save_mode=1, show_mode=1,
            vehicle_logging_timestep_interval=1, 
            hard_deterministic_mode=False,
            random_seed=42
        )

        W.addNode("1", 0, 1)
        W.addNode("2", 1, 1)
        W.addNode("3", 5, 1)
        W.addNode("4", 0, 0)
        W.addNode("5", 1, 0)
        W.addNode("6", 5, 0)
        W.addNode("7", 6, 0.5)

        def toll(t):
            toll = 0
            if 3000 < t:
                toll = 120
            if 6000 < t:
                toll = 10000
            return toll

        W.addLink("highway12", "1", "2", length=1000, number_of_lanes=1, merge_priority=1)
        link_highway = W.addLink("highway23", "2", "3", length=3000, number_of_lanes=1, jam_density=0.5, merge_priority=1, capacity_out=0.6, congestion_pricing=toll)
        W.addLink("highway37", "3", "7", length=1000, number_of_lanes=1, merge_priority=1)
        W.addLink("onramp", "5", "2", length=1000, number_of_lanes=1, merge_priority=0.5)
        W.addLink("arterial45", "4", "5", length=1000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)
        W.addLink("arterial56", "5", "6", length=3000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)
        W.addLink("arterial67", "6", "7", length=1000, free_flow_speed=10, number_of_lanes=2, merge_priority=0.5)

        W.adddemand("1", "7", 0, 6000, 0.3)
        W.adddemand("4", "7", 0, 6000, 0.4*3)

        return W

    # DUE
    solver_DUE = SolverDUE(create_World)
    solver_DUE.solve(max_iter=40, print_progress=False)
    W_DUE = solver_DUE.W_sol
    W_DUE.analyzer.print_simple_stats(force_print=False)

    l = W_DUE.get_link("highway23")
    print("inflows:", l.inflow_between(100,500), l.inflow_between(4000,6000))

    assert l.inflow_between(100,500) > 0.6  #initial: demand > bottleneck capacity
    assert 0.5 < l.inflow_between(4000,6000) < 0.65 #with pricing: demand = BN capacity
    assert l.average_travel_time_between(0,10) == 150   #initial: free-flow
    assert l.average_travel_time_between(500,3000) > 300    #without pricing: congested
    assert 150 < l.average_travel_time_between(4000,6000) < 200 #with pricing: close to free-flow
    assert l.average_travel_time_between(7000,8000) == 150  #with too much pricing: free-flow (no travelers)



# ======================================================================
# From test_other_functions.py (filtered)
# ======================================================================

def test_analyzer():
    import matplotlib
    matplotlib.use('Agg')
    # Define the main simulation
    # Units are standardized to seconds (s) and meters (m)
    W = World(cpp=True, 
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


def test_k_shortest_path():
    W = World(cpp=True, 
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

    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
        W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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


@pytest.mark.flaky(reruns=10)
def test_area_stats():

    rec_volume_areaN = []
    rec_volume_areaS = []
    rec_ttt_areaN = []
    rec_delay_areaN = []

    for i in range(10):
        W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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


def test_get_linkstats():
    W = World(cpp=True, name="simple", tmax=2000, show_mode=1)

    W.addNode("start", x=0, y=0)
    W.addNode("end", x=7500, y=0)

    l = W.addLink("road", start_node="start", end_node="end", length=10000, free_flow_speed=10)

    W.adddemand(orig="start", dest="end", t_start=100, t_end=600, volume=500)

    W.exec_simulation()

    t = 50
    assert equal_tolerance(l.num_vehicles_t(t), 0)
    assert equal_tolerance(l.average_density(t), 0.0)
    assert equal_tolerance(l.average_speed(t), 10.0)
    assert equal_tolerance(l.average_flow(t), 0.0)

    t = 1000
    assert equal_tolerance(l.num_vehicles_t(t), 500)
    assert equal_tolerance(l.average_density(t), 0.05)
    assert equal_tolerance(l.average_speed(t), 10.0)
    assert equal_tolerance(l.average_flow(t), 0.5)


def test_reduce_memory_delete_vehicle_route_pref():
    W = World(cpp=True, 
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

    W = World(cpp=True, 
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
    
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
    W = World(cpp=True, 
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

    r_acutal0 = W.VEHICLES["0"].traveled_route()[0]
    r_acutal300 = W.VEHICLES["300"].traveled_route()[0]

    assert r_acutal0 == r1 or r_acutal0 == r2 
    assert r_acutal300 == r1 or r_acutal300 == r2 
    
    assert W.VEHICLES["300"].traveled_route()[1][0] == 3260
    assert W.VEHICLES["300"].traveled_route()[1][-1] > 3260

    tt_from_vehicle_route = W.VEHICLES["300"].traveled_route()[1][-1]-W.VEHICLES["300"].traveled_route()[1][0]
    tt_from_route_by_departure_time = W.VEHICLES["300"].traveled_route()[0].actual_travel_time(W.VEHICLES["300"].traveled_route()[1][0])
    assert equal_tolerance(tt_from_vehicle_route, tt_from_route_by_departure_time)



def test_route_enforce_route_old():
    W = World(cpp=True, 
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

    W = World(cpp=True, 
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
    W = World(cpp=True, 
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
        W = World(cpp=True, 
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


# ======================================================================
# Custom C++ mode tests (analyzer, DTA)
# ======================================================================

def _make_analyzer_world():
    """Create a 1-link scenario for analyzer tests."""
    from uxsim import World
    W = World(cpp=True, name="analyzer_test", deltan=5, tmax=1200,
              print_mode=0, save_mode=0, random_seed=0)
    W.addNode("orig", 0, 0)
    W.addNode("dest", 0, 1000)
    W.addLink("link", "orig", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig", "dest", 0, 1000, 0.4)
    W.exec_simulation()
    return W


def test_analyzer_basic_to_pandas():
    W = _make_analyzer_world()
    df = W.analyzer.basic_to_pandas()
    assert df is not None and len(df) > 0


def test_analyzer_link_to_pandas():
    W = _make_analyzer_world()
    df = W.analyzer.link_to_pandas()
    assert df is not None and len(df) > 0


def test_analyzer_od_to_pandas():
    W = _make_analyzer_world()
    df = W.analyzer.od_to_pandas()
    assert df is not None and len(df) > 0


def test_analyzer_compute_edie_state():
    W = _make_analyzer_world()
    W.analyzer.compute_edie_state()
    link = W.get_link("link")
    assert hasattr(link, "k_mat")
    assert link.k_mat is not None


def test_analyzer_object_identity():
    W = _make_analyzer_world()
    W.analyzer.basic_to_pandas()
    for veh in W.VEHICLES.values():
        for i, lnk in enumerate(veh.log_link):
            if lnk != -1:
                assert lnk is W.LINKS[lnk.id], (
                    f"log_link[{i}] object identity mismatch for vehicle {veh.name}"
                )


def test_analyzer_log_cache_reuse():
    W = _make_analyzer_world()
    W.analyzer.basic_to_pandas()
    for veh in list(W.VEHICLES.values())[:5]:
        log_t_1 = veh.log_t
        log_t_2 = veh.log_t
        assert log_t_1 is log_t_2, (
            f"log_t cache not reused for vehicle {veh.name}"
        )


def test_analyzer_vehicles_to_pandas():
    W = _make_analyzer_world()
    df = W.analyzer.vehicles_to_pandas()
    assert df is not None and len(df) > 0


def test_analyzer_vehicle_trip_to_pandas():
    W = _make_analyzer_world()
    W.analyzer.basic_to_pandas()
    df = W.analyzer.vehicle_trip_to_pandas()
    assert df is not None and len(df) > 0


def test_analyzer_link_cumulative_to_pandas():
    W = _make_analyzer_world()
    df = W.analyzer.link_cumulative_to_pandas()
    assert df is not None and len(df) > 0


def test_analyzer_large_network():
    from uxsim import World
    W = World(cpp=True, name="grid3x3", deltan=5, tmax=1200,
              print_mode=0, save_mode=0, random_seed=0)
    # 3x3 grid nodes
    for i in range(3):
        for j in range(3):
            W.addNode(f"n{i}{j}", i*500, j*500)
    # horizontal links (6)
    for i in range(2):
        for j in range(3):
            W.addLink(f"h{i}{j}", f"n{i}{j}", f"n{i+1}{j}",
                      length=500, free_flow_speed=20, jam_density=0.2)
    # vertical links (6)
    for i in range(3):
        for j in range(2):
            W.addLink(f"v{i}{j}", f"n{i}{j}", f"n{i}{j+1}",
                      length=500, free_flow_speed=20, jam_density=0.2)
    # demand from corners
    W.adddemand("n00", "n22", 0, 1000, 0.3)
    W.adddemand("n20", "n02", 0, 1000, 0.3)
    W.exec_simulation()
    # all analyzer methods should work
    df = W.analyzer.basic_to_pandas()
    assert df is not None and len(df) > 0
    df = W.analyzer.od_to_pandas()
    assert df is not None and len(df) > 0
    df = W.analyzer.link_to_pandas()
    assert df is not None and len(df) > 0
    df = W.analyzer.vehicles_to_pandas()
    assert df is not None and len(df) > 0
    df = W.analyzer.vehicle_trip_to_pandas()
    assert df is not None and len(df) > 0
    df = W.analyzer.link_cumulative_to_pandas()
    assert df is not None and len(df) > 0
    W.analyzer.compute_edie_state()
    for link in W.LINKS:
        assert hasattr(link, "k_mat")


def test_dta_solver_due_cpp():
    """SolverDUE runs on a small network with cpp=True and returns positive TTT."""
    import uxsim
    from uxsim.DTAsolvers import SolverDUE

    def create_World():
        W = uxsim.World(name="", deltan=10, tmax=4000, print_mode=0, save_mode=1,
                        show_mode=0, vehicle_logging_timestep_interval=1,
                        random_seed=42, cpp=True)
        W.addNode("A", 0, 0)
        W.addNode("B", 1, 0)
        W.addNode("C", 2, 0)
        W.addNode("D", 2, 1)
        W.addLink("AB", "A", "B", length=1000, number_of_lanes=2)
        W.addLink("BC", "B", "C", length=2000, number_of_lanes=1)
        W.addLink("BD", "B", "D", length=2000, number_of_lanes=1)
        W.addLink("CD", "C", "D", length=1000, number_of_lanes=2)
        W.adddemand("A", "D", 0, 2000, 0.4)
        return W

    solver = SolverDUE(create_World)
    solver.solve(max_iter=3, print_progress=False)
    W_sol = solver.W_sol
    W_sol.analyzer.print_simple_stats(force_print=False)
    assert W_sol.analyzer.total_travel_time > 0
    assert W_sol.analyzer.trip_completed > 0



@pytest.mark.flaky(reruns=5)
def test_congestion_pricing_toll_timeseries_cpp():
    """congestion_pricing callback is converted to toll_timeseries and affects route choice."""
    from uxsim import World
    from uxsim.utils import eq_tol

    def toll(t):
        return 50 if t < 2000 else 0

    W = World(name="", deltan=5, tmax=4000, print_mode=0, save_mode=0, show_mode=0,
              duo_update_time=100, duo_update_weight=0.8, cpp=True,
              hard_deterministic_mode=True)
    W.addNode("O", 0, 0)
    W.addNode("M1", 1, 1)
    W.addNode("M2", 1, -1)
    W.addNode("D", 2, 0)
    link_toll = W.addLink("fast", "O", "M1", length=1000, free_flow_speed=25,
                          number_of_lanes=1, congestion_pricing=toll)
    W.addLink("fast2", "M1", "D", length=1000, free_flow_speed=25, number_of_lanes=1)
    link_free = W.addLink("slow", "O", "M2", length=1000, free_flow_speed=20,
                          number_of_lanes=1)
    W.addLink("slow2", "M2", "D", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.adddemand("O", "D", 0, 4000, 0.5)
    W.exec_simulation()

    # t<2000: toll=50 on fast link → slow link preferred
    assert eq_tol(link_toll.average_flow(1000), 0.0)
    assert link_free.average_flow(1000) > 0.3
    # t>2000: toll=0 → fast link preferred (shorter travel time)
    assert link_toll.average_flow(3000) > 0.3



def test_dta_solver_due_with_congestion_pricing_cpp():
    """SolverDUE works with congestion_pricing on cpp=True."""
    import uxsim
    from uxsim.DTAsolvers import SolverDUE

    def create_World():
        W = uxsim.World(name="", deltan=10, tmax=5000, print_mode=0, save_mode=1,
                        show_mode=0, vehicle_logging_timestep_interval=1,
                        random_seed=42, cpp=True)
        W.addNode("1", 0, 0)
        W.addNode("2", 1, 0)
        W.addNode("3", 2, 0)

        def toll(t):
            return 100 if t < 2000 else 0

        W.addLink("L12", "1", "2", length=2000, number_of_lanes=1,
                  congestion_pricing=toll)
        W.addLink("L23", "2", "3", length=1000, number_of_lanes=1)
        W.adddemand("1", "3", 0, 3000, 0.3)
        return W

    solver = SolverDUE(create_World)
    solver.solve(max_iter=3, print_progress=False)
    W_sol = solver.W_sol
    assert W_sol.analyzer.total_travel_time > 0
    assert W_sol.analyzer.trip_completed > 0


# ======================================================================
# Notebook demo tests (cpp=True via injected patch cell)
# ======================================================================

import nbformat
import os
from nbconvert.preprocessors import ExecutePreprocessor, CellExecutionError

class _CppSkipCellsPreprocessor(ExecutePreprocessor):
    """Preprocessor that skips cells containing specified keywords."""

    def __init__(self, exception_words=None, **kwargs):
        super().__init__(**kwargs)
        self.exception_words = exception_words or []

    def preprocess_cell(self, cell, resources, cell_index):
        if cell.cell_type == "code":
            if any(word in cell.source for word in self.exception_words):
                return cell, resources
        return super().preprocess_cell(cell, resources, cell_index)

_CPP_PATCH_CELL_SOURCE = """
import uxsim.uxsim as _mod
_OrigWorld = _mod.World
_orig_new = _OrigWorld.__new__
_orig_init = _OrigWorld.__init__
def _patched_new(cls, *args, cpp=True, **kwargs):
    return _orig_new(cls, *args, cpp=cpp, **kwargs)
def _patched_init(self, *args, cpp=True, **kwargs):
    return _orig_init(self, *args, cpp=cpp, **kwargs)
_OrigWorld.__new__ = _patched_new
_OrigWorld.__init__ = _patched_init
""".strip()

def _run_notebook_cpp(notebook_pattern, exception_words, timeout=1800):
    """Run a notebook with cpp=True patch injected at the beginning."""
    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and notebook_pattern in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            # Inject cpp=True patch as the first cell
            patch_cell = nbformat.v4.new_code_cell(source=_CPP_PATCH_CELL_SOURCE)
            nb.cells.insert(0, patch_cell)
            ep = _CppSkipCellsPreprocessor(
                exception_words=exception_words,
                timeout=timeout,
                kernel_name='python3'
            )
            ep.preprocess(nb, {'metadata': {'path': notebook_dir}})

def test_cpp_demo_notebook_01en():
    _run_notebook_cpp(
        "demo_notebook_01en",
        ["ResultGUIViewer", "%matplotlib", "load_scenario", "save_scenario"]
    )

def test_cpp_demo_notebook_01jp():
    _run_notebook_cpp(
        "demo_notebook_01jp",
        ["ResultGUIViewer", "%matplotlib", "load_scenario", "save_scenario"]
    )

def test_cpp_demo_notebook_09en_dta():
    _run_notebook_cpp(
        "demo_notebook_09en_dynamic_traffic_assignment",
        ["ResultGUIViewer", "%matplotlib"],
        timeout=7200
    )

def test_cpp_demo_notebook_10en_signal():
    _run_notebook_cpp(
        "demo_notebook_10en_traffic_signal_tutorial",
        ["ResultGUIViewer", "%matplotlib"]
    )


# ======================================================================
# Example script tests (cpp=True via sed injection)
# ======================================================================

import subprocess

_CPP_EXAMPLE_SCRIPTS = [
    "example_00en_simple.py",
    "example_01en_basic.py",
    "example_01jp_basic.py",
    "example_02en_bottleneck.py",
    "example_02jp_bottleneck.py",
    "example_03en_Nguyen-Dupuis_network.py",
    "example_04en_automatic_network_generation.py",
    "example_04jp_automatic_network_generation.py",
    "example_05en_gridlock_and_prevention.py",
    "example_05jp_gridlock_and_prevention.py",
    "example_06en_dynamic_parameter_change.py",
    "example_06jp_dynamic_parameter_change.py",
    "example_07en_signal.py",
    "example_07jp_signal.py",
    "example_08en_signal_reactive_control.py",
    "example_08jp_signal_reactive_control.py",
    "example_10en_signal_4legged_intersection.py",
    "example_11en_signal_4legged_intersection_reactive_control.py",
    "example_13en_multiple_signals.py",
    "example_15jp_4phase_signal_japanese_style.py",
    "example_19en_multilane_link.py",
    "example_26en_dynamic_traffic_assignment_DUO_DUE_DSO.py",
]

@pytest.mark.parametrize("example_script", _CPP_EXAMPLE_SCRIPTS)
def test_cpp_example_runs(example_script):
    """Test that example scripts run successfully with cpp=True."""
    script_path = os.path.join("demos_and_examples", example_script)
    with open(script_path, "r") as f:
        code = f.read()
    # Inject cpp=True into all World() instantiations
    patched = code.replace("World(", "World(cpp=True, ")
    # Fix doubled cpp=True if already present
    patched = patched.replace("cpp=True, cpp=True", "cpp=True")
    result = subprocess.run(
        ["python3", "-c", patched],
        capture_output=True, text=True,
        cwd="demos_and_examples",
        timeout=600
    )
    assert result.returncode == 0, (
        f"Script {example_script} failed with cpp=True:\n{result.stdout}\n{result.stderr}"
    )



@pytest.mark.flaky(reruns=5)
def test_route_choice_no_cyclic_routing_cpp():
    from uxsim import World
    W = World(
        cpp=True,
        name="",
        deltan=10,
        tmax=7200,
        print_mode=1, save_mode=0, show_mode=1,
        random_seed=None,
        no_cyclic_routing=True
    )

    # scenario
    #automated network generation
    #deploy nodes as an imax x jmax grid
    imax = 13
    jmax = 13
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j)

    #create links between neighborhood nodes
    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=10, jam_density=0.2)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=10, jam_density=0.2)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=10, jam_density=0.2)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=10, jam_density=0.2)

    #generate traffic demand between the boundary nodes
    demand_flow = 0.025
    demand_duration = 3600
    for n1 in [(0,j) for j in range(jmax)]:
        for n2 in [(imax-1,j) for j in range(jmax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i,0) for i in range(imax)]:
        for n2 in [(i,jmax-1) for i in range(imax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    # execute simulation
    W.exec_simulation()

    # visualize
    W.analyzer.print_simple_stats()

    # count vehicles that use the same link or inverse link during 1 trip
    print("\nStats for same or inverse link traveling")

    n_same = 0
    n_inv = 0
    n_all = 0

    n_same_veh = 0
    n_inv_veh = 0

    for veh in W.VEHICLES.values():
        route_nodes = [(l.start_node, l.end_node) for l in veh.traveled_route()[0]]
        n_same_veh_tmp = 0
        n_inv_veh_tmp = 0
        for i in range(len(route_nodes)):
            for j in range(i+1, len(route_nodes)):
                ll = route_nodes[i]
                kk = route_nodes[j]
                if ll == kk:
                    n_same += 1
                    n_same_veh_tmp = 1
                if (ll[0], ll[1]) == (kk[1], kk[0]):
                    n_inv += 1
                    n_inv_veh_tmp = 1

            n_all += 1
        n_same_veh += n_same_veh_tmp
        n_inv_veh += n_inv_veh_tmp

    print(
        f"all_links={n_all:}\n"
        f"same_link={n_same:} ({n_same / n_all:.2%})\n"
        f"inverse_link={n_inv:} ({n_inv / n_all:.2%})\n"
    )

    print(
        f"vehicles={len(W.VEHICLES):}\n"
        f"same_link={n_same_veh:} ({n_same_veh / len(W.VEHICLES):.2%})\n"
        f"inverse_link={n_inv_veh:} ({n_inv_veh / len(W.VEHICLES):.2%})\n"
    )

    ttt_nocyclic = W.analyzer.total_travel_time
    trip_completed_nocyclic = W.analyzer.trip_completed

    assert n_same == 0
    assert n_inv == 0


    W = World(
        cpp=True,
        name="",
        deltan=10,
        tmax=7200,
        print_mode=1, save_mode=0, show_mode=1,
        random_seed=None,
        no_cyclic_routing=False
    )

    # scenario
    #automated network generation
    #deploy nodes as an imax x jmax grid
    imax = 13
    jmax = 13
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j)

    #create links between neighborhood nodes
    links = {}
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=10, jam_density=0.2)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=10, jam_density=0.2)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=10, jam_density=0.2)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=10, jam_density=0.2)

    #generate traffic demand between the boundary nodes
    demand_flow = 0.025
    demand_duration = 3600
    for n1 in [(0,j) for j in range(jmax)]:
        for n2 in [(imax-1,j) for j in range(jmax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i,0) for i in range(imax)]:
        for n2 in [(i,jmax-1) for i in range(imax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    # execute simulation
    W.exec_simulation()

    # visualize
    W.analyzer.print_simple_stats()

    # count vehicles that use the same link or inverse link during 1 trip
    print("\nStats for same or inverse link traveling")

    n_same = 0
    n_inv = 0
    n_all = 0

    n_same_veh = 0
    n_inv_veh = 0

    for veh in W.VEHICLES.values():
        route_nodes = [(l.start_node, l.end_node) for l in veh.traveled_route()[0]]
        n_same_veh_tmp = 0
        n_inv_veh_tmp = 0
        for i in range(len(route_nodes)):
            for j in range(i+1, len(route_nodes)):
                ll = route_nodes[i]
                kk = route_nodes[j]
                if ll == kk:
                    n_same += 1
                    n_same_veh_tmp = 1
                if (ll[0], ll[1]) == (kk[1], kk[0]):
                    n_inv += 1
                    n_inv_veh_tmp = 1

            n_all += 1
        n_same_veh += n_same_veh_tmp
        n_inv_veh += n_inv_veh_tmp

    print(
        f"all_links={n_all:}\n"
        f"same_link={n_same:} ({n_same / n_all:.2%})\n"
        f"inverse_link={n_inv:} ({n_inv / n_all:.2%})\n"
    )

    print(
        f"vehicles={len(W.VEHICLES):}\n"
        f"same_link={n_same_veh:} ({n_same_veh / len(W.VEHICLES):.2%})\n"
        f"inverse_link={n_inv_veh:} ({n_inv_veh / len(W.VEHICLES):.2%})\n"
    )

    ttt_cyclic = W.analyzer.total_travel_time
    trip_completed_cyclic = W.analyzer.trip_completed


    assert trip_completed_cyclic == 60840
    assert trip_completed_nocyclic == 60840
    assert eq_tol(ttt_cyclic, ttt_nocyclic)


###########################################
# Coverage improvement tests (2026-04-02)
###########################################

# --- Vehicle abort, log access, route manipulation ---

def test_coverage_vehicle_abort_and_all_logs():
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("orig", 0, 0)
    W.addNode("deadend", 500, 0)
    W.addNode("orig2", 0, 500)
    W.addNode("dest", 500, 500)

    W.addLink("orig_deadend", "orig", "deadend", length=500, free_flow_speed=20)
    W.addLink("orig2_dest", "orig2", "dest", length=500, free_flow_speed=20)

    for t in range(0, 50, 10):
        W.addVehicle("orig", "dest", departure_time=t, trip_abort=1, auto_rename=True)
    for t in range(0, 50, 10):
        W.addVehicle("orig2", "dest", departure_time=t, auto_rename=True)

    W.exec_simulation()

    aborted = [v for v in W.VEHICLES.values() if v.orig == W.get_node("orig")]
    ended = [v for v in W.VEHICLES.values() if v.orig == W.get_node("orig2")]

    for veh in aborted:
        assert veh.state == "abort"
        assert veh.arrival_time == -1
        assert veh.travel_time == -1
        assert veh.flag_trip_aborted == True

    for veh in ended:
        assert veh.state == "end"

    for veh in list(W.VEHICLES.values()):
        repr(veh)
        assert len(veh.log_t) > 0
        assert len(veh.log_x) > 0
        assert len(veh.log_v) > 0
        assert len(veh.log_s) > 0
        assert len(veh.log_lane) > 0
        assert len(veh.log_state) > 0
        assert len(veh.log_link) > 0
        assert len(veh.log_t_link) > 0
        veh.traveled_route()

    veh0 = list(W.VEHICLES.values())[0]
    with pytest.raises(AttributeError):
        _ = veh0.nonexistent_attr


def test_coverage_vehicle_route_manipulation():
    W = World(cpp=True, deltan=5, tmax=500, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("orig", 0, 0)
    W.addNode("mid", 500, 0)
    W.addNode("dest1", 1000, 0)
    W.addNode("dest2", 1000, 500)
    W.addNode("dest3", 1000, -500)

    link_om = W.addLink("orig_mid", "orig", "mid", length=500, free_flow_speed=20)
    link_md1 = W.addLink("mid_dest1", "mid", "dest1", length=500, free_flow_speed=20)
    link_md2 = W.addLink("mid_dest2", "mid", "dest2", length=500, free_flow_speed=20)
    link_md3 = W.addLink("mid_dest3", "mid", "dest3", length=500, free_flow_speed=20)

    W.addVehicle("orig", "dest1", departure_time=0, auto_rename=True)
    W.addVehicle("orig", "dest2", departure_time=50, auto_rename=True)
    W.addVehicle("orig", "dest1", departure_time=100, auto_rename=True)

    vehs = sorted(W.VEHICLES.values(), key=lambda v: v.departure_time)
    veh1, veh2, veh3 = vehs[0], vehs[1], vehs[2]

    veh1.enforce_route([link_om, link_md1])
    veh2.set_links_prefer([link_md2])
    veh3.set_links_avoid([link_md3])

    W.exec_simulation()

    route1 = veh1.traveled_route()[0]
    assert route1[-1] == link_md1


# --- Link mid-simulation queries, setters, edie_dx ---

def test_coverage_link_mid_simulation_queries():
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("A", 0, 0)
    W.addNode("B", 1000, 0)
    link = W.addLink("AB", "A", "B", length=1000, free_flow_speed=20)
    W.adddemand("A", "B", 0, 200, flow=0.5)

    W.exec_simulation(until_t=100)

    assert link.speed >= 0
    assert link.density >= 0
    assert link.flow >= 0
    assert link.arrival_count(0) >= 0
    assert link.departure_count(0) >= 0

    for t in [50, -1, 99999]:
        itt = link.instant_travel_time(t)
        att = link.actual_travel_time(t)
        assert itt >= 0 or itt == -1
        assert att >= 0 or att == -1

    W.exec_simulation()


def test_coverage_link_setters_and_eular_dx():
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("n1", 0, 0)
    W.addNode("n2", 1, 0)
    link = W.addLink("l1", "n1", "n2", length=1000, free_flow_speed=20, eular_dx=200)
    W.adddemand("n1", "n2", 0, 200, 0.5)

    W.exec_simulation(until_t=100)

    link.capacity_in_remain = 100.0
    assert link.capacity_in_remain == pytest.approx(100.0)
    link.capacity_out_remain = 100.0
    assert link.capacity_out_remain == pytest.approx(100.0)

    assert isinstance(link.cum_arrival, np.ndarray)
    assert isinstance(link.cum_departure, np.ndarray)
    assert isinstance(link.traveltime_instant, np.ndarray)
    assert isinstance(link.traveltime_actual, np.ndarray)
    assert link.edie_dx == 200

    W.exec_simulation()


# --- World misc methods, NotImplementedError stubs ---

def test_coverage_misc_world_methods():
    W = World(cpp=True, deltan=5, tmax=200, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    n1 = W.addNode("n1", 0, 0)
    n2 = W.addNode("n2", 1, 0)
    link = W.addLink("l1", "n1", "n2", length=500, free_flow_speed=20)
    W.adddemand("n1", "n2", 0, 100, 0.5)
    W.exec_simulation()

    assert W.get_node(n1).name == "n1"
    assert W.get_node("n2").name == "n2"
    assert W.get_node(None) is None
    with pytest.raises(Exception):
        W.get_node(99999)

    assert W.get_link(link).name == "l1"
    assert W.get_link("l1").name == "l1"
    assert W.get_link(None) is None
    with pytest.raises(Exception):
        W.get_link(99999)

    with pytest.raises(NotImplementedError):
        W.save_scenario("dummy")
    with pytest.raises(NotImplementedError):
        W.load_scenario("dummy")
    with pytest.raises(NotImplementedError):
        W.copy()
    with pytest.raises(NotImplementedError):
        W.save("dummy")

    result = W.on_time(50)
    assert result is True or result is False
    W.change_print_mode(0)


# --- Node signal, rename ---

def test_coverage_node_signal_and_rename():
    W = World(cpp=True, deltan=5, tmax=200, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    n1 = W.addNode("sig_node", 0, 0, signal=[60, 30])
    W.addNode("n2", 1, 0)
    W.addLink("l1", "sig_node", "n2", length=500, free_flow_speed=20, signal_group=[0])

    with pytest.raises((ValueError, RuntimeError)):
        W.addNode("sig_node", 2, 0, auto_rename=True)
    with pytest.raises(ValueError):
        W.addNode("sig_node", 3, 0)

    W.adddemand("sig_node", "n2", 0, 100, 0.3)
    W.exec_simulation()

    assert "sig_node" in repr(n1)
    sp = n1.signal_phase
    assert isinstance(sp, (int, np.integer))
    st = n1.signal_t
    assert isinstance(st, (int, float, np.floating))
    n1.signal_phase = 0
    n1.signal_t = 0


# --- Route dunder methods ---

def test_coverage_route_dunder_methods():
    W = World(cpp=True, deltan=5, tmax=200, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 0)
    W.addNode("dest", 2, 0)
    l1 = W.addLink("l1", "orig", "mid", length=500, free_flow_speed=20)
    l2 = W.addLink("l2", "mid", "dest", length=500, free_flow_speed=20)

    route = W.defRoute(["l1", "l2"])
    assert "Route" in repr(route)
    assert list(route) == [l1, l2]
    assert len(route) == 2
    assert route[0] == l1


# --- addVehicle: taxi NotImplementedError, links_avoid ---

def test_coverage_addVehicle_taxi_and_links_avoid():
    W = World(cpp=True, deltan=5, tmax=200, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 0)
    W.addNode("dest", 2, 0)
    link1 = W.addLink("link1", "orig", "mid", length=500, free_flow_speed=20)
    W.addLink("link2", "mid", "dest", length=500, free_flow_speed=20)

    with pytest.raises(NotImplementedError):
        W.addVehicle("orig", "dest", 0, mode="taxi")

    W.addVehicle("orig", "dest", 0, links_avoid=[link1])
    W.exec_simulation()
    assert len(W.VEHICLES) >= 1


# --- Demand methods: point2point, area2area, area2area2, nodes2nodes2 ---

def test_coverage_demand_methods():
    import warnings as _w
    W = World(cpp=True, deltan=5, tmax=500, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    nodes = {}
    for i in range(3):
        for j in range(3):
            nodes[(i, j)] = W.addNode(f"n{i}{j}", i, j)
    for i in range(3):
        for j in range(3):
            if i + 1 < 3:
                W.addLink(f"l{i}{j}_{i+1}{j}", f"n{i}{j}", f"n{i+1}{j}", length=500, free_flow_speed=20)
                W.addLink(f"l{i+1}{j}_{i}{j}", f"n{i+1}{j}", f"n{i}{j}", length=500, free_flow_speed=20)
            if j + 1 < 3:
                W.addLink(f"l{i}{j}_{i}{j+1}", f"n{i}{j}", f"n{i}{j+1}", length=500, free_flow_speed=20)
                W.addLink(f"l{i}{j+1}_{i}{j}", f"n{i}{j+1}", f"n{i}{j}", length=500, free_flow_speed=20)

    W.adddemand_point2point(0, 0, 2, 2, 0, 300, flow=0.5)
    with _w.catch_warnings():
        _w.simplefilter("ignore", FutureWarning)
        W.adddemand_area2area(0, 0, 1.5, 2, 2, 1.5, 0, 300, flow=0.2)
    W.adddemand_area2area2(0, 0, 1.5, 2, 2, 1.5, 0, 300, volume=50)
    W.adddemand_nodes2nodes2([nodes[(0,0)], nodes[(0,1)]], [nodes[(2,1)], nodes[(2,2)]], 0, 300, volume=50)

    W.exec_simulation()
    assert len(W.VEHICLES) > 0


# --- Link auto_rename ---

def test_coverage_link_auto_rename():
    W = World(cpp=True, deltan=5, tmax=100, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("A", 0, 0)
    W.addNode("B", 1, 0)
    link1 = W.addLink("duplink", "A", "B", length=500, free_flow_speed=20)
    assert link1.name == "duplink"
    link2 = W.addLink("duplink", "A", "B", length=500, free_flow_speed=20, auto_rename=True)
    assert link2.name.startswith("duplink_renamed")
    with pytest.raises(ValueError):
        W.addLink("duplink", "A", "B", length=500, free_flow_speed=20, auto_rename=False)


def test_coverage_vehicle_logs_and_analysis():
    """Access vehicle log data and analyzer through public API.
    Internally triggers build_all_vehicle_logs_flat_compact, build_enter_log_data, etc."""
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("orig", 0, 0)
    W.addNode("deadend", 1, 0)
    W.addNode("orig2", 0, 1)
    W.addNode("dest", 1, 1)
    link_dead = W.addLink("link_dead", "orig", "deadend", length=500, free_flow_speed=20)
    link_ok = W.addLink("link_ok", "orig2", "dest", length=500, free_flow_speed=20)

    for t in [0, 10, 20]:
        W.addVehicle("orig", "dest", departure_time=t, trip_abort=1, auto_rename=True)
    for t in [0, 10, 20, 30, 40]:
        W.addVehicle("orig2", "dest", departure_time=t, auto_rename=True)

    W.exec_simulation()
    # print_simple_stats triggers simulation_terminated → batch log build + enter_log build
    W.analyzer.print_simple_stats()

    # Access all vehicle log properties through public API
    for veh in W.VEHICLES.values():
        assert veh.state in ("home", "wait", "run", "end", "abort")
        assert len(veh.log_t) > 0
        assert len(veh.log_x) > 0
        assert len(veh.log_v) > 0
        assert len(veh.log_s) > 0
        assert len(veh.log_lane) > 0
        assert len(veh.log_state) > 0
        assert len(veh.log_link) > 0
        assert len(veh.log_t_link) > 0
        veh.traveled_route()
        _ = veh.departure_time
        _ = veh.travel_time

    # Access vehicles_enter_log through link (triggers build_enter_log_data internally)
    assert isinstance(link_ok.vehicles_enter_log, dict)

    # Verify abort and end states
    aborted = [v for v in W.VEHICLES.values() if v.state == "abort"]
    ended = [v for v in W.VEHICLES.values() if v.state == "end"]
    assert len(aborted) > 0
    assert len(ended) > 0


def test_coverage_simulation_lifecycle():
    """Test simulation lifecycle methods through public API.
    Covers check_simulation_ongoing, iterative exec, and print_scenario_stats."""
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    orig = W.addNode("orig", 0, 0)
    dest = W.addNode("dest", 1, 0)
    link1 = W.addLink("link1", "orig", "dest", length=500, free_flow_speed=20)

    W.adddemand("orig", "dest", 0, 200, 0.5)

    # Iterative execution
    W.exec_simulation(until_t=100)
    assert W.check_simulation_ongoing() == True

    W.exec_simulation()
    assert W.check_simulation_ongoing() == False

    # All vehicles should have ended
    for veh in W.VEHICLES.values():
        assert veh.state == "end"

    # Print methods through public API
    W.analyzer.print_simple_stats()


def test_coverage_signal_capacity_enforce_route():
    """Test signals, explicit capacity, and enforce_route through public API."""
    W = World(cpp=True, deltan=5, tmax=200, print_mode=0, save_mode=0, show_mode=0, random_seed=42)

    n1 = W.addNode("n1", 0, 0, signal=[60, 30])
    n2 = W.addNode("n2", 1, 0)
    n3 = W.addNode("n3", 2, 0)

    link1 = W.addLink("l1", "n1", "n2", length=500, free_flow_speed=20,
                       capacity_out=0.8, capacity_in=0.8, signal_group=[0])
    link2 = W.addLink("l2", "n2", "n3", length=500, free_flow_speed=20)

    W.addVehicle("n1", "n3", 0, auto_rename=True)
    vehs = list(W.VEHICLES.values())
    vehs[0].enforce_route([link1, link2])

    W.adddemand("n1", "n3", 0, 100, 0.3)
    W.exec_simulation()

    ended_count = len([v for v in W.VEHICLES.values() if v.state == "end"])
    assert ended_count > 0

    # Verify the enforced-route vehicle used the specified path
    route = vehs[0].traveled_route()[0]
    assert route[-1] == link2


def test_coverage_per_vehicle_log_mid_simulation():
    """Access vehicle log properties mid-simulation.
    Verifies that log data is accessible during iterative execution
    and that post-simulation analysis still works correctly afterward."""
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("A", 0, 0)
    W.addNode("B", 1000, 0)
    W.addLink("AB", "A", "B", length=1000, free_flow_speed=20)
    W.adddemand("A", "B", 0, 200, 0.5)

    # Partial execution
    W.exec_simulation(until_t=150)

    # Access vehicle logs mid-simulation for run/end vehicles
    accessed = 0
    for veh in W.VEHICLES.values():
        if veh.state in ("run", "end"):
            assert len(veh.log_t) > 0
            assert len(veh.log_x) > 0
            assert len(veh.log_v) > 0
            assert len(veh.log_s) > 0
            assert len(veh.log_lane) > 0
            assert len(veh.log_state) > 0
            assert len(veh.log_link) > 0
            assert len(veh.log_t_link) > 0
            accessed += 1
            if accessed >= 3:
                break
    assert accessed >= 3

    # Complete simulation — log caches are rebuilt at simulation_terminated
    W.exec_simulation()

    # Verify post-simulation analysis works correctly with fresh logs
    W.analyzer.print_simple_stats()
    ended = [v for v in W.VEHICLES.values() if v.state == "end"]
    assert len(ended) > 0
    for veh in ended[:3]:
        assert len(veh.log_t) > 0
        assert veh.travel_time > 0


def test_coverage_main_loop_parameter_branches():
    """Test different exec_simulation parameter combinations to cover main_loop branches."""
    W = World(cpp=True, deltan=5, tmax=300, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("A", 0, 0)
    W.addNode("B", 1000, 0)
    W.addLink("AB", "A", "B", length=1000, free_flow_speed=20)
    W.adddemand("A", "B", 0, 200, 0.5)

    # Branch 1: until_t
    W.exec_simulation(until_t=50)
    assert W.check_simulation_ongoing() == True

    # Branch 2: duration_t2 (runs for exactly duration_t2 seconds)
    W.exec_simulation(duration_t2=50)
    assert W.check_simulation_ongoing() == True

    # Complete
    W.exec_simulation()
    assert W.check_simulation_ongoing() == False


def test_coverage_route_choice_edge_cases():
    """Test route_next_link_choice with unreachable destination.
    A vehicle heading to an isolated node cannot find a route at the origin,
    triggering the route_next_link == nullptr branch in Node::generate."""
    W = World(cpp=True, deltan=5, tmax=200, print_mode=0, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("orig", 0, 0)
    W.addNode("mid", 1, 0)
    W.addNode("dest", 2, 0)
    W.addNode("island", 5, 5)  # isolated node, no links to/from
    W.addLink("l1", "orig", "mid", length=500, free_flow_speed=20)
    W.addLink("l2", "mid", "dest", length=500, free_flow_speed=20)

    # Vehicle heading to unreachable "island" node — should never depart
    W.addVehicle("orig", "island", 0, auto_rename=True)
    # Normal vehicles
    W.adddemand("orig", "dest", 0, 100, 0.3)

    W.exec_simulation()

    # Normal vehicles should complete
    ended = [v for v in W.VEHICLES.values() if v.state == "end"]
    assert len(ended) > 0

    # Unreachable vehicle should NOT have ended (still waiting or never departed)
    island_vehs = [v for v in W.VEHICLES.values() if v.dest == W.get_node("island")]
    assert len(island_vehs) == 1
    assert island_vehs[0].state != "end"


def test_coverage_no_cyclic_all_filtered():
    """Test no_cyclic_routing where all outlinks lead to visited nodes."""
    W = World(cpp=True, deltan=5, tmax=500, print_mode=0, save_mode=0, show_mode=0,
              random_seed=42, no_cyclic_routing=True)
    # Triangle: A -> B -> C -> A (cycle)
    W.addNode("A", 0, 0)
    W.addNode("B", 1, 0)
    W.addNode("C", 0.5, 1)
    W.addNode("D", 2, 0)  # final destination
    W.addLink("AB", "A", "B", length=500, free_flow_speed=20)
    W.addLink("BC", "B", "C", length=500, free_flow_speed=20)
    W.addLink("CA", "C", "A", length=500, free_flow_speed=20)
    W.addLink("BD", "B", "D", length=500, free_flow_speed=20)

    W.adddemand("A", "D", 0, 200, 0.3)
    W.exec_simulation()

    ended = [v for v in W.VEHICLES.values() if v.state == "end"]
    assert len(ended) > 0


def test_coverage_print_mode_enabled():
    """Run simulation with print_mode=1 to cover C++ print branches in main_loop."""
    W = World(cpp=True, deltan=5, tmax=200, print_mode=1, save_mode=0, show_mode=0, random_seed=42)
    W.addNode("A", 0, 0)
    W.addNode("B", 1, 0)
    W.addLink("AB", "A", "B", length=500, free_flow_speed=20)
    W.adddemand("A", "B", 0, 100, 0.3)
    W.exec_simulation()
    assert len(W.VEHICLES) > 0


# ======================================================================
# DTA solver tests for C++ mode (SolverDUE, SolverDSO_D2D)
# Based on 9x9 grid scenario from devlog/benchtestdta.py
# ======================================================================

def _create_dta_grid_world(seed, cpp):
    """Helper: build a 9x9 grid DTA scenario (from benchtestdta.py)."""
    W = uxsim.World(
        name="", deltan=10, tmax=4800,
        duo_update_time=300,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=seed, cpp=cpp,
    )
    imax, jmax = 9, 9
    id_center = 4
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i, j] = W.addNode(f"n{(i, j)}", i, j, flow_capacity=1.6)
    for i in range(imax):
        for j in range(jmax):
            free_flow_speed = 10
            if i != imax - 1:
                if j == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i+1, j)}", nodes[i, j], nodes[i+1, j],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if i != 0:
                if j == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i-1, j)}", nodes[i, j], nodes[i-1, j],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if j != jmax - 1:
                if i == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i, j+1)}", nodes[i, j], nodes[i, j+1],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if j != 0:
                if i == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i, j-1)}", nodes[i, j], nodes[i, j-1],
                           length=1000, free_flow_speed=free_flow_speed)

    demand_flow = 0.08
    demand_duration = 2400
    outer_ids = 3
    for n1 in [(0, j) for j in range(outer_ids, jmax - outer_ids)]:
        for n2 in [(imax - 1, j) for j in range(outer_ids, jmax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(i, jmax - 1) for i in range(outer_ids, imax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i, 0) for i in range(outer_ids, imax - outer_ids)]:
        for n2 in [(i, jmax - 1) for i in range(outer_ids, imax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(imax - 1, j) for j in range(outer_ids, jmax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    return W


def test_dta_solver_due_grid_cpp():
    """SolverDUE runs on a 9x9 grid with cpp=True and returns positive TTT."""
    from uxsim.DTAsolvers import SolverDUE

    def create_World():
        return _create_dta_grid_world(seed=42, cpp=True)

    solver = SolverDUE(create_World)
    solver.solve(max_iter=1, print_progress=False)
    W_sol = solver.W_sol
    W_sol.analyzer.print_simple_stats(force_print=False)
    assert W_sol.analyzer.total_travel_time > 0
    assert W_sol.analyzer.trip_completed > 0


def test_dta_solver_dso_d2d_grid_cpp():
    """SolverDSO_D2D runs on a 9x9 grid with cpp=True and returns positive TTT."""
    from uxsim.DTAsolvers import SolverDSO_D2D

    def create_World():
        return _create_dta_grid_world(seed=42, cpp=True)

    solver = SolverDSO_D2D(create_World)
    solver.solve(max_iter=1, print_progress=False)
    W_sol = solver.W_sol
    W_sol.analyzer.print_simple_stats(force_print=False)
    assert W_sol.analyzer.total_travel_time > 0
    assert W_sol.analyzer.trip_completed > 0


@pytest.mark.flaky(reruns=5)
def test_dta_solver_due_grid_cpp_vs_python():
    """SolverDUE on 9x9 grid: C++ and Python produce similar TTT."""
    from uxsim.DTAsolvers import SolverDUE

    solver_cpp = SolverDUE(lambda: _create_dta_grid_world(seed=42, cpp=True))
    solver_cpp.solve(max_iter=1, print_progress=False)
    ttt_cpp = solver_cpp.W_sol.analyzer.total_travel_time
    trips_cpp = solver_cpp.W_sol.analyzer.trip_completed

    solver_py = SolverDUE(lambda: _create_dta_grid_world(seed=42, cpp=False))
    solver_py.solve(max_iter=1, print_progress=False)
    ttt_py = solver_py.W_sol.analyzer.total_travel_time
    trips_py = solver_py.W_sol.analyzer.trip_completed

    assert trips_cpp == trips_py
    assert ttt_cpp == pytest.approx(ttt_py, rel=0.2)


@pytest.mark.flaky(reruns=5)
def test_dta_solver_dso_d2d_grid_cpp_vs_python():
    """SolverDSO_D2D on 9x9 grid: C++ and Python produce similar TTT."""
    from uxsim.DTAsolvers import SolverDSO_D2D

    solver_cpp = SolverDSO_D2D(lambda: _create_dta_grid_world(seed=42, cpp=True))
    solver_cpp.solve(max_iter=1, print_progress=False)
    ttt_cpp = solver_cpp.W_sol.analyzer.total_travel_time
    trips_cpp = solver_cpp.W_sol.analyzer.trip_completed

    solver_py = SolverDSO_D2D(lambda: _create_dta_grid_world(seed=42, cpp=False))
    solver_py.solve(max_iter=1, print_progress=False)
    ttt_py = solver_py.W_sol.analyzer.total_travel_time
    trips_py = solver_py.W_sol.analyzer.trip_completed

    assert trips_cpp == trips_py
    assert ttt_cpp == pytest.approx(ttt_py, rel=0.2)


def test_dta_solver_due_grid_convergence_metrics_cpp():
    """SolverDUE on 9x9 grid: tracks convergence metrics (ttts, n_swaps) with cpp=True."""
    from uxsim.DTAsolvers import SolverDUE

    n_iter = 3
    solver = SolverDUE(lambda: _create_dta_grid_world(seed=42, cpp=True))
    solver.solve(max_iter=n_iter, print_progress=False)

    assert len(solver.ttts) == n_iter
    assert len(solver.n_swaps) == n_iter
    assert all(t > 0 for t in solver.ttts)
    assert solver.W_sol is not None


def test_dta_solver_dso_d2d_grid_convergence_metrics_cpp():
    """SolverDSO_D2D on 9x9 grid: tracks convergence metrics (ttts, n_swaps) with cpp=True."""
    from uxsim.DTAsolvers import SolverDSO_D2D

    n_iter = 3
    solver = SolverDSO_D2D(lambda: _create_dta_grid_world(seed=42, cpp=True))
    solver.solve(max_iter=n_iter, print_progress=False)

    assert len(solver.ttts) == n_iter
    assert len(solver.n_swaps) == n_iter
    assert all(t > 0 for t in solver.ttts)
    assert solver.W_sol is not None


def test_dta_solver_due_grid_duo_baseline_cpp():
    """DUO on 9x9 grid with cpp=True produces same TTT as Python DUO."""
    W_cpp = _create_dta_grid_world(seed=42, cpp=True)
    W_cpp.exec_simulation()
    W_cpp.analyzer.print_simple_stats(force_print=False)
    ttt_cpp = W_cpp.analyzer.total_travel_time

    W_py = _create_dta_grid_world(seed=42, cpp=False)
    W_py.exec_simulation()
    W_py.analyzer.print_simple_stats(force_print=False)
    ttt_py = W_py.analyzer.total_travel_time

    assert ttt_cpp > 0
    assert ttt_cpp == pytest.approx(ttt_py, rel=0.1)


def test_dta_solver_due_grid_benchmark_cpp():
    """Benchmark: SolverDUE on 9x9 grid with cpp=True, prints wall-clock time."""
    import time
    from uxsim.DTAsolvers import SolverDUE

    t0 = time.perf_counter()
    solver = SolverDUE(lambda: _create_dta_grid_world(seed=42, cpp=True))
    solver.solve(max_iter=1, print_progress=False)
    elapsed_cpp = time.perf_counter() - t0

    t0 = time.perf_counter()
    solver_py = SolverDUE(lambda: _create_dta_grid_world(seed=42, cpp=False))
    solver_py.solve(max_iter=1, print_progress=False)
    elapsed_py = time.perf_counter() - t0

    speedup = elapsed_py / elapsed_cpp if elapsed_cpp > 0 else float("inf")
    print(f"\n[DTA Benchmark] SolverDUE 9x9 grid (max_iter=1):")
    print(f"  C++:    {elapsed_cpp:.3f}s")
    print(f"  Python: {elapsed_py:.3f}s")
    print(f"  Speedup: {speedup:.2f}x")

    assert solver.W_sol.analyzer.total_travel_time > 0


def test_dta_solver_dso_d2d_grid_benchmark_cpp():
    """Benchmark: SolverDSO_D2D on 9x9 grid with cpp=True, prints wall-clock time."""
    import time
    from uxsim.DTAsolvers import SolverDSO_D2D

    t0 = time.perf_counter()
    solver = SolverDSO_D2D(lambda: _create_dta_grid_world(seed=42, cpp=True))
    solver.solve(max_iter=1, print_progress=False)
    elapsed_cpp = time.perf_counter() - t0

    t0 = time.perf_counter()
    solver_py = SolverDSO_D2D(lambda: _create_dta_grid_world(seed=42, cpp=False))
    solver_py.solve(max_iter=1, print_progress=False)
    elapsed_py = time.perf_counter() - t0

    speedup = elapsed_py / elapsed_cpp if elapsed_cpp > 0 else float("inf")
    print(f"\n[DTA Benchmark] SolverDSO_D2D 9x9 grid (max_iter=1):")
    print(f"  C++:    {elapsed_cpp:.3f}s")
    print(f"  Python: {elapsed_py:.3f}s")
    print(f"  Speedup: {speedup:.2f}x")

    assert solver.W_sol.analyzer.total_travel_time > 0
