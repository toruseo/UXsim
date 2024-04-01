"""
This script verifies whether UXsim outputs reasonable solutions for multiple route networks in various configurations.
Note that it uses random numbers for rouce choice behavior, so the results may vary slightly between runs.
"""

import pytest
from uxsim import *
import pandas as pd

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
        W = World(
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
        W = World(
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
        W = World(
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
        W = World(
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
        W = World(
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
        W = World(
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
        link11.free_flow_speed = 5
        link12.free_flow_speed = 5
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
        W = World(
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
        W = World(
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
        W = World(
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