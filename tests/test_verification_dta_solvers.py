"""
This script verifies whether UXsim outputs reasonable solutions for DTA.
"""

import pytest
import uxsim
from uxsim.DTAsolvers import *

import matplotlib
matplotlib.use('Agg')

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

@pytest.mark.flaky(reruns=10)
def test_DTA_total_travel_time_comparison():
    # scenario definition
    def create_World():
        """
        A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.
        """
        W = uxsim.World(
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
    W_DUO = create_World()
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
    solver_DSO_GA.solve(max_iter=20, pop_size=20)   # max_iter should be larger (e.g., 100). this is just for demonstration
    W_DSO_GA = solver_DSO_GA.W_sol
    W_DSO_GA.analyzer.print_simple_stats(force_print=True)
    df_DSO_GA = W_DSO_GA.analyzer.basic_to_pandas()

    #################################
    # DSO by ALNS: this is obsolete
    solver_DSO_ALNS = DTAsolvers.SolverDSO_ALNS(create_World)
    solver_DSO_ALNS.solve(max_iter=200)   # max_iter should be larger (e.g., 500). this is just for demonstration
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
    assert W_DUE.analyzer.total_travel_time > W_DSO_GA.analyzer.total_travel_time
    assert W_DUE.analyzer.total_travel_time > W_DSO_ALNS.analyzer.total_travel_time
    assert W_DUE.analyzer.total_travel_time > W_DSO_D2D.analyzer.total_travel_time


@pytest.mark.flaky(reruns=10)
def test_DTA_total_travel_time_comparison_with_initial_solutions_for_DSO():
    # scenario definition
    def create_World():
        """
        A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.
        """
        W = uxsim.World(
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
    W_DUO = create_World()
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
    solver_DSO_GA.solve(max_iter=20, pop_size=20, initial_solution_World=W_DUE)   # max_iter should be larger (e.g., 100). this is just for demonstration
    W_DSO_GA = solver_DSO_GA.W_sol
    W_DSO_GA.analyzer.print_simple_stats(force_print=True)
    df_DSO_GA = W_DSO_GA.analyzer.basic_to_pandas()

    #################################
    # DSO by ALNS
    solver_DSO_ALNS = SolverDSO_ALNS(create_World)
    solver_DSO_ALNS.solve(max_iter=200, initial_solution_World=W_DUE)   # max_iter should be larger (e.g., 500). this is just for demonstration
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
    assert W_DUE.analyzer.total_travel_time > W_DSO_GA.analyzer.total_travel_time
    assert W_DUE.analyzer.total_travel_time > W_DSO_ALNS.analyzer.total_travel_time