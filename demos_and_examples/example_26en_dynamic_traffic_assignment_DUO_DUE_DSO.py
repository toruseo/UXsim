"""
Demonstration of dynamic traffic assginment (DTA) in a simple network.
- Dynamic User Optimum (DUO)
- Dynamic User Equilibrium (DUE)
- Dynamic System Optimum (DSO)
"""

from pylab import *
import uxsim
from uxsim.DTAsolvers import *

#################################
# scenario definition
def create_World():
    """
    A function that returns World object with scenario informaiton. This is faster way to reuse the same scenario, as `World.copy` or `World.load_scenario` takes some computation time.

    This scenario mimics a stuation where 2 pararell routes, namely highway (fast but small capacpity) and arterial road (slow but large capacity), exist between an OD pair. Travelers choose an appropriate route depending on the routing princigple.
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
W = create_World()
W.exec_simulation()
W.analyzer.print_simple_stats(force_print=True)
df_DUO = W.analyzer.basic_to_pandas()

#################################
# DUE
solver_DUE = SolverDUE(create_World)
solver_DUE.solve(max_iter=50)
W_DUE = solver_DUE.W_sol
W_DUE.analyzer.print_simple_stats(force_print=True)
df_DUE = W_DUE.analyzer.basic_to_pandas()


#################################
# DSO
solver_DSO = SolverDSO_D2D(create_World)
solver_DSO.solve(max_iter=50)
W_DSO = solver_DSO.W_sol
W_DSO.analyzer.print_simple_stats(force_print=True)
df_DSO = W_DSO.analyzer.basic_to_pandas()

#################################
# Results

# print stats
print("DUO")
print(df_DUO)
print("DUE")
print(df_DUE)
print("DSO")
print(df_DSO)


# visualizations
solver_DUE.plot_convergence()
solver_DUE.plot_link_stats()
solver_DUE.plot_vehicle_stats(orig="4", dest="7")

solver_DSO.plot_convergence()
solver_DSO.plot_link_stats()
solver_DSO.plot_vehicle_stats(orig="4", dest="7")
