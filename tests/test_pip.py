"""
This script tests whether UXsim in pypi installed via pip (especially C++ mode) is valid. 
"""

from uxsim import *

def run_small(cpp):
    """Small-scale merge scenario."""
    W = World(
        name="", deltan=5, tmax=1200,
        no_cyclic_routing=True,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=None, cpp=cpp,
    )
    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.adddemand("orig1", "dest", 0, 1000, 0.45)
    W.adddemand("orig2", "dest", 400, 1000, 0.6)

    W.exec_simulation()
    
    df = W.analyzer.basic_to_pandas()

    return df["total_travel_time"].sum()

def test_run_small_scenario_pip():
    ttt_pyt = []
    ttt_cpp = []
    
    for i in range(10):
        ttt_pyt.append(run_small(False))
        ttt_cpp.append(run_small(True))

    assert eq_tol(sum(ttt_pyt), sum(ttt_cpp))
