"""
This script verifies whether UXsim outputs reasonable solutions for Sioux Falls network, checking the overall behavior.
Note that it uses random numbers for node and route choice behavior, so the results may vary slightly between runs.
"""

import pytest
from uxsim import *
import pandas as pd

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

@pytest.mark.flaky(reruns=5)
def test_sioux_falls():
    total_trips_list = []
    completed_trips_list = []
    total_travel_time_list = []
    average_travel_time_list = []
    average_delay_list = []
    link_traffic_volume_mean_list = []
    link_traffic_volume_std_list = []

    for i in range(5):
        W = World(
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

    assert equal_tolerance(np.average(total_trips_list), 34690.0)
    assert equal_tolerance(np.average(completed_trips_list), 33247.0)
    assert equal_tolerance(np.average(total_travel_time_list), 58423065.0)
    assert equal_tolerance(np.average(average_travel_time_list), 1757.2519806618154)
    assert equal_tolerance(np.average(average_delay_list), 386.9874140444886)
    assert equal_tolerance(np.average(link_traffic_volume_mean_list), 1185.8552631578946, rel_tol=0.2)
    assert equal_tolerance(np.average(link_traffic_volume_std_list), 750.477574487115, rel_tol=0.2)

    # assert equal_tolerance(np.std(total_trips_list), 0.0, rel_tol=1)
    # assert equal_tolerance(np.std(completed_trips_list), 65.46754921333164, rel_tol=1)
    # assert equal_tolerance(np.std(total_travel_time_list), 138625.4193862006, rel_tol=1)
    # assert equal_tolerance(np.std(average_travel_time_list), 6.016662655879493, rel_tol=1)
    # assert equal_tolerance(np.std(average_delay_list), 7.43415746253196, rel_tol=1)
    # assert equal_tolerance(np.std(link_traffic_volume_mean_list), 8.806259578063365, rel_tol=1)
    # assert equal_tolerance(np.std(link_traffic_volume_std_list), 10.943099195703097, rel_tol=1)