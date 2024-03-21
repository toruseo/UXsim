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