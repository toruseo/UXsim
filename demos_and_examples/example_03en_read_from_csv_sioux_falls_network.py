from uxsim import *
import pandas as pd


if __name__ == "__main__":

    W = World(
        name="",
        deltan=5,
        tmax=7200,
        print_mode=1, save_mode=1, show_mode=0,
        random_seed=0
    )

    # you can read CSV files like this.
    W.load_scenario_from_csv("dat/siouxfalls_nodes.csv", "dat/siouxfalls_links.csv", "dat/siouxfalls_demand.csv")

    W.exec_simulation()

    W.analyzer.print_simple_stats()
    #W.analyzer.time_space_diagram_density()
    #W.analyzer.time_space_diagram_traj()
    W.analyzer.time_space_diagram_traj_links([["21-22", "22-15", "15-10", "10-9", "9-5"]])
    #W.analyzer.cumulative_curves()
    W.analyzer.macroscopic_fundamental_diagram()
    for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(8,8))
    #for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    #    W.analyzer.network(t, detailed=1, network_font_size=0)
    #W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
    #W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(6,6))
    #W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.8, interval=5, trace_length=5)

    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())