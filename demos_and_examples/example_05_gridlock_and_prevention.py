from uxsim import *
import pandas as pd


if __name__ == "__main__":

    print("####"*10)
    print("# gridlock scenario")
    # シミュレーション本体の定義
    W = World(
        name="gridlock",
        deltan=5,
        tmax=6000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # シナリオ定義
    #uroborosネットワークのgridlock：ファイル読込
    W.load_scenario_from_csv("dat/uroboros_nodes.csv", "dat/uroboros_links.csv", "dat/uroboros_demand.csv")

    # gridlockを防止する制御．コメントアウトして実行
    #W.get_link("NE").merge_priority = 2
    #W.get_link("SW").merge_priority = 2

    # シミュレーション実行
    #最後までシミュを回す
    W.exec_simulation()

    # 結果可視化
    W.analyzer.print_simple_stats()
    #W.analyzer.time_space_diagram_density()
    #W.analyzer.time_space_diagram_traj()
    W.analyzer.time_space_diagram_traj_links([["WN", "NE", "ES", "SW"]])
    #W.analyzer.cumulative_curves()
    W.analyzer.macroscopic_fundamental_diagram()
    #for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    #    W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))
    #for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    #    W.analyzer.network(t, detailed=1, network_font_size=0)
    #W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
    #W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=1, interval=5, trace_length=5)

    # 結果をpandas.DataFrameに変換
    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())
    print(W.analyzer.vehicles_to_pandas())



    print("####"*10)
    print("# no gridlock scenario")
    # シミュレーション本体の定義
    W = World(
        name="gridlock_prevented",
        deltan=5,
        tmax=6000,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # シナリオ定義
    #uroborosネットワークのgridlock回避：ファイル読込
    W.load_scenario_from_csv("dat/uroboros_nodes.csv", "dat/uroboros_links.csv", "dat/uroboros_demand.csv")

    # gridlockを防止する制御．コメントアウトして実行
    W.get_link("NE").merge_priority = 2
    W.get_link("SW").merge_priority = 2

    # シミュレーション実行
    #最後までシミュを回す
    W.exec_simulation()

    # 結果可視化
    W.analyzer.print_simple_stats()
    #W.analyzer.time_space_diagram_density()
    #W.analyzer.time_space_diagram_traj()
    W.analyzer.time_space_diagram_traj_links([["WN", "NE", "ES", "SW"]])
    #W.analyzer.cumulative_curves()
    W.analyzer.macroscopic_fundamental_diagram()
    #for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    #    W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))
    #for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    #    W.analyzer.network(t, detailed=1, network_font_size=0)
    #W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
    #W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=1, interval=5, trace_length=5)

    # 結果をpandas.DataFrameに変換
    print(W.analyzer.basic_to_pandas())
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())
    print(W.analyzer.vehicles_to_pandas())