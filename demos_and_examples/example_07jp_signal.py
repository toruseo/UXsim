import pandas as pd
from uxsim import *

if __name__ == "__main__":

    # シミュレーション本体の定義
    W = World(
        name="",
        deltan=1,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # シナリオ定義
    #合流ネットワーク：信号制御
    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1, signal=[30,60]) #変数signalは，[phase 0の時間, phase 1の時間, ...]のリスト
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0) #変数signal_groupの値nは，このリンクの出口はphase nで青になることを意味する
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2, capacity_in=1)
    W.adddemand("orig1", "dest", 0, 1000, 0.2)
    W.adddemand("orig2", "dest", 500, 1000, 0.6)

    # シミュレーション実行
    #最後までシミュを回す
    W.exec_simulation()

    #特定時刻までシミュを回す（途中で介入したいとき用）
    #W.exec_simulation(until_t=3600)

    #特定時間だけシミュを回す（途中で介入したいとき用）
    #while W.check_simulation_ongoing():
    #    W.exec_simulation(duration_t=1000)


    # 結果可視化
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["link1", "link3"], ["link2", "link3"]])
    W.analyzer.cumulative_curves()
    #W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    #W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.3, interval=5, trace_length=5)