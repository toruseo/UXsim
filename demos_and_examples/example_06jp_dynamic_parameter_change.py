from uxsim import *
import pandas as pd


if __name__ == "__main__":

    # シミュレーション本体の定義
    W = World(
        name="",
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # シナリオ定義
    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    link1 = W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=0.5)
    link2 = W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2)
    link3 = W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig1", "dest", 0, 1000, 0.4)
    W.adddemand("orig2", "dest", 300, 1000, 0.6)

    #特定時間だけシミュを回す（途中で介入したいとき用）
    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t=100) #100秒づつシミュレーションを回す

        print("\t", link1.num_vehicles_queue, "/", link1.num_vehicles)    #リンク車両台数を出力
        print("\t", link1.flow, link1.density, link1.speed) #リンク交通状態を出力

        if W.on_time(600):
            link2.merge_priority = 0.2    #リンク合流優先率を変更

        if W.on_time(800):
            link1.free_flow_speed = 5    #リンク自由流速度を変更

        if W.on_time(1000):
            link3.capacity_in = link3.capacity/2    #リンクへの流入容量を変更

    # 結果可視化
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_traj_links([["link1", "link3"], ["link2", "link3"]])
    W.analyzer.macroscopic_fundamental_diagram()

    # 結果をpandas.DataFrameに変換
    print(W.analyzer.basic_to_pandas()) #動的にモデルパラメータを変化させた場合，自由旅行時間と遅れの計算が不適切になる
    print(W.analyzer.od_to_pandas())
    print(W.analyzer.mfd_to_pandas())
    print(W.analyzer.link_to_pandas())
    print(W.analyzer.link_traffic_state_to_pandas())
    print(W.analyzer.vehicles_to_pandas())