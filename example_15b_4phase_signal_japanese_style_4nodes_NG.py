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
    #4現示信号:
    #    現示0: 南北直左のみ50秒
    #    現示1: 南北右折のみ10秒
    #    現示2: 東西直左のみ40秒
    #    現示3: 東西右折のみ5秒
    W.addNode("S", 0, -2)    #起終点ノード
    W.addNode("N", 0, 2)
    W.addNode("W", -2, 0)
    W.addNode("E", 2, 0)
    
    signal_phase_setting = [50,10,40,5]
    W.addNode("S_in", 0, -1, signal=signal_phase_setting)  #信号交差点の南側出入口ノード
    W.addNode("N_in", 0, 1, signal=signal_phase_setting)
    W.addNode("W_in", -1, 0, signal=signal_phase_setting)
    W.addNode("E_in", 1, 0, signal=signal_phase_setting)
    
    W.addLink("linkSin", "S", "S_in", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[0,1])   #交差点への流入リンク
    W.addLink("linkNin", "N", "N_in", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[0,1])
    W.addLink("linkWin", "W", "W_in", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[2,3])
    W.addLink("linkEin", "E", "E_in", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[2,3])
    W.addLink("linkSout", "S_in", "S", length=200, free_flow_speed=20, jam_density=0.2)   #交差点からの流出リンク
    W.addLink("linkNout", "N_in", "N", length=200, free_flow_speed=20, jam_density=0.2)
    W.addLink("linkWout", "W_in", "W", length=200, free_flow_speed=20, jam_density=0.2)
    W.addLink("linkEout", "E_in", "E", length=200, free_flow_speed=20, jam_density=0.2)
    
    #交差点内部ダミーリンク
    W.addLink("signal_SN_s", "S_in", "N_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0)  #直進．lengthは仮のもの．あまりに短すぎると丸め誤差で変なことがおきるかも
    W.addLink("signal_NS_s", "N_in", "S_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0)
    W.addLink("signal_SW_l", "S_in", "W_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0)  #左折
    W.addLink("signal_NE_l", "N_in", "E_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=0)
    W.addLink("signal_SE_r", "S_in", "E_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=10, signal_group=1)  #右折．lengthは右折待ち行列の最大長さに相当
    W.addLink("signal_NW_r", "N_in", "W_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=10, signal_group=1)
    W.addLink("signal_WE_s", "W_in", "E_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=2)
    W.addLink("signal_EW_s", "E_in", "W_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=2)
    W.addLink("signal_WN_l", "W_in", "N_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=2)
    W.addLink("signal_ES_l", "E_in", "S_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=1, signal_group=2)
    W.addLink("signal_WS_r", "W_in", "S_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=10, signal_group=3)
    W.addLink("signal_EN_r", "E_in", "N_in", length=20, free_flow_speed=20, jam_density=0.2, merge_priority=10, signal_group=3)
    
    demand_s = 0.3
    demand_l = 0.01
    demand_r = 0.01
    duration = 1200
    W.adddemand("S", "N", 0, duration, demand_s)
    W.adddemand("S", "E", 0, duration, demand_l)
    W.adddemand("S", "W", 0, duration, demand_r)
    W.adddemand("N", "S", 0, duration, demand_s)
    W.adddemand("N", "E", 0, duration, demand_l)
    W.adddemand("N", "W", 0, duration, demand_r)
    W.adddemand("W", "E", 0, duration, demand_s)
    W.adddemand("W", "N", 0, duration, demand_l)
    W.adddemand("W", "S", 0, duration, demand_r)
    W.adddemand("E", "W", 0, duration, demand_s)
    W.adddemand("E", "N", 0, duration, demand_l)
    W.adddemand("E", "S", 0, duration, demand_r)
    
    # シミュレーション実行
    #最後までシミュを回す
    W.exec_simulation()
    
    # 結果可視化
    W.analyzer.print_simple_stats()
    #W.analyzer.cumulative_curves()
    W.analyzer.time_space_diagram_traj()
    for t in list(range(0,W.TMAX,int(W.TMAX/10))):   
        W.analyzer.network(t, detailed=1, network_font_size=0, figsize=(4,4))
    #W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.3, interval=5, trace_length=5)