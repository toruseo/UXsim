from uxsim import *

# シミュレーション本体の定義
W = World(
    name="",
    deltan=1,
    tmax=1500,
    print_mode=1, save_mode=1, show_mode=1,
    random_seed=None
)

# シナリオ定義
#4現示信号:
#    現示0: 南北直左のみ50秒
#    現示1: 南北右折のみ10秒
#    現示2: 東西直左のみ40秒
#    現示3: 東西右折のみ5秒
W.addNode("S_orig", 0, -3)    #起点ノード
W.addNode("N_orig", 0, 3)
W.addNode("W_orig", -3, 0)
W.addNode("E_orig", 3, 0)
W.addNode("S_dest", 0, -3)    #終点ノード
W.addNode("N_dest", 0, 3)
W.addNode("W_dest", -3, 0)
W.addNode("E_dest", 3, 0)

signal_phase_setting = [50,10,40,5]
W.addNode("S_i", 0, -1, signal=signal_phase_setting)  #信号交差点の南側入口ノード
W.addNode("N_i", 0.2, 1, signal=signal_phase_setting)
W.addNode("W_i", -1, 0.2, signal=signal_phase_setting)
W.addNode("E_i", 1, 0, signal=signal_phase_setting)
W.addNode("S_o", 0.2, -1, signal=signal_phase_setting)  #信号交差点の南側出口ノード
W.addNode("N_o", 0, 1, signal=signal_phase_setting)
W.addNode("W_o", -1, 0, signal=signal_phase_setting)
W.addNode("E_o", 1, 0.2, signal=signal_phase_setting)

W.addLink("linkSin", "S_orig", "S_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[0,1])   #交差点への流入リンク
W.addLink("linkNin", "N_orig", "N_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[0,1])
W.addLink("linkWin", "W_orig", "W_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[2,3])
W.addLink("linkEin", "E_orig", "E_i", length=200, free_flow_speed=20, jam_density=0.2, signal_group=[2,3])
W.addLink("linkSout", "S_o", "S_dest", length=200, free_flow_speed=20, jam_density=0.2)   #交差点からの流出リンク
W.addLink("linkNout", "N_o", "N_dest", length=200, free_flow_speed=20, jam_density=0.2)
W.addLink("linkWout", "W_o", "W_dest", length=200, free_flow_speed=20, jam_density=0.2)
W.addLink("linkEout", "E_o", "E_dest", length=200, free_flow_speed=20, jam_density=0.2)

#交差点内部ダミーリンク
W.addLink("signal_SN_s", "S_i", "N_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)  #直進．lengthは仮のもの．あまりに短すぎると丸め誤差で変なことがおきるかも
W.addLink("signal_NS_s", "N_i", "S_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)
W.addLink("signal_SW_l", "S_i", "W_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)  #左折
W.addLink("signal_NE_l", "N_i", "E_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=0)
W.addLink("signal_SE_r", "S_i", "E_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=1)  #右折．lengthは右折待ち行列の最大長さに相当
W.addLink("signal_NW_r", "N_i", "W_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=1)
W.addLink("signal_WE_s", "W_i", "E_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
W.addLink("signal_EW_s", "E_i", "W_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
W.addLink("signal_WN_l", "W_i", "N_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
W.addLink("signal_ES_l", "E_i", "S_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=2)
W.addLink("signal_WS_r", "W_i", "S_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=3)
W.addLink("signal_EN_r", "E_i", "N_o", length=20, free_flow_speed=20, jam_density=0.2, signal_group=3)

demand_s = 0.2
demand_l = 0.03
demand_r = 0.03
duration = 1000
W.adddemand("S_orig", "N_dest", 0, duration, demand_s)
W.adddemand("S_orig", "E_dest", 0, duration, demand_l)
W.adddemand("S_orig", "W_dest", 0, duration, demand_r)
W.adddemand("N_orig", "S_dest", 0, duration, demand_s)
W.adddemand("N_orig", "E_dest", 0, duration, demand_l)
W.adddemand("N_orig", "W_dest", 0, duration, demand_r)
W.adddemand("W_orig", "E_dest", 0, duration, demand_s)
W.adddemand("W_orig", "N_dest", 0, duration, demand_l)
W.adddemand("W_orig", "S_dest", 0, duration, demand_r)
W.adddemand("E_orig", "W_dest", 0, duration, demand_s)
W.adddemand("E_orig", "N_dest", 0, duration, demand_l)
W.adddemand("E_orig", "S_dest", 0, duration, demand_r)

# シミュレーション実行
#最後までシミュを回す
W.exec_simulation()

# 結果可視化
W.analyzer.print_simple_stats()

print("南から北へ直進する経路")
W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SN_s", "linkNout"], xlim=[0,300])
W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SN_s", "linkNout"])
print("南から西へ左折する経路")
W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SW_l", "linkWout"], xlim=[0,300])
W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SW_l", "linkWout"])
print("南から東へ右折する経路")
W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SE_r", "linkEout"], xlim=[0,300])
W.analyzer.time_space_diagram_traj_links(["linkSin","signal_SE_r", "linkEout"])

for t in list(range(0,W.TMAX,int(W.TMAX/10))):   
    W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))