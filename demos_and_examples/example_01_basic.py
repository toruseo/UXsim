from uxsim import *
import pandas as pd


if __name__ == "__main__":

    # シミュレーション本体の定義
    #単位系は全てsとmで統一
    W = World(
        name="",    #シナリオ名称．空白でも可．結果保存のフォルダ名に使われる
        deltan=5,   #シミュレーション集計単位Δn．何台の車両をまとめて計算するか．計算コストは基本的にdeltan^2に逆比例する
        tmax=1200,  #総シミュレーション時間（s）
        print_mode=1, save_mode=1, show_mode=0,    #各種オプション．print_modeは各種情報をprintするかどうか．普段は1とし，自動で多数のシミュレーションを回すときは0を推奨．save_modeは可視化結果等を保存するかどうか．show_mode可視化結果を表示するかどうか．Jupyter Notebook上ではshow_mode=1が良いが，それ以外は0を推奨．
        random_seed=0    #乱数シードの設定．再現性のある実験をしたいときは指定，そうでないときはNone．Jupyter Notebook上では乱数が固定されない場合有り（要修正）
    )

    # シナリオ定義
    #合流ネットワーク：ベタ打ち定義の例
    W.addNode("orig1", 0, 0) #ノードの作成．ノード名，可視化x座標，可視化y座標
    node_orig2 = W.addNode("orig2", 0, 2) #W.addNodeは作成したノードインスタンスを返すので，変数に代入しても良い
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=0.5) #リンクの作成．リンク名，起点ノード，終点ノード，length=長さ, free_flow_speed=自由流速度, jam_density=渋滞密度, merge_priority=合流時優先率
    W.addLink("link2", node_orig2, "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2) #ノードは名称ではなく変数で指定しても良い
    link3 = W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2) #リンク作成もインスタンスを返す．
    W.adddemand("orig1", "dest", 0, 1000, 0.4) #OD交通需要の作成．出発地ノード，目的地ノード，開始時刻，終了時刻，需要率
    W.adddemand("orig2", "dest", 500, 1000, 0.6)

    # シミュレーション実行
    #最後までシミュを回す
    W.exec_simulation()

    #特定時間だけシミュを回す（途中で介入したいとき用）
    #while W.check_simulation_ongoing():
    #    W.exec_simulation(duration_t=100) #100秒づつシミュレーションを回す

    # 結果可視化：一部は超遅いので注意．やらなくてもシミュレーション機能は変化しないので，なくても良い．
    W.analyzer.print_simple_stats()
    W.analyzer.time_space_diagram_density()
    W.analyzer.time_space_diagram_traj()
    W.analyzer.time_space_diagram_traj_links([["link1", "link3"], ["link2", "link3"]])
    W.analyzer.cumulative_curves()
    W.analyzer.plot_vehicle_log("110")
    W.analyzer.macroscopic_fundamental_diagram()
    for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(4,4))
    for t in list(range(0,W.TMAX,int(W.TMAX/6))):
        W.analyzer.network(t, detailed=1, network_font_size=0)
    W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
    W.analyzer.network_anim(detailed=1, network_font_size=0, figsize=(12,12))
    W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=1.0, interval=3, trace_length=5)

    # 結果をpandas.DataFrameに変換して分析しやすいようにする
    print(W.analyzer.basic_to_pandas()) #基礎統計
    print(W.analyzer.od_to_pandas())    #OD別情報
    print(W.analyzer.mfd_to_pandas())   #MFD
    print(W.analyzer.link_to_pandas())  #リンク単位
    print(W.analyzer.link_traffic_state_to_pandas())    #リンク内部の交通状態
    print(W.analyzer.vehicles_to_pandas())  #車両ごと

    # 結果をCSVに保存
    W.analyzer.output_data()