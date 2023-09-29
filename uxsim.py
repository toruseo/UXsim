import numpy as np
import matplotlib.pyplot as plt
import random, copy, glob, os, csv, time
import pandas as pd
from PIL import Image, ImageDraw, ImageFont, ImageOps
from PIL.Image import Resampling, Transpose
from tqdm.auto import tqdm
from collections import deque, OrderedDict
from collections import defaultdict as ddict
from utils.utils  import *

from scipy.sparse.csgraph import floyd_warshall

import warnings

plt.rcParams["font.family"] = "monospace"

# ノードクラス
class Node:
    def __init__(s, W, name, x, y, signal=[0]):
        """Create a node
        
        Parameters
        ----------
        W : object
            The network to which the node belongs.
        name : str
            The name of the node.
        x : float
            The x-coordinate of the node (for visualization purposes).
        y : float
            The y-coordinate of the node (for visualization purposes).
        signal : list of int, optional
            A list representing the signal at the node. Default is [0], representing no signal.
            If a signal is present, the list contains the green times for each group, e.g., [green_time_group0, green_time_group1, ...].
        """
        
        s.W = W
        #ノード位置（可視化用）
        s.x = x
        s.y = y
        
        #流入・流出リンク
        s.inlinks = {}
        s.outlinks = {}
        
        #リンク間遷移リクエスト（デマンド）
        s.incoming_vehicles = []
        
        #発生車両流入待ち行列（vertical queue）
        s.generation_queue = deque()
        
        #信号交差点
        #signal=[0]であれば信号無し
        #信号ありにするときはsignal=[group0の青時間，group1の青時間，...]とする
        s.signal = signal
        s.signal_phase = 0
        s.signal_t = 0
        
        s.id = len(s.W.NODES)
        s.name = name
        s.W.NODES.append(s)
    
    def __repr__(s):
        return f"<Node {s.name}>"
    
    def signal_control(s):
        if s.signal_t > s.signal[s.signal_phase]:
            s.signal_phase += 1
            s.signal_t = 0
            if s.signal_phase >= len(s.signal):
                s.signal_phase = 0
        s.signal_t += s.W.DELTAT
    
    def generate(s):
        #出発待ち行列から出発
        if len(s.generation_queue) > 0:
            veh = s.generation_queue[0]
            outlinks = list(s.outlinks.values())
            
            if len(outlinks):
                preference = [veh.route_pref[l] for l in outlinks]
                if sum(preference) > 0:
                    outlink = random.choices(outlinks, preference)[0]
                else:
                    outlink = random.choices(outlinks)[0]
                
                if (len(outlink.vehicles) == 0 or outlink.vehicles[-1].x > outlink.delta*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN:
                    #受け入れ可能な場合，リンク優先度に応じて選択
                    veh = s.generation_queue.popleft()
                    
                    veh.state = "run"
                    veh.link = outlink
                    veh.x = 0
                    veh.v = outlink.u #端部の挙動改善
                    s.W.VEHICLES_RUNNING[veh.name] = veh
                    
                    if len(outlink.vehicles) > 0:
                        veh.leader = outlink.vehicles[-1]
                        veh.leader.follower = veh 
                    
                    outlink.vehicles.append(veh)
                    
                    outlink.cum_arrival[-1] += s.W.DELTAN
                    veh.link_arrival_time = s.W.T*s.W.DELTAT
                    
                    outlink.capacity_in_remain -= s.W.DELTAN
    
    def transfer(s):
        #リンク間遷移
        for outlink in {veh.route_next_link for veh in s.incoming_vehicles if veh.route_next_link != None}:
            if (len(outlink.vehicles) == 0 or outlink.vehicles[-1].x > outlink.delta*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN:
                #受け入れ可能かつ流出可能の場合，リンク優先度に応じて選択
                vehs = [
                    veh for veh in s.incoming_vehicles 
                    if veh.route_next_link == outlink and veh.link.signal_group == s.signal_phase and veh.link.capacity_out_remain >= s.W.DELTAN
                ]
                if len(vehs) == 0:
                    continue
                veh = random.choices(vehs, [veh.link.merge_priority for veh in vehs])[0]
                inlink = veh.link
                
                #累積台数関連更新
                inlink.cum_departure[-1] += s.W.DELTAN
                outlink.cum_arrival[-1] += s.W.DELTAN
                inlink.traveltime_actual[-1] = s.W.T*s.W.DELTAT - veh.link_arrival_time
                veh.link_arrival_time = s.W.T*s.W.DELTAT
                
                inlink.capacity_out_remain -= s.W.DELTAN
                outlink.capacity_in_remain -= s.W.DELTAN
                
                #リンク間遷移実行
                inlink.vehicles.popleft()
                veh.link = outlink
                veh.x = 0
                
                if veh.follower != None:
                    veh.follower.leader = None
                    veh.follower = None
                veh.leader = None
                if len(outlink.vehicles):
                    veh.leader = outlink.vehicles[-1]
                    veh.leader.follower = veh
                
                #走り残し処理
                x_next = veh.move_remain*outlink.u/inlink.u
                if veh.leader != None:
                    x_cong = veh.leader.x_old - veh.link.delta*veh.W.DELTAN
                    if x_cong < veh.x:
                        x_cong = veh.x
                    if x_next > x_cong:
                        x_next = x_cong
                    if x_next >= outlink.length:
                        x_next = outlink.length
                veh.x = x_next
                
                outlink.vehicles.append(veh)
                s.incoming_vehicles.remove(veh)
        
        s.incoming_vehicles = []
    
    def update(s):
        s.signal_control()

# リンククラス
class Link:
    def __init__(s, W, name, start_node, end_node, length, free_flow_speed, jam_density, merge_priority=1, signal_group=0, capacity_out=None, capacity_in=None, eular_dx=None):
        """Create a link
        
        Parameters
        ----------
        W : object
            The network to which the link belongs.
        name : str
            The name of the link.
        start_node : str
            The name of the start node of the link.
        end_node : str
            The name of the end node of the link.
        length : float
            The length of the link.
        free_flow_speed : float
            The free flow speed on the link.
        jam_density : float
            The jam density on the link.
        merge_priority : int, optional
            The priority of the link when merging, default is 1.
        signal_group : int, optional
            The signal group to which the link belongs, default is 0.
        capacity_out : float, optional
            The capacity out of the link, default is calculated based on other parameters.
        capacity_in : float, optional
            The capacity into the link, default is calculated based on other parameters.
        eular_dx : float, optional
            The default space aggregation size for link traffic state computation, default is None. If None, the global eular_dx value is used.
        
        Notes
        -----
        The `capacity_out` and `capacity_in` parameters are used to set the capacities, and if not provided, they are calculated based on other parameters.
        Real-time link status for external reference is maintained with attributes `speed`, `density`, `flow`, `num_vehicles`, and `num_vehicles_queue`.
        """
        
        s.W = W
        #起点・終点ノード
        s.start_node = s.W.get_node(start_node)
        s.end_node = s.W.get_node(end_node)
        
        #リンク長
        s.length = length
        
        #フローモデルパラメータ
        s.u = free_flow_speed
        s.kappa = jam_density
        s.tau = s.W.REACTION_TIME
        s.w = 1/s.tau/s.kappa
        s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
        s.delta = 1/s.kappa
        
        #合流時優先度
        s.merge_priority = merge_priority
        
        #リンク内車両一覧
        s.vehicles = deque()
        
        #旅行時間
        s.traveltime_instant = []
        
        #経路選択補正
        s.route_choice_penalty = 0
        
        #累積図関係
        s.cum_arrival = []
        s.cum_departure = []
        s.traveltime_actual = []
        
        #信号関係
        s.signal_group = signal_group
        
        #流出容量
        s.capacity_out = capacity_out
        if capacity_out == None:
            s.capacity_out = s.capacity*2
            #todo: capacity_outは微妙にバグがあるらしい．少なくとも未設定時にはバグが顕在化しないように2倍にしている
        s.capacity_out_remain = s.capacity_out*s.W.DELTAT
        
        #流入容量
        s.capacity_in = capacity_in
        if capacity_in == None:
            s.capacity_in = s.capacity*2
            #todo: capacity_inは微妙にバグがあるらしい．少なくとも未設定時にはバグが顕在化しないように2倍にしている
        s.capacity_in_remain = s.capacity_in*s.W.DELTAT
        
        s.id = len(s.W.LINKS)
        s.name = name
        s.W.LINKS.append(s)
        s.start_node.outlinks[s.name] = s
        s.end_node.inlinks[s.name] = s
        
        #リアルタイムリンク状態（外部から参照する用）
        s._speed = -1 #リンク全体の平均速度
        s._density = -1
        s._flow = -1
        s._num_vehicles = -1 #車両数
        s._num_vehicles_queue = -1 #自由流速度未満の車両数
        
        #より正確な車両軌跡
        s.tss = []
        s.xss = []
        s.cs = []
        s.names = []
        
        if eular_dx == None:
            s.eular_dx = s.length/10
            if s.eular_dx < s.u*s.W.DELTAT:
                s.eular_dx = s.u*s.W.DELTAT
    
    def __repr__(s):
        return f"<Link {s.name}>"
    
    def init_after_tmax_fix(s):
        #TMAXを決めた後の初期化
        
        #Euler型交通状態
        s.edie_dt = s.W.EULAR_DT
        s.edie_dx = s.eular_dx
        s.k_mat = np.zeros([int(s.W.TMAX/s.edie_dt)+1, int(s.length/s.edie_dx)])
        s.q_mat = np.zeros(s.k_mat.shape)
        s.v_mat = np.zeros(s.k_mat.shape)
        s.tn_mat = np.zeros(s.k_mat.shape)
        s.dn_mat = np.zeros(s.k_mat.shape)
        s.an = s.edie_dt*s.edie_dx
    
    def update(s):
        #更新
        s.in_out_flow_constraint()
        
        s.set_traveltime_instant()
        s.cum_arrival.append(0)
        s.cum_departure.append(0)
        s.traveltime_actual.append(0)
        if len(s.cum_arrival) > 1:
            s.cum_arrival[-1] = s.cum_arrival[-2]
            s.cum_departure[-1] = s.cum_departure[-2]
            if len(s.vehicles):
                s.traveltime_actual[-1] = s.traveltime_actual[-2]
        
        #リアルタイム状態リセット
        s._speed = -1
        s._density = -1
        s._flow = -1
        s._num_vehicles = -1
        s._num_vehicles_queue = -1
    
    def in_out_flow_constraint(s):
        #リンク流入出率を流出容量以下にするための処理
        if s.capacity_out_remain < s.W.DELTAN:
            s.capacity_out_remain += s.capacity_out*s.W.DELTAT
        if s.capacity_in_remain < s.W.DELTAN:
            s.capacity_in_remain += s.capacity_in*s.W.DELTAT
    
    def set_traveltime_instant(s):
        #瞬間旅行時間算出
        if s.speed > 0:
            s.traveltime_instant.append(s.length/s.speed)
        else:
            s.traveltime_instant.append(s.length/(s.u/100))

    #getter/setter
    @property
    def speed(s):
        if s._speed == -1:
            if len(s.vehicles):
                s._speed = np.average([veh.v for veh in s.vehicles])
            else:
                s._speed = s.u
        return s._speed
    
    @property
    def density(s):
        if s._density == -1:
            s._density = s.num_vehicles/s.length
        return s._density
    
    @property
    def flow(s):
        if s._flow == -1:
            s._flow = s.density*s.speed
        return s._flow
    
    @property
    def num_vehicles(s):
        if s._num_vehicles == -1:
            s._num_vehicles = len(s.vehicles)*s.W.DELTAN
        return s._num_vehicles
    
    @property
    def num_vehicles_queue(s):
        if s._num_vehicles_queue == -1:
            s._num_vehicles_queue = sum([veh.v < s.u for veh in s.vehicles])*s.W.DELTAN
        return s._num_vehicles_queue

    @property
    def free_flow_speed(s):
        return s.u
    
    @free_flow_speed.setter
    def free_flow_speed(s, new_value):
        if new_value >= 0:
            s.u = new_value
            s.w = 1/s.tau/s.kappa
            s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
            s.delta = 1/s.kappa
        else:
            warnings.warn(f"ignored negative free_flow_speed at {s}", UserWarning)
    
    @property
    def jam_density(s):
        return s.kappa
    
    @jam_density.setter
    def jam_density(s, new_value):
        if new_value >= 0:
            s.kappa = new_value
            s.w = 1/s.tau/s.kappa
            s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
            s.delta = 1/s.kappa
        else:
            warnings.warn(f"ignored negative jam_density at {s}", UserWarning)
    

# 車両クラス
class Vehicle:
    def __init__(s, W, orig, dest, departure_time, name=None, route_pref=None, route_choice_principle=None, links_prefer=[], links_avoid=[], trip_abort=1, departure_time_is_time_step=0):
        """Create a vehicle (more precisely, platoon)
        
        Parameters
        ----------
        W : object
            The network to which the vehicle belongs.
        orig : str
            The origin node.
        dest : str
            The destination node.
        departure_time : int
            The departure time step of the vehicle.
        name : str, optional
            The name of the vehicle, default is the id of the vehicle.
        route_pref : dict, optional
            The preference weights for links, default is 0 for all links.
        route_choice_principle : str, optional
            The route choice principle of the vehicle, default is the network's route choice principle.
        links_prefer : list of str, optional
            The names of the links the vehicle prefers, default is empty list.
        links_avoid : list of str, optional
            The names of the links the vehicle avoids, default is empty list.
        trip_abort : int, optional
            Whether to abort the trip if a dead end is reached, default is 1.
        """
        
        s.W = W
        #出発・目的地ノード
        s.orig = s.W.get_node(orig)
        s.dest = s.W.get_node(dest)
        
        #出発・到着時刻
        if departure_time_is_time_step:#互換性のため，タイムステップ表記
            s.departure_time = departure_time
        else:
            s.departure_time = int(departure_time/s.W.DELTAT)
        s.arrival_time = -1
        s.link_arrival_time = -1
        s.travel_time = -1
        
        #状態：home, wait, run，end
        s.state = "home"
        
        #リンク内位置
        s.link = None
        s.x = 0
        s.x_next = 0
        s.x_old = 0
        s.v = 0
        
        #先行・後行車
        s.leader = None
        s.follower = None
        
        #リンク端部の走り残し処理
        s.move_remain = 0
        
        #経路選択
        if route_choice_principle == None:
            s.route_choice_principle = s.W.route_choice_principle
        else:
            s.route_choice_principle = route_choice_principle
        
        #希望リンク重み：{link:重み}
        s.route_pref = route_pref
        if s.route_pref == None:
            s.route_pref = {l:0 for l in s.W.LINKS}
        
        #好むリンクと避けるリンク（近視眼的）
        s.links_prefer = [get_link(l) for l in links_prefer]
        s.links_avoid = [get_link(l) for l in links_avoid]
        
        #行き止まりに行ってしまったときにトリップを止める
        s.trip_abort = trip_abort
        s.flag_trip_aborted = 0
        
        #ログなど
        s.log_t = [] #時刻
        s.log_state = [] #状態
        s.log_link = [] #リンク
        s.log_x = [] #位置
        s.log_s = [] #車頭距離
        s.log_v = [] #現在速度
        s.color = (random.random(), random.random(), random.random())
        
        s.id = len(s.W.VEHICLES)
        if name != None:
            s.name = name
        else:
            s.name = str(s.id)
        s.W.VEHICLES[s.name] = s
        s.W.VEHICLES_LIVING[s.name] = s
    
    def __repr__(s):
        return f"<Vehicle {s.name}: {s.state}, x={s.x}, link={s.link}>"

    def update(s):
        #更新
        s.record_log()
        
        if s.state == "home":
            #需要生成
            if s.W.T >= s.departure_time:
                s.state = "wait"
                s.orig.generation_queue.append(s)
        if s.state == "wait":
            #出発ノードで待つ
            pass
        if s.state == "run":
            #走行
            s.v = (s.x_next-s.x)/s.W.DELTAT
            s.x_old = s.x
            s.x = s.x_next
            
            #リンク下流端
            if s.x == s.link.length:
                if s.link.end_node == s.dest:
                    #トリップ終了
                    s.end_trip()
                elif len(s.link.end_node.outlinks.values()) == 0 and s.trip_abort == 1:
                    s.flag_trip_aborted = 1
                    s.route_next_link = None
                    s.end_trip()
                else:
                    #リンク間遷移リクエスト
                    s.route_next_link_choice()
                    s.link.end_node.incoming_vehicles.append(s)
        if s.state in ["end", "abort"] :
            #終わり
            pass
    
    def end_trip(s):
        #トリップ終了処理
        s.state = "end"
        
        s.link.cum_departure[-1] += s.W.DELTAN
        s.link.traveltime_actual[-1] = (s.W.T+1)*s.W.DELTAT - s.link_arrival_time  #端部の挙動改善
        
        if s.follower != None:
            s.follower.leader = None
        
        s.link.vehicles.popleft()
        s.link = None
        s.x = 0
        s.arrival_time = s.W.T
        s.travel_time = (s.arrival_time - s.departure_time)*s.W.DELTAT
        s.W.VEHICLES_RUNNING.pop(s.name)
        s.W.VEHICLES_LIVING.pop(s.name)
        
        if s.flag_trip_aborted:
            s.state = "abort"
            s.arrival_time = -1
            s.travel_time = -1
        
        s.record_log()
    
    def carfollow(s):
        #リンク内走行
        s.x_next = s.x + s.link.u*s.W.DELTAT
        if s.leader != None:
            x_cong = s.leader.x - s.link.delta*s.W.DELTAN
            if x_cong < s.x:
                x_cong = s.x
            if s.x_next > x_cong:
                s.x_next = x_cong
        
        if s.x_next > s.link.length:
            s.move_remain = s.x_next - s.link.length
            s.x_next = s.link.length
    
    def route_pref_update(s, weight):
        #経路選択のためのリンク選好を更新
        if s.route_choice_principle == "homogeneous_DUO":
            s.route_pref = s.W.ROUTECHOICE.route_pref[s.dest.id]
        elif s.route_choice_principle == "heterogeneous_DUO":
            route_pref_new = {l:0 for l in s.W.LINKS}
            k = s.dest.id
            for l in s.W.LINKS:
                i = l.start_node.id
                j = l.end_node.id
                if j == s.W.ROUTECHOICE.next[i,k]:
                    route_pref_new[l] = 1
            
            if sum(list(s.route_pref.values())) == 0:
                #最初にpreferenceが空なら確定的に初期化
                weight = 1
            for l in s.route_pref.keys():
                s.route_pref[l] = (1-weight)*s.route_pref[l] + weight*route_pref_new[l]
    
    def route_next_link_choice(s):
        #現在のリンクから次に移るリンクを選択
        if s.dest != s.link.end_node:
            outlinks = list(s.link.end_node.outlinks.values())
            
            if len(outlinks):
                if set(outlinks) & set(s.links_prefer):
                    outlinks = list(set(outlinks) & set(s.links_prefer))
                if set(outlinks) & set(s.links_avoid):
                    outlinks = list(set(outlinks) - set(s.links_avoid))
                
                preference = [s.route_pref[l] for l in outlinks]
                
                if sum(preference) > 0:
                    s.route_next_link = random.choices(outlinks, preference)[0]
                else:
                    s.route_next_link = random.choices(outlinks)[0]
            else:
                s.route_next_link = None
    
    def record_log(s):
        #走行履歴保存
        if s.state != "run":
            s.log_t.append(s.W.T*s.W.DELTAT)
            s.log_state.append(s.state)
            s.log_link.append(-1)
            s.log_x.append(-1)
            s.log_s.append(-1)
            s.log_v.append(-1)
            
            if s.state == "wait":
                s.W.analyzer.average_speed_count += 1
                s.W.analyzer.average_speed += 0
        else:
            s.log_t.append(s.W.T*s.W.DELTAT)
            s.log_state.append(s.state)
            s.log_link.append(s.link)
            s.log_x.append(s.x)
            s.log_v.append(s.v)
            if s.leader != None and s.link == s.leader.link:
                s.log_s.append(s.leader.x-s.x)
            else:
                s.log_s.append(-1)
            
            s.W.analyzer.average_speed_count += 1
            s.W.analyzer.average_speed += (s.v - s.W.analyzer.average_speed)/s.W.analyzer.average_speed_count

# 経路選択クラス
class RouteChoice:
    """class for computing shortest path for all vehicles"""
    
    def __init__(s, W):
        s.W = W
        #リンク旅行時間行列
        s.adj_mat_time = np.zeros([len(s.W.NODES), len(s.W.NODES)]) 
        #ij間最短距離
        s.dist = np.zeros([len(s.W.NODES), len(s.W.NODES)]) 
        #iからjに行くために次に進むべきノード
        s.next = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        #iからjに行くために来たノード
        s.pred = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        
        #homogeneous DUO用．kに行くための最短経路的上にあれば1
        s.route_pref = {k.id: {l:0 for l in s.W.LINKS} for k in s.W.NODES}
    
    def route_search_all(s, infty=np.inf, noise=0):
        #現時刻の最短経路を計算．scipy版
        for link in s.W.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if s.W.ADJ_MAT[i,j]:
                s.adj_mat_time[i,j] = link.traveltime_instant[-1]*random.uniform(1, 1+noise) + link.route_choice_penalty
                if link.capacity_in == 0: #流入禁止の場合は通行不可
                    s.adj_mat_time[i,j] = np.inf
            else:
                s.adj_mat_time[i,j] = np.inf
        
        s.dist, s.pred = floyd_warshall(s.adj_mat_time, return_predecessors=True)
        
        n_vertices = s.pred.shape[0]
        s.next = -np.ones((n_vertices, n_vertices), dtype=int)
        for i in range(n_vertices):
            for j in range(n_vertices):
                # iからjへの最短経路を逆にたどる．．．
                if i != j:
                    prev = j
                    while s.pred[i, prev] != i and s.pred[i, prev] != -9999:
                        prev = s.pred[i, prev]
                    s.next[i, j] = prev
    
    def route_search_all_old(s, infty=9999999999999999999, noise=0):
        #現時刻の最短経路を計算．自家版．さすがに大規模ネットワークでは少し遅い
        for link in s.W.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if s.W.ADJ_MAT[i,j]:
                s.adj_mat_time[i,j] = link.traveltime_instant[-1]*random.uniform(1, 1+noise) + link.route_choice_penalty
                if link.capacity_in == 0: #流入禁止の場合は通行不可
                    s.adj_mat_time[i,j] = 0
            else:
                s.adj_mat_time[i,j] = 0
        
        dist = np.zeros([len(s.W.NODES), len(s.W.NODES)]) 
        next = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        for i in range(len(s.W.NODES)):
            for j in range(len(s.W.NODES)):
                if s.adj_mat_time[i,j] > 0:
                    dist[i,j] = s.adj_mat_time[i,j]
                    next[i,j] = j
                elif i == j:
                    next[i,j] = j
                else:
                    dist[i,j] = infty
                    next[i,j] = -1
        
        for k in range(len(s.W.NODES)):
            for i in range(len(s.W.NODES)):
                for j in range(len(s.W.NODES)):
                    if dist[i,j] > dist[i,k]+dist[k,j]:
                        dist[i,j] = dist[i,k]+dist[k,j]
                        next[i,j] = next[i,k]
        s.dist = dist
        s.next = next
    
    def homogeneous_DUO_update_old(s):
        #全員が同一の基準に従うDUOの場合，一括で計算する
        for dest in s.W.NODES:
            k = dest.id
            route_pref_new = {l:0 for l in s.W.LINKS}
            for l in s.W.LINKS:
                i = l.start_node.id
                j = l.end_node.id
                if j == s.W.ROUTECHOICE.next[i,k]:
                    route_pref_new[l] = 1
            
            weight = s.W.DUO_UPDATE_WEIGHT
            if sum(list(s.route_pref[k].values())) == 0:
                #最初にpreferenceが空なら確定的に初期化
                weight = 1
            for l in s.route_pref[k].keys():
                s.route_pref[k][l] = (1-weight)*s.route_pref[k][l] + weight*route_pref_new[l]

    
    def homogeneous_DUO_update(s):
        #全員が同一の基準に従うDUOの場合，一括で計算する
        for dest in s.W.NODES:
            k = dest.id
            weight = s.W.DUO_UPDATE_WEIGHT
            if sum(list(s.route_pref[k].values())) == 0:
                #最初にpreferenceが空なら確定的に初期化
                weight = 1
            for l in s.W.LINKS:
                i = l.start_node.id
                j = l.end_node.id
                if j == s.W.ROUTECHOICE.next[i,k]:
                    s.route_pref[k][l] = (1-weight)*s.route_pref[k][l] + weight
                else:
                    s.route_pref[k][l] = (1-weight)*s.route_pref[k][l]

# 結果分析クラス
class Analyzer:
    """class for analyzing and visualizing a simulation result"""
    
    def __init__(s, W):
        s.W = W
        
        os.makedirs(f"out{s.W.name}", exist_ok=True)
        
        #基礎統計量
        s.average_speed = 0
        s.average_speed_count = 0
        s.trip_completed = 0
        s.trip_all = 0
        s.total_travel_time = 0
        s.average_travel_time = 0
        
        #フラグ
        s.flag_edie_state_computed = 0
        s.flag_trajectory_computed = 0
        s.flag_pandas_convert = 0
        s.flag_od_analysis = 0
    
    def basic_analysis(s):
        #基礎統計量の計算
        df = s.W.analyzer.od_to_pandas()
        
        s.trip_completed = np.sum(df["completed_trips"])
        s.trip_all = np.sum(df["total_trips"])
        
        if s.trip_completed:
            s.total_travel_time = np.sum(df["completed_trips"]*df["average_travel_time"])
            s.average_travel_time = s.total_travel_time/s.trip_completed
            s.total_delay = np.sum(df["completed_trips"]*(df["average_travel_time"]-df["free_travel_time"]))
            s.average_delay = s.total_delay/s.trip_completed
        else:
            s.total_travel_time = -1
            s.average_travel_time = -1
            s.total_delay = -1
            s.average_delay = -1

    
    def od_analysis(s):
        #OD別の分析
        #トリップ数，トリップ完了数，自由旅行時間，平均旅行時間，標準偏差
        if s.flag_od_analysis:
            return 0
        else:
            s.flag_od_analysis = 1
        
        s.od_trips = ddict(lambda: 0)
        s.od_trips_comp = ddict(lambda: 0)
        s.od_tt_free = ddict(lambda: 0)
        s.od_tt = ddict(lambda: [])
        s.od_tt_ave = ddict(lambda: 0)
        s.od_tt_std = ddict(lambda: 0)
        dn = s.W.DELTAN
        
        #自由旅行時間
        adj_mat_time = np.zeros([len(s.W.NODES), len(s.W.NODES)]) 
        for link in s.W.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if s.W.ADJ_MAT[i,j]:
                adj_mat_time[i,j] = link.length/link.u
                if link.capacity_in == 0: #流入禁止の場合は通行不可
                    adj_mat_time[i,j] = np.inf
            else:
                adj_mat_time[i,j] = np.inf
        dist = floyd_warshall(adj_mat_time)
        
        for veh in s.W.VEHICLES.values():
            o = veh.orig
            d = veh.dest
            s.od_trips[o,d] += dn
            if veh.travel_time != -1:
                s.od_trips_comp[o,d] += dn
                s.od_tt[o,d].append(veh.travel_time)
        for o,d in s.od_tt.keys():
            s.od_tt_ave[o,d] = np.average(s.od_tt[o,d])
            s.od_tt_std[o,d] = np.std(s.od_tt[o,d])
            s.od_tt_free[o,d] = dist[o.id, d.id]
    
    def link_analysis_coarse(s):
        #リンクレベルの粗い分析
        #交通量，取り残し車両数，自由旅行時間，平均旅行時間，標準偏差
        s.linkc_volume = ddict(lambda:0)
        s.linkc_tt_free = ddict(lambda:0)
        s.linkc_tt_ave = ddict(lambda:-1)
        s.linkc_tt_std = ddict(lambda:-1)
        s.linkc_remain = ddict(lambda:0)
        
        for l in s.W.LINKS:
            s.linkc_volume[l] = l.cum_departure[-1]
            s.linkc_remain[l] = l.cum_arrival[-1]-l.cum_departure[-1]
            s.linkc_tt_free[l] = l.length/l.u
            if s.linkc_volume[l]:
                s.linkc_tt_ave[l] = np.average([t for t in l.traveltime_actual if t>0])
                s.linkc_tt_std[l] = np.std([t for t in l.traveltime_actual if t>0])
    
    def compute_accurate_traj(s):
        #リンク端部の車両軌跡を補ったより正確な軌跡を生成する．端部は自由流走行と仮定
        if s.flag_trajectory_computed:
            return 0
        else:
            s.flag_trajectory_computed = 1
        
        for veh in s.W.VEHICLES.values():
            l_old = None
            for i in lange(veh.log_t):
                if veh.log_link[i] != -1:
                    l = s.W.get_link(veh.log_link[i])
                    if l_old != l:
                        l.tss.append([])
                        l.xss.append([])
                        l.cs.append(veh.color)
                        l.names.append(veh.name)

                    l_old = l
                    l.tss[-1].append(veh.log_t[i])
                    l.xss[-1].append(veh.log_x[i])
                    
        for l in s.W.LINKS:
            #端部を外挿
            for i in lange(l.xss):
                if len(l.xss[i]):
                    if l.xss[i][0] != 0:
                        x_remain = l.xss[i][0]
                        l.xss[i].insert(0, 0)
                        l.tss[i].insert(0, l.tss[i][0]-x_remain/l.u)
                    if l.length-l.u*s.W.DELTAT <= l.xss[i][-1] < l.length:
                        x_remain = l.length-l.xss[i][-1]
                        l.xss[i].append(l.length)
                        l.tss[i].append(l.tss[i][-1]+x_remain/l.u)

    def compute_edie_state(s):
        #Euler型交通状態計算．精緻版
        if s.flag_edie_state_computed:
            return 0
        else:
            s.flag_edie_state_computed = 1
        
        s.compute_accurate_traj()
        for l in s.W.LINKS:
            DELTAX = l.edie_dx
            DELTATE = l.edie_dt
            MAXX = l.length
            MAXT = s.W.TMAX
            
            dt = DELTATE
            dx = DELTAX
            tn = [[ddict(lambda: 0) for i in range(int(MAXX/DELTAX))] for j in range(int(MAXT/DELTATE))]
            dn = [[ddict(lambda: 0) for i in range(int(MAXX/DELTAX))] for j in range(int(MAXT/DELTATE))]

            l.k_mat = np.zeros([int(MAXT/DELTATE), int(MAXX/DELTAX)])
            l.q_mat = np.zeros([int(MAXT/DELTATE), int(MAXX/DELTAX)])
            l.v_mat = np.zeros([int(MAXT/DELTATE), int(MAXX/DELTAX)])

            for v in lange(l.xss):
                for i in lange(l.xss[v][:-1]):
                    i0 = l.names[v]
                    x0 = l.xss[v][i]
                    x1 = l.xss[v][i+1]
                    t0 = l.tss[v][i]
                    t1 = l.tss[v][i+1]
                    if t1-t0 != 0:
                        v0 = (x1-x0)/(t1-t0)
                    else:
                        #todo: why?
                        v0 = 0

                    tt = int(t0//dt)
                    xx = int(x0//dx)

                    if v0 > 0:
                        #残り
                        xl0 = dx-x0%dx
                        xl1 = x1%dx
                        tl0 = xl0/v0
                        tl1 = xl1/v0

                        if tt < int(MAXT/DELTATE) and xx < int(MAXX/DELTAX):
                            if xx == x1//dx:
                                #(x,t)
                                dn[tt][xx][i0] += x1-x0
                                tn[tt][xx][i0] += t1-t0
                            else:
                                #(x+n, t)
                                jj = int(x1//dx-xx+1)
                                for j in range(jj):
                                    if xx+j < int(MAXX/DELTAX):
                                        if j == 0:
                                            dn[tt][xx+j][i0] += xl0
                                            tn[tt][xx+j][i0] += tl0
                                        elif j == jj-1:
                                            dn[tt][xx+j][i0] += xl1
                                            tn[tt][xx+j][i0] += tl1
                                        else:
                                            dn[tt][xx+j][i0] += dx
                                            tn[tt][xx+j][i0] += dx/v0
                    else:
                        if tt < int(MAXT/DELTATE) and xx < int(MAXX/DELTAX):
                            dn[tt][xx][i0] += 0
                            tn[tt][xx][i0] += t1-t0
            
            for i in lange(tn):
                for j in lange(tn[i]):
                    t = list(tn[i][j].values())*s.W.DELTAN
                    d = list(dn[i][j].values())*s.W.DELTAN
                    l.tn_mat[i,j] = sum(t)
                    l.dn_mat[i,j] = sum(d)
                    l.k_mat[i,j] = l.tn_mat[i,j]/DELTATE/DELTAX
                    l.q_mat[i,j] = l.dn_mat[i,j]/DELTATE/DELTAX
            with np.errstate(invalid="ignore"):
                l.v_mat = l.q_mat/l.k_mat
            l.v_mat = np.nan_to_num(l.v_mat, nan=l.u)
    
    @catch_exceptions_and_warn()
    def print_simple_stats(s):
        #簡単な結果表示
        
        s.W.print("results:")
        s.W.print(f" average speed:\t {s.average_speed:.1f} m/s")
        s.W.print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
        if s.trip_completed > 0:
            s.W.print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
            s.W.print(f" average delay of trips:\t {s.average_delay:.1f} s")
            s.W.print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")
    
    @catch_exceptions_and_warn()
    def time_space_diagram_traj(s, links=None, figsize=(12,4)):
        """Draws the time-space diagram of vehicle trajectories for vehicles on specified links.

        Parameters
        ----------
        links : list of link or link, optional
            The names of the links for which the time-space diagram is to be plotted. 
            If None, the diagram is plotted for all links in the network. Default is None.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        """
        
        #リンク車両軌跡の時空間図
        s.W.print(" drawing trajectories...")
        s.compute_accurate_traj()
        
        #対象がlistであればOKで，単一な場合にはlistに変換する．未指定であれば全部にする．
        if links == None:
            links = s.W.LINKS
        try:
            iter(links)
            if type(links) == str:
                links = [links]
        except TypeError:
            links = [links]
        
        for lll in tqdm(links, disable=(s.W.print_mode==0)):
            l = s.W.get_link(lll)
            
            plt.figure(figsize=figsize)
            plt.title(l)
            for i in range(len(l.xss)):
                plt.plot(l.tss[i], l.xss[i], c=l.cs[i], lw=0.5)
            plt.xlabel("time (s)")
            plt.ylabel("space (m)")
            plt.xlim([0, s.W.TMAX])
            plt.ylim([0, l.length])
            plt.grid()
            plt.tight_layout()
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/tsd_traj_{l.name}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")
    
    @catch_exceptions_and_warn()
    def time_space_diagram_density(s, links=None, figsize=(12,4)):
        """Draws the time-space diagram of traffic density on specified links.

        Parameters
        ----------
        links : list of link or link, optional
            The names of the links for which the time-space diagram is to be plotted. 
            If None, the diagram is plotted for all links in the network. Default is None.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        """
        
        #リンク密度の時空間図
        s.W.print(" drawing traffic states...")
        s.compute_edie_state()
        
        #対象がlistであればOKで，単一な場合にはlistに変換する．未指定であれば全部にする．
        if links == None:
            links = s.W.LINKS
        try:
            iter(links)
            if type(links) == str:
                links = [links]
        except TypeError:
            links = [links]
        
        s.compute_edie_state()
        
        for lll in tqdm(links, disable=(s.W.print_mode==0)):
            l = s.W.get_link(lll)
            
            plt.figure(figsize=figsize)
            plt.title(l)
            plt.imshow(l.k_mat.T, origin="lower", aspect="auto", 
                extent=(0, int(s.W.TMAX/l.edie_dt)*l.edie_dt, 0, int(l.length/l.edie_dx)*l.edie_dx), 
                interpolation="none", vmin=0, vmax=1/l.delta, cmap="inferno")
            plt.colorbar().set_label("density (veh/m)")
            plt.xlabel("time (s)")
            plt.ylabel("space (m)")
            plt.xlim([0, s.W.TMAX])
            plt.ylim([0, l.length])
            plt.tight_layout()
            
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/tsd_k_{l.name}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")
    
    @catch_exceptions_and_warn()
    def time_space_diagram_traj_links(s, linkslist, figsize=(12,4)):
        """Draws the time-space diagram of vehicle trajectories for vehicles on concective links.

        Parameters
        ----------
        linkslist : list of link or list of list of link
            The names of the concective links for which the time-space diagram is to be plotted. 
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        """
        #複数リンクの連続した車両軌跡の時空間図
        s.W.print(" drawing trajectories in consecutive links...")
        s.compute_accurate_traj()
        
        #リンクリストのリストであればそのまま，そうでなければリスト化
        try:
            iter(linkslist[0])
            if type(linkslist[0]) == str:
                linkslist = [linkslist]
        except TypeError:
            linkslist = [linkslist]
        
        for links in linkslist:
            linkdict = {}
            d = 0
            for ll in links:
                l = s.W.get_link(ll)
                linkdict[l] = d
                d += l.length
                
            plt.figure(figsize=figsize)
            for ll in links:
                l = s.W.get_link(ll)
                for i in range(len(l.xss)):
                    plt.plot(l.tss[i], np.array(l.xss[i])+linkdict[l], c=l.cs[i], lw=0.5)
            for l in linkdict.keys():
                plt.plot([0, s.W.TMAX], [linkdict[l], linkdict[l]], "k--", lw=0.7)
                plt.plot([0, s.W.TMAX], [linkdict[l]+l.length, linkdict[l]+l.length], "k--", lw=0.7)
                plt.text(0, linkdict[l]+l.length/2, l.name, va="center", c="b")
                plt.text(0, linkdict[l], l.start_node.name, va="center", c="g")
                plt.text(0, linkdict[l]+l.length, l.end_node.name, va="center", c="g")
            plt.xlabel("time (s)")
            plt.ylabel("space (m)")
            plt.xlim([0, s.W.TMAX])
            plt.grid()
            plt.tight_layout()
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/tsd_traj_links_{'-'.join([s.W.get_link(l).name for l in links])}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")
    
    @catch_exceptions_and_warn()
    def cumulative_curves(s, links=None, figsize=(6,4)):
        #累積図と旅行時間
        
        #対象がlistであればOKで，単一な場合にはlistに変換する．未指定であれば全部にする．
        if links == None:
            links = s.W.LINKS
        try:
            iter(links)
            if type(links) == str:
                flag_single = 1
        except TypeError:
            links = [links]
        
        for link in links:
            fig, ax1 = plt.subplots(figsize=figsize)
            plt.title(link)
            ax2 = ax1.twinx()

            ax1.plot(range(0, s.W.TMAX, s.W.DELTAT), link.cum_arrival, "r-", label="arrival")
            ax1.plot(range(0, s.W.TMAX, s.W.DELTAT), link.cum_departure, "b-", label="departure")

            ax2.plot(range(0, s.W.TMAX, s.W.DELTAT), link.traveltime_instant, ".", c="gray", ms=0.5, label="instantaneous")
            ax2.plot(range(0, s.W.TMAX, s.W.DELTAT), link.traveltime_actual, "k.", ms=1, label="actual")

            ax1.set_ylim(ymin=0)
            ax2.set_ylim(ymin=0)

            ax1.set_xlabel("time(s)")
            ax1.set_ylabel("cumlative count (veh)")
            ax2.set_ylabel("travel time (s)")

            ax1.legend(loc="upper left")
            ax2.legend(loc="upper right")

            ax1.grid()
            plt.tight_layout()
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/cumlative_curves_{link.name}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")
    
    @catch_exceptions_and_warn()
    def network(s, t=None, detailed=1, minwidth=0.5, maxwidth=12, left_handed=1, tmp_anim=0, figsize=(6,6), network_font_size=4, node_size=2):
        #ネットワーク全体の交通状況
        #detailed=1の時，リンク内部を詳細に描画，0の時簡略化
        s.compute_edie_state()
        
        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")
        plt.title(f"t = {t :>8} (s)")
        for n in s.W.NODES:
            plt.plot(n.x, n.y, "ko", ms=node_size, zorder=10)
            if network_font_size > 0:
                plt.text(n.x, n.y, n.name, c="g", horizontalalignment="center", verticalalignment="top", zorder=20, fontsize=network_font_size)
        for l in s.W.LINKS:
            x1, y1 = l.start_node.x, l.start_node.y
            x2, y2 = l.end_node.x, l.end_node.y
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            if detailed:
                #詳細モード
                xsize = l.k_mat.shape[1]-1
                c = ["k" for i in range(xsize)]
                lw = [1 for i in range(xsize)]
                for i in range(xsize):
                    k = l.k_mat[int(t/l.edie_dt), i+1]
                    v = l.v_mat[int(t/l.edie_dt), i+1]
                    lw[i] = k*l.delta*(maxwidth-minwidth)+minwidth
                    c[i] = plt.colormaps["viridis"](v/l.u)
                xmid = [((xsize-i)*x1+(i+1)*x2)/(xsize+1)+vx for i in range(xsize)]
                ymid = [((xsize-i)*y1+(i+1)*y2)/(xsize+1)+vy for i in range(xsize)]
                plt.plot([x1]+xmid+[x2], [y1]+ymid+[y2], "k--", lw=0.25, zorder=5)
                for i in range(xsize-1):
                    plt.plot([xmid[i], xmid[i+1]], [ymid[i], ymid[i+1]], c=c[i], lw=lw[i], zorder=6)
                if network_font_size > 0:
                    plt.text(xmid[int(len(xmid)/2)], ymid[int(len(xmid)/2)], l.name, c="b", zorder=20, fontsize=network_font_size)
            else:
                #簡略モード
                k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
                v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
                width = k*l.delta*(maxwidth-minwidth)+minwidth
                c = plt.colormaps["viridis"](v/l.u)
                xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
                xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
                plt.plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], "k--", lw=0.25, zorder=5)
                plt.plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], c=c, lw=width, zorder=6, solid_capstyle="butt")
                if network_font_size > 0:
                    plt.text(xmid1, ymid1, l.name, c="b", zorder=20, fontsize=network_font_size)
        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        maxy = max([n.y for n in s.W.NODES])
        miny = min([n.y for n in s.W.NODES])
        buffx, buffy = (maxx-minx)/10, (maxy-miny)/10
        if buffx == 0:
            buffx = buffy
        if buffy == 0:
            buffy = buffx
        plt.xlim([minx-buffx, maxx+buffx])
        plt.ylim([miny-buffy, maxy+buffy])
        plt.tight_layout()
        if tmp_anim:
            plt.savefig(f"out{s.W.name}/tmp_anim_{t}.png")
            plt.close("all")
        else:
            if s.W.save_mode:
                plt.savefig(f"out{s.W.name}/network{detailed}_{t}.png")
            if s.W.show_mode:
                plt.show()
            else:
                plt.close("all")
    
    @catch_exceptions_and_warn()
    def network_pillow(s, t=None, detailed=1, minwidth=0.5, maxwidth=12, left_handed=1, tmp_anim=0, figsize=6, network_font_size=20, node_size=2, image_return=0):
        #ネットワーク全体の交通状況．pillowを使った高速な可視化

        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        maxy = max([n.y for n in s.W.NODES])
        miny = min([n.y for n in s.W.NODES])

        scale = 2
        try:
            coef = figsize*100*scale/(maxx-minx)
        except:
            coef = figsize[0]*100*scale/(maxx-minx)
        maxx *= coef
        minx *= coef
        maxy *= coef
        miny *= coef
        minwidth *= scale
        maxwidth *= scale

        buffer = (maxx-minx)/10
        maxx += buffer
        minx -= buffer
        maxy += buffer
        miny -= buffer

        img = Image.new("RGBA", (int(maxx-minx), int(maxy-miny)), (255, 255, 255, 255))
        draw = ImageDraw.Draw(img)
        font = ImageFont.truetype("utils/Inconsolata.otf", int(network_font_size))

        def flip(y):
            return img.size[1]-y

        for l in s.W.LINKS:
            x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
            x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            k = (l.cum_arrival[int(t/s.W.DELTAT)]-l.cum_departure[int(t/s.W.DELTAT)])/l.length
            v = l.length/l.traveltime_instant[int(t/s.W.DELTAT)]
            width = k*l.delta*(maxwidth-minwidth)+minwidth
            c = plt.colormaps["viridis"](v/l.u)
            xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
            xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
            draw.line([(x1, flip(y1)), (xmid1, flip(ymid1)), (xmid2, flip(ymid2)), (x2, flip(y2))], fill=(int(c[0]*255), int(c[1]*255), int(c[2]*255)), width=int(width), joint="curve")
            
            if network_font_size > 0:
                draw.text((xmid1, flip(ymid1)), l.name, font=font, fill="blue", anchor="mm")

        for n in s.W.NODES:
            if network_font_size > 0:
                draw.text(((n.x)*coef-minx, flip((n.y)*coef-miny)), n.name, font=font, fill="green", anchor="mm")
                draw.text(((n.x)*coef-minx, flip((n.y)*coef-miny)), n.name, font=font, fill="green", anchor="mm")

        font = ImageFont.truetype("utils/Inconsolata.otf", int(30))       
        draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

        img = img.resize((int((maxx-minx)/scale), int((maxy-miny)/scale)), resample=Resampling.LANCZOS)    
        if image_return:
            return img
        elif tmp_anim:
            img.save(f"out{s.W.name}/tmp_anim_{t}.png")
        else:
            if s.W.save_mode:
                img.save(f"out{s.W.name}/network{detailed}_{t}.png")
    
    @catch_exceptions_and_warn()
    def show_simulation_progress(s):
        #シミュレーション途中経過を表示する
        
        vehs = [l.density*l.length for l in s.W.LINKS]
        sum_vehs = sum(vehs)
        
        vs = [l.density*l.length*l.speed for l in s.W.LINKS]
        if sum_vehs > 0:
            avev = sum(vs)/sum_vehs
        else:
            avev = 0
        
        print(f"{s.W.TIME:>8.0f} s| {sum_vehs:>8.0f} vehs|  {avev:>4.1f} m/s | {time.time()-s.W.sim_start_time:8.2f} s", flush=True)
        
    @catch_exceptions_and_warn()
    def network_anim(s, animation_speed_inverse=10, detailed=0, minwidth=0.5, maxwidth=12, left_handed=1, figsize=(6,6), node_size=2, network_font_size=20):
        #ネットワーク全体の交通状況のアニメーション
        if s.W.save_mode:
            s.W.print(" generating animation...")
            pics = []
            speed_coef = 8
            for t in tqdm(range(0,s.W.TMAX,s.W.DELTAT*speed_coef), disable=(s.W.print_mode==0)):
                if detailed:
                    #todo: 今後はこちらもpillowにする
                    s.network(int(t), detailed=detailed, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size)
                else:
                    s.network_pillow(int(t), detailed=detailed, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size)
                pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))
            pics[0].save(f"out{s.W.name}/anim_network{detailed}.gif", save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*speed_coef, loop=0)
            for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
                os.remove(f)
    
    @catch_exceptions_and_warn()
    def network_fancy(s, animation_speed_inverse=10, figsize=6, sample_ratio=0.3, interval=5, network_font_size=0, trace_length=3, speed_coef=2):
        #ネットワーク全体の車両軌跡をいい感じにアニメーション
        s.W.print(" generating animation...")
        
        # ベジエ補間
        from scipy.interpolate import make_interp_spline

        #{t: ["xs":[], "ys":[], "v": v, "c":c]}
        draw_dict = ddict(lambda: [])

        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        dcoef = (maxx-minx)/20
        
        for veh in s.W.VEHICLES.values():
            if random.random() > sample_ratio:
                continue

            ts = []
            xs = []
            ys = []
            vs = []
            dx = (random.random()-0.5)*dcoef
            dy = (random.random()-0.5)*dcoef
            for i in range(0, len(veh.log_t), interval):
                if veh.log_state[i] == "run":
                    link = veh.log_link[i]
                    x0 = link.start_node.x+dx
                    y0 = link.start_node.y+dy
                    x1 = link.end_node.x+dx
                    y1 = link.end_node.y+dy
                    alpha = veh.log_x[i]/link.length
                    ts.append(veh.log_t[i])
                    xs.append(x0*(1-alpha)+x1*alpha)
                    ys.append(y0*(1-alpha)+y1*alpha)
                    c = veh.color
            for i in range(0, len(veh.log_t)):
                if veh.log_state[i] == "run":
                    vs.append(veh.log_v[i]/veh.log_link[i].u)
            if len(ts) <= interval:
                continue

            # 点列
            points = np.array([xs, ys]).T

            # x, y 座標を取得
            x = points[:, 0]
            y = points[:, 1]

            # ベジエ曲線による補間
            t = np.linspace(0, 1, len(points))
            interp_size = len(ts)*interval
            t_smooth = np.linspace(0, 1, interp_size)
            bezier_spline = make_interp_spline(t, points, k=3)
            smooth_points = bezier_spline(t_smooth)
            for i in lange(t_smooth):
                ii = max(0, i-trace_length)
                if i < len(vs):
                    v = vs[i]
                else:
                    v = vs[-1]
                draw_dict[int(ts[0]+i*s.W.DELTAT)].append({
                    "xs": smooth_points[ii:i+1, 0], 
                    "ys": smooth_points[ii:i+1, 1],
                    "c": veh.color,
                    "v": v
                })

        # 可視化
        maxx = max([n.x for n in s.W.NODES])
        minx = min([n.x for n in s.W.NODES])
        maxy = max([n.y for n in s.W.NODES])
        miny = min([n.y for n in s.W.NODES])

        scale = 2
        try:
            coef = figsize*100*scale/(maxx-minx)
        except:
            coef = figsize[0]*100*scale/(maxx-minx)
        maxx *= coef
        minx *= coef
        maxy *= coef
        miny *= coef

        buffer = (maxx-minx)/10
        maxx += buffer
        minx -= buffer
        maxy += buffer
        miny -= buffer

        pics = []
        for t in tqdm(range(int(s.W.TMAX*0), int(s.W.TMAX*1), s.W.DELTAT*speed_coef)):
            img = Image.new("RGBA", (int(maxx-minx), int(maxy-miny)), (255, 255, 255, 255))
            draw = ImageDraw.Draw(img)
            font = ImageFont.truetype("utils/Inconsolata.otf", int(network_font_size))

            def flip(y):
                return img.size[1]-y

            for l in s.W.LINKS:
                x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
                x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
                draw.line([(x1, flip(y1)), (x2, flip(y2))], fill=(200,200,200), width=int(1), joint="curve")

                if network_font_size > 0:
                    draw.text((xmid1, flip(ymid1)), l.name, font=font, fill="blue", anchor="mm")

            traces = draw_dict[t]
            for trace in traces:
                xs = trace["xs"]*coef-minx
                ys = trace["ys"]*coef-miny
                size = 3*(1-trace["v"])
                coords = [(l[0], flip(l[1])) for l in list(np.vstack([xs, ys]).T)]
                draw.line(coords,
                          fill=(int(trace["c"][0]*255), int(trace["c"][1]*255), int(trace["c"][2]*255)), width=2, joint="curve")
                draw.ellipse((xs[-1]-size, flip(ys[-1])-size, xs[-1]+size, flip(ys[-1])+size), fill=(int(trace["c"][0]*255), int(trace["c"][1]*255), int(trace["c"][2]*255)))
                #draw.line([(x1, flip(y1)), (xmid1, flip(ymid1)), (xmid2, flip(ymid2)), (x2, flip(y2))]

            font = ImageFont.truetype("utils/Inconsolata.otf", int(30))       
            draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

            img = img.resize((int((maxx-minx)/scale), int((maxy-miny)/scale)), resample=Resampling.LANCZOS)
            img.save(f"out{s.W.name}/tmp_anim_{t}.png")
            pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))

        pics[0].save(f"out{s.W.name}/anim_network_fancy.gif", save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*speed_coef, loop=0)
        
        for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
            os.remove(f)
    
    def compute_mfd(s, links=None):
        s.compute_edie_state()
        if links == None:
            links = s.W.LINKS
        
        for i in range(len(s.W.Q_AREA)):
            tn = sum([l.tn_mat[i,:].sum() for l in s.W.LINKS if l in links])
            dn = sum([l.dn_mat[i,:].sum() for l in s.W.LINKS if l in links])
            an = sum([l.length*s.W.EULAR_DT for l in s.W.LINKS if l in links])
            s.W.K_AREA[i] = tn/an
            s.W.Q_AREA[i] = dn/an
    
    @catch_exceptions_and_warn()
    def macroscopic_fundamental_diagram(s, kappa=0.2, qmax=1, links=None, figsize=(4,4)):
        #MFDの描画
        if links == None:
            links = s.W.LINKS
        s.compute_mfd(links)
        
        plt.figure(figsize=figsize)
        plt.title(f"# of links: {len(links)}")
        plt.plot(s.W.K_AREA, s.W.Q_AREA, "ko-")
        plt.xlabel("network average density (veh/m)")
        plt.ylabel("network average flow (veh/s)")
        plt.xlim([0, kappa])
        plt.ylim([0, qmax])
        plt.grid()        
        plt.tight_layout()
        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/mfd.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")
    
    @catch_exceptions_and_warn()
    def plot_vehicle_log(s, vehname):
        #車両1台の走行リンクと速度をプロット
        veh = s.W.VEHICLES[vehname]

        fig, ax1 = plt.subplots()
        plt.title(f"vehicle: {veh.name}")
        ax1.fill_between(veh.log_t, 0, veh.log_v, color="c", zorder=10)
        ax1.set_ylabel('speed (m/s)', color='c')
        plt.ylim([0, None])
        plt.xlabel("time (s)")

        ax2 = ax1.twinx()
        vehlinks = [str(l.name) if l != -1 else "not in network" for l in veh.log_link]
        ax2.plot([veh.log_t[i] for i in lange(veh.log_t) if veh.log_state[i] != "home"], [vehlinks[i] for i in lange(vehlinks) if veh.log_state[i] != "home"], 'k-', zorder=20)
        ax2.grid()
        ax2.set_ylabel('link', color='k')
        plt.ylim([0, None])
        plt.tight_layout()
        
        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/vehicle_{vehname}.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")
    
    def log_vehicles_to_pandas(s):
        #車両走行ログをpandas.DataFrameに変換して返す
        return s.vehicles_to_pandas()
    
    def vehicles_to_pandas(s):
        #車両走行ログをpandas.DataFrameに変換して返す．名称変更
        if s.flag_pandas_convert == 0:
            s.flag_pandas_convert = 1
            
            out = [["name", "dn", "orig", "dest", "t", "link", "x", "s", "v"]]
            for veh in s.W.VEHICLES.values():
                for i in range(len(veh.log_t)):
                    if veh.log_state[i] in ("wait", "run", "end", "abort"):
                        if veh.log_link[i] != -1:
                            linkname = veh.log_link[i].name
                        else:
                            if veh.log_state[i] == "wait":
                                linkname = "waiting_at_origin_node"
                            else:
                                linkname = "trip_end"
                        out.append([veh.name, s.W.DELTAN, veh.orig.name, veh.dest.name, veh.log_t[i], linkname, veh.log_x[i], veh.log_s[i], veh.log_v[i]])
            s.df_vehicles = pd.DataFrame(out[1:], columns=out[0])
        return s.df_vehicles
    
    def basic_to_pandas(s):
        #基礎情報をdfに変換して返す
        out = [["total_trips", "completed_trips", "total_travel_time", "average_travel_time", "total_delay", "average_delay"], [s.trip_all, s.trip_completed, s.total_travel_time, s.average_travel_time, s.total_delay, s.average_delay]]
        
        s.df_basic = pd.DataFrame(out[1:], columns=out[0])
        return s.df_basic
    
    def od_to_pandas(s):
        #OD別分析をdfに変換して返す
        s.od_analysis()
        
        out = [["orig", "dest", "total_trips", "completed_trips", "free_travel_time", "average_travel_time", "stddiv_travel_time"]]
        for o,d in s.od_trips.keys():
            out.append([o.name, d.name, s.od_trips[o,d], s.od_trips_comp[o,d], s.od_tt_free[o,d], s.od_tt_ave[o,d], s.od_tt_std[o,d]])
        
        s.df_od = pd.DataFrame(out[1:], columns=out[0])
        return s.df_od
        
    def mfd_to_pandas(s, links=None):
        #エリア流率・密度をdfに変換して返す
        s.compute_mfd(links)
        
        out = [["t", "network_k", "network_q"]]
        for i in lange(s.W.K_AREA):
            out.append([i*s.W.EULAR_DT, s.W.K_AREA[i], s.W.Q_AREA[i]])
        s.df_mfd = pd.DataFrame(out[1:], columns=out[0])
        return s.df_mfd
    
    def link_to_pandas(s):
        #リンク粗情報をdfに変換して返す
        s.link_analysis_coarse()
        
        out = [["link", "traffic_volume", "vehicles_remain", "free_travel_time", "average_travel_time", "stddiv_travel_time"]]
        for l in s.W.LINKS:
            out.append([l.name, s.linkc_volume[l], s.linkc_remain[l], s.linkc_tt_free[l], s.linkc_tt_ave[l], s.linkc_tt_std[l]])
        s.df_linkc = pd.DataFrame(out[1:], columns=out[0])
        return s.df_linkc
    
    def link_traffic_state_to_pandas(s):
        #リンクの時空間交通状態をdfに変換して返す
        s.compute_edie_state()
        
        out = [["link", "t", "x", "delta_t", "delta_x", "q", "k", "v"]]
        for l in s.W.LINKS:
            for i in range(l.k_mat.shape[0]):
                for j in range(l.k_mat.shape[1]):
                    out.append([l.name, i*l.edie_dt, j*l.edie_dx, l.edie_dt, l.edie_dx, l.q_mat[i,j], l.k_mat[i,j], l.v_mat[i,j]])
        s.df_link_traffic_state = pd.DataFrame(out[1:], columns=out[0])
        return s.df_link_traffic_state
    
    @catch_exceptions_and_warn()
    def output_data(s, fname=None):
        #車両走行ログをCSVとして保存する
        if fname == None:
            fname = f"out{s.W.name}/data"
        s.basic_to_pandas().to_csv(fname+"_basic.csv", index=False)
        s.od_to_pandas().to_csv(fname+"_od.csv", index=False)
        s.mfd_to_pandas().to_csv(fname+"_mfd.csv", index=False)
        s.link_to_pandas().to_csv(fname+"_link.csv", index=False)
        s.link_traffic_state_to_pandas().to_csv(fname+"_link_traffic_state.csv", index=False)
        s.log_vehicles_to_pandas().to_csv(fname+"_vehicles.csv", index=False)


class World:
    """network simulation environment (i.e., world)"""
    
    def __init__(W, name="", deltan=5, reaction_time=1, duo_update_time=600, duo_update_weight=0.5, duo_noise=0.01, eular_dt=120, eular_dx=100, random_seed=None, print_mode=1, save_mode=1, show_mode=0, route_choice_principle="homogeneous_DUO", show_progress=1, show_progress_deltat=600, tmax=None):
        """Create a world.

        Parameters
        ----------
        name : str, optional
            The name of the world, default is an empty string.
        deltan : int, optional
            The platoon size, default is 5 vehicles.
        reaction_time : float, optional
            The reaction time, default is 1 second. This is also related to simulation time step width.
        duo_update_time : float, optional
            The time interval for route choice update, default is 600 seconds.
        duo_update_weight : float, optional
            The update weight for route choice, default is 0.5.
        duo_noise : float, optional
            The noise in route choice, default is 0.01.
        eular_dt : float, optional
            The time aggregation size for eularian traffic state computation, default is 120.
        random_seed : int or None, optional
            The random seed, default is None.
        print_mode : int, optional
            The print mode, whether print the simulation progress or not. Default is 1 (enabled).
        save_mode : int, optional
            The save mode,. whether save the simulation results or not.  Default is 1 (enabled).
        show_mode : int, optional
            The show mode, whether show the matplotlib visualization results or not. Default is 0 (disabled).
        route_choice_principle : str, optional
            The route choice principle, default is "homogeneous_DUO".
        show_progress : int, optional
            Whether show network progress, default is 1 (enabled).
        show_progress_deltat : float, optional
            The time interval for showing network progress, default is 600 seconds.
        tmax : float or None, optional
            The simulation duration, default is None (automatically determined).
        
        Notes
        -----
        A World object must be defined firstly to initiate simulation.
        """
        
        ## パラメータ設定
        random.seed(random_seed)
        np.random.seed(random_seed)
        
        W.TMAX = tmax   #シミュレーション時間（s）
        
        W.DELTAN = deltan         #車群サイズ（veh）
        W.REACTION_TIME = reaction_time         #反応時間（s）
        
        W.DUO_UPDATE_TIME = duo_update_time     #経路選択時間間隔（s）
        W.DUO_UPDATE_WEIGHT = duo_update_weight    #経路選択の更新重み
        W.DUO_NOISE = duo_noise    #経路選択時のノイズ（完全に対称なネットワークで極端な経路選択になるのを防ぐ）
        W.EULAR_DT = eular_dt     #Eular型データのデフォルト時間離散化幅
        
        W.DELTAT = W.REACTION_TIME*W.DELTAN
        W.DELTAT_ROUTE = int(W.DUO_UPDATE_TIME/W.DELTAT)
        
        ## データ格納先定義
        W.VEHICLES = OrderedDict()            #home, wait, run, end
        W.VEHICLES_LIVING = OrderedDict()     #home, wait, run
        W.VEHICLES_RUNNING = OrderedDict()    #run
        W.NODES = []
        W.LINKS = []
        
        W.route_choice_principle = route_choice_principle
        
        ## リアルタイム経過表示
        W.show_progress = show_progress
        W.show_progress_deltat_timestep = int(show_progress_deltat/W.DELTAT)
        
        ## システム設定
        W.name = name
        
        W.finalized = 0
        W.world_start_time = time.time()
        
        W.print_mode = print_mode
        if print_mode:
            W.print = print
        else:
            def noprint(*args, **kwargs):
                pass
            W.print = noprint
        W.save_mode = save_mode
        W.show_mode = show_mode
        
    
    def addNode(W, *args, **kwargs):
        """add a node to world
        
        Parameters
        ----------
        name : str
            The name of the node.
        x : float
            The x-coordinate of the node (for visualization purposes).
        y : float
            The y-coordinate of the node (for visualization purposes).
        signal : list of int, optional
            A list representing the signal at the node. Default is [0], representing no signal.
            If a signal is present, the list contains the green times for each group, e.g., [green_time_group0, green_time_group1, ...].
        inlinks : dict
            A dictionary holding the incoming links to the node.
        outlinks : dict
            A dictionary holding the outgoing links from the node.
        incoming_vehicles : list
            A list holding the vehicles that are incoming to the node.
        generation_queue : deque
            A queue holding the generated vehicles waiting to enter the network via this node.
        signal_phase : int
            The current phase of the signal.
        signal_t : int
            The current time of the signal.
        id : int
            The unique identifier of the node.
        name : str
            The name of the node.
        
        Returns
        -------
        object
            the added Node object.
        
        Notes
        -----
        This function acts as a wrapper for creating a Node object and adding it to the network.
        It passes all given arguments and keyword arguments to the Node class initialization.
        """
        return Node(W, *args, **kwargs)
        
    def addLink(W, *args, **kwargs):
        """add a link to world
        
        Parameters
        ----------
        name : str
            The name of the node.
        x : float
            The x-coordinate of the node (for visualization purposes).
        y : float
            The y-coordinate of the node (for visualization purposes).
        signal : list of int, optional
            A list representing the signal at the node. Default is [0], representing no signal.
            If a signal is present, the list contains the green times for each group, e.g., [green_time_group0, green_time_group1, ...].
        inlinks : dict
            A dictionary holding the incoming links to the node.
        outlinks : dict
            A dictionary holding the outgoing links from the node.
        incoming_vehicles : list
            A list holding the vehicles that are incoming to the node.
        generation_queue : deque
            A queue holding the generated vehicles waiting to enter the network via this node.
        signal_phase : int
            The current phase of the signal.
        signal_t : int
            The current time of the signal.
        id : int
            The unique identifier of the node.
        name : str
            The name of the node.
        
        Returns
        -------
        object
            the added Link object.
        
        Notes
        -----
        This function acts as a wrapper for creating a Link object and adding it to the network.
        It passes all given arguments and keyword arguments to the Link class initialization.
        """
        return Link(W, *args, **kwargs)
        
    def addVehicle(W, *args, **kwargs):
        """add a vehicle to world
        
        Parameters
        ----------
        orig : str
            The origin node.
        dest : str
            The destination node.
        departure_time : int
            The departure time of the vehicle.
        name : str, optional
            The name of the vehicle, default is the id of the vehicle.
        route_pref : dict, optional
            The preference weights for links, default is 0 for all links.
        route_choice_principle : str, optional
            The route choice principle of the vehicle, default is the network's route choice principle.
        links_prefer : list of str, optional
            The names of the links the vehicle prefers, default is empty list.
        links_avoid : list of str, optional
            The names of the links the vehicle avoids, default is empty list.
        trip_abort : int, optional
            Whether to abort the trip if a dead end is reached, default is 1.
        
        Returns
        -------
        object
            the added Vehicle object.
        
        Notes
        -----
        This function acts as a wrapper for creating a Vehicle object and adding it to the network.
        It passes all given arguments and keyword arguments to the Vehicle class initialization.
        """
        return Vehicle(W, *args, **kwargs)
    
    def adddemand(W, orig, dest, t_start, t_end, flow):
        """Generate vehicles by specifying time-dependent origin-destination demand.

        Parameters
        ----------
        orig : str
            The name of the origin node.
        dest : str
            The name of the destination node.
        t_start : float
            The start time for the demand in seconds.
        t_end : float
            The end time for the demand in seconds.
        flow : float
            The flow rate from the origin to the destination in vehicles per second.
        """
        #時間帯OD需要の生成関数
        #時刻t_start (s)からt_end (s)までorigからdestに
        #向かう流率flow (veh/s)の需要を生成
        f = 0
        for t in range(int(t_start/W.DELTAT), int(t_end/W.DELTAT)):
            f += flow*W.DELTAT
            if f >= W.DELTAN:
                W.addVehicle(orig, dest, t, departure_time_is_time_step=1)
                f -= W.DELTAN
                
    def finalize_scenario(W, tmax=None):
        
        #シミュレーション時間の決定
        if W.TMAX == None:
            if tmax == None:
                tmax = 0
                for veh in W.VEHICLES.values():
                    if veh.departure_time*W.DELTAT > tmax:
                        tmax = veh.departure_time*W.DELTAT
                W.TMAX = (tmax//1800+2)*1800
            else:
                W.TMAX = tmax    #s
        
        W.T = 0 #timestep
        W.TIME = 0 #s
        
        W.TSIZE = int(W.TMAX/W.DELTAT)
        W.Q_AREA = np.zeros(int(W.TMAX/W.EULAR_DT))
        W.K_AREA = np.zeros(int(W.TMAX/W.EULAR_DT))
        for l in W.LINKS:
            l.init_after_tmax_fix()
        
        #隣接行列の生成
        W.ROUTECHOICE = RouteChoice(W)
        W.ADJ_MAT = np.zeros([len(W.NODES), len(W.NODES)])
        W.ADJ_MAT_LINKS = {} #リンクオブジェクトが入った隣接行列（的な辞書）
        for link in W.LINKS:
            for i,node in enumerate(W.NODES):
                if node == link.start_node:
                    break
            for j,node in enumerate(W.NODES):
                if node == link.end_node:
                    break
            W.ADJ_MAT[i,j] = 1
            W.ADJ_MAT_LINKS[i,j] = link
        
        W.analyzer = Analyzer(W)
        
        W.finalized = 1
        
        ## 問題規模表示
        W.print("simulation setting:")
        W.print(" scenario name:", W.name)
        W.print(" simulation duration:\t", W.TMAX, "s")
        W.print(" number of vehicles:\t", len(W.VEHICLES)*W.DELTAN, "veh")
        W.print(" total road length:\t", sum([l.length for l in W.LINKS]),"m")
        W.print(" time discret. width:\t", W.DELTAT, "s")
        W.print(" platoon size:\t\t", W.DELTAN, "veh")
        W.print(" number of timesteps:\t", W.TSIZE)
        W.print(" number of platoons:\t", len(W.VEHICLES))
        W.print(" number of links:\t", len(W.LINKS))
        W.print(" number of nodes:\t", len(W.NODES))
        W.print(" setup time:\t\t", f"{time.time()-W.world_start_time:.2f}", "s")
        
        W.sim_start_time = time.time()
        W.print("simulating...")
    
    def exec_simulation(W, until_t=None, duration_t=None):
        """Execute the main loop of the simulation.

        Parameters
        ----------
        until_t : float or None, optional
            The time until the simulation is to be executed in seconds. If both `until_t` and `duration_t` are None, the simulation is executed until the end. Default is None.
        duration_t : float or None, optional
            The duration for which the simulation is to be executed in seconds. If both `until_t` and `duration_t` are None, the simulation is executed until the end. Default is None.

        Returns
        -------
        int
            Returns 1 if the simulation is finished, 0 otherwise.

        Notes
        -----
        The function executes the main simulation loop that update links, nodes, and vehicles for each time step.
        It also performs route search and updates the route preference for vehicles at specified intervals.
        The simulation is executed until the end time is reached or until the maximum simulation time is exceeded.
        The nodes, links, and vehicles must be defined before calling this function.
        """
        
        #シミュレーションのメインループを回す
        
        
        #シナリオ未確定であればここで確定
        if W.finalized == 0:
            W.finalize_scenario()
        
        #時間決定
        start_ts = W.T
        if until_t != None:
            end_ts = int(until_t/W.DELTAT)
        elif duration_t != None:
            end_ts = start_ts + int(duration_t/W.DELTAT)
        else:
            end_ts = W.TSIZE
        if end_ts > W.TSIZE:
            end_ts = W.TSIZE
        
        #W.print(f"simulating: from t = {start_ts*W.DELTAT} to {(end_ts-1)*W.DELTAT} s...")
        
        if start_ts == end_ts == W.TSIZE:
            if W.print_mode and W.show_progress:
                W.analyzer.show_simulation_progress()
            W.simulation_terminated()
            return 1 #シミュレーション終わり
        if end_ts < start_ts:
            raise Exception("exec_simulation error: Simulation duration is negative. Check until_t or duration_t")
        
        #メインループ
        for W.T in range(start_ts, end_ts):
            if W.T == 0:
                W.print("      time| # of vehicles| ave speed | computation time", flush=True)
                W.analyzer.show_simulation_progress()
            
            for link in W.LINKS:
                link.update()
            
            for node in W.NODES:
                node.generate()
                node.update()
            
            for node in W.NODES:
                node.transfer()
            
            for veh in W.VEHICLES_RUNNING.values():
                veh.carfollow()
            
            for veh in copy.copy(W.VEHICLES_LIVING).values():
                veh.update()
            
            if W.T % W.DELTAT_ROUTE == 0:
                W.ROUTECHOICE.route_search_all(noise=W.DUO_NOISE)
                W.ROUTECHOICE.homogeneous_DUO_update()
                for veh in W.VEHICLES_LIVING.values():
                    veh.route_pref_update(weight=W.DUO_UPDATE_WEIGHT)
            
            W.TIME = W.T*W.DELTAT
            
            if W.print_mode and W.show_progress and W.T%W.show_progress_deltat_timestep == 0 and W.T > 0:
                W.analyzer.show_simulation_progress()
        
        if W.T == W.TSIZE-1:
            if W.print_mode and W.show_progress:
                W.analyzer.show_simulation_progress()
            W.simulation_terminated()
            return 1
        
        W.T += 1
        return 0 #まだ終わってない
    
    def check_simulation_ongoing(W):
        """Check whether the simulation is has not reached its final time.
        
        Returns
        -------
        int
            Returns 1 if the simulation is ongoing and has not reached its final time.
        """
        #シミュレーションが進行中か判定
        if W.finalized == 0:
            return 1
        return W.T < W.TSIZE-1
    
    def simulation_terminated(W):
        """postprocessing after simulation finished"""
        W.print(" simulation finished")
        W.analyzer.basic_analysis()
    
    def get_node(W, node):
        """Get a Node instance by name or object.

        Parameters
        ----------
        node : str or Node object
            The name of the node or the Node object itself.

        Returns
        -------
        Node object
            The found Node object.
        """
        if type(node) is Node:
            return node
        elif type(node) is str:
            for n in W.NODES:
                if n.name == node:
                    return n
        raise Exception(f"'{node}' is not Node")
    
    def get_link(W, link):
        """Get a Link instance by name or object.

        Parameters
        ----------
        link : str or Link object
            The name of the link or the Link object itself.

        Returns
        -------
        Link object
            The found Link object.
        """
        if type(link) is Link:
            return link
        elif type(link) is str:
            for l in W.LINKS:
                if l.name == link:
                    return l
        raise Exception(f"'{link}' is not Link")
    
    #def generate_demand(W, orig, dest, t_start, t_end, flow):
    #    #時間帯OD需要の生成関数
    #    #時刻t_start (s)からt_end (s)までorigからdestに
    #    #向かう流率flow (veh/s)の需要を生成
    #    f = 0
    #    for t in range(int(t_start/W.DELTAT), int(t_end/W.DELTAT)):
    #        f += flow*W.DELTAT
    #        if f >= W.DELTAN:
    #            Vehicle(W, orig, dest, t)
    #            f -= W.DELTAN
    
    def load_scenario_from_csv(W, fname_node, fname_link, fname_demand, tmax=None):
        """Load a scenario from CSV files.

        Parameters
        ----------
        fname_node : str
            The file name of the CSV file containing node data.
        fname_link : str
            The file name of the CSV file containing link data.
        fname_demand : str
            The file name of the CSV file containing demand data.
        tmax : float or None, optional
            The maximum simulation time in seconds, default is None.
        """
        W.generate_Nodes_from_csv(fname_node)
        W.generate_Links_from_csv(fname_link)
        W.generate_demand_from_csv(fname_demand)
        
        if tmax != None:
            W.TMAX = tmax
    
    def generate_Nodes_from_csv(W, fname):
        """Generate nodes in the network from a CSV file.

        Parameters
        ----------
        fname : str
            The file name of the CSV file containing node data.
        """

        with open(fname) as f:
            for r in csv.reader(f):
                if r[1] != "x":
                    W.addNode(r[0], float(r[1]), float(r[2]))

    def generate_Links_from_csv(W, fname):
        """Generate links in the network from a CSV file.

        Parameters
        ----------
        fname : str
            The file name of the CSV file containing link data.
        """

        with open(fname) as f:
            for r in csv.reader(f):
                if r[3] != "length":
                    W.addLink(r[0], r[1], r[2], length=float(r[3]), free_flow_speed=float(r[4]), jam_density=float(r[5]), merge_priority=float(r[6]))
                    
    def generate_demand_from_csv(W, fname):
        """Generate demand in the network from a CSV file.

        Parameters
        ----------
        fname : str
            The file name of the CSV file containing demand data.
        """
        with open(fname) as f:
            for r in csv.reader(f):
                if r[2] != "start_t":
                    W.adddemand(r[0], r[1], float(r[2]), float(r[3]), float(r[4]))

    def on_time(W, time):
        """Check if the current time step is close to the specified time.

        Parameters
        ----------
        time : float
            The specified time in seconds.

        Returns
        -------
        bool
            Returns True if the current time step is close to the specified time, False otherwise.
        """
        if W.T == int(time/W.DELTAT):
            return True
        else:
            return False
