"""
Macroscopic/mesoscopic traffic flow simulator in a network.
"""

import numpy as np
import matplotlib.pyplot as plt
import random, glob, os, csv, time, math, string
import pandas as pd
from PIL import Image, ImageDraw, ImageFont, ImageOps
from PIL.Image import Resampling, Transpose
from tqdm.auto import tqdm
from collections import deque, OrderedDict
from collections import defaultdict as ddict
from .utils  import *
import pkg_resources

from scipy.sparse.csgraph import floyd_warshall

import warnings

plt.rcParams["font.family"] = "monospace"

# ノードクラス
class Node:
    """
    Node in a network.
    """
    def __init__(s, W, name, x, y, signal=[0], signal_offset=0, flow_capacity=None, auto_rename=False):
        """
        Create a node

        Parameters
        ----------
        W : object
            The world to which the node belongs.
        name : str
            The name of the node.
        x : float
            The x-coordinate of the node (for visualization purposes).
        y : float
            The y-coordinate of the node (for visualization purposes).
        signal : list of int, optional
            A list representing the signal at the node. Default is [0], representing no signal.
            If a signal is present, the list contains the green times for each group.
            For example, `signal`=[60, 10, 50, 5] means that this signal has 4 phases, and green time for the 1st group is 60 s.
        signal_offset : float, optional
            The offset of the signal. Default is 0.
        flow_capacity : float, optional
            The maximum flow capacity of the node. Default is None, meaning infinite capacity.
        auto_rename : bool, optional
            Whether to automatically rename the node if the name is already used. Default is False.

        Attributes
        ----------
        signal_phase : int
            The phase of current signal. Links that have the same `signal_group` have a green signal.
        signal_t : float
            The elapsed time since the current signal phase started. When it is larger than `Link.signal[Link.signal_phase]`, the phase changes to the next one.
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
        s.signal_offset = signal_offset
        offset = s.signal_offset
        if s.signal != [0]:
            i = 0
            while 1:
                if offset < s.signal[i]:
                    s.signal_phase = i
                    s.signal_t = offset
                    break
                offset -= s.signal[i]

                i += 1
                if i >= len(s.signal):
                    i = 0

        s.signal_log = []

        #流量制限
        if flow_capacity != None:
            s.flow_capacity = flow_capacity
            s.flow_capacity_remain = flow_capacity*s.W.DELTAT
        else:
            s.flow_capacity = None
            s.flow_capacity_remain = 99999999999

        s.id = len(s.W.NODES)
        s.name = name
        if s.name in [n.name for n in s.W.NODES]:
            if auto_rename:
                s.name = s.name+"_renamed"+"".join(random.choices(string.ascii_letters + string.digits, k=8))
            else:
                raise ValueError(f"Node name {s.name} already used by another node. Please specify a unique name.")
        s.W.NODES.append(s)

    def __repr__(s):
        return f"<Node {s.name}>"

    def signal_control(s):
        """
        Updates the signal timings for a traffic signal node.
        """
        if s.signal_t > s.signal[s.signal_phase]:
            s.signal_phase += 1
            s.signal_t = 0
            if s.signal_phase >= len(s.signal):
                s.signal_phase = 0
        s.signal_t += s.W.DELTAT

        s.signal_log.append(s.signal_phase)

    def flow_capacity_update(s):
        """
        flow capacity updates.
        """
        if s.flow_capacity != None and s.flow_capacity_remain < s.W.DELTAN:
            s.flow_capacity_remain += s.flow_capacity*s.W.DELTAT

    def generate(s):
        """
        Departs vehicles from the waiting queue.

        Notes
        -----
        If there are vehicles in the generation queue of the node, this method attempts to depart a vehicle to one of the outgoing links.
        The choice of the outgoing link is based on the vehicle's route preference for each link. Once a vehicle is departed, it is removed from the generation queue, added to the list of vehicles on the chosen link, and its state is set to "run".
        """
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
        """
        Transfers vehicles between links at the node.

        Notes
        -----
        This method handles the transfer of vehicles from one link to another at the node.
        A vehicle is eligible for transfer if:
        - The next link it intends to move to has space.
        - The vehicle has the right signal phase to proceed.
        - The current link has enough capacity to allow the vehicle to exit.
        - The node capacity is not exceeded.
        """
        for outlink in {veh.route_next_link for veh in s.incoming_vehicles if veh.route_next_link != None}:
            if (len(outlink.vehicles) == 0 or outlink.vehicles[-1].x > outlink.delta*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN and s.flow_capacity_remain >= s.W.DELTAN:
                #受け入れ可能かつ流出可能の場合，リンク優先度に応じて選択
                vehs = [
                    veh for veh in s.incoming_vehicles 
                    if veh.route_next_link == outlink and
                      (s.signal_phase in veh.link.signal_group or len(s.signal)<=1) and 
                      veh.link.capacity_out_remain >= s.W.DELTAN
                ]
                if len(vehs) == 0:
                    continue
                veh = random.choices(vehs, [veh.link.merge_priority for veh in vehs])[0]
                inlink = veh.link

                #累積台数関連更新
                inlink.cum_departure[-1] += s.W.DELTAN
                outlink.cum_arrival[-1] += s.W.DELTAN
                inlink.traveltime_actual[int(veh.link_arrival_time/s.W.DELTAT):] = s.W.T*s.W.DELTAT - veh.link_arrival_time #自分の流入時刻より後の実旅行時間も今の実旅行時間で仮決め．後に流出した車両が上書きする前提

                veh.link_arrival_time = s.W.T*s.W.DELTAT

                inlink.capacity_out_remain -= s.W.DELTAN
                outlink.capacity_in_remain -= s.W.DELTAN
                if s.flow_capacity != None:
                    s.flow_capacity_remain -= s.W.DELTAN

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
        """
        Make necessary updates when the timestep is incremented.
        """
        s.signal_control()
        s.flow_capacity_update()

# リンククラス
class Link:
    """
    Link in a network.
    """
    def __init__(s, W, name, start_node, end_node, length, free_flow_speed, jam_density, merge_priority=1, signal_group=0, capacity_out=None, capacity_in=None, eular_dx=None, attribute=None, auto_rename=False):
        """
        Create a link

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
        merge_priority : float, optional
            The priority of the link when merging at the downstream node, default is 1.
        signal_group : int or list, optional
            The signal group to which the link belongs, default is 0. If `signal_group` is int, say 0, it becomes green if `end_node.signal_phase` is 0.  the If `signal_group` is list, say [0,1], it becomes green if the `end_node.signal_phase` is 0 or 1.
        capacity_out : float, optional
            The capacity out of the link, default is calculated based on other parameters.
        capacity_in : float, optional
            The capacity into the link, default is calculated based on other parameters.
        eular_dx : float, optional
            The default space aggregation size for link traffic state computation, default is None. If None, the global eular_dx value is used.
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        auto_rename : bool, optional
            Whether to automatically rename the link if the name is already used. Default is False.

        Attributes
        ----------
        speed : float
            Average speed of traffic on the link.
        density : float
            Density of traffic on the link.
        flow : float
            Flow of traffic on the link.
        num_vehicles : float
            Number of vehicles on the link.
        num_vehicles_queue : float
            Number of slow vehicles (due to congestion) on the link.
        free_flow_speed : float
            Free flow speed of the link.
        jam_density : float
            Jam density of the link.
        capacity_out : float
            Capacity for outflow from the link.
        capacity_in : float
            Capacity for inflow to the link.
        merge_priority : float
            The priority of the link when merging at the downstream node.

        Notes
        -----
        The `capacity_out` and `capacity_in` parameters are used to set the capacities, and if not provided, they are calculated based on other parameters.
        Real-time link status for external reference is maintained with attributes `speed`, `density`, `flow`, `num_vehicles`, and `num_vehicles_queue`.
        Some of the traffic flow model parameters can be altered during simulation by changing `free_flow_speed`, `jam_density`, `capacity_out`, `capacity_in`, and `merge_priority`.
        
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
        if type(s.signal_group) == int:
            s.signal_group = [s.signal_group]

        #流出容量
        s.capacity_out = capacity_out
        if capacity_out == None:
            s.capacity_out = s.capacity*2
            #todo_later: capacity_outは微妙にバグがあるらしい（多分離散化誤差）．少なくとも未設定時にはバグが顕在化しないように2倍にしている
        s.capacity_out_remain = s.capacity_out*s.W.DELTAT

        #流入容量
        s.capacity_in = capacity_in
        if capacity_in == None:
            s.capacity_in = s.capacity*2
            #todo_later: capacity_inは微妙にバグがあるらしい（多分離散化誤差）．少なくとも未設定時にはバグが顕在化しないように2倍にしている
        s.capacity_in_remain = s.capacity_in*s.W.DELTAT

        s.id = len(s.W.LINKS)
        s.name = name
        if s.name in [l.name for l in s.W.LINKS]:
            if auto_rename:
                s.name = s.name+"_renamed"+"".join(random.choices(string.ascii_letters + string.digits, k=8))
            else:
                raise ValueError(f"Link name {s.name} already used by another link. Please specify a unique name.")
        s.W.LINKS.append(s)
        s.start_node.outlinks[s.name] = s
        s.end_node.inlinks[s.name] = s

        s.attribute = attribute

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
        """
        Initalization before simulation execution.
        """

        #Euler型交通状態
        s.edie_dt = s.W.EULAR_DT
        s.edie_dx = s.eular_dx
        s.k_mat = np.zeros([int(s.W.TMAX/s.edie_dt)+1, int(s.length/s.edie_dx)])
        s.q_mat = np.zeros(s.k_mat.shape)
        s.v_mat = np.zeros(s.k_mat.shape)
        s.tn_mat = np.zeros(s.k_mat.shape)
        s.dn_mat = np.zeros(s.k_mat.shape)
        s.an = s.edie_dt*s.edie_dx

        #累積
        s.traveltime_actual = np.array([s.length/s.u for t in range(s.W.TSIZE)])

    def update(s):
        """
        Make necessary updates when the timestep is incremented.
        """
        s.in_out_flow_constraint()

        s.set_traveltime_instant()
        s.cum_arrival.append(0)
        s.cum_departure.append(0)
        if len(s.cum_arrival) > 1:
            s.cum_arrival[-1] = s.cum_arrival[-2]
            s.cum_departure[-1] = s.cum_departure[-2]

        #リアルタイム状態リセット
        s._speed = -1
        s._density = -1
        s._flow = -1
        s._num_vehicles = -1
        s._num_vehicles_queue = -1

    def in_out_flow_constraint(s):
        """
        Link capacity updates.
        """
        #リンク流入出率を流出容量以下にするための処理
        if s.capacity_out_remain < s.W.DELTAN:
            s.capacity_out_remain += s.capacity_out*s.W.DELTAT
        if s.capacity_in_remain < s.W.DELTAN:
            s.capacity_in_remain += s.capacity_in*s.W.DELTAT

    def set_traveltime_instant(s):
        """
        Compute instantanious travel time.
        """
        if s.speed > 0:
            s.traveltime_instant.append(s.length/s.speed)
        else:
            s.traveltime_instant.append(s.length/(s.u/100))

    def arrival_count(s, t):
        """
        Get cumulative vehicle count of arrival to this link on time t

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            The cumulative arrival vehicle count.
        """
        tt = int(t//s.W.DELTAT)
        if tt >= len(s.cum_arrival):
            return s.cum_arrival[-1]
        if tt < 0:
            return s.cum_arrival[0]
        return s.cum_arrival[tt]

    def departure_count(s, t):
        """
        Get cumulative vehicle count of departure from this link on time t

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            The cumulative departure vehicle count.
        """
        tt = int(t//s.W.DELTAT)
        if tt >= len(s.cum_departure):
            return s.cum_departure[-1]
        if tt < 0:
            return s.cum_departure[0]
        return s.cum_departure[tt]

    def instant_travel_time(s, t):
        """
        Get instantanious travel time of this link on time t

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            The instantanious travel time.
        """
        tt = int(t//s.W.DELTAT)
        if tt >= len(s.traveltime_instant):
            return s.traveltime_instant[-1]
        if tt < 0:
            return s.traveltime_instant[0]
        return s.traveltime_instant[tt]

    def actual_travel_time(s, t):
        """
        Get actual travel time of vehicle who enters this link on time t. Note that small error may occur due to fractional processing.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            The actual travel time.
        """
        tt = int(t//s.W.DELTAT)
        if tt >= len(s.traveltime_actual):
            return s.traveltime_actual[-1]
        if tt < 0:
            return s.traveltime_actual[0]
        return s.traveltime_actual[tt]

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
    """
    Vehicle or platoon in a network.
    """
    def __init__(s, W, orig, dest, departure_time, name=None, route_pref=None, route_choice_principle=None, links_prefer=[], links_avoid=[], trip_abort=1, departure_time_is_time_step=0, attribute=None, auto_rename=False):
        """
        Create a vehicle (more precisely, platoon)

        Parameters
        ----------
        W : object
            The world to which the vehicle belongs.
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
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        auto_rename : bool, optional
            Whether to automatically rename the vehicle if the name is already used. Default is False.
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
        s.links_prefer = [s.W.get_link(l) for l in links_prefer]
        s.links_avoid = [s.W.get_link(l) for l in links_avoid]

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

        s.log_t_link = [[s.departure_time, "home"]] #新たなリンクに入った時にその時刻とリンクのみを保存．経路分析用
        #todo: s.departure_timeがタイムステップ表記の事がある

        s.attribute = attribute

        s.id = len(s.W.VEHICLES)
        if name != None:
            s.name = name
        else:
            s.name = str(s.id)+"_autoid"
        if s.name in [veh.name for veh in s.W.VEHICLES.values()]:
            if auto_rename:
                s.name = s.name+"_renamed"+"".join(random.choices(string.ascii_letters + string.digits, k=8))
            else:
                raise ValueError(f"Vehicle name {s.name} already used by another vehicle. Please specify a unique name.")
        s.W.VEHICLES[s.name] = s
        s.W.VEHICLES_LIVING[s.name] = s


    def __repr__(s):
        return f"<Vehicle {s.name}: {s.state}, x={s.x}, link={s.link}>"

    def update(s):
        """
        Updates the vehicle's state and position.

        Notes
        -----
        This method updates the state and position of the vehicle based on its current situation.

        - If the vehicle is at "home", it checks if the current time matches its departure time. If so, the vehicle's state is set to "wait" and it is added to the generation queue of its origin node.
        - If the vehicle is in the "wait" state, it remains waiting at its departure node.
        - If the vehicle is in the "run" state, it updates its speed and position. If the vehicle reaches the end of its current link, it either ends its trip if it has reached its destination, or requests a transfer to the next link.
        - If the vehicle's state is "end" or "abort", no further actions are taken.
        """
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
        """
        Procedure when the vehicle finishes its trip.
        """
        s.state = "end"

        s.link.cum_departure[-1] += s.W.DELTAN
        s.link.traveltime_actual[int(s.link_arrival_time/s.W.DELTAT):] = (s.W.T+1)*s.W.DELTAT - s.link_arrival_time  #端部の挙動改善 todo: 精査

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
        """
        Drive withing a link.
        """
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
        """
        Updates the vehicle's link preferences for route choice.

        Parameters
        ----------
        weight : float
            The weight for updating the link preferences based on the recent travel time.
            Should be in the range [0, 1], where 0 means the old preferences are fully retained and 1 means the preferences are completely updated.

        Notes
        -----
        This method updates the link preferences used by the vehicle to select its route based on its current understanding of the system.

        - If the vehicle's route choice principle is "homogeneous_DUO", it will update its preferences based on a global, homogenous dynamic user optimization (DUO) model.
        - If the route choice principle is "heterogeneous_DUO", it will update its preferences based on a heterogeneous DUO model, considering both its past preferences and the system's current state.

        The updated preferences guide the vehicle's decisions in subsequent route choices.
        """
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
        """
        Select a next link from the current link.
        """
        if s.dest != s.link.end_node:
            outlinks = list(s.link.end_node.outlinks.values())

            if len(outlinks):

                #好むリンク・避けるリンクがあれば優先する
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

    def traveled_route(s):
        """
        Returns the route this vehicle traveled.
        """
        link_old = -1
        t = -1
        route = []
        ts = []
        for i, link in enumerate(s.log_link):
            if link_old != link:
                route.append(link)
                ts.append(s.log_t[i])
                link_old = link

        return Route(s.W, route[:-1]), ts

    def record_log(s):
        """
        Record travel logs.
        """
        if s.state != "run":
            if s.state == "end" and s.log_t_link[-1][1] != "end":
                s.log_t_link.append([s.W.T*s.W.DELTAT, "end"])

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
            if len(s.log_link) == 0 or s.log_link[-1] != s.link:
                s.log_t_link.append([s.W.T*s.W.DELTAT, s.link])

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
    """
    Class for computing shortest path for all vehicles.
    """

    def __init__(s, W):
        """
        Create route choice computation object.

        Parameters
        ----------
        W : object
            The world to which this belongs.
        """
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
        """
        Compute the current shortest path based on instantanious travel time.

        Parameters
        ----------
        infty : float
            value representing infinity.
        noise : float
            very small noise to slightly randomize route choice. useful to eliminate strange results at an initial stage of simulation where many routes has identical travel time.
        """
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
                # iからjへの最短経路を逆にたどる．．． -> todo: 起終点を逆にした最短経路探索にすればよい
                if i != j:
                    prev = j
                    while s.pred[i, prev] != i and s.pred[i, prev] != -9999:
                        prev = s.pred[i, prev]
                    s.next[i, j] = prev

    def route_search_all_old(s, infty=9999999999999999999, noise=0):
        """
        Compute the current shortest path based on instantanious travel time. Old version, slow for large networks.

        Parameters
        ----------
        infty : float
            value representing infinity.
        noise : float
            very small noise to slightly randomize route choice. useful to eliminate strange results at an initial stage of simulation where many routes has identical travel time.
        """
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

    def homogeneous_DUO_update(s):
        """
        Update link preference of all homogeneous travelers based on DUO principle.
        """
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
    """
    Class for analyzing and visualizing a simulation result.
    """

    def __init__(s, W):
        """
        Create result analysis object.

        Parameters
        ----------
        W : object
            The world to which this belongs.
        """
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
        """
        Analyze basic stats.
        """
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
        """
        Analyze OD-specific stats: number of trips, number of completed trips, free-flow travel time, average travel time, its std
        """
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
        """
        Analyze link-level coarse stats: traffic volume, remaining vehicles, free-flow travel time, average travel time, its std.
        """
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
        """
        Generate more complete vehicle trajectories for each link by extrapolating recorded trajectories. It is assumed that vehicles are in free-flow travel at the end of the link.
        """
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
                        if x_remain/l.u > s.W.DELTAT*0.01:
                            l.xss[i].insert(0, 0)
                            l.tss[i].insert(0, l.tss[i][0]-x_remain/l.u)
                    if l.length-l.u*s.W.DELTAT <= l.xss[i][-1] < l.length:
                        x_remain = l.length-l.xss[i][-1]
                        if x_remain/l.u > s.W.DELTAT*0.01:
                            l.xss[i].append(l.length)
                            l.tss[i].append(l.tss[i][-1]+x_remain/l.u)

    def compute_edie_state(s):
        """
        Compute Edie's traffic state for each link.
        """
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
                        #compute_accurate_traj()の外挿で極稀にt1=t0になったのでエラー回避（もう起きないはずだが念のため）
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
    def print_simple_stats(s, force_print=False):
        """
        Prints basic statistics of simulation result.

        Parameters
        ----------
        force_print : bool, optional
            print the stats regardless of the value of `print_mode`
        """
        s.W.print("results:")
        s.W.print(f" average speed:\t {s.average_speed:.1f} m/s")
        s.W.print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
        if s.trip_completed > 0:
            s.W.print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
            s.W.print(f" average delay of trips:\t {s.average_delay:.1f} s")
            s.W.print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")

        if force_print == 1 and s.W.print_mode == 0:
            print("results:")
            print(f" average speed:\t {s.average_speed:.1f} m/s")
            print(" number of completed trips:\t", s.trip_completed, "/", len(s.W.VEHICLES)*s.W.DELTAN)
            if s.trip_completed > 0:
                print(f" average travel time of trips:\t {s.average_travel_time:.1f} s")
                print(f" average delay of trips:\t {s.average_delay:.1f} s")
                print(f" delay ratio:\t\t\t {s.average_delay/s.average_travel_time:.3f}")

    def comp_route_travel_time(s, t, route):
        pass

    @catch_exceptions_and_warn()
    def time_space_diagram_traj(s, links=None, figsize=(12,4), plot_signal=True):
        """
        Draws the time-space diagram of vehicle trajectories for vehicles on specified links.

        Parameters
        ----------
        links : list of link or link, optional
            The names of the links for which the time-space diagram is to be plotted.
            If None, the diagram is plotted for all links in the network. Default is None.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        plot_signal : bool, optional
            Plot the downstream signal red light.
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
            if plot_signal:
                signal_log = [i*s.W.DELTAT for i in lange(l.end_node.signal_log) if (l.end_node.signal_log[i] not in l.signal_group and len(l.end_node.signal)>1)]
                plt.plot(signal_log, [l.length for i in lange(signal_log)], "r.")
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
    def time_space_diagram_density(s, links=None, figsize=(12,4), plot_signal=True):
        """
        Draws the time-space diagram of traffic density on specified links.

        Parameters
        ----------
        links : list of link or link, optional
            The names of the links for which the time-space diagram is to be plotted.
            If None, the diagram is plotted for all links in the network. Default is None.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        plot_signal : bool, optional
            Plot the downstream signal red light.
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
            if plot_signal:
                signal_log = [i*s.W.DELTAT for i in lange(l.end_node.signal_log) if (l.end_node.signal_log[i] not in l.signal_group and len(l.end_node.signal)>1)]
                plt.plot(signal_log, [l.length for i in lange(signal_log)], "r.")
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
    def time_space_diagram_traj_links(s, linkslist, figsize=(12,4), plot_signal=True):
        """
        Draws the time-space diagram of vehicle trajectories for vehicles on concective links.

        Parameters
        ----------
        linkslist : list of link or list of list of link
            The names of the concective links for which the time-space diagram is to be plotted.
        figsize : tuple of int, optional
            The size of the figure to be plotted, default is (12,4).
        plot_signal : bool, optional
            Plot the signal red light.
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
                if plot_signal:
                    signal_log = [i*s.W.DELTAT for i in lange(l.end_node.signal_log) if (l.end_node.signal_log[i] not in l.signal_group and len(l.end_node.signal)>1)]
                    plt.plot(signal_log, [l.length+linkdict[l] for i in lange(signal_log)], "r.")
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
        """
        Plots the cumulative curves and travel times for the provided links.

        Parameters
        ----------
        links : list or object, optional
            A list of links or a single link for which the cumulative curves and travel times are to be plotted.
            If not provided, the cumulative curves and travel times for all the links in the network will be plotted.
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (6, 4).

        Notes
        -----
        This method plots the cumulative curves for vehicle arrivals and departures, as well as the instantaneous and actual travel times.
        The plots are saved to the directory `out<W.name>` with the filename format `cumulative_curves_<link.name>.png`.
        """

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
        """
        Visualizes the entire transportation network and its current traffic conditions.

        Parameters
        ----------
        t : float, optional
            The current time for which the traffic conditions are visualized.
        detailed : int, optional
            Determines the level of detail in the visualization.
            If set to 1, the link internals (cell) are displayed in detail.
            If set to 0, the visualization is simplified to link-level. Default is 1.
        minwidth : float, optional
            The minimum width of the link visualization. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link visualization. Default is 12.
        left_handed : int, optional
            If set to 1, the left-handed traffic system (e.g., Japan, UK) is used. If set to 0, the right-handed one is used. Default is 1.
        tmp_anim : int, optional
            If set to 1, the visualization will be saved as a temporary animation frame. Default is 0.
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (6, 6).
        network_font_size : int, optional
            The font size for the network labels. Default is 4.
        node_size : int, optional
            The size of the nodes in the visualization. Default is 2.

        Notes
        -----
        This method visualizes the entire transportation network and its current traffic conditions.
        The visualization provides information on vehicle density, velocity, link names, node locations, and more.
        The plots are saved to the directory `out<W.name>` with filenames depending on the `detailed` and `t` parameters.
        """
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
                    try:
                        k = l.k_mat[int(t/l.edie_dt), i+1]
                        v = l.v_mat[int(t/l.edie_dt), i+1]
                    except:
                        warnings.warn(f"invalid time {t} is specified for network visualization", UserWarning)
                        return -1
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
        """
        Visualizes the entire transportation network and its current traffic conditions. Faster implementation using Pillow.

        Parameters
        ----------
        t : float, optional
            The current time for which the traffic conditions are visualized.
        detailed : int, optional
            Determines the level of detail in the visualization.
            If set to 1, the link internals (cell) are displayed in detail.
            If set to 0, the visualization is simplified to link-level. Default is 1.
        minwidth : float, optional
            The minimum width of the link visualization. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link visualization. Default is 12.
        left_handed : int, optional
            If set to 1, the left-handed traffic system (e.g., Japan, UK) is used. If set to 0, the right-handed one is used. Default is 1.
        tmp_anim : int, optional
            If set to 1, the visualization will be saved as a temporary animation frame. Default is 0.
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (6, 6).
        network_font_size : int, optional
            The font size for the network labels. Default is 4.
        node_size : int, optional
            The size of the nodes in the visualization. Default is 2.

        Notes
        -----
        This method visualizes the entire transportation network and its current traffic conditions.
        The visualization provides information on vehicle density, velocity, link names, node locations, and more.
        The plots are saved to the directory `out<W.name>` with filenames depending on the `detailed` and `t` parameters.
        """

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
        font_fname = pkg_resources.resource_filename('uxsim', 'utils/Inconsolata.otf')
        font = ImageFont.truetype(font_fname, int(network_font_size))

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

        font_fname = pkg_resources.resource_filename('uxsim', 'utils/Inconsolata.otf')
        font = ImageFont.truetype(font_fname, int(30))
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
        """
        Print simulation progress.
        """
        if s.W.print_mode:
            vehs = [l.density*l.length for l in s.W.LINKS]
            sum_vehs = sum(vehs)

            vs = [l.density*l.length*l.speed for l in s.W.LINKS]
            if sum_vehs > 0:
                avev = sum(vs)/sum_vehs
            else:
                avev = 0

            print(f"{s.W.TIME:>8.0f} s| {sum_vehs:>8.0f} vehs|  {avev:>4.1f} m/s| {time.time()-s.W.sim_start_time:8.2f} s", flush=True)

    @catch_exceptions_and_warn()
    def network_anim(s, animation_speed_inverse=10, detailed=0, minwidth=0.5, maxwidth=12, left_handed=1, figsize=(6,6), node_size=2, network_font_size=20, timestep_skip=24):
        """
        Generates an animation of the entire transportation network and its traffic states over time.

        Parameters
        ----------
        animation_speed_inverse : int, optional
            The inverse of the animation speed. A higher value will result in a slower animation. Default is 10.
        detailed : int, optional
            Determines the level of detail in the animation.
            If set to 1, the link internals (cell) are displayed in detail.
            Under some conditions, the detailed mode will produce inappropriate visualization.
            If set to 0, the visualization is simplified to link-level. Default is 0.
        minwidth : float, optional
            The minimum width of the link visualization in the animation. Default is 0.5.
        maxwidth : float, optional
            The maximum width of the link visualization in the animation. Default is 12.
        left_handed : int, optional
            If set to 1, the left-handed traffic system (e.g., Japan, UK) is used. If set to 0, the right-handed one is used. Default is 1.
        figsize : tuple of int, optional
            The size of the figures in the animation. Default is (6, 6).
        node_size : int, optional
            The size of the nodes in the animation. Default is 2.
        network_font_size : int, optional
            The font size for the network labels in the animation. Default is 20.
        timestep_skip : int, optional
            How many timesteps are skipped per frame. Large value means coarse and lightweight animation. Default is 8.

        Notes
        -----
        This method generates an animation visualizing the entire transportation network and its traffic conditions over time.
        The animation provides information on vehicle density, velocity, link names, node locations, and more.
        The generated animation is saved to the directory `out<W.name>` with a filename based on the `detailed` parameter.

        Temporary images used to create the animation are removed after the animation is generated.
        """
        s.W.print(" generating animation...")
        pics = []
        for t in tqdm(range(0, s.W.TMAX, s.W.DELTAT*timestep_skip), disable=(s.W.print_mode==0)):
            if int(t/s.W.LINKS[0].edie_dt) < s.W.LINKS[0].k_mat.shape[0]:
                if detailed:
                    #todo_later: 今後はこちらもpillowにする
                    s.network(int(t), detailed=detailed, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size)
                else:
                    s.network_pillow(int(t), detailed=detailed, minwidth=minwidth, maxwidth=maxwidth, left_handed=left_handed, tmp_anim=1, figsize=figsize, node_size=node_size, network_font_size=network_font_size)
                pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))
        pics[0].save(f"out{s.W.name}/anim_network{detailed}.gif", save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*timestep_skip, loop=0)
        for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
            os.remove(f)

    @catch_exceptions_and_warn()
    def network_fancy(s, animation_speed_inverse=10, figsize=6, sample_ratio=0.3, interval=5, network_font_size=0, trace_length=3, speed_coef=2):
        """
        Generates a visually appealing animation of vehicles' trajectories across the entire transportation network over time.

        Parameters
        ----------
        animation_speed_inverse : int, optional
            The inverse of the animation speed. A higher value will result in a slower animation. Default is 10.
        figsize : int or tuple of int, optional
            The size of the figures in the animation. Default is 6.
        sample_ratio : float, optional
            The fraction of vehicles to be visualized. Default is 0.3.
        interval : int, optional
            The interval at which vehicle positions are sampled. Default is 5.
        network_font_size : int, optional
            The font size for the network labels in the animation. Default is 0.
        trace_length : int, optional
            The length of the vehicles' trajectory trails in the animation. Default is 3.
        speed_coef : int, optional
            A coefficient that adjusts the animation speed. Default is 2.

        Notes
        -----
        This method generates a visually appealing animation that visualizes vehicles' trajectories across the transportation network over time.
        The animation provides information on vehicle positions, speeds, link names, node locations, and more, with Bezier curves used for smooth transitions.
        The generated animation is saved to the directory `out<W.name>` with a filename `anim_network_fancy.gif`.

        Temporary images used to create the animation are removed after the animation is generated.
        """

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
            font_fname = pkg_resources.resource_filename('uxsim', 'utils/Inconsolata.otf')
            font = ImageFont.truetype(font_fname, int(network_font_size))

            def flip(y):
                return img.size[1]-y

            for l in s.W.LINKS:
                x1, y1 = l.start_node.x*coef-minx, l.start_node.y*coef-miny
                x2, y2 = l.end_node.x*coef-minx, l.end_node.y*coef-miny
                draw.line([(x1, flip(y1)), (x2, flip(y2))], fill=(200,200,200), width=int(1), joint="curve")

                if network_font_size > 0:
                    draw.text(((x1+x2)/2, flip((y1+y2)/2)), l.name, font=font, fill="blue", anchor="mm")

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

            font_fname = pkg_resources.resource_filename('uxsim', 'utils/Inconsolata.otf')
            font = ImageFont.truetype(font_fname, int(30))
            draw.text((img.size[0]/2,20), f"t = {t :>8} (s)", font=font, fill="black", anchor="mm")

            img = img.resize((int((maxx-minx)/scale), int((maxy-miny)/scale)), resample=Resampling.LANCZOS)
            img.save(f"out{s.W.name}/tmp_anim_{t}.png")
            pics.append(Image.open(f"out{s.W.name}/tmp_anim_{t}.png"))

        pics[0].save(f"out{s.W.name}/anim_network_fancy.gif", save_all=True, append_images=pics[1:], optimize=False, duration=animation_speed_inverse*speed_coef, loop=0)

        for f in glob.glob(f"out{s.W.name}/tmp_anim_*.png"):
            os.remove(f)

    def compute_mfd(s, links=None):
        """
        Compute network average flow and density for MFD.
        """
        s.compute_edie_state()
        if links == None:
            links = s.W.LINKS
        links = [s.W.get_link(link) for link in links]
        links = frozenset(links)


        for i in range(len(s.W.Q_AREA[links])):
            tn = sum([l.tn_mat[i,:].sum() for l in s.W.LINKS if l in links])
            dn = sum([l.dn_mat[i,:].sum() for l in s.W.LINKS if l in links])
            an = sum([l.length*s.W.EULAR_DT for l in s.W.LINKS if l in links])
            s.W.K_AREA[links][i] = tn/an
            s.W.Q_AREA[links][i] = dn/an

    @catch_exceptions_and_warn()
    def macroscopic_fundamental_diagram(s, kappa=0.2, qmax=1, figtitle="", links=None, fname="", figsize=(4,4)):
        """
        Plots the Macroscopic Fundamental Diagram (MFD) for the provided links.

        Parameters
        ----------
        kappa : float, optional
            The maximum network average density for the x-axis of the MFD plot. Default is 0.2.
        qmax : float, optional
            The maximum network average flow for the y-axis of the MFD plot. Default is 1.
        links : list or object, optional
            A list of links or a single link for which the MFD is to be plotted.
            If not provided, the MFD for all the links in the network will be plotted.
        fname : str
            File name for saving (postfix). Default is "".
        figsize : tuple of int, optional
            The size of the figure to be plotted. Default is (4, 4).

        Notes
        -----
        This method plots the Macroscopic Fundamental Diagram (MFD) for the provided links.
        The MFD provides a relationship between the network average density and the network average flow.
        The plot is saved to the directory `out<W.name>` with the filename `mfd<fname>.png`.
        """

        if links == None:
            links = s.W.LINKS
        links = [s.W.get_link(link) for link in links]
        links = frozenset(links)
        s.compute_mfd(links)

        plt.figure(figsize=figsize)
        plt.title(f"{figtitle} (# of links: {len(links)})")
        plt.plot(s.W.K_AREA[links], s.W.Q_AREA[links], "ko-")
        plt.xlabel("network average density (veh/m)")
        plt.ylabel("network average flow (veh/s)")
        plt.xlim([0, kappa])
        plt.ylim([0, qmax])
        plt.grid()
        plt.tight_layout()
        if s.W.save_mode:
            plt.savefig(f"out{s.W.name}/mfd{fname}.png")
        if s.W.show_mode:
            plt.show()
        else:
            plt.close("all")

    @catch_exceptions_and_warn()
    def plot_vehicle_log(s, vehname):
        """
        Plots the driving link and speed for a single vehicle.

        Parameters
        ----------
        vehname : str
            The name of the vehicle for which the driving link and speed are to be plotted.

        Notes
        -----
        This method visualizes the speed profile and the links traversed by a specific vehicle over time.
        The speed is plotted on the primary y-axis, and the links are plotted on the secondary y-axis.
        The plot is saved to the directory `out<W.name>` with the filename `vehicle_<vehname>.png`.
        """
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


    def vehicles_to_pandas(s):
        """
        Converts the vehicle travel logs to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
            A DataFrame containing the travel logs of vehicles, with the columns:
            - 'name': the name of the vehicle (platoon).
            - 'dn': the platoon size.
            - 'orig': the origin node of the vehicle's trip.
            - 'dest': the destination node of the vehicle's trip.
            - 't': the timestep.
            - 'link': the link the vehicle is on (or relevant status).
            - 'x': the position of the vehicle on the link.
            - 's': the spacing of the vehicle.
            - 'v': the speed of the vehicle.
        """
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
                            elif veh.log_state[i] == "abort":
                                linkname = "trip_aborted"
                            else:
                                linkname = "trip_end"
                        out.append([veh.name, s.W.DELTAN, veh.orig.name, veh.dest.name, veh.log_t[i], linkname, veh.log_x[i], veh.log_s[i], veh.log_v[i]])
            s.df_vehicles = pd.DataFrame(out[1:], columns=out[0])
        return s.df_vehicles

    def log_vehicles_to_pandas(s):
        """
        same to `vehicles_to_pandas`, just for backward compatibility
        """
        return s.vehicles_to_pandas()

    def basic_to_pandas(s):
        """
        Converts the basic stats to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        out = [["total_trips", "completed_trips", "total_travel_time", "average_travel_time", "total_delay", "average_delay"], [s.trip_all, s.trip_completed, s.total_travel_time, s.average_travel_time, s.total_delay, s.average_delay]]

        s.df_basic = pd.DataFrame(out[1:], columns=out[0])
        return s.df_basic

    def od_to_pandas(s):
        """
        Converts the OD-specific analysis results to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """

        s.od_analysis()

        out = [["orig", "dest", "total_trips", "completed_trips", "free_travel_time", "average_travel_time", "stddiv_travel_time"]]
        for o,d in s.od_trips.keys():
            out.append([o.name, d.name, s.od_trips[o,d], s.od_trips_comp[o,d], s.od_tt_free[o,d], s.od_tt_ave[o,d], s.od_tt_std[o,d]])

        s.df_od = pd.DataFrame(out[1:], columns=out[0])
        return s.df_od

    def mfd_to_pandas(s, links=None):
        """
        Converts the MFD to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        if links == None:
            links = s.W.LINKS
        s.compute_mfd(links)
        links = [s.W.get_link(link) for link in links]
        links = frozenset(links)

        out = [["t", "network_k", "network_q"]]
        for i in lange(s.W.K_AREA):
            out.append([i*s.W.EULAR_DT, s.W.K_AREA[links][i], s.W.Q_AREA[links][i]])
        s.df_mfd = pd.DataFrame(out[1:], columns=out[0])
        return s.df_mfd

    def link_to_pandas(s):
        """
        Converts the link-level analysis results to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        s.link_analysis_coarse()

        out = [["link", "traffic_volume", "vehicles_remain", "free_travel_time", "average_travel_time", "stddiv_travel_time"]]
        for l in s.W.LINKS:
            out.append([l.name, s.linkc_volume[l], s.linkc_remain[l], s.linkc_tt_free[l], s.linkc_tt_ave[l], s.linkc_tt_std[l]])
        s.df_linkc = pd.DataFrame(out[1:], columns=out[0])
        return s.df_linkc

    def link_traffic_state_to_pandas(s):
        """
        Converts the traffic states in links to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        s.compute_edie_state()

        out = [["link", "t", "x", "delta_t", "delta_x", "q", "k", "v"]]
        for l in s.W.LINKS:
            for i in range(l.k_mat.shape[0]):
                for j in range(l.k_mat.shape[1]):
                    out.append([l.name, i*l.edie_dt, j*l.edie_dx, l.edie_dt, l.edie_dx, l.q_mat[i,j], l.k_mat[i,j], l.v_mat[i,j]])
        s.df_link_traffic_state = pd.DataFrame(out[1:], columns=out[0])
        return s.df_link_traffic_state

    def link_cumulative_to_pandas(s):
        """
        Converts the cumulative counts etc. in links to a pandas DataFrame.

        Returns
        -------
        pd.DataFrame
        """
        out = [["link", "t", "arrival_count", "departure_count", "actual_travel_time", "instantanious_travel_time"]]
        for link in s.W.LINKS:
            for i in range(s.W.TSIZE):
                out.append([link.name, i*s.W.DELTAT, link.cum_arrival[i], link.cum_departure[i], link.traveltime_actual[i], link.traveltime_instant[i]])
        s.df_link_cumulative = pd.DataFrame(out[1:], columns=out[0])
        return s.df_link_cumulative

    @catch_exceptions_and_warn()
    def output_data(s, fname=None):
        """
        Save all results to CSV files
        """
        if fname == None:
            fname = f"out{s.W.name}/data"
        s.basic_to_pandas().to_csv(fname+"_basic.csv", index=False)
        s.od_to_pandas().to_csv(fname+"_od.csv", index=False)
        s.mfd_to_pandas().to_csv(fname+"_mfd.csv", index=False)
        s.link_to_pandas().to_csv(fname+"_link.csv", index=False)
        s.link_traffic_state_to_pandas().to_csv(fname+"_link_traffic_state.csv", index=False)
        s.vehicles_to_pandas().to_csv(fname+"_vehicles.csv", index=False)

# 経路クラス
class Route:
    """
    Class for a route that store concective links.
    """
    def __init__(s, W, links, name="", trust_input=False):
        """
        Define a route.

        Attributes
        ----------
        links : list
            List of links. The contents are Link objects.
        links_name : list
            List of name of links. The contents are str.
        trust_input : bool
            True if you trust the `links` in order to reduce the computation cost by omitting verification.
        """
        s.W = W
        s.name = name
        s.links = []
        if trust_input == False:
            #入力データを検査する場合
            for i in range(0, len(links)-1):
                l1 = W.get_link(links[i])
                l2 = W.get_link(links[i+1])
                if l2 in l1.end_node.outlinks.values():
                    s.links.append(l1)
                else:
                    raise Exception(f"route is not defined by concective links: {links}, {l1}")
                    #todo: interpolation based on shotest path
            s.links.append(l2)
        else:
            #検査せずそのまま使用（計算コスト削減）
            s.links = links
        s.links_name = [l.name for l in s.links]

    def __repr__(s):
        return f"<Route {s.name}: {s.links}>"

    def __eq__(self, other):
        """
        Override `==` operator. If the links of two route are the same, then the routes are the same.
        """
        if isinstance(other, Route):
            return self.links == other.links
        return NotImplemented

    def actual_travel_time(s, t, return_details=False):
        """
        Actual travel time for a (hypothetical) vehicle who start traveling this route on time t.

        Parameters
        ----------
        t : float
            Time in seconds.
        return_details : bool
            True if you want travel time per link.

        Returns
        -------
        float
            The actual travel time.
        list (if `return_details` is True)
            List of travel time per link.
        """
        tt = 0
        tts = []

        for l in s.links:
            link_tt = l.actual_travel_time(t)
            tt += link_tt
            t += link_tt
            tts.append(link_tt)

        if return_details:
            return tt, tts
        else:
            return tt

class OSMImporter:
    """
    OpenStreetMap importer using OSMnx.
    Work in progress. Import from OSM is experimental and may not work as expected. It is functional but produces inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.
    """
    def import_osm_data(north, south, east, west, custom_filter='["highway"~"trunk|primary"]', 
                        default_number_of_lanes_mortorway=3, default_number_of_lanes_trunk=3, 
                        default_number_of_lanes_primary=2, default_number_of_lanes_secondary=2, 
                        default_number_of_lanes_residential=1, default_number_of_lanes_tertiary=1, 
                        default_number_of_lanes_others=1, 
                        default_maxspeed_mortorway=100, default_maxspeed_trunk=60, 
                        default_maxspeed_primary=50, default_maxspeed_secondary=50, 
                        default_maxspeed_residential=30, default_maxspeed_tertiary=30, 
                        default_maxspeed_others=30):
        """
        Import road network data from OpenStreetMap using OSMnx.

        Parameters
        ----------
        north, south, east, west: float
            The latitudes and longitudes of the area to be imported.
        custom_filter: str
            The filter to be used for importing the data. 
            The default is '["highway"~"trunk|primary"]', which means that only trunk and primary roads (usually correspond to major arterial roads) are imported.
        default_number_of_lanes_*: int
            The default number of lanes for * {road_type}.
        default_maxspeed_*: float
            The default maximum speed for * {road_type}.    

        Returns
        -------
        links: list
            A list of links, where each element is a list of [name, from, to, lanes, maxspeed].
        nodes: dict
            A dictionary of nodes, where the key is the node ID and the value is a list of [node_id, x, y].
        """

        #experimental warning
        print("WARNING: Import from OSM is experimental and may not work as expected. It is functional but produces inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.")
        warnings.warn("Import from OSM is experimental and may not work as expected. It is functional but produces inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.")

        try:
            import osmnx as ox
        except:
            raise ImportError("Optional module 'osmnx' is not installed. Please install it by 'pip install osmnx' to use this function.")

        print("Start downloading OSM data. This may take some time.")
        G = ox.graph.graph_from_bbox(north=north, south=south, east=east, west=west, network_type="drive", 
                                    custom_filter=custom_filter)
        print("Download completed")
        """
        motorway: 高速道路
        trunk: 一般国道
        primary: 主要地方道（2桁番号県道・市道）
        secondary: 一般地方道（3桁番号県道・市道）
        """

        # データ抽出
        node_dict = {}
        for n in G.nodes:
            nd = G.nodes[n]
            node_dict[n]=[n, nd["x"], nd["y"]]

        links = []
        nodes = {}
        for e in G.edges:
            ed = G.get_edge_data(e[0], e[1])[0]
            
            if "highway" in ed:
                road_type = ed["highway"]
                try:
                    name = ed["name"]
                    if type(name) == list:
                        name = name[0]
                    osmid = ed["osmid"]
                    if type(osmid) == list:
                        osmid = osmid[0]
                    name += "-"+str(osmid)
                except:
                    name = ""
                    osmid = ""
                try:
                    lanes = int(ed["lanes"])
                except:
                    try:
                        if "mortorway" in road_type:
                            lanes = default_number_of_lanes_mortorway
                        elif "trunk" in road_type:
                            lanes = default_number_of_lanes_trunk
                        elif "primary" in road_type:
                            lanes = default_number_of_lanes_primary
                        elif "secondary" in road_type:
                            lanes = default_number_of_lanes_secondary
                        elif "residential" in road_type:
                            lanes = default_number_of_lanes_residential
                        elif "tertiary" in road_type:
                            lanes = default_number_of_lanes_tertiary
                        else:
                            lanes = default_number_of_lanes_others
                    except:
                        lanes = default_number_of_lanes_others
                try:
                    maxspeed = float(ed["maxspeed"])/3.6
                except:
                    try:
                        if "mortorway" in road_type:
                            maxspeed = default_maxspeed_mortorway/3.6
                        elif "trunk" in road_type:
                            maxspeed = default_maxspeed_trunk/3.6
                        elif "primary" in road_type:
                            maxspeed = default_maxspeed_primary/3.6
                        elif "secondary" in road_type:
                            maxspeed = default_maxspeed_secondary/3.6
                        elif "residential" in road_type:
                            maxspeed = default_maxspeed_residential/3.6
                        elif "tertiary" in road_type:
                            maxspeed = default_maxspeed_tertiary/3.6
                        else:
                            maxspeed = default_maxspeed_others/3.6
                    except:
                        maxspeed = default_maxspeed_others/3.6
            

                links.append([name, e[0], e[1], lanes, maxspeed]) # name, from, to, number_of_lanes, maxspeed
                nodes[e[0]] = node_dict[e[0]]
                nodes[e[1]] = node_dict[e[1]]

        nodes = list(nodes.values())
        
        print("imported network size:")
        print(" number of links:", len(links))
        print(" number of nodes:", len(nodes))

        return nodes, links
                    
    def osm_network_postprocessing(nodes, links, node_merge_threshold, node_merge_iteration=5, enforce_bidirectional=False):
        """
        Postprocess the network to make it suitable for simulation. First, it aggregates the network by merging nodes that are closer than the threshold. Second, if `enforce_bidirectional` is True, it adds reverse links for each link to eliminate deadend nodes as much as possible.

        Parameters
        ----------
        nodes: list
            A list of nodes, where each element is a list of [node_id, x, y].
        links: list 
            A list of links, where each element is a list of [name, from, to, lanes, maxspeed].
        node_merge_threshold: float
            If two nodes are connected by a link that is shorter than this threshold, the nodes are merged and the link is removed.
        node_merge_iteration: int
            The number of iterations for the node merge.
        enforce_bidirectional: bool
            True if you want to enforce bidirectional links. It will automatically add a reverse link for each link. This will eliminate deadend nodes as much as possible, but the original network topology is not preserved rigorously.

        Returns
        -------
        nodes: list
            A list of nodes, where each element is a list of [node_id, x, y].
        links: list
            A list of links, where each element is a list of [name, from, to, lanes, maxspeed].
        """

        #ネットワーク縮約：リンクベース
        def distance(pos1, pos2):
            return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        
        for i in range(node_merge_iteration):
            # ノード座標の辞書を作成
            node_positions = {node[0]: (node[1], node[2]) for node in nodes}

            # 長さが node_merge_threshold 以下のリンクを省略し、起点ノードと終点ノードを融合
            #縮約対象になったノードのマッピング：keyが縮約元，valueが縮約先
            delete_node_map = {}

            number_of_deleted_links = 0
            for link in links:
                start_node, end_node = link[1], link[2]
                start_pos = node_positions[start_node]
                end_pos = node_positions[end_node]
                link_length = distance(start_pos, end_pos)
                #print(link_length, link[0], start_node, end_node)

                if link_length <= node_merge_threshold:
                    #縮約先に登録されていなければ登録
                    if end_node not in delete_node_map.values() and start_node not in delete_node_map.keys():
                        delete_node_map[end_node] = start_node
                    if start_node not in delete_node_map.values() and end_node not in delete_node_map.keys():
                        delete_node_map[start_node] = end_node
                    
                    number_of_deleted_links += 1
                
            # 新しいノードリストとリンクリストを作成
            new_nodes = []
            new_links = {}

            number_of_deleted_nodes = 0
            number_of_new_nodes = 0
            for node in nodes:
                # for group in delete_node_groups:
                #     if node[0] in delete_node_map.keys():
                #         break
                if node[0] in delete_node_map.keys():
                    number_of_deleted_nodes += 1
                    continue
                else:
                    new_nodes.append(node)
                    number_of_new_nodes += 1

            for link in links:
                name = link[0]
                start_node = link[1]
                end_node = link[2]
                lanes = link[3]
                maxspeed = link[4]
                if start_node in delete_node_map.keys():
                    start_node = delete_node_map[start_node]
                if end_node in delete_node_map.keys():
                    end_node = delete_node_map[end_node]

                length = distance(node_positions[start_node], node_positions[end_node])

                if start_node != end_node:
                    new_links[start_node, end_node] = [name, start_node, end_node, lanes, maxspeed, length]

            if enforce_bidirectional:
                for link in list(new_links.values()):
                    name = link[0]
                    start_node = link[1]
                    end_node = link[2]
                    lanes = link[3]
                    maxspeed = link[4]
                    length = link[5]
                    if (end_node, start_node) not in new_links:
                        new_links[end_node, start_node] = [name+"-reverse", end_node, start_node, lanes, maxspeed, length]

            new_links = list(new_links.values())

            # 孤立したノードの除去
            used_nodes = set()
            for l in new_links:
                used_nodes.add(l[1])
                used_nodes.add(l[2])

            new_nodes_used = []
            for n in new_nodes:
                if n[0] in used_nodes:
                    new_nodes_used.append(n)

            new_nodes = new_nodes_used

            nodes = new_nodes
            links = new_links
        
        print("aggregated network size:")
        print(" number of links:", len(links))
        print(" number of nodes:", len(nodes))

        return nodes, links

    def osm_network_visualize(nodes, links, figsize=(12,12), xlim=[None,None], ylim=[None,None], show_link_name=False): 
        """
        Visualize the imported network. Mainly for test purpose.
        """
        node_positions = {node[0]: (node[1], node[2]) for node in nodes}

        # グラフを描画
        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")

        # ノードをプロット
        for node, pos in node_positions.items():
            plt.plot(pos[0], pos[1], 'ro', markersize=2)

        # リンクをプロット
        for link in links:
            start_node, end_node = link[1], link[2]
            start_pos = node_positions[start_node]
            end_pos = node_positions[end_node]
            x1 = start_pos[0]
            y1 = start_pos[1]
            x2 = end_pos[0]
            y2 = end_pos[1]
            vx, vy = (y1-y2)*0.025, (x2-x1)*0.025
            xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
            xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
            plt.plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], 'b-', linewidth=1)
            if show_link_name:
                plt.text((x1+x2)/2, (y1+y2)/2, link[0], fontsize=8)
        
        plt.xlim(xlim)
        plt.ylim(ylim)
        plt.show()

    def osm_network_to_World(W, nodes, links, default_jam_density=0.2, coef_degree_to_meter=111000):
        """
        Load the imported network to the World object of UXsim.

        Parameters
        ----------
        nodes: list
            A list of nodes, where each element is a list of [node_id, x, y].
        links: list
            A list of links, where each element is a list of [name, from, to, lanes, maxspeed, length].
        default_jam_density: float
            The default jam density for the links.
        coef_degree_to_meter: float
            The coefficient to convert lon/lat degree to meter. Default is 111000.
        """
        for i, node in enumerate(nodes):
            nname = str(node[0])
            if nname in [n.name for n in W.NODES]:
                nname + f"_osm{i}"
            W.addNode(str(node[0]), x=node[1], y=node[2], auto_rename=True)
        for i, link in enumerate(links):
            lname = str(link[0])
            if lname in [l.name for l in W.LINKS.values()]:
                lname + f"_osm{i}"
            W.addLink(lname, str(link[1]), str(link[2]), length=link[5]*coef_degree_to_meter, free_flow_speed=link[4], jam_density=default_jam_density, auto_rename=True)


class World:
    """
    World (i.e., simulation environment). A World object is consistently referred to as `W` in this code.
    """

    def __init__(W, name="", deltan=5, reaction_time=1, duo_update_time=600, duo_update_weight=0.5, duo_noise=0.01, eular_dt=120, eular_dx=100, random_seed=None, print_mode=1, save_mode=1, show_mode=0, route_choice_principle="homogeneous_DUO", show_progress=1, show_progress_deltat=600, tmax=None):
        """
        Create a World.

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
        """
        add a node to world

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
            If a signal is present, the list contains the green times for each group.
            For example, `signal`=[60, 10, 50, 5] means that this signal has 4 phases, and green time for the 1st group is 60 s.
        signal_offset : float, optional
            The offset of the signal. Default is 0.
        flow_capacity : float, optional
            The maximum flow capacity of the node. Default is None, meaning infinite capacity.
        auto_rename : bool, optional
            Whether to automatically rename the node if the name is already used. Default is False.

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
        """
        add a link to world

        Parameters
        ----------
        name : str
            The name of the link.
        start_node : str | Node
            The name or object of the start node of the link.
        end_node : str | Node
            The name or object of the end node of the link.
        length : float
            The length of the link.
        free_flow_speed : float
            The free flow speed on the link.
        jam_density : float
            The jam density on the link.
        merge_priority : float, optional
            The priority of the link when merging at the downstream node, default is 1.
        signal_group : int or list, optional
            The signal group to which the link belongs, default is 0. If `signal_group` is int, say 0, it becomes green if `end_node.signal_phase` is 0.  the If `signal_group` is list, say [0,1], it becomes green if the `end_node.signal_phase` is 0 or 1.
        capacity_out : float, optional
            The capacity out of the link, default is calculated based on other parameters.
        capacity_in : float, optional
            The capacity into the link, default is calculated based on other parameters.
        eular_dx : float, optional
            The default space aggregation size for link traffic state computation, default is None. If None, the global eular_dx value is used.
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        auto_rename : bool, optional
            Whether to automatically rename the link if the name is already used. Default is False.

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
        """
        add a vehicle to world

        Parameters
        ----------
        orig : str | Node
            The origin node.
        dest : str | Node
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
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        auto_rename : bool, optional
            Whether to automatically rename the vehicle if the name is already used. Default is False.

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

    def adddemand(W, orig, dest, t_start, t_end, flow=-1, volume=-1, attribute=None):
        """
        Generate vehicles by specifying time-dependent origin-destination demand.

        Parameters
        ----------
        orig : str | Node
            The name or object of the origin node.
        dest : str | Node
            The name or object of the destination node.
        t_start : float
            The start time for the demand in seconds.
        t_end : float
            The end time for the demand in seconds.
        flow : float, optional
            The flow rate from the origin to the destination in vehicles per second.
        volume: float, optional
            The demand volume from the origin to the destination. If volume is specified, the flow is ignored.
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        """
        if volume > 0:
            flow = volume/(t_end-t_start)

        f = 0
        for t in range(int(t_start/W.DELTAT), int(t_end/W.DELTAT)):
            f += flow*W.DELTAT
            while f >= W.DELTAN:
                W.addVehicle(orig, dest, t, departure_time_is_time_step=1, attribute=attribute)
                f -= W.DELTAN

    def adddemand_point2point(W, x_orig, y_orig, x_dest, y_dest, t_start, t_end, flow=-1, volume=-1, attribute=None):
        """
        Generate vehicles by specifying time-dependent origin-destination demand using coordinates.

        Parameters
        ----------
        x_orig : float
            The x-coordinate of the origin.
        y_orig : float
            The y-coordinate of the origin.
        x_dest : float
            The x-coordinate of the destination.
        y_dest : float
            The y-coordinate of the destination.
        t_start : float
            The start time for the demand in seconds.
        t_end : float
            The end time for the demand in seconds.
        flow : float, optional
            The flow rate from the origin to the destination in vehicles per second.
        volume: float, optional
            The demand volume from the origin to the destination. If volume is specified, the flow is ignored.
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        """
        orig = W.get_nearest_node(x_orig, y_orig)
        dest = W.get_nearest_node(x_dest, y_dest)
        W.adddemand(orig, dest, t_start, t_end, flow, volume, attribute)

    def adddemand_area2area(W, x_orig, y_orig,  radious_orig, x_dest, y_dest, radious_dest, t_start, t_end, flow=-1, volume=-1, attribute=None):
        """
        Generate vehicles by specifying time-dependent origin-destination demand by specifying circular areas.

        Parameters
        ----------
        x_orig : float
            The x-coordinate of the center of the origin area.
        y_orig : float
            The y-coordinate of the center of the origin area.
        radious_orig : float
            The radious of the origin area. Note that too large radious may generate too sparse demand that is rounded to zero.
        x_dest : float
            The x-coordinate of the center of the destination area.
        y_dest : float
            The y-coordinate of the center of the destination area.
        radious_dest : float
            The radious of the destination area.
        t_start : float
            The start time for the demand in seconds.
        t_end : float
            The end time for the demand in seconds.
        flow : float, optional
            The flow rate from the origin to the destination in vehicles per second.
        volume: float, optional
            The demand volume from the origin to the destination. If volume is specified, the flow is ignored.
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        """
        origs = W.get_nodes_in_area(x_orig, y_orig, radious_orig)
        dests = W.get_nodes_in_area(x_dest, y_dest, radious_dest)

        origs_new = []
        dests_new = []
        for o in origs:
            if len(o.outlinks) != 0:
                origs_new.append(o)
        for d in dests:
            if len(d.inlinks) != 0:
                dests_new.append(d)
        origs = origs_new
        dests = dests_new
        if len(origs) == 0:
            origs.append(W.get_nearest_node(x_orig, y_orig))
        if len(dests) == 0:
            dests.append(W.get_nearest_node(x_dest, y_dest))
        
        if flow != -1:
            flow = flow/(len(origs)*len(dests))
        if volume != -1:
            volume = volume/(len(origs)*len(dests))
        for o in origs:
            for d in dests:
                W.adddemand(o, d, t_start, t_end, flow, volume, attribute)
    
    def finalize_scenario(W, tmax=None):
        """
        Finalizes the settings and preparations for the simulation scenario execution.

        Parameters
        ----------
        tmax : float, optional
            The maximum simulation time. If not provided, it will be determined based on the departure times of the vehicles.

        Notes
        -----
        This function automatically called by `exec_simulation()` if it has not been called manually.
        """
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
        W.Q_AREA = ddict(lambda: np.zeros(int(W.TMAX/W.EULAR_DT)))
        W.K_AREA = ddict(lambda: np.zeros(int(W.TMAX/W.EULAR_DT)))
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
        """
        Execute the main loop of the simulation.

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
                W.print("      time| # of vehicles| ave speed| computation time", flush=True)
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

            for name in list(W.VEHICLES_LIVING.keys()):
                W.VEHICLES_LIVING[name].update()

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
        """
        Check whether the simulation is has not reached its final time.

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
        """
        Postprocessing after simulation finished
        """
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
            if node in W.NODES:
                return node
            else:
                for n in W.NODES:
                    if n.name == node.name:
                        return n
        elif type(node) is str:
            for n in W.NODES:
                if n.name == node:
                    return n
        raise Exception(f"'{node}' is not Node in this World")

    def get_link(W, link):
        """
        Get a Link instance by name or object.

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
            if link in W.LINKS:
                return link
            else:
                for l in W.LINKS:
                    if l.name == link.name:
                        return l
        elif type(link) is str:
            for l in W.LINKS:
                if l.name == link:
                    return l
        raise Exception(f"'{link}' is not Link in this World")

    def get_nearest_node(W, x, y):
        """
        Get the nearest node to the given coordinates.

        Parameters
        ----------
        x : float
            The x-coordinate.
        y : float
            The y-coordinate.

        Returns
        -------
        object
            The nearest Node object.
        """
        min_dist = 1e10
        nearest_node = None
        for node in W.NODES:
            dist = (node.x-x)**2 + (node.y-y)**2
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def get_nodes_in_area(W, x, y, r):
        """
        Get the nodes in the area defined by the center coordinates and radius.
        Parameters
        ----------
        x : float
            The x-coordinate of the center.
        y : float
            The y-coordinate of the center.
        r : float
            The radius of the area.

        Returns
        -------
        list
            A list of Node objects in the area.
        """
        nodes = []
        for node in W.NODES:
            if (node.x-x)**2 + (node.y-y)**2 < r**2:
                nodes.append(node)
        return nodes

    def load_scenario_from_csv(W, fname_node, fname_link, fname_demand, tmax=None):
        """
        Load a scenario from CSV files.

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
        """
        Generate nodes in the network from a CSV file.

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
        """
        Generate links in the network from a CSV file.

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
        """
        Generate demand in the network from a CSV file.

        Parameters
        ----------
        fname : str
            The file name of the CSV file containing demand data.
        """
        with open(fname) as f:
            for r in csv.reader(f):
                if r[2] != "start_t":
                    try:
                        W.adddemand(r[0], r[1], float(r[2]), float(r[3]), float(r[4]), float(r[5]))
                    except:
                        W.adddemand(r[0], r[1], float(r[2]), float(r[3]), float(r[4]))

    def on_time(W, time):
        """
        Check if the current time step is close to the specified time.

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

    #@catch_exceptions_and_warn()
    def show_network(W, width=1, left_handed=1, figsize=(6,6), network_font_size=10, node_size=6):
        """
        Visualizes the entire transportation network shape.
        """
        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")
        for n in W.NODES:
            plt.plot(n.x, n.y, "o", c="gray", ms=node_size, zorder=10)
            if network_font_size > 0:
                plt.text(n.x, n.y, n.name, c="g", horizontalalignment="center", verticalalignment="top", zorder=20, fontsize=network_font_size)
        for l in W.LINKS:
            x1, y1 = l.start_node.x, l.start_node.y
            x2, y2 = l.end_node.x, l.end_node.y
            vx, vy = (y1-y2)*0.05, (x2-x1)*0.05
            if not left_handed:
                vx, vy = -vx, -vy
            #簡略モード
            xmid1, ymid1 = (2*x1+x2)/3+vx, (2*y1+y2)/3+vy
            xmid2, ymid2 = (x1+2*x2)/3+vx, (y1+2*y2)/3+vy
            plt.plot([x1, xmid1, xmid2, x2], [y1, ymid1, ymid2, y2], "gray", lw=width, zorder=6, solid_capstyle="butt")
            if network_font_size > 0:
                plt.text(xmid1, ymid1, l.name, c="b", zorder=20, fontsize=network_font_size)
        maxx = max([n.x for n in W.NODES])
        minx = min([n.x for n in W.NODES])
        maxy = max([n.y for n in W.NODES])
        miny = min([n.y for n in W.NODES])
        buffx, buffy = (maxx-minx)/10, (maxy-miny)/10
        if buffx == 0:
            buffx = buffy
        if buffy == 0:
            buffy = buffx
        plt.xlim([minx-buffx, maxx+buffx])
        plt.ylim([miny-buffy, maxy+buffy])
        plt.tight_layout()
        plt.show()