"""
UXsim: Macroscopic/mesoscopic traffic flow simulator in a network.
This `uxsim.py` is the core of UXsim. It summarizes the classes and methods that are essential for the simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
import random, csv, time, math, string, warnings
from collections import deque, OrderedDict
from collections import defaultdict as ddict
from scipy.sparse.csgraph import floyd_warshall

from .analyzer import *
from .utils  import *

class Node:
    """
    Node in a network.
    """
    def __init__(s, W, name, x, y, signal=[0], signal_offset=0, flow_capacity=None, auto_rename=False, number_of_lanes=None):
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
        number_of_lanes : int, optional
            The number of lanes that can be green simultaniously at the node. Default is None.

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

        #流量制限（マクロ信号）
        s.flag_lanes_automatically_determined = False
        if flow_capacity != None:
            s.flow_capacity = flow_capacity
            s.flow_capacity_remain = flow_capacity*s.W.DELTAT
            if number_of_lanes != None:
                s.lanes = number_of_lanes
            else:
                s.lanes = math.ceil(flow_capacity/0.8) #TODO: 要調整．現状は1車線0.8 veh/sと見なし，車線数を決定している
                s.flag_lanes_automatically_determined = True
        else:
            s.flow_capacity = None
            s.flow_capacity_remain = 10e10
            s.lanes = None

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
        if s.flow_capacity != None:
            if s.flow_capacity_remain < s.W.DELTAN*s.lanes:
                s.flow_capacity_remain += s.flow_capacity*s.W.DELTAT
        else:
            s.flow_capacity_remain = 10e10

    def generate(s):
        """
        Departs vehicles from the waiting queue.

        Notes
        -----
        If there are vehicles in the generation queue of the node, this method attempts to depart a vehicle to one of the outgoing links.
        The choice of the outgoing link is based on the vehicle's route preference for each link. Once a vehicle is departed, it is removed from the generation queue, added to the list of vehicles on the chosen link, and its state is set to "run".
        """
        outlinks = list(s.outlinks.values())
        if len(outlinks):
            for i in range(sum([l.lanes for l in outlinks])):
                if len(s.generation_queue) > 0:
                    veh = s.generation_queue[0]
                    
                    preference = [veh.route_pref[l] for l in outlinks]
                    if sum(preference) > 0:
                        outlink = random.choices(outlinks, preference)[0]
                    else:
                        outlink = random.choices(outlinks)[0]

                    if (len(outlink.vehicles) < outlink.lanes or outlink.vehicles[-outlink.lanes].x > outlink.delta_per_lane*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN:
                        #受け入れ可能な場合，リンク優先度に応じて選択
                        veh = s.generation_queue.popleft()

                        veh.state = "run"
                        veh.link = outlink
                        veh.x = 0
                        veh.v = outlink.u #端部の挙動改善
                        s.W.VEHICLES_RUNNING[veh.name] = veh

                        if len(outlink.vehicles) > 0:
                            veh.lane = (outlink.vehicles[-1].lane + 1)%outlink.lanes
                        else:
                            veh.lane = 0

                        veh.leader = None
                        if len(outlink.vehicles) >= outlink.lanes:
                            veh.leader = outlink.vehicles[-outlink.lanes]
                            veh.leader.follower = veh
                            assert veh.leader.lane == veh.lane

                        outlink.vehicles.append(veh)

                        outlink.cum_arrival[-1] += s.W.DELTAN
                        veh.link_arrival_time = s.W.T*s.W.DELTAT

                        outlink.capacity_in_remain -= s.W.DELTAN
                    else:
                        #受入れ不可能ならこのノードから流入しない
                        break
                else:
                    break

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
        outlinks = []
        for outlink in {veh.route_next_link for veh in s.incoming_vehicles if veh.route_next_link != None}:
            for i in range(outlink.lanes):#車線の数だけ受け入れ試行回数あり
                outlinks.append(outlink)
        random.shuffle(outlinks)

        for outlink in outlinks: 
            if (len(outlink.vehicles) < outlink.lanes or outlink.vehicles[-outlink.lanes].x > outlink.delta_per_lane*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN and s.flow_capacity_remain >= s.W.DELTAN:
                #受け入れ可能かつ流出可能の場合，リンク優先度に応じて選択
                vehs = [
                    veh for veh in s.incoming_vehicles 
                    if veh == veh.link.vehicles[0] and #送り出しリンクで先頭車線の車両
                    veh.route_next_link == outlink and #行先リンクが受け入れリンク
                    (s.signal_phase in veh.link.signal_group or len(s.signal)<=1) and #信号が合致
                    veh.link.capacity_out_remain >= s.W.DELTAN
                ] 
                if len(vehs) == 0:
                    continue
                veh = random.choices(vehs, [veh.link.merge_priority for veh in vehs])[0] #車線の少ないリンクは，車線の多いリンクの試行回数の恩恵を受けて少し有利になる．大きな差はでないので許容する
                
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

                if len(outlink.vehicles) > 0:
                    veh.lane = (outlink.vehicles[-1].lane + 1)%outlink.lanes
                else:
                    veh.lane = 0
                
                veh.leader = None
                if len(outlink.vehicles) >= outlink.lanes:
                    veh.leader = outlink.vehicles[-outlink.lanes]
                    veh.leader.follower = veh
                    assert veh.leader.lane == veh.lane

                #走り残し処理
                x_next = veh.move_remain*outlink.u/inlink.u
                if veh.leader != None:
                    x_cong = veh.leader.x_old - veh.link.delta_per_lane*veh.W.DELTAN
                    if x_cong < veh.x:
                        x_cong = veh.x
                    if x_next > x_cong:
                        x_next = x_cong
                    if x_next >= outlink.length:
                        x_next = outlink.length
                veh.x = x_next

                #今移動した車両の後続車両がトリップ終了待ちの場合，トリップ終了させる
                if len(inlink.vehicles) and inlink.vehicles[0].flag_waiting_for_trip_end:
                    inlink.vehicles[0].end_trip()

                outlink.vehicles.append(veh)
                s.incoming_vehicles.remove(veh)

        #各リンクの先頭のトリップ終了待ち車両をトリップ終了させる
        for link in s.inlinks.values():
            for lane in range(link.lanes):
                if len(link.vehicles) and link.vehicles[0].flag_waiting_for_trip_end:
                    link.vehicles[0].end_trip()
                else:
                    break

        s.incoming_vehicles = []

    def update(s):
        """
        Make necessary updates when the timestep is incremented.
        """
        s.signal_control()
        s.flow_capacity_update()


class Link:
    """
    Link in a network.
    """
    def __init__(s, W, name, start_node, end_node, length, free_flow_speed=20, jam_density=0.2, jam_density_per_lane=None, number_of_lanes=1, merge_priority=1, signal_group=0, capacity_out=None, capacity_in=None, eular_dx=None, attribute=None, auto_rename=False):
        """
        Create a link

        Parameters
        ----------
        W : object
            The world to which the link belongs.
        name : str
            The name of the link.
        start_node : str
            The name of the start node of the link.
        end_node : str
            The name of the end node of the link.
        length : float
            The length of the link.
        free_flow_speed : float, optional
            The free flow speed on the link, default is 20.
        jam_density : float, optional  
            The jam density on the link, default is 0.2. If jam_density_per_lane is specified, this value is ignored.
        jam_density_per_lane : float, optional
            The jam density per lane on the link. If specified, it overrides the jam_density value.
        number_of_lanes : int, optional
            The number of lanes on the link, default is 1.
        merge_priority : float, optional
            The priority of the link when merging at the downstream node, default is 1.
        signal_group : int or list, optional
            The signal group(s) to which the link belongs, default is 0. If `signal_group` is int, say 0, it becomes green if `end_node.signal_phase` is 0. If `signal_group` is list, say [0,1], it becomes green if the `end_node.signal_phase` is 0 or 1.
        capacity_out : float, optional
            The capacity out of the link, default is calculated based on other parameters.
        capacity_in : float, optional
            The capacity into the link, default is calculated based on other parameters.
        eular_dx : float, optional
            The space aggregation size for link traffic state computation, default is 1/10 of link length or free flow distance per simulation step, whichever is larger.
        attribute : any, optional
            Additional (meta) attributes defined by users.
        auto_rename : bool, optional
            Whether to automatically rename the link if the name is already used. Default is False (raise an exception).

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
        Traffic Flow Model:
        - The link model follows a multi-lane, single-pipe approach where FIFO is guaranteed per link and no lane changing occurs.
        - Fundamental diagram parameters such as free_flow_speed, jam_density (or jam_density_per_lane), and number_of_lanes determine the link's flow characteristics. Reaction time of drivers `REACTION_TIME` is a grobal parameter.
        - Real-time link status for external reference is maintained with attributes `speed`, `density`, `flow`, `num_vehicles`, and `num_vehicles_queue`.

        Capacity and Bottlenecks:
        - The `capacity_out` and `capacity_in` parameters set the outflow and inflow capacities of the link. If not provided, the capacities are unlimited.
        - These capacities can represent bottlenecks at the beginning or end of the link.

        Connection to Node Model:
        - At the downstream end of a sending link, vehicles in all lanes have the right to be sent out, but FIFO order is maintained.
        - At the upstream end of a receiving link, all lanes can accept vehicles.

        Parameter Adjustments:
        - Some traffic flow model parameters like `free_flow_speed`, `jam_density`, `capacity_out`, `capacity_in`, and `merge_priority` can be altered during simulation to reflect changing conditions.
            
        Details on Multi-lane model:
        - Link model:
            - Multiple lanes with single-pipe model. FIFO is guaranteed per link. No lane changing.
            - Links have a `lanes` attribute representing the number of lanes. 
            - Each vehicle has a `lane` attribute.
            - Each vehicle follows the leader vehicle in the same lane, i.e., the vehicle `lanes` steps ahead on the link.
        - Node model: 
            - Sending links:
                - Vehicles in all lanes at the downstream end of the link have the right to be sent out.
                - However, to ensure link FIFO, vehicles are tried to be sent out in the order they entered the link. If a vehicle cannot be accepted, the outflow from that link stops.
            - Receiving links:  
                - All lanes at the upstream end of the link can accept vehicles.

        Details on Fundamental diagram parameters (*: input, **: alternative input):
        - *free_flow_speed (m/s)
        - *jam_density (veh/m/LINK)  
        - **jam_density_per_lane (veh/m/lane)
        - *lanes, number_of_lane (lane) 
        - tau: y-intercept of link FD (s/veh*LINK)
        - REACTION_TIME (s/veh*lane) 
        - w (m/s)
        - capacity (veh/s/LINK)
        - capacity_per_lane (veh/s/lane)
        - delta: minimum spacing (m/veh*LINK)
        - delta_per_lane: minimum spacing in lane (m/veh*lane) 
        - q_star: capacity (veh/s/LINK)
        - k_star: critical density (veh/s/LINK)
        - *capacity_in, capacity_out: bottleneck capacity at beginning/end of link (veh/s/LINK)
        - *Node.flow_capacity: node flow capacity (veh/s/LINK-LIKE) 
        """

        s.W = W
        #起点・終点ノード
        s.start_node = s.W.get_node(start_node)
        s.end_node = s.W.get_node(end_node)

        #リンク長
        s.length = length

        #車線数
        s.lanes = int(number_of_lanes)
        if s.lanes != number_of_lanes:
            raise ValueError(f"number_of_lanes must be an integer. Got {number_of_lanes} at {s}.")
        

        #フローモデルパラメータ:per link
        s.u = free_flow_speed
        s.kappa = jam_density
        if jam_density == 0.2 and jam_density_per_lane != None:
            s.kappa = jam_density_per_lane*number_of_lanes
        if jam_density != 0.2 and jam_density_per_lane != None:
            s.kappa = jam_density_per_lane*number_of_lanes
            warnings.warn(f"{s}: jam_density is ignored because jam_density_per_lane is set.", UserWarning)
        s.tau = s.W.REACTION_TIME/s.lanes
        s.w = 1/s.tau/s.kappa
        s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
        s.delta = 1/s.kappa
        s.delta_per_lane = s.delta*s.lanes #m/veh for each lane. used for car-following model per lane
        s.q_star = s.capacity   #flow capacity
        s.k_star = s.capacity/s.u   #critical density


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
            s.capacity_in = 10e10
            s.capacity_in_remain = 10e10
        else:
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
        #リンク流入出率を流出容量以下にするための処理．一タイムステップ当りに通り抜ける最大数を確保する
        if s.capacity_in != None:
            if s.capacity_out_remain < s.W.DELTAN*s.lanes:
                s.capacity_out_remain += s.capacity_out*s.W.DELTAT
            if s.capacity_in_remain < s.W.DELTAN*s.lanes:
                s.capacity_in_remain += s.capacity_in*s.W.DELTAT
        else:
            s.capacity_out_remain = 10e10
            s.capacity_in_remain = 10e10

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
        if departure_time_is_time_step:#互換性のため，departure_timeは常にタイムステップ表記 -> TODO: 要訂正！
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

        #走行車線
        s.lane = 0

        #先行・後行車
        s.leader = None
        s.follower = None

        #トリップ終了準備フラグ
        s.flag_waiting_for_trip_end = 0

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

        s.log_t_link = [[int(s.departure_time*s.W.DELTAT), "home"]] #新たなリンクに入った時にその時刻とリンクのみを保存．経路分析用

        s.attribute = attribute

        s.id = len(s.W.VEHICLES)
        if name != None:
            s.name = name
        else:
            s.name = str(s.id)
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
                    #トリップ終了待ちにする
                    s.flag_waiting_for_trip_end = 1
                    if s.link.vehicles[0] == s:
                        s.end_trip()
                elif len(s.link.end_node.outlinks.values()) == 0 and s.trip_abort == 1:
                    #トリップ終了待ち（目的地到達不可）にする
                    s.flag_trip_aborted = 1
                    s.route_next_link = None
                    s.flag_waiting_for_trip_end = 1
                    if s.link.vehicles[0] == s:
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
        s.arrival_time = s.W.T  #TODO: arrival_timeもタイムステップ表記．要修正
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
            x_cong = s.leader.x - s.link.delta_per_lane*s.W.DELTAN
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

    @catch_exceptions_and_warn()
    def show_network(W, width=1, left_handed=1, figsize=(6,6), network_font_size=10, node_size=6):
        """
        Visualizes the entire transportation network shape.
        """
        plt.rcParams["font.family"] = "monospace"
        if "MS Gothic" in plt.rcParams["font.family"]:
            plt.rcParams["font.family"] = "MS Gothic"

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