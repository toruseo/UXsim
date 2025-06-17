"""
UXsim: Macroscopic/mesoscopic traffic flow simulator in a network.
This `uxsim.py` is the core of UXsim. It summarizes the classes and methods that are essential for the simulation.
"""

import csv, time, math, string, warnings, copy
from collections import deque, OrderedDict
from collections import defaultdict as ddict
import warnings

import numpy as np
import matplotlib.pyplot as plt
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
import dill as pickle

from .analyzer import *
from .utils import *
from .scenario_reader_writer import *

class Node:
    """
    Node in a network.
    """
    def __init__(s, W, name, x, y, signal=[0], signal_offset=0, signal_offset_old=None, flow_capacity=None, auto_rename=False, number_of_lanes=None, attribute=None, user_attribute=None, user_function=None):
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
        signal_offset_old : float, optional
            The old parameter used to set offset of the signal prior to v1.8.1. This is the opposite of the usual definition of offset. Default is None, meaning it is `signal_offset` is used.
        flow_capacity : float, optional
            The maximum flow capacity of the node. Default is None, meaning infinite capacity.
        auto_rename : bool, optional
            Whether to automatically rename the node if the name is already used. Default is False.
        number_of_lanes : int, optional
            The number of lanes that can be green simultaniously at the node. Default is None.
        attribute : any, optional
            Additional (meta) attributes defined by users.
        user_attribute : any, optional
            Additional (meta) attributes defined by users. Same functionality to `attribute`, but with more understandable name.
        user_function : func, optinal
            User-defined custom function that is automatically called when timestep is incremented (more precisely, when `update()` is called). It takes only one argument: the Node object itself. Example: The following code prints the current number of incoming vehicles to the node at each timestep. If user_function=None (default), no functions will be executed.
            >>> def user_function(node):
            >>>     print(len(node.incoming_vehicles))
            >>> W = World(...)
            >>> W.addNode("node", 0, 0, user_function=user_function)
            >>> ... #define your scenario
            >>> W.exec_simulation()

        Attributes
        ----------
        signal_phase : int
            The phase of current signal. Links that have the same `signal_group` have a green signal.
        signal_t : float
            The elapsed time since the current signal phase started. When it is larger than `Link.signal[Link.signal_phase]`, the phase changes to the next one.
        """

        s.W = W
        #node position (for visualization)
        s.x = x
        s.y = y

        #custom attibutes
        s.attribute = attribute
        s.user_attribute = user_attribute
        s.user_function = user_function
        
        #incoming/outgoing links
        s.inlinks = dict()
        s.outlinks = dict()

        #request for inter-link transfer (demand for node model)
        s.incoming_vehicles = []

        #vertical queue for vehicle generation
        s.generation_queue = deque()

        #signal settings
        #If this node does not have a signal, set `signal=[0]`
        #If this node has a signal, set `signal=[green time for group0, green time for group1. ...]`
        s.signal = signal
        s.cycle_length = sum(s.signal)
        s.signal_phase = 0
        s.signal_t = 0
        s.signal_offset = signal_offset
        if signal_offset_old != None:
            s.signal_offset = s.cycle_length-signal_offset_old
        
        offset = s.cycle_length-s.signal_offset
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

        #flow capacity (macroscopic/continious representation for signal)
        s.flag_lanes_automatically_determined = False
        s.number_of_lanes = number_of_lanes
        if flow_capacity != None:
            s.flow_capacity = flow_capacity
            s.flow_capacity_remain = flow_capacity*s.W.DELTAT
            if number_of_lanes != None:
                s.number_of_lanes = number_of_lanes
            else:
                s.number_of_lanes = math.ceil(flow_capacity/0.8) #the number of lanes is determined by assuming 0.8 veh/s capacity per lane
                s.flag_lanes_automatically_determined = True
        else:
            s.flow_capacity = None
            s.flow_capacity_remain = 10e10
            s.number_of_lanes = None

        s.id = len(s.W.NODES)
        s.name = name
        s.auto_rename = auto_rename
        if s.name in s.W.NODES_NAME_DICT.keys():
            if auto_rename:
                s.name = s.name+"_renamed"+"".join(s.W.rng.choice(list(string.ascii_letters + string.digits), size=8))
            else:
                raise ValueError(f"Node name {s.name} already used by another node. Please specify a unique name.")
        s.W.NODES.append(s)
        s.W.NODES_NAME_DICT[s.name] = s
    
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
        s.cycle_length = sum(s.signal)

        s.signal_log.append(s.signal_phase)

    def flow_capacity_update(s):
        """
        flow capacity updates.
        """
        if s.flow_capacity != None:
            if s.flow_capacity_remain < s.W.DELTAN*s.number_of_lanes:
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
        outlinks0 = list(s.outlinks.values())
        if len(outlinks0):
            for i in range(sum([l.number_of_lanes for l in outlinks0])):
                if len(s.generation_queue) > 0:                    
                    veh = s.generation_queue[0]

                    #consider the link preferences
                    outlinks = list(s.outlinks.values())
                    if set(outlinks) & set(veh.links_prefer):
                        outlinks = sorted(set(outlinks) & set(veh.links_prefer), key=lambda l:l.name)
                    if set(outlinks) & set(veh.links_avoid):
                        outlinks = sorted(set(outlinks) - set(veh.links_avoid), key=lambda l:l.name)
                    
                    preference = np.array([veh.route_pref[l.id] for l in outlinks], dtype=float)
                    if s.W.hard_deterministic_mode == False:
                        if sum(preference) > 0:
                            outlink = s.W.rng.choice(outlinks, p=preference/sum(preference))
                        else:
                            outlink = s.W.rng.choice(outlinks)
                    else:
                        outlink = max(zip(preference, outlinks), key=lambda x:x[0])[1]

                    if (len(outlink.vehicles) < outlink.number_of_lanes or outlink.vehicles[-outlink.number_of_lanes].x > outlink.delta_per_lane*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN:
                        #受け入れ可能な場合，リンク優先度に応じて選択
                        veh = s.generation_queue.popleft()

                        veh.state = "run"
                        veh.link = outlink
                        veh.x = 0
                        veh.v = outlink.u #端部の挙動改善
                        s.W.VEHICLES_RUNNING[veh.name] = veh

                        if len(outlink.vehicles) > 0:
                            veh.lane = (outlink.vehicles[-1].lane + 1)%outlink.number_of_lanes
                        else:
                            veh.lane = 0

                        veh.leader = None
                        if len(outlink.vehicles) >= outlink.number_of_lanes:
                            veh.leader = outlink.vehicles[-outlink.number_of_lanes]
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
        outlink_candidates = {veh.route_next_link:0 for veh in s.incoming_vehicles if veh.route_next_link != None}
        for outlink in outlink_candidates.keys():
            for i in range(outlink.number_of_lanes):#車線の数だけ受け入れ試行回数あり
                outlinks.append(outlink)
        
        if s.W.hard_deterministic_mode == False:
            s.W.rng.shuffle(outlinks)

        for outlink in outlinks: 
            if (len(outlink.vehicles) < outlink.number_of_lanes or outlink.vehicles[-outlink.number_of_lanes].x > outlink.delta_per_lane*s.W.DELTAN) and outlink.capacity_in_remain >= s.W.DELTAN and s.flow_capacity_remain >= s.W.DELTAN:
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
                merge_priorities = np.array([veh.link.merge_priority for veh in vehs], dtype=float)
                if sum(merge_priorities) == 0:
                    merge_priorities = np.ones(len(merge_priorities))
                if s.W.hard_deterministic_mode == False:
                    veh = s.W.rng.choice(vehs, p=merge_priorities/sum(merge_priorities)) #車線の少ないリンクは，車線の多いリンクの試行回数の恩恵を受けて少し有利になる．大きな差はでないので許容する
                else:
                    veh = max(zip(merge_priorities, vehs), key=lambda x:x[0])[1]
                
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
                    veh.lane = (outlink.vehicles[-1].lane + 1)%outlink.number_of_lanes
                else:
                    veh.lane = 0
                
                veh.leader = None
                if len(outlink.vehicles) >= outlink.number_of_lanes:
                    veh.leader = outlink.vehicles[-outlink.number_of_lanes]
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
                veh.v += veh.x/s.W.DELTAT
                veh.move_remain = 0

                #今移動した車両の後続車両がトリップ終了待ちの場合，トリップ終了させる
                if len(inlink.vehicles) and inlink.vehicles[0].flag_waiting_for_trip_end:
                    inlink.vehicles[0].end_trip()

                outlink.vehicles.append(veh)
                s.incoming_vehicles.remove(veh)

        #各リンクの先頭のトリップ終了待ち車両をトリップ終了させる
        for link in s.inlinks.values():
            for lane in range(link.number_of_lanes):
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
        if s.user_function is not None:
            s.user_function(s)


class Link:
    """
    Link in a network.
    """
    def __init__(s, W, name, start_node, end_node, length, free_flow_speed=20, jam_density=0.2, jam_density_per_lane=None, number_of_lanes=1, merge_priority=1, signal_group=[0], capacity_out=None, capacity_in=None, eular_dx=None, attribute=None, user_attribute=None, user_function=None, auto_rename=False):
        """
        Create a link

        Parameters
        ----------
        W : object
            The world to which the link belongs.
        name : str
            The name of the link.
        start_node : str | Node
            The name of the start node of the link.
        end_node : str | Node
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
        user_attribute : any, optional
            Additional (meta) attributes defined by users. Same functionality to `attribute`, but with more understandable name.
        user_function : func, optinal
            User-defined custom function that is automatically called when timestep is incremented (more precisely, when `update()` is called). It takes only one argument: the Link object itself. Example: The following code prints the current number of vehicles on the link at each timestep. If user_function=None (default), no functions will be executed.
            >>> def user_function(link):
            >>>     print(len(link.vehicles))
            >>> W = World(...)
            >>> W.addLink("link", "node1", "node2, 1000, user_function=user_function)
            >>> ... #define your scenario
            >>> W.exec_simulation()
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

        Traffic Flow Model Parameters:

        - Their definition is illustrated as https://toruseo.jp/UXsim/docs/_images/fundamental_diagram.png
        - If you are not familiar to the traffic flow theory, it is recommended that you adjust only `free_flow_speed` and `number_of_lanes` for the traffic flow model parameters, leaving the other parameters at their default values.

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

        Details on Fundamental diagram parameters (+: input, ++: alternative input):

        - free_flow_speed (m/s)+
        - jam_density (veh/m/LINK)+
        - jam_density_per_lane (veh/m/lane)++
        - lanes, number_of_lane (lane)+
        - tau: y-intercept of link FD (s/veh*LINK)
        - REACTION_TIME, World.reaction_time (s/veh*lane) 
        - w (m/s)
        - capacity (veh/s/LINK)
        - capacity_per_lane (veh/s/lane)
        - delta: minimum spacing (m/veh*LINK)
        - delta_per_lane: minimum spacing in lane (m/veh*lane) 
        - q_star: capacity (veh/s/LINK)
        - k_star: critical density (veh/s/LINK)
        - capacity_in, capacity_out: bottleneck capacity at beginning/end of link (veh/s/LINK)+
        - Node.flow_capacity: node flow capacity (veh/s/LINK-LIKE)+
        """

        s.W = W
        #起点・終点ノード
        s.start_node = s.W.get_node(start_node)
        s.end_node = s.W.get_node(end_node)

        #リンク長
        s.length = length

        #車線数
        s.number_of_lanes = int(number_of_lanes)
        if s.number_of_lanes != number_of_lanes:
            raise ValueError(f"number_of_lanes must be an integer. Got {number_of_lanes} at {s}.")
        

        #フローモデルパラメータ:per link

        s.u = free_flow_speed
        s.kappa = jam_density
        if jam_density == 0.2 and jam_density_per_lane != None:
            s.kappa = jam_density_per_lane*number_of_lanes
        if jam_density != 0.2 and jam_density_per_lane != None:
            s.kappa = jam_density_per_lane*number_of_lanes
            warnings.warn(f"{s}: jam_density is ignored because jam_density_per_lane is set.", UserWarning)
        s.tau = s.W.REACTION_TIME/s.number_of_lanes
        s.w = 1/s.tau/s.kappa
        s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
        s.delta = 1/s.kappa
        s.delta_per_lane = s.delta*s.number_of_lanes #m/veh for each lane. used for car-following model per lane
        s.q_star = s.capacity   #flow capacity
        s.k_star = s.capacity/s.u   #critical density
        
        s.free_flow_speed = free_flow_speed
        s.jam_density = jam_density
        s.jam_density_per_lane = jam_density_per_lane


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
            s.capacity_in_remain = s.capacity*2
        else:
            s.capacity_in_remain = s.capacity_in*s.W.DELTAT

        s.id = len(s.W.LINKS)
        s.name = name
        s.auto_rename = auto_rename
        if s.name in s.W.LINKS_NAME_DICT.keys():
            if auto_rename:
                s.name = s.name+"_renamed"+"".join(s.W.rng.choice(list(string.ascii_letters + string.digits), size=8))
            else:
                raise ValueError(f"Link name {s.name} already used by another link. Please specify a unique name.")
        s.W.LINKS.append(s)
        s.W.LINKS_NAME_DICT[s.name] = s
        s.start_node.outlinks[s.name] = s
        s.end_node.inlinks[s.name] = s

        s.attribute = attribute
        s.user_attribute = user_attribute
        s.user_function = user_function


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
        s.ls = []
        s.names = []

        s.eular_dx = eular_dx
        if eular_dx == None:
            s.edie_dx = s.length/10
            if s.edie_dx < s.u*s.W.DELTAT:
                s.edie_dx = s.u*s.W.DELTAT


    def __repr__(s):
        return f"<Link {s.name}>"

    def init_after_tmax_fix(s):
        """
        Initalization before simulation execution.
        """

        #Euler型交通状態
        s.edie_dt = s.W.EULAR_DT
        s.edie_dx = s.edie_dx
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

        if s.user_function is not None:
            s.user_function(s)

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
            if s.capacity_out_remain < s.W.DELTAN*s.number_of_lanes:
                s.capacity_out_remain += s.capacity_out*s.W.DELTAT
            if s.capacity_in_remain < s.W.DELTAN*s.number_of_lanes:
                s.capacity_in_remain += s.capacity_in*s.W.DELTAT
        else:
            s.capacity_out_remain = 10e10
            s.capacity_in_remain = 10e10

    def set_traveltime_instant(s):
        """
        Compute instantaneous travel time.
        """
        if s.W.T%s.W.instantaneous_TT_timestep_interval == 0:
            if s.speed > 0:
                s.traveltime_instant.append(s.length/s.speed)
            else:
                s.traveltime_instant.append(s.length/(s.u/100))
        else:
            s.traveltime_instant.append(s.traveltime_instant[-1])


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
        Get instantaneous travel time of this link on time t

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            The instantaneous travel time.
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

    #Disabled temporally to avoid cirtical bugs related to scenario writer/reader. Instead, change_*() are added
    # @property
    # def free_flow_speed(s):
    #     return s.u

    # @free_flow_speed.setter
    # def free_flow_speed(s, new_value):
    #     if new_value >= 0:
    #         s.u = new_value
    #         s.w = 1/s.tau/s.kappa
    #         s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
    #         s.delta = 1/s.kappa
    #     else:
    #         warnings.warn(f"ignored negative free_flow_speed at {s}", UserWarning)

    # @property
    # def jam_density(s):
    #     return s.kappa

    # @jam_density.setter
    # def jam_density(s, new_value):
    #     if new_value >= 0:
    #         s.kappa = new_value
    #         s.w = 1/s.tau/s.kappa
    #         s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
    #         s.delta = 1/s.kappa
    #     else:
    #         warnings.warn(f"ignored negative jam_density at {s}", UserWarning)

    def change_free_flow_speed(s, new_value):
        if new_value >= 0:
            s.free_flow_speed = new_value
            s.u = new_value
            s.w = 1/s.tau/s.kappa
            s.capacity = s.u*s.w*s.kappa/(s.u+s.w)
            s.delta = 1/s.kappa
        else:
            warnings.warn(f"ignored negative free_flow_speed at {s}", UserWarning)

    def change_jam_density(s, new_value):
        if new_value >= 0:
            s.free_jam_density = new_value
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
    def __init__(s, W, orig, dest, departure_time, name=None, route_pref=None, route_choice_principle=None, mode="single_trip", links_prefer=[], links_avoid=[], trip_abort=1, departure_time_is_time_step=0, attribute=None, user_attribute=None, user_function=None, auto_rename=False):
        """
        Create a vehicle (more precisely, platoon)

        Parameters
        ----------
        W : object
            The world to which the vehicle belongs.
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
        mode : str, optional
            The mode of the vehicle. Available options are "single_trip" and "taxi", default is "single_trip".
            "single_trip": The vehicle makes a single trip from the origin to the destination.
            "taxi": The vehicle serves multiple trips by specifying sequence of destinations. The destination list `Vehicle.dest_list` can be dynamically updated externaly.
        links_prefer : list of str, optional
            The names of the links the vehicle prefers, default is empty list.
        links_avoid : list of str, optional
            The names of the links the vehicle avoids, default is empty list.
        trip_abort : int, optional
            Whether to abort the trip if a dead end is reached, default is 1.
        attribute : any, optinonal
            Additional (meta) attributes defined by users.
        user_attribute : any, optional
            Additional (meta) attributes defined by users. Same functionality to `attribute`, but with more understandable name.
        user_function : func, optinal
            User-defined custom function that is automatically called when timestep is incremented (more precisely, when `update()` is called). It takes only one argument: the Vehicle object itself. Example: The following code prints the current speed of vehicle at each timestep. If user_function=None (default), no functions will be executed.
            >>> def user_function(veh):
            >>>     print(veh.speed))
            >>> W = World(...)
            >>> ... #define your scenario
            >>> W.addVehicle("orig", "dest", 100, user_function=user_function)
            >>> W.exec_simulation()
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
        s.departure_time_in_second = departure_time*s.W.DELTAT  #TODO: temporal workaround
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
        s.move_remain = 0   #リンク移動後に走り続ける余力

        #経路選択
        if route_choice_principle == None:
            s.route_choice_principle = s.W.route_choice_principle
        else:
            s.route_choice_principle = route_choice_principle

        #private vehicle or taxi
        s.mode = mode
        s.dest_list = []

        #dict of events that are triggered when this vehicle reaches a certain node {Node: func}
        s.node_event = dict()

        #希望リンク重み：{link.id:重み}
        if route_pref == None:
            if s.W.route_pref_for_vehs == None:
                s.W.route_pref_for_vehs = {l.id:0 for l in s.W.LINKS}
            s.route_pref = copy.copy(s.W.route_pref_for_vehs)
        else:
            s.route_pref = {l.id:route_pref[l] for l in route_pref.keys()}

        #these links will be always chosen or not chosen when choosing next link at each node
        s.links_prefer = [s.W.get_link(l) for l in links_prefer]
        s.links_avoid = [s.W.get_link(l) for l in links_avoid]

        #行き止まりに行ってしまったときにトリップを止める
        s.trip_abort = trip_abort
        s.flag_trip_aborted = 0

        #log
        s.log_t = [] #時刻
        s.log_state = [] #状態
        s.log_link = [] #リンク
        s.log_x = [] #位置
        s.log_s = [] #車頭距離
        s.log_v = [] #現在速度
        s.log_lane = [] #車線
        s.color = (s.W.rng.random(), s.W.rng.random(), s.W.rng.random())

        s.log_t_link = [[int(s.departure_time*s.W.DELTAT), "home"]] #route-level log. It records the time and link when the vehicle entered new link
        s.link_old = None
        
        s.distance_traveled = 0

        s.attribute = attribute
        s.user_attribute = user_attribute
        s.user_function = user_function

        s.id = len(s.W.VEHICLES)
        if name != None:
            s.name = name
        else:
            s.name = str(s.id)
        if s.name in s.W.VEHICLES.keys():
            if auto_rename:
                s.name = s.name+"_renamed"+"".join(s.W.rng.choice(list(string.ascii_letters + string.digits), size=8))
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
            #depart
            if s.W.T >= s.departure_time:
                s.state = "wait"
                s.orig.generation_queue.append(s)
        if s.state == "wait":
            #wait at the vertical queue at the origin node
            if s.W.route_choice_update_gradual:
                s.route_pref_update()
            pass
        if s.state == "run":
            #drive within the link
            s.v = (s.x_next-s.x)/s.W.DELTAT
            s.x_old = s.x
            s.x = s.x_next

            #at the end of the link
            if s.x == s.link.length:
                if s.link.end_node in s.node_event.keys():
                    s.node_event[s.link.end_node]()
                
                if s.W.route_choice_update_gradual:
                    s.route_pref_update()
                
                if s.link.end_node == s.dest:
                    if s.mode == "single_trip":
                        #prepare for trip end
                        s.flag_waiting_for_trip_end = 1
                        if s.link.vehicles[0] == s:
                            s.end_trip()
                    elif s.mode == "taxi":
                        #proceed to next destination
                        if len(s.dest_list) > 0:
                            s.dest = s.dest_list.pop(0)
                        else:
                            s.dest = None
                            s.dest_list = []
                        s.route_pref_update(weight=1)
                        s.route_next_link_choice()
                        s.link.end_node.incoming_vehicles.append(s)

                elif len(s.link.end_node.outlinks.values()) == 0 and s.trip_abort == 1:
                    #prepare for trip abort due to dead end
                    s.flag_trip_aborted = 1
                    s.route_next_link = None
                    s.flag_waiting_for_trip_end = 1
                    if s.link.vehicles[0] == s:
                        s.end_trip()

                else:
                    #request link transfer
                    s.route_next_link_choice()
                    s.link.end_node.incoming_vehicles.append(s)
        
        if s.state in ["end", "abort"] :
            #ended the trip
            pass

        if s.user_function is not None:
            s.user_function(s)

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

        s.record_log(enforce_log=1)

        if s.W.reduce_memory_delete_vehicle_route_pref:
            s.route_pref = None

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
    
    def enforce_route(s, route, set_avoid=False):
        """
        Enforce the vehicle to use the specified route. The route should connect the origin to the destination. TODO: add consistency check

        Parameters
        ----------
        route : List of Link
            The route for this vehicle.
        set_avoid : bool
            If True, the vehicle only travel the links in `route` argument. This is very strict. If the route is not consistent (e.g., the route does not completely connect between the origin and the destination), it will raise an exception.
        """
        s.links_prefer = [s.W.get_link(l) for l in route]
        if set_avoid:
            s.links_avoid = [s.W.get_link(l) for l in s.W.LINKS if l not in s.links_prefer]

    def route_pref_update(s, weight=1):
        """
        Updates the vehicle's link preferences for route choice.

        Parameters
        ----------
        weight : float
            The weight for updating the link preferences based on the recent travel time.
            Should be in the range [0, 1], where 0 means the old preferences are fully retained and 1 means the preferences are completely updated. THIS IS DISABLED FOR NOW.

        Notes
        -----
        This method updates the link preferences used by the vehicle to select its route based on its current understanding of the system.

        - If the vehicle's route choice principle is "homogeneous_DUO", it will update its preferences based on a global, homogenous dynamic user optimization (DUO) model.
        - If the route choice principle is "heterogeneous_DUO", it will update its preferences based on a heterogeneous DUO model, considering both its past preferences and the system's current state. This is imcomplete feature. Not recommended.

        The updated preferences guide the vehicle's decisions in subsequent route choices.
        """
        if s.route_choice_principle == "homogeneous_DUO":
            if s.dest != None:
                s.route_pref = s.W.ROUTECHOICE.route_pref[s.dest.id]
            else:
                s.route_pref = {l.id:0 for l in s.W.LINKS}
        elif s.route_choice_principle == "heterogeneous_DUO":
            route_pref_new = {l.id:0 for l in s.W.LINKS}
            k = s.dest.id
            for l in s.W.LINKS:
                i = l.start_node.id
                j = l.end_node.id
                if j == s.W.ROUTECHOICE.next[i,k]:
                    route_pref_new[l.id] = 1

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

                #if links_prefer is given and available at the node, select only from the links in the list. if links_avoid is given, select links not in the list.
                if set(outlinks) & set(s.links_prefer):
                    outlinks = sorted(set(outlinks) & set(s.links_prefer), key=lambda l:l.name)
                if set(outlinks) & set(s.links_avoid):
                    outlinks = sorted(set(outlinks) - set(s.links_avoid), key=lambda l:l.name)

                preference = np.array([s.route_pref[l.id] for l in outlinks], dtype=float)
                if s.W.hard_deterministic_mode == False:
                    if sum(preference) > 0:
                        s.route_next_link = s.W.rng.choice(outlinks, p=preference/sum(preference))
                    else:
                        s.route_next_link = s.W.rng.choice(outlinks)
                else:
                    s.route_next_link = max(zip(preference, outlinks), key=lambda x:x[0])[1]

            else:
                s.route_next_link = None

    def add_dest(s, dest, order=-1):
        """
        Add a destination to the vehicle's destination list.

        Parameters
        ----------
        dest : str | Node
            The destination node to be added.
        order : int, optional
            The order of the destination in the list. Default is -1, which appends the destination to the end of the list.
        """
        if s.mode == "taxi":
            if s.dest == None:
                s.dest = dest
                s.route_pref_update(weight=1)
            else:
                if order == -1:
                    s.dest_list.append(s.W.get_node(dest))
                else:
                    s.dest_list.insert(order, s.W.get_node(dest))
        else:
            raise ValueError(f"Vehicle {s.name} is not in taxi mode. Cannot add destination.")
    
    def set_links_prefer(s, links):
        """
        Set the links the vehicle prefers.

        Parameters
        ----------
        links : list of str
            The list of link names the vehicle prefers.
        """
        s.links_prefer = [s.W.get_link(l) for l in links]
    
    def set_links_avoid(s, links):
        """
        Set the links the vehicle avoids.

        Parameters
        ----------
        links : list of str
            The list of link names the vehicle avoids.
        """
        s.links_avoid = [s.W.get_link(l) for l in links]

    def add_dests(s, dests):
        """
        Add multiple destinations to the vehicle's destination list.

        Parameters
        ----------
        dests : list of str | Node
            The list of destinations to be added.
        """
        for dest in dests:
            s.add_dest(dest)
    
    def traveled_route(s, include_arrival_time=True, include_departure_time=False):
        """
        Returns the route this vehicle traveled.

        Parameters
        ----------
        include_arrival_time : bool
            If true, return the arrival time to the destination as well.  `-1` means it did not reach the destination.
        include_departure_time : bool
            If true, return the departure time from the origin as well. It will be different from the entering time to the first link if there are congestion (i.e., the vehicle need to enter the network). 

        Returns
        -------
        Route
            The route this vehicle traveled.
        list
            The time at which the vehicle entered each link. If `include_arrival_time` is true, the last element is the time the vehicle reached the destination. If `include_departure_time` is true, the first element is the time the vehicle departed from the origin. This complexity is actually due to a design failure in the past. These options are added to keep the backward compatibility.
        """
        # link_old = -1
        # t = -1
        # route = []
        # ts = []
        # for i, link in enumerate(s.log_link):
        #     if link_old != link:
        #         route.append(link)
        #         ts.append(s.log_t[i])
        #         link_old = link

        # return Route(s.W, route[:-1]), ts

        route = []
        ts = []

        for log in s.log_t_link:
            t = log[0]
            l = log[1]
            if l == "home" and include_departure_time:
                ts.append(t)
            if type(l) == Link:
                ts.append(t)
                route.append(l)
        
        log = s.log_t_link[-1]
        t = log[0]
        l = log[1]
        if include_arrival_time:
            if l == "end":
                ts.append(t)
            else:
                ts.append(-1)
        
        return Route(s.W, route), ts

    def get_xy_coords(s, t=-1):
        """
        Get the x-y coordinates of the vehicle. If t is given, the position at time t is returned based on the logs.

        Parameters
        ----------
        t : int | float, optional
            Time in seconds. If it is -1, the latest position is returned.
        """
        if t != -1:
            link = s.log_link[int(t/s.W.DELTAT/s.W.vehicle_logging_timestep_interval)]
            xx = s.log_x[int(t/s.W.DELTAT/s.W.vehicle_logging_timestep_interval)]
        else:
            link = s.link
            xx = s.x
        if link == -1:
            return (-1, -1)
        link = s.W.get_link(link)
        x0 = link.start_node.x
        y0 = link.start_node.y
        x1 = link.end_node.x
        y1 = link.end_node.y
        x = x0 + (x1-x0)*xx/link.length
        y = y0 + (y1-y0)*xx/link.length
        return (x, y)

    def record_log(s, enforce_log=0):
        """
        Record travel logs.

        Parameters
        ----------
        enforce_log : bool, optional
            Record log regardless of the logging interval, default is 0.
        """
        if s.W.vehicle_logging_timestep_interval != -1:
            if s.W.vehicle_logging_timestep_interval == 1 or s.W.T%s.W.vehicle_logging_timestep_interval == 0 or enforce_log:
                if s.state != "run":
                    # if s.state == "end" and s.log_t_link[-1][1] != "end":
                    #     s.log_t_link.append([t, "end"])

                    s.log_t.append(s.W.T*s.W.DELTAT)
                    s.log_state.append(s.state)
                    s.log_link.append(-1)
                    s.log_x.append(-1)
                    s.log_s.append(-1)
                    s.log_v.append(-1)
                    s.log_lane.append(-1)

                    if s.state == "wait":
                        s.W.analyzer.average_speed_count += 1
                        s.W.analyzer.average_speed += 0
                else:
                    # if len(s.log_link) == 0 or s.log_link[-1] != s.link:
                    #     s.log_t_link.append([t, s.link])

                    s.log_t.append(s.W.T*s.W.DELTAT)
                    s.log_state.append(s.state)
                    s.log_link.append(s.link)
                    s.log_x.append(s.x)
                    s.log_v.append(s.v)
                    s.log_lane.append(s.lane)
                    if s.leader != None and s.link == s.leader.link:
                        s.log_s.append(s.leader.x-s.x)
                    else:
                        s.log_s.append(-1)

                    s.W.analyzer.average_speed_count += 1
                    s.W.analyzer.average_speed += (s.v - s.W.analyzer.average_speed)/s.W.analyzer.average_speed_count
        
        if s.link != s.link_old:
            if s.state == "run":
                s.log_t_link.append([s.W.T*s.W.DELTAT, s.link])
            elif s.state == "end":
                s.log_t_link.append([s.W.T*s.W.DELTAT, "end"])
            s.link_old = s.link


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
        #リンク旅行時間行列: adjacency matrix whose cost is link travel cost
        s.adj_mat_time = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        #ij間最短距離: the shortest path cost (based on the current instantaneous travel time) from node i to j
        s.dist = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        #iからjに行くために次に進むべきノード: the node to proceed from i when the destination is j
        s.next = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        #iからjに行くために来たノード. This is not used anymore
        s.pred = np.zeros([len(s.W.NODES), len(s.W.NODES)])

        s.dist_record = {}

        #homogeneous DUO用．kに行くための最短経路的上にあれば1: array[dest,link]==1 if link is on the shortest path to dest
        #s.route_pref = {k.id: {l:0 for l in s.W.LINKS} for k in s.W.NODES} #old definition, {node_id: {link: preference}. preference of link is 1 if it is on the shortest path to node_id
        s.route_pref = np.zeros([len(W.NODES), len(W.LINKS)])

    def route_search_all(s, infty=np.inf, noise=0):
        """
        Compute the current shortest path based on instantaneous travel time.

        Parameters
        ----------
        infty : float
            value representing infinity.
        noise : float
            very small noise to slightly randomize route choice. useful to eliminate strange results at an initial stage of simulation where many routes has identical travel time.
        """
        s.adj_mat_time = np.zeros([len(s.W.NODES), len(s.W.NODES)])
        adj_mat_link_count = np.zeros([len(s.W.NODES), len(s.W.NODES)])

        for link in s.W.LINKS:
            i = link.start_node.id
            j = link.end_node.id
            if s.W.ADJ_MAT[i,j]:
                if s.W.hard_deterministic_mode == False:
                    new_link_tt = link.traveltime_instant[-1]*s.W.rng.uniform(1, 1+noise) + link.route_choice_penalty
                else:
                    new_link_tt = link.traveltime_instant[-1] + link.route_choice_penalty
                n = adj_mat_link_count[i,j]
                s.adj_mat_time[i,j] = s.adj_mat_time[i,j]*n/(n+1) + new_link_tt/(n+1) # if there are multiple links between the same nodes, average the travel time
                # s.adj_mat_time[i,j] = new_link_tt #if there is only one link between the nodes, this line is fine, but for generality we use the above line
                adj_mat_link_count[i,j] += 1
                if link.capacity_in == 0: #if the inflow is profibited, travel time is assumed to be infinite
                    s.adj_mat_time[i,j] = np.inf
            else:
                s.adj_mat_time[i,j] = np.inf
        
        #computes the shortest path from *destination* to *origin*, so that the pred_matrix becomes the next_matrix in the original problem. It is simply achieved by tranposing the matrices twice.
        dist, pred = dijkstra(csr_matrix(s.adj_mat_time).T, return_predecessors=True)
        s.dist = dist.T
        s.next = pred.T

        s.dist_record[s.W.T] = s.dist

    def homogeneous_DUO_update(s):
        """
        Update link preference of all homogeneous travelers based on DUO principle.
        Vectorized by Claude 3.5 Sonnet.
        """
        if s.W.route_choice_update_gradual:
            weight0 = s.W.DUO_UPDATE_WEIGHT * (s.W.DELTAT / s.W.DUO_UPDATE_TIME)
        else:
            weight0 = s.W.DUO_UPDATE_WEIGHT

        num_nodes = len(s.W.NODES)
        num_links = len(s.W.LINKS)

        # Create a mask for empty preferences
        empty_pref_mask = np.sum(s.route_pref, axis=1) == 0

        # Initialize weight array
        weights = np.full(num_nodes, weight0)
        weights[empty_pref_mask] = 1

        # Create arrays for start and end nodes of links
        start_nodes = np.array([l.start_node.id for l in s.W.LINKS])
        end_nodes = np.array([l.end_node.id for l in s.W.LINKS])

        # Create the next_node_mask
        next_node_mask = np.zeros((num_nodes, num_links), dtype=bool)
        for k in range(num_nodes):
            next_node_mask[k] = end_nodes == s.next[start_nodes, k]

        # Update route preferences
        # In-place scaling of all elements
        s.route_pref *= (1 - weights[:, np.newaxis])

        # In-place addition of weights where next_node_mask is True
        s.route_pref += weights[:, np.newaxis] * next_node_mask


class World:
    """
    World (i.e., simulation environment). A World object is consistently referred to as `W` in this code.
    """

    def __init__(W, name="", deltan=5, reaction_time=1, 
                 duo_update_time=600, duo_update_weight=0.5, duo_noise=0.01, route_choice_principle="homogeneous_DUO", route_choice_update_gradual=False, instantaneous_TT_timestep_interval=5, 
                 eular_dt=120, eular_dx=100, 
                 random_seed=None, 
                 print_mode=1, save_mode=1, show_mode=0, show_progress=1, show_progress_deltat=600, 
                 tmax=None, 
                 vehicle_logging_timestep_interval=1, 
                 reduce_memory_delete_vehicle_route_pref=False,
                 hard_deterministic_mode=False, 
                 meta_data={}, user_attribute=None, user_function=None):
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
        route_choice_update_gradual : bool, optional
            Whether to update route choice ratio gradually or not. True is recommended. Default is False for backward compatibility.
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
        vehicle_logging_timestep_interval : int, optional
            The interval for logging vehicle data, default is 1. Logging is off if set to -1.
            Setting large intervel (2 or more) or turn off the logging makes the simulation significantly faster in large-scale scenarios without loosing simulation internal accuracy, but outputed vehicle trajecotry and other related data will become inaccurate.
        instantaneous_TT_timestep_interval : int, optional
            The interval for computing instantaneous travel time of each link. Default is 5.
            If it is longer than the DUO update timestep interval, it is substituted by DUO update timestep interval to maintain reasonable route choice behavior.
        hard_deterministic_mode : bool, optional
            If True, the simulation will not use any random variables. At a merging node, a link with higher merge_priority will be always prioritized, and vehicles always choose the shortest path. This may be useful for analysis that need strict predictability. Be aware that the simulation results will be significantly different from ones with `hard_deterministic_mode=False`.
        reduce_memory_delete_vehicle_route_pref : bool, optional
            If True, the simulation will delete the route preference of vehicles after its ends. This is useful when the route preference is not needed after the simulation ends.
        meta_data : dict, optinal
            Meta data for simulation scenario. Can store arbitrary data, such as licences and simulation explanation.
        user_attribute : any, optinal
            Optinonal meta attributes that can be freely defined by a user.
        user_function : func, optinal
            User-defined custom function that is automatically called when timestep is incremented (more precisely, just before timesptep is incremented). It takes only one argument: the World object itself. Example: The following code prints the current simulation time at each timestep. If user_function=None (default), no functions will be executed.
            >>> def user_function(W):
            >>>     print(W.TIME)
            >>> W = World(user_function=user_function)
            >>> ... #define your scenario
            >>> W.exec_simulation()

        Notes
        -----
        A World object must be defined firstly to initiate simulation.
        """

        ## parameter setting
        
        W.rng = np.random.default_rng(seed=random_seed)
        W.random_seed = random_seed

        W.TMAX = tmax   #simulation time (s)

        W.DELTAN = deltan         #platoon size (veh)
        W.REACTION_TIME = reaction_time         #reaction time = simulation time step size (s)

        W.DUO_UPDATE_TIME = duo_update_time     #time interval for route choice update (s)
        W.DUO_UPDATE_WEIGHT = duo_update_weight    #weight for route choice update
        W.DUO_NOISE = duo_noise    #very small noise for route choice to avoid singular results
        W.EULAR_DT = eular_dt     #time discretization size for Eular-type data (aggregated traffic state)

        W.DELTAT = W.REACTION_TIME*W.DELTAN
        W.DELTAT_ROUTE = int(W.DUO_UPDATE_TIME/W.DELTAT) #this unit is timestep
        if W.DELTAT_ROUTE == 0:
            W.DELTAT_ROUTE = 1

        ## data storage
        W.VEHICLES = OrderedDict()            #home, wait, run, end
        W.VEHICLES_LIVING = OrderedDict()     #home, wait, run
        W.VEHICLES_RUNNING = OrderedDict()    #run
        W.NODES = []
        W.LINKS = []

        W.NODES_NAME_DICT = {} #map from name to node object
        W.LINKS_NAME_DICT = {}

        W.vehicle_logging_timestep_interval = vehicle_logging_timestep_interval

        W.route_choice_principle = route_choice_principle

        W.route_choice_update_gradual = route_choice_update_gradual

        W.instantaneous_TT_timestep_interval = int(instantaneous_TT_timestep_interval)
        if W.DELTAT_ROUTE < W.instantaneous_TT_timestep_interval:
            W.instantaneous_TT_timestep_interval = W.DELTAT_ROUTE

        W.route_pref_for_vehs = None

        ## progress print setting
        W.show_progress = show_progress
        W.show_progress_deltat_timestep = int(show_progress_deltat/W.DELTAT)

        ## memory setting
        W.reduce_memory_delete_vehicle_route_pref = reduce_memory_delete_vehicle_route_pref

        ## system setting
        W.name = name

        W.hard_deterministic_mode = hard_deterministic_mode

        W.meta_data = meta_data
        W.network_info = ddict(list)
        W.demand_info = ddict(list)

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

        W.user_attribute = user_attribute
        W.user_function = user_function

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
        number_of_lanes : int, optional
            The number of lanes that can be green simultaniously at the node. Default is None.

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
            The name of the start node of the link.
        end_node : str | Node
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

    def addVehicle(W, *args, direct_call=True, **kwargs):
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
        mode : str, optional
            The mode of the vehicle. Available options are "single_trip" and "taxi", default is "single_trip".
            "single_trip": The vehicle makes a single trip from the origin to the destination.
            "taxi": The vehicle serves multiple trips by specifying sequence of destinations. The destination list `Vehicle.dest_list` can be dynamically updated externaly.
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

    @demand_info_record
    def adddemand(W, orig, dest, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
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
                W.addVehicle(orig, dest, t, departure_time_is_time_step=1, attribute=attribute, direct_call=False)
                f -= W.DELTAN

        

    @demand_info_record
    def adddemand_point2point(W, x_orig, y_orig, x_dest, y_dest, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
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
        W.adddemand(orig, dest, t_start, t_end, flow, volume, attribute, direct_call=False)

    @demand_info_record
    def adddemand_area2area(W, x_orig, y_orig,  radious_orig, x_dest, y_dest, radious_dest, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        """
        Generate vehicles by specifying time-dependent origin-destination demand by specifying circular areas. `adddemand_area2area()` is not recommended as it may truncate demand. Consider to use new `adddemand_area2area2()` which is more accurate, smooth, and fast.

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
        warnings.warn(
            "`adddemand_area2area()` is not recommended as it may truncate demand. Consider to use new `adddemand_area2area2()` which is more accurate, smooth, and fast.",
            FutureWarning
        )

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
                W.adddemand(o, d, t_start, t_end, flow, volume, attribute, direct_call=False)
    
    @demand_info_record
    def adddemand_nodes2nodes(W, origs, dests, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        """
        Generate vehicles by specifying time-dependent origin-destination demand by specifying origin area (i.e., list of nodes) and destination one. `adddemand_nodes2nodes()` is not recommended as it may truncate demand. Consider to use new `adddemand_nodes2nodes2() which is more accurate, smooth, and fast.

        Parameters
        ----------
        origs : list
            The list of origin nodes. The items can be Node objects or names of Nodes.
        dests : list
            The list of destination nodes. The items can be Node objects or names of Nodes.
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
        warnings.warn(
            "`adddemand_nodes2nodes()` is not recommended as it may truncate demand. Consider to use new `adddemand_nodes2nodes2() which is more accurate, smooth, and fast.`",
            FutureWarning
        )

        origs_new = []
        dests_new = []
        for oo in origs:
            o = W.get_node(oo)
            if len(o.outlinks) != 0:
                origs_new.append(o)
        for dd in dests:
            d = W.get_node(dd)
            if len(d.inlinks) != 0:
                dests_new.append(d)
        origs = origs_new
        dests = dests_new
        
        if flow != -1:
            flow = flow/(len(origs)*len(dests))
        if volume != -1:
            volume = volume/(len(origs)*len(dests))
        for o in origs:
            for d in dests:
                W.adddemand(o, d, t_start, t_end, flow, volume, attribute, direct_call=False)

    @demand_info_record
    def adddemand_area2area2(W, x_orig, y_orig,  radious_orig, x_dest, y_dest, radious_dest, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        """
        Generate vehicles by specifying time-dependent origin-destination demand by specifying circular areas. This is new version of `adddemand_area2area`, more efficient, more smooth, and more accurate. However, it introduces some randomness.

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

        if flow >= 0 and volume == -1:
            volume = flow*(t_end-t_start)
        size = int(volume/W.DELTAN)

        origs_new = []
        dests_new = []
        for oo in origs:
            o = W.get_node(oo)
            if len(o.outlinks) != 0:
                origs_new.append(o)
        for dd in dests:
            d = W.get_node(dd)
            if len(d.inlinks) != 0:
                dests_new.append(d)
        origs = origs_new
        dests = dests_new
        if len(origs) == 0:
            origs.append(W.get_nearest_node(x_orig, y_orig))
        if len(dests) == 0:
            dests.append(W.get_nearest_node(x_dest, y_dest))

        ts = np.linspace(t_start, t_end, size)
        
        os = W.rng.choice(origs, size=size)
        ds = W.rng.choice(dests, size=size)
        
        for t, o, d, in zip(ts, os, ds):
            d2 = d
            if o == d:
                d2 = W.rng.choice([dd for dd in dests if dd!=d])
            W.addVehicle(o, d2, t, attribute=attribute, direct_call=False)


    @demand_info_record
    def adddemand_nodes2nodes2(W, origs, dests, t_start, t_end, flow=-1, volume=-1, attribute=None, direct_call=True):
        """
        Generate vehicles by specifying time-dependent origin-destination demand by specifying origin area (i.e., list of nodes) and destination one. This is new version of `adddemand_nodes2nodes`, more efficient, more smooth, and more accurate. However, it introduces some randomness.

        Parameters
        ----------
        origs : list
            The list of origin nodes. The items can be Node objects or names of Nodes.
        dests : list
            The list of destination nodes. The items can be Node objects or names of Nodes.
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
        
        if flow >= 0 and volume == -1:
            volume = flow*(t_end-t_start)
        size = int(volume/W.DELTAN)

        origs_new = []
        dests_new = []
        for oo in origs:
            o = W.get_node(oo)
            if len(o.outlinks) != 0:
                origs_new.append(o)
        for dd in dests:
            d = W.get_node(dd)
            if len(d.inlinks) != 0:
                dests_new.append(d)
        origs = origs_new
        dests = dests_new
        ts = np.linspace(t_start, t_end, size)
        
        os = W.rng.choice(origs, size=size)
        ds = W.rng.choice(dests, size=size)
        
        for t, o, d, in zip(ts, os, ds):
            d2 = d
            if o == d:
                d2 = W.rng.choice([dd for dd in dests if dd!=d])
            W.addVehicle(o, d2, t, attribute=attribute, direct_call=False)

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

        #generate adjacency matrix
        W.ROUTECHOICE = RouteChoice(W)
        W.ADJ_MAT = np.zeros([len(W.NODES), len(W.NODES)])
        W.ADJ_MAT_LINKS = dict() #リンクオブジェクトが入った隣接行列（的な辞書）
        W.NODE_PAIR_LINKS = dict() #リンクオブジェクトが入った隣接行列（的な辞書）．キーはノード名
        for link in W.LINKS:
            for i,start_node in enumerate(W.NODES):
                if start_node == link.start_node:
                    break
            for j,end_node in enumerate(W.NODES):
                if end_node == link.end_node:
                    break
            W.ADJ_MAT[i,j] = 1
            W.ADJ_MAT_LINKS[i,j] = link
            W.NODE_PAIR_LINKS[start_node.name,end_node.name] = link

        W.analyzer = Analyzer(W)

        W.finalized = 1

        ## 問題規模表示
        if W.print_mode:
            W.print_scenario_stats()

        W.sim_start_time = time.time()
        W.print("simulating...")

    def print_scenario_stats(W):
        """
        Print scenario stats
        """
        if W.finalized:
            print("simulation setting:")
        else:
            print("simulation setting (not finalized):")
        print(" scenario name:", W.name)
        print(" simulation duration:\t", W.TMAX, "s")
        print(" number of vehicles:\t", len(W.VEHICLES)*W.DELTAN, "veh")
        print(" total road length:\t", sum([l.length for l in W.LINKS]),"m")
        print(" time discret. width:\t", W.DELTAT, "s")
        print(" platoon size:\t\t", W.DELTAN, "veh")
        if W.finalized:
            print(" number of timesteps:\t", W.TSIZE)
        else:
            print(" number of timesteps:\t", W.TMAX/W.DELTAT)
        print(" number of platoons:\t", len(W.VEHICLES))
        print(" number of links:\t", len(W.LINKS))
        print(" number of nodes:\t", len(W.NODES))
        print(" setup time:\t\t", f"{time.time()-W.world_start_time:.2f}", "s")

    def exec_simulation(W, until_t=None, duration_t=None, duration_t2=None):
        """
        Execute the main loop of the simulation.

        Parameters
        ----------
        until_t : float or None, optional
            The time until the simulation is to be executed in seconds. 
            If all of `until_t` and `duration_t` and `duration_t2` are None, the simulation is executed until the end. Default is None.
        duration_t : float or None, optional
            The duration for which the simulation is to be executed in seconds. Simulation runs `duration_t+1` seconds for a technical reason. Old setting, not recommended.
            If all of `until_t` and `duration_t` and `duration_t2` are None, the simulation is executed until the end. Default is None. Recommended. 
        duration_t2 : float or None, optional
            The duration for which the simulation is to be executed in seconds. Simulation runs `duration_t2` seconds.
            If all of `until_t` and `duration_t` and `duration_t2` are None, the simulation is executed until the end. Default is None. 

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

        #finalize the scenario if it has not been done
        if W.finalized == 0:
            W.finalize_scenario()

        #determine the simulation time
        start_ts = W.T
        if until_t != None:
            end_ts = int(until_t/W.DELTAT)
        elif duration_t2 != None:
            end_ts = start_ts + int(duration_t2/W.DELTAT)-1
        elif duration_t != None:
            end_ts = start_ts + int(duration_t/W.DELTAT)
        else:
            end_ts = W.TSIZE
        if end_ts > W.TSIZE:
            end_ts = W.TSIZE

        if start_ts == end_ts == W.TSIZE:
            if W.print_mode and W.show_progress:
                W.analyzer.show_simulation_progress()
            W.simulation_terminated()
            return 1 #end of simulation
        if end_ts < start_ts:
            raise Exception("exec_simulation error: Simulation duration is not positive. Check until_t or duration_t or duration_t2")

        #the main loop
        #print("preping:", W.T, start_ts, end_ts, W.check_simulation_ongoing())
        for W.T in range(start_ts, end_ts+1):
            #print("execing:", W.T, start_ts, end_ts, W.check_simulation_ongoing())
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

            if W.route_choice_update_gradual == True:
                if W.T % W.DELTAT_ROUTE == 0:
                    W.ROUTECHOICE.route_search_all(noise=W.DUO_NOISE)
                W.ROUTECHOICE.homogeneous_DUO_update()
            else:
                if W.T % W.DELTAT_ROUTE == 0:
                    W.ROUTECHOICE.route_search_all(noise=W.DUO_NOISE)
                    W.ROUTECHOICE.homogeneous_DUO_update()
                    for veh in W.VEHICLES_LIVING.values():
                        veh.route_pref_update(weight=W.DUO_UPDATE_WEIGHT)

            W.TIME = W.T*W.DELTAT

            if W.print_mode and W.show_progress and W.T%W.show_progress_deltat_timestep == 0 and W.T > 0:
                W.analyzer.show_simulation_progress()
            
            if W.user_function is not None:
                W.user_function(W)

            if W.T == W.TSIZE-1:
                #print("break")
                break

        W.T += 1
        W.TIME = W.T*W.DELTAT

        if W.T == W.TSIZE:
            if W.print_mode and W.show_progress:
                W.analyzer.show_simulation_progress()
            W.simulation_terminated()
            return 1
        
        return 0 #simulation not yet finished

    def check_simulation_ongoing(W):
        """
        Check whether the simulation is has not reached its final time.

        Returns
        -------
        int
            Returns 1 if the simulation is ongoing and has not reached its final time.
        """
        if W.finalized == 0:
            return 1
        return W.T <= W.TSIZE-1

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
        if node == None:
            return None
        
        if type(node) is Node:
            if node in W.NODES:
                return node
            else:
                if node.name in W.NODES_NAME_DICT:
                    return W.NODES_NAME_DICT[node.name]
        elif type(node) is str:
            if node in W.NODES_NAME_DICT:
                return W.NODES_NAME_DICT[node]
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
        if link == None:
            return None
        
        if type(link) is Link:
            if link in W.LINKS:
                return link
            else:
                if link.name in W.LINKS_NAME_DICT:
                    return W.LINKS_NAME_DICT[link.name]
        elif type(link) is str:
            if link in W.LINKS_NAME_DICT:
                return W.LINKS_NAME_DICT[link]
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

    def defRoute(W, *args, **kwargs):
        """
        Define Route object for this World.

        Parameters
        ----------
        links : list
            List of links (Link object or str)
        name : str
            Name of the route.
        trust_input : bool
            True if you trust the `links` in order to reduce the computation cost by omitting verification.
        """

        return Route(W, *args, **kwargs)


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

    def save_scenario(W, fname, network=True, demand=True):
        """
        Save the scenario (Node, Link, demand) to a file.

        Parameters
        ----------
        fname : str
            The file name to save the scenario.
        network : bool, optional
            Whether to save the network (Node, Link) data, default is True.
        demand : bool, optional
            Whether to save the demand (adddemand, etc.) data, default is True.
        """
        save_scenario(W, fname, network, demand)

    def load_scenario(W, fname, network=True, demand=True):
        """
        Load a scenario from a file.

        Parameters
        ----------
        fname : str
            The file name to load the scenario.
        network : bool, optional
            Whether to load the network data, default is True.
        demand : bool, optional
            Whether to load the demand data, default is True.
        """
        load_scenario(W, fname, network=network, demand=demand)

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

    def change_print_mode(W, print_mode):
        """
        Change the print mode.

        Parameters
        ----------
        print_mode : bool
            The print mode. If True, the print function is enabled. If False, the print function is disabled.
        """

        W.print_mode = print_mode
        if print_mode:
            W.print = print
        else:
            def noprint(*args, **kwargs):
                pass
            W.print = noprint

    @catch_exceptions_and_warn()
    def show_network(W, width=1, left_handed=1, figsize=(6,6), network_font_size=10, node_size=6, show_id=True):
        """
        Visualizes the entire transportation network shape.

        Parameters
        ----------
        W : Network
            The transportation network object.
        width : int, optional
            The width of the links in the visualization. Default is 1.
        left_handed : int, optional
            Determines the direction of the links. If 1, the links drawn with left-handed traffic rule. If 0, the links are right-handed. Default is 1.
        figsize : tuple, optional
            The size of the figure in inches. Default is (6, 6).
        network_font_size : int, optional
            The font size of the node and link labels. If 0, no labels will be displayed. Default is 10.
        node_size : int, optional
            The size of the nodes in the visualization. Default is 6.
        """
        
        os.makedirs(f"out{W.name}", exist_ok=True)

        plt.rcParams["font.family"] = get_font_for_matplotlib()

        plt.figure(figsize=figsize)
        plt.subplot(111, aspect="equal")
        for n in W.NODES:
            plt.plot(n.x, n.y, "o", c="gray", ms=node_size, zorder=10)
            if network_font_size > 0:
                if show_id:
                    label = f"{n.id}: {n.name}"
                else:
                    label = f"{n.name}"
                plt.text(n.x, n.y, label, c="g", horizontalalignment="center", verticalalignment="top", zorder=20, fontsize=network_font_size)
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
                if show_id:
                    label = f"{l.id}: {l.name}"
                else:
                    label = f"{l.name}"
                plt.text(xmid1, ymid1, label, c="b", zorder=20, fontsize=network_font_size)
        maxx = max([n.x for n in W.NODES])
        minx = min([n.x for n in W.NODES])
        maxy = max([n.y for n in W.NODES])
        miny = min([n.y for n in W.NODES])
        buffxy = max([(maxx-minx)/10, (maxy-miny)/10])
        plt.xlim([minx-buffxy, maxx+buffxy])
        plt.ylim([miny-buffxy, maxy+buffxy])
        plt.tight_layout()
        if W.save_mode:
            plt.savefig(f"out{W.name}/network.png")
        if W.show_mode:
            plt.show()
        else:
            plt.close("all")
    
    def copy(W):
        """
        Copy the World object.

        Returns
        -------
        World object
            The copied World object.
        """
        return pickle.loads(pickle.dumps(W))

    def save(W, fname):
        """
        Save the World object.

        Notes
        -----
        The World object is saved as a pickle file.
        """
        with open(f"{fname}.pkl", "wb") as f:
            pickle.dump(W, f)


class Route:
    """
    Class for a route that store concective links.
    """
    def __init__(s, W, links, name="", trust_input=False):
        """
        Define a route.

        Parameters
        ----------
        W : World
            The World to which this Route belongs
        links : list
            List of links (Link object or str)
        name : str
            Name of the route.
        trust_input : bool
            True if you trust the `links` in order to reduce the computation cost by omitting verification.

        Examples
        --------
        >>> route = Route(W, ["l1", "l2", "l3"])
        ... vehicle_object.links_prefer = route
        This will enforce the vehicle to travel the route if the route covers the entire links between the OD nodes of the vehicle.

        >>> route = Route(W, ["l1", "l2", "l3"])
        ... for link in route:
        ...     print(link)
        This will print the links in the route.
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
            if len(links) >= 2:
                s.links.append(l2)
            elif len(links) >= 1:
                s.links.append(W.get_link(links[0]))
            else:
                s.links = []
        else:
            #検査せずそのまま使用（計算コスト削減）
            s.links = links
        s.links_name = [l.name for l in s.links]

    def __repr__(s):
        return f"<Route {s.name}: {s.links}>"

    def __iter__(s):
        """
        Override `iter()` function. Iterate the links of the route.
        """
        return iter(s.links)
    
    def __len__(s):
        return len(s.links)
    
    def __eq__(self, other):
        """
        Override `==` operator. If the links of two route are the same, then the routes are the same.
        """
        if isinstance(other, Route):
            return [l.name for l in self.links] == [l.name for l in other.links]
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