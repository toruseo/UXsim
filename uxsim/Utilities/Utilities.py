"""
Submodule for general utilities.
This contains functions that are not essential for simulation but useful to specific analysis.
"""
import networkx as nx
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
import tqdm
import random
from collections import defaultdict
import warnings

def generate_grid_network(W, imax, jmax, **kwargs):
    """
    Generate a grid network with imax x jmax nodes.

    Parameters
    ----------
    W : World
        The world object to which the network will be added.
    imax : int
        The number of nodes in the x direction.
    jmax : int
        The number of nodes in the y direction. 
    **kwargs : dict
        Additional keyword arguments to be passed to the addLink function.
    """
    
    # Define the scenario
    #deploy nodes as an imax x jmax grid
    nodes = dict()
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    #create links between neighborhood nodes
    links = dict()
    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], **kwargs)
            if i != 0:
                links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], **kwargs)
            if j != jmax-1:
                links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], **kwargs)
            if j != 0:
                links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], **kwargs)
    
    return nodes, links


def enumerate_k_shortest_routes(W, source, target, k=1, cost_function=lambda l: l.length/l.u, print_stats=0, return_cost=False):
    """
    Enumerate the k shortest routes between two nodes in a network. By default, `enumerate_k_shortest_routes(W, "O", "D")` returns the shortest path from node "O" to "D" based on the free-flow travel time. 

    Parameters
    ----------
    W : World
        The world object containing the network.   
    source : str | Node
        The source node.
    target : str | Node
        The target node.
    k : int
        The number of shortest routes to enumerate. Default is 1.
    cost_function : function
        A link cost function to compute shortest path. The argument is Link object. Default is the free-flow travel time, `lambda l: l.length/l.u`.
    print_stats : bool
        Print the statistics of the paths.
    return_cost : bool
        Return the cost of the paths.
    
    Returns
    -------
    routes : list
        A list of k shortest routes. Each route is a list of link names. 
    costs : list
        A list of costs of the routes if return_cost is True.
    """
    G = nx.DiGraph()
    for l in W.LINKS:
        G.add_edge(l.start_node.name, l.end_node.name, weight=cost_function(l))

    k_shortest_paths = list(nx.shortest_simple_paths(G, W.get_node(source).name, W.get_node(target).name, weight='weight'))[:k]
    routes = []
    costs = []
    for path in k_shortest_paths:
        route = []
        for n in path[:-1]:
            for l in W.LINKS:
                if l.start_node.name == n and l.end_node.name == path[path.index(n)+1]:
                    route.append(l.name)

        path_weight = sum([G[u][v]['weight'] for u, v in zip(path[:-1], path[1:])])
        if print_stats:
            print(f"Route: {route}, Cost: {path_weight}")
            
        routes.append(route)
        costs.append(path_weight)
    
    if return_cost:
        return routes, costs
    else:
        return routes


def enumerate_k_shortest_routes_on_t(W, source, target, t, k=1, cost_function=lambda l, t: l.instant_travel_time(t), print_stats=0, return_cost=False):
    """
    Enumerate the k shortest routes between two nodes in a network. By default, `enumerate_k_shortest_routes_on_t(W, "O", "D", t=t)` returns the shortest path from node "O" to "D" based on instantanious travel time on time t.

    Parameters
    ----------
    W : World
        The world object containing the network.   
    source : str | Node
        The source node.
    target : str | Node
        The target node.
    t : float
        The time point to compute shortest routes.
    k : int
        The number of shortest routes to enumerate. Default is 1.
    cost_function : function
        A link cost function to compute shortest path. The argument is Link object and time t. Default is the instantaneous travel time, `lambda l, t: l.instant_travel_time(t)`.
    print_stats : bool
        Print the statistics of the paths.
    return_cost : bool
        Return the cost of the paths.
    
    Returns
    -------
    routes : list
        A list of k shortest routes. Each route is a list of link names. 
    costs : list
        A list of costs of the routes if return_cost is True.
    """
    G = nx.DiGraph()
    for l in W.LINKS:
        G.add_edge(l.start_node.name, l.end_node.name, weight=cost_function(l, t))

    k_shortest_paths = list(nx.shortest_simple_paths(G, W.get_node(source).name, W.get_node(target).name, weight='weight'))[:k]
    routes = []
    costs = []
    for path in k_shortest_paths:
        route = []
        for n in path[:-1]:
            for l in W.LINKS:
                if l.start_node.name == n and l.end_node.name == path[path.index(n)+1]:
                    route.append(l.name)

        path_weight = sum([G[u][v]['weight'] for u, v in zip(path[:-1], path[1:])])
        if print_stats:
            print(f"Route: {route}, Cost: {path_weight}")
            
        routes.append(route)
        costs.append(path_weight)
    
    if return_cost:
        return routes, costs
    else:
        return routes

def enumerate_k_random_routes(W, k):
    """
    Enumerate k random routes between all node pairs in a network. The shortest path with free flow travel time is always included. This is much faster than `enumerate_k_shortest_routes` and could be useful for a plausible choice set generation for route choice problems.

    Parameters
    ----------
    W : World
        The world object containing the network.   
    k : int
        The number of random routes to enumerate.
    
    Returns
    -------
    dict_routes : dict
        A dictionary of k random routes. The key is a tuple of the origin and destination nodes. The value is a list of link names.
    """
    dict_routes = defaultdict(list)
    counter = 0
    iteration = 0
    average_n_routes_old = 0
    while 1:
        G = nx.DiGraph()
        for l in W.LINKS:
            if iteration == 0:
                G.add_edge(l.start_node.name, l.end_node.name, weight=l.length/l.u)
            else:
                G.add_edge(l.start_node.name, l.end_node.name, weight=l.length/l.u*random.uniform(0,100))

        paths = dict(nx.all_pairs_dijkstra_path(G))
        for source in paths:
            for dest in paths[source]:
                if len(dict_routes[source, dest]) < k:
                    path = []
                    for i in range(0,len(paths[source][dest])-1):
                        l = W.NODE_PAIR_LINKS[paths[source][dest][i], paths[source][dest][i+1]]
                        #print(W.NODE_PAIR_LINKS[paths[source][dest][i], paths[source][dest][i+1]], end="")
                        path.append(l.name)
                    if path not in dict_routes[source, dest]:
                        dict_routes[source, dest].append(path)

        average_n_routes = sum([len(value) for key,value in dict_routes.items()])/len(dict_routes.keys())
        if average_n_routes == average_n_routes_old:
            counter += 1
        else:
            counter = 0
            average_n_routes_old = average_n_routes
        
        iteration += 1
        if counter > 10:
            break

    return dict_routes


def get_shortest_path_distance_between_all_nodes(W, return_matrix=False):
    """
    Get the shortest distances (in meters) between all node pairs based on link lengths

    Parameters
    ----------
    W : World
        The World object.
    return_matrix : bool, optional
        Whether to return the distance matrix as a numpy array. Default is False.

    Returns
    -------
    dict or numpy array
        Returns a dictionary of distances between nodes whose key is node pair if `return_matrix` is False.
        Returns a numpy array of distances between nodes whose index is node.id pair if `return_matrix` is True.
    """
    num_nodes = len(W.NODES)
    distances = np.full((num_nodes, num_nodes), np.inf)  # Initialize with infinity

    # Fill in the distances based on the link lengths
    for link in W.LINKS:
        i = link.start_node.id
        j = link.end_node.id
        distances[i, j] = min(distances[i, j], link.length)

    # Use Dijkstra algorithm to compute shortest distances
    distances = dijkstra(csr_matrix(distances), directed=True, return_predecessors=False)

    if return_matrix == True:
        return distances
    else:
        distances_dict = dict()
        for node1 in W.NODES:
            for node2 in W.NODES:
                distances_dict[node1, node2] = distances[node1.id, node2.id]
                distances_dict[node1.name, node2.name] = distances[node1.id, node2.id]
        return distances_dict

def get_shortest_path_instantaneous_travel_time_between_all_nodes(W, return_matrix=False):
    """
    Get the shortest instantaneous travel time (in seconds) between all node pairs based on the current instantaneous travel time of each link.

    Parameters
    ----------
    W : World
        The World object.
    return_matrix : bool, optional
        Whether to return the distance matrix as a numpy array. Default is False.

    Returns
    -------
    dict or numpy array
        Returns a dictionary of distances between nodes whose key is node pair if `return_matrix` is False.
        Returns a numpy array of distances between nodes whose index is node.id pair if `return_matrix` is True.
    """
    distances = W.ROUTECHOICE.dist

    if return_matrix == True:
        return distances
    else:
        distances_dict = dict()
        for node1 in W.NODES:
            for node2 in W.NODES:
                distances_dict[node1, node2] = distances[node1.id, node2.id]
                distances_dict[node1.name, node2.name] = distances[node1.id, node2.id]
        return distances_dict
    
def get_shortest_path_instantaneous_travel_time_between_all_nodes_on_t(W, t, return_time=False, return_matrix=False):
    """
    Get the shortest instantaneous travel time (in seconds) between all node pairs based on the instantaneous travel time of each link near time t (based on the route choice update interval).

    Parameters
    ----------
    W : World
        The World object.
    t : float
        The time point to compute shortest travel time.
    return_time : bool, optional
        Whether to return the actual time of computing shortest path cost. Default is False.
    return_matrix : bool, optional
        Whether to return the distance matrix as a numpy array. Default is False.

    Returns
    -------
    dict or numpy array or tuple
        Returns a dictionary of distances between nodes whose key is node pair if `return_matrix` is False and `return_time` is False.
        Returns a numpy array of distances between nodes whose index is node.id pair if `return_matrix` is True and `return_time` is False.
        Returns a tuple of the abode distances and the actual time of computing shortest path cost if `return_time` is True.
    """
    if t >= W.TMAX:
        t = W.TMAX-W.DELTAT
    duo_t = t//W.DUO_UPDATE_TIME
    duo_timestep = int(duo_t*W.DUO_UPDATE_TIME/W.DELTAT)

    distances = W.ROUTECHOICE.dist_record[duo_timestep]

    if return_matrix == True:
        ret = distances
    else:
        distances_dict = dict()
        for node1 in W.NODES:
            for node2 in W.NODES:
                distances_dict[node1, node2] = distances[node1.id, node2.id]
                distances_dict[node1.name, node2.name] = distances[node1.id, node2.id]
        ret = distances_dict

    if return_time == True:
        return ret, duo_t
    else:
        return ret

def construct_time_space_network(W, dt=None, from_origin_only=True):
    """
    Construct a time-space network (TSN) that includes the time-dependent shortest path infomation.

    Parameters
    ----------
    W : World
        The World object.
    dt : int, optional
        The time interval to construct the TSN. Default is None, which sets to the simulation timestep.
    from_origin_only : bool, optional
        Whether to compute the shortest path from the origin only. Default is True

    Notes
    -----
        In the default setting, `W.TSN_paths` contains time-dependent shortest paths from the origin nodes to all nodes, for all departure time. The time-dependent link cost is based on the actual travel time. For example, `W.TSN_paths["orig_node_name", 0]` contains the shortest path from the node "orig_node_name" with departure time 0. `W.TSN_paths["orig_node_name", 0]["dest_node_name", "end"]` contains the shortest path from the node "orig_node_name" to "dest_node_name" with departure time 0. The travel time between these nodes on this time can be obtained by `W.TSN_costs["orig_node_name", 0]["dest_node_name", "end"]`.

        Not efficient for large networks.
    """
    if dt == None:
        dt = int(W.DELTAT)
    tmax = int(W.TMAX)
    coef_t_to_xy = 1/10000  #for visualization

    G = nx.DiGraph()
    for t in range(0, tmax, dt):
        for node in W.NODES:
            G.add_node((node.name, t), pos=(node.x+t*coef_t_to_xy, node.y+t*coef_t_to_xy))

    for t in range(0, tmax, dt):
        for node in W.NODES:
            for link in node.outlinks.values():
                travel_time = link.traveltime_actual[int(t/W.DELTAT)]
                t_arrival = int((t + travel_time)/dt)*dt
                if t_arrival < W.TMAX:
                    u = (node.name, t)
                    v = (link.end_node.name, t_arrival)
                    if G.has_node(u) and G.has_node(v):
                        G.add_edge(u, v, weight=travel_time)
                    else:
                        print(u,v)

    for t in range(0, tmax, dt):
        for node in W.NODES:
            v = (node.name, "end")
            if not G.has_node(v):
                G.add_node((node.name, "end"), pos=(node.x+tmax*coef_t_to_xy, node.y+tmax*coef_t_to_xy))

            u = (node.name, t)
            if G.has_node(u) and G.has_node(v):
                G.add_edge(u, v, weight=0)

    #print(G.number_of_nodes(), G.number_of_edges())

    paths = {}
    costs = {}     
    if from_origin_only:
        sources = {}
        for veh in W.VEHICLES.values():
            u = (veh.orig.name, int(veh.departure_time_in_second/dt)*dt)
            if G.has_node(u):
                sources[u] = True
            else:
                print(u)
        for source in sources.keys():
            costs[source], paths[source] = nx.single_source_dijkstra(G, source)
    else:
        for source, (distance_dict, path_dict) in nx.all_pairs_dijkstra(G):
            costs[source] = distance_dict
            paths[source] = path_dict

    W.TSN = G
    W.TSN_paths = paths
    W.TSN_costs = costs


    # # Extract positions
    # pos = nx.get_node_attributes(G, 'pos')
    # edge_labels = nx.get_edge_attributes(G, 'weight')
    #
    # # Create a plot
    # fig, ax = plt.subplots()
    # nx.draw(G, pos, with_labels=False, node_color='lightblue', edge_color='gray', node_size=50, ax=ax)
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='black', bbox=dict(facecolor='none', edgecolor='none'))
    # plt.show()
    
    # # Display the paths and costs
    # for source in paths:
    #     for target in paths[source]:
    #         if target[1] == "end":
    #             print(f"Shortest path from {source[0]} to {target[0]} on t = {source[1]}: cost = {costs[source][target]}: {paths[source][target]}")



def estimate_congestion_externality_link(W, link, t):
    """
    Estimate externality caused by a hypothetical vehicle traveling the link in the world. Subfunction for `estimate_congestion_externality_route`.

    Parameters
    ----------
    W : World
        The World object.
    link : str | Link
        Link object or link name.
    t : float
        The departure time of the hypothetical vehicle.
    """
    try:
        link = W.get_link(link)
        ts = int(t/W.DELTAT)

        #同じ待ち行列で自分より後ろで待っている車両台数の算出
        ts_end = ts
        for i in range(ts, len(link.traveltime_actual), 1):
            if link.traveltime_actual[i] > link.length/link.free_flow_speed*1.01 and link.cum_arrival[i+int(link.traveltime_actual[i]/W.DELTAT)]-link.cum_departure[i+int(link.traveltime_actual[i]/W.DELTAT)] > 0:
            #if link.traveltime_actual[i] > link.length/link.free_flow_speed*1.01:
                ts_end = i
            else:
                break
        
        veh_count = link.cum_arrival[ts_end]-link.cum_arrival[ts]

        #局所的容量の算出
        #TODO:微妙に系統的にずれる
        #TODO:スピルオーバー時の除外 
        #TODO:信号待ちの一台目のときに破綻してしまう．頑健で賢い方法あるか？信号，このリンクの容量制約，下流側リンク・ノード容量制約を全部考えられるもの
        ts_out = int(ts+int(link.traveltime_actual[ts]/W.DELTAT))+1
        ts_out_leader = ts_out
        for i in range(ts_out, 0, -1):
            if link.cum_departure[i] < link.cum_departure[ts_out]:
                ts_out_leader = i
                break

        headway_consumed = (ts_out-ts_out_leader)*W.DELTAT 

        ext = headway_consumed*veh_count
        
        return ext
    
    except Exception as e:
        #細かいインデックスエラーなどを一時的に回避 TODO: fix

        warnings.warn(f"Error at `estimate_congestion_externality_link`: {e}", UserWarning)

        return 0

def estimate_congestion_externality_route(W, route, t):
    """
    Estimate externality caused by a hypothetical vehicle traveling the route in the world. 
    
    Specifically, it estimates the total travel time that would be achieved if the vehicle did not exist and then calculates the difference. This estimation is based on a simple queuing model for each link of the route that the vehicle passes through. Does not fully capture queue spilovers.

    Parameters
    ----------
    W : World
        The World object.  
    route : Route
        Route object.
    t : float
        The departure time of the hypothetical vehicle.
    """

    tts = route.actual_travel_time(t, return_details=True)[1]
    exts = 0
    for i,link in enumerate(route):
        exts += estimate_congestion_externality_link(W, link, t)
        t += tts[i]
    
    return exts

