"""
Submodule for general utilities.
This contains functions that are not essential for simulation but useful to specific analysis.
"""
import networkx as nx
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

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