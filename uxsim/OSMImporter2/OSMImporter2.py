"""
OpenStreetMap importer using OSMnx and neatnet.
Work in progress. Import from OSM is experimental and may not work as expected. It is functional but may produce inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.

Confirmed version: neatnet 0.1.0
"""

import warnings
import numpy as np
import pandas as pd

try:
    import osmnx as ox
except:
    raise ImportError("Optional module 'osmnx' is not installed.")

try:
    import neatnet
except:
    raise ImportError("Optional module 'neatnet' is not installed. Note that 'neatnet' requires Python 3.11 or later.")

try:
    import geopandas as gpd
except:
    raise ImportError("Optional module 'geopandas' is not installed.")

try:
    from shapely.geometry import Point, LineString
except:
    raise ImportError("Optional module 'shapely' is not installed.")


def _obtain_simplify_osm_by_osmnx_neatnet(bbox, custom_filter, kwargs_dict_for_osmnx_graph_from_bbox, kwargs_dict_for_osmnx_project_graph, kwargs_dict_for_neatnet_neatify):
    """
    Obtains and simplifies OSM data using OSMnx and neatnet libraries. This function retrieves a road network from OpenStreetMap within the specified bounding box, applies projection, and then simplifies the network links using the neatnet library's neatify algorithm.

    Parameters
    ----------
    bbox : tuple
        Bounding box coordinates expected by OSMnx.
    custom_filter : str
        OSM filter query to use when downloading the network data.

    Returns
    -------
    GeoDataFrame
        A GeoDataFrame containing the simplified road network links.

    Notes
    -----
    This function requires the OSMnx and neatnet libraries to be installed.
    The returned network is filtered for driving ("drive" network type).
    """
    
    G = ox.graph.graph_from_bbox(bbox=bbox, network_type="drive", custom_filter=custom_filter, **kwargs_dict_for_osmnx_graph_from_bbox)
    G_projected = ox.project_graph(G, **kwargs_dict_for_osmnx_project_graph)

    simplified_links = neatnet.neatify(ox.graph_to_gdfs(G_projected)[1], **kwargs_dict_for_neatnet_neatify)
    
    return simplified_links


def _extract_network_from_geodataframe(gdf):
    """
    Function to extract road network nodes and links from a GeoDataFrame
    
    Parameters:
    -----------
    gdf : GeoDataFrame
        GeoDataFrame containing linear features (LineString)
    
    Returns:
    --------
    nodes_df : GeoDataFrame
        Node list (node_id, x, y, geometry)
    links_df : GeoDataFrame
        Link list (link_id, from_node, to_node, length, and all other columns from the original GeoDataFrame)
    """
    # LineStringの形状のみを処理
    if not all(isinstance(geom, LineString) for geom in gdf.geometry):
        raise ValueError("GeoDataFrameにはLineString型のジオメトリのみが含まれている必要があります")
    
    # ノードの収集（始点と終点）
    nodes = []
    node_id_map = {}  # 座標からノードIDへのマッピング
    node_counter = 0
    
    # リンク情報の収集
    links = []
    
    for idx, row in gdf.iterrows():
        line = row.geometry
        
        # 始点の座標を取得
        start_point = line.coords[0]
        # 終点の座標を取得
        end_point = line.coords[-1]
        
        # 始点が既に登録されているかチェック
        start_key = (round(start_point[0], 6), round(start_point[1], 6))
        if start_key not in node_id_map:
            node_id_map[start_key] = node_counter
            nodes.append({
                'node_id': node_counter,
                'x': start_point[0],
                'y': start_point[1],
                'geometry': Point(start_point)
            })
            node_counter += 1
        
        # 終点が既に登録されているかチェック
        end_key = (round(end_point[0], 6), round(end_point[1], 6))
        if end_key not in node_id_map:
            node_id_map[end_key] = node_counter
            nodes.append({
                'node_id': node_counter,
                'x': end_point[0],
                'y': end_point[1],
                'geometry': Point(end_point)
            })
            node_counter += 1
        
        # リンク情報の作成（元のGeoDataFrameの全カラムを保持）
        link_data = {
            'link_id': idx,
            'from_node': node_id_map[start_key],
            'to_node': node_id_map[end_key],
            'length': line.length,
        }
        
        # 元のGeoDataFrameの全カラムをコピー
        for col in gdf.columns:
            if col not in ['geometry', 'link_id', 'from_node', 'to_node', 'length']:  # ジオメトリは別途処理するので除外
                link_data[col] = row[col]
                
        # ジオメトリはリンクデータに追加
        link_data['geometry'] = line
        
        links.append(link_data)
    
    # ノードとリンクのDataFrameを作成
    nodes_df = pd.DataFrame(nodes)
    links_df = pd.DataFrame(links)
    
    # GeoDataFrameに変換
    nodes_gdf = gpd.GeoDataFrame(nodes_df, geometry='geometry', crs=gdf.crs)
    links_gdf = gpd.GeoDataFrame(links_df, geometry='geometry', crs=gdf.crs)
    
    return nodes_gdf, links_gdf

def _attach_osm_nodes_links(W, nodes, links, set_node_capacity):
    """
    Adds OSM nodes and links to the network. This function processes the OSM nodes and links dataframes and adds them to the provided network object (W). For nodes, it extracts coordinates and ID. For links, it processes attributes like speed limits, number of lanes, and handles one-way/two-way streets accordingly.

    Parameters
    ----------
    W : Network object
        The network object to which nodes and links will be added.
    nodes : pandas.DataFrame
        DataFrame containing node information with columns:
        - node_id: unique identifier for each node
        - x: x-coordinate
        - y: y-coordinate
    links : pandas.DataFrame
        DataFrame containing link information with columns:
        - from_node: source node ID
        - to_node: destination node ID
        - length: length of the link
        - maxspeed: maximum speed (km/h)
        - lanes: number of lanes
        - name: name of the link
        - oneway: boolean indicating if the link is one-way
    """


    added_node_names = []
    for i in nodes.index:
        x = nodes.at[i, "x"]
        y = nodes.at[i, "y"]
        node_id = str(nodes.at[i, "node_id"])
        W.addNode(node_id, x, y, auto_rename=True)

        added_node_names.append(node_id)

    for i in links.index:
        start_node = str(links.at[i, "from_node"])
        end_node = str(links.at[i, "to_node"])
        length = links.at[i, "length"]

        maxspeed = links.at[i, "maxspeed"]
        if type(maxspeed) is list:
            maxspeed = np.average([float(v) for v in maxspeed])
        if maxspeed is None:
            maxspeed = 10 #TODO: this should be type-specific
        if type(maxspeed) is str:
            maxspeed = float(maxspeed)
        maxspeed = maxspeed/3.6

        lanes = links.at[i, "lanes"]
        if type(lanes) is list:
            lanes = max([int(l) for l in lanes])
        if lanes is None:
            lanes = 1  #TODO: this should be type-specific
        lanes = int(lanes)

        name = links.at[i, "name"]
        if type(name) is list:
            name = name[0]
        if type(name) is not str:
            name = f"{i}"

        oneway = links.at[i, "oneway"]

        if oneway:
            W.addLink(name=name, start_node=start_node, end_node=end_node, length=length, free_flow_speed=maxspeed, number_of_lanes=lanes, auto_rename=True)
        else:
            lanes = int(lanes/2)
            if lanes <= 0:
                lanes = 1
            W.addLink(name=name, start_node=start_node, end_node=end_node, length=length, free_flow_speed=maxspeed, number_of_lanes=lanes, auto_rename=True)
            W.addLink(name=name+"-reverse", start_node=end_node, end_node=start_node, length=length, free_flow_speed=maxspeed, number_of_lanes=lanes, auto_rename=True)
    
    if set_node_capacity:
        for node in W.NODES:
            if node.name in added_node_names:
                max_lanes = max([l.number_of_lanes for l in node.inlinks.values()] + [l.number_of_lanes for l in node.outlinks.values()])
                node.flow_capacity = max_lanes*0.8
                node.number_of_lanes = max_lanes


def osm_network_to_World(W, north=None, south=None, east=None, west=None, bbox=None, custom_filter='["highway"~"trunk|primary"]', set_node_capacity=True, kwargs_dict_for_osmnx_graph_from_bbox={}, kwargs_dict_for_osmnx_project_graph={}, kwargs_dict_for_neatnet_neatify={}):
    """
    Imports OpenStreetMap (OSM) network data into a World object. This function retrieves OSM road network data within the specified bounding box, simplifies the network, and extracts nodes and links using OSMnx and neatnet. Then it attaches them to the World object.

    Parameters
    ----------
    W : World
        The World object to which the OSM network will be added.
    north, south, east, west: float
        The latitudes and longitudes of the area to be imported.
    bbox: list
        The bounding box of the area to be imported. The order is [north, south, east, west] for OSMnx ver 1, and [west, south, east, north] for OSMnx ver 2. This is prioritized over north, south, east, west arguments.
    custom_filter: str
        The filter to be used for importing the data. 
        The default is '["highway"~"trunk|primary"]', which means that only trunk and primary roads (usually correspond to major arterial roads) are imported.
        Examples
        - custom_filter='["highway"~"motorway"]' #highway only
        - custom_filter='["highway"~"motorway|trunk|primary|secondary|tertiary"]' #highway and major/mid arterials
    set_node_capacity: bool
        Automatically determines flow capacity of nodes based on its connected links. If false, the capacity is unbounded. This simulates a traffic signal in a continious manner. Default is True.
    kwargs_dict_for_osmnx_graph_from_bbox: dict
        Additional keyword arguments to pass to OSMnx's graph.graph_from_bbox function.
    kwargs_dict_for_osmnx_project_graph: dict
        Additional keyword arguments to pass to OSMnx's project_graph function.
    kwargs_dict_for_neatnet_neatify: dict
        Additional keyword arguments to pass to neatnet's neatify function.

    Returns
    -------
    None
        The function modifies the World object in-place by adding OSM nodes and links.
    
    Notes
    -----
    The function performs three main steps:
    1. Obtains and simplifies the OSM network using OSMnx and neatnet
    2. Extracts node and link information from the simplified network
    3. Attaches the extracted nodes and links to the World object
    """
    
    warnings.warn("Import from OSM is experimental and may not work as expected. It is functional but produces inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.")
    
    if bbox == None:
        bbox = [west,south,east,north]
        
    simplified_links = _obtain_simplify_osm_by_osmnx_neatnet(bbox, custom_filter, 
                                                             kwargs_dict_for_osmnx_graph_from_bbox, kwargs_dict_for_osmnx_project_graph, kwargs_dict_for_neatnet_neatify)
    nodes, links = _extract_network_from_geodataframe(simplified_links)  
    _attach_osm_nodes_links(W, nodes, links, set_node_capacity)

    print(f"Loaded {len(nodes)} nodes and {len(links)} links from OSM")