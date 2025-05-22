"""
OpenStreetMap importer using OSMnx and neatnet.
Work in progress. Import from OSM is experimental and may not work as expected. It is functional but may produce inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.

Confirmed version: neatnet 0.1.0
"""

import sys
import warnings
from collections import defaultdict, Counter
import numpy as np
import pandas as pd
import networkx as nx


if 1: #to avoid edior's failure
    if sys.version_info < (3, 11):
        raise RuntimeError("Python 3.11 or higher is required to use 'OSMImporter2'.")

try:
    import osmnx as ox
except:
    raise ImportError("Optional module 'osmnx' is not installed. Use 'pip install uxsim[advanced]'.")

try:
    import neatnet # type: ignore
except:
    raise ImportError("Optional module 'neatnet' is not installed. Use 'pip install uxsim[advanced]'. Note that 'neatnet' requires Python 3.11 or later.")

try:
    import geopandas as gpd
except:
    raise ImportError("Optional module 'geopandas' is not installed. Use 'pip install uxsim[advanced]'.")

try:
    from shapely.geometry import Point, LineString
except:
    raise ImportError("Optional module 'shapely' is not installed. Use 'pip install uxsim[advanced]'.")


def _obtain_simplify_osm_by_osmnx_neatnet(bbox, custom_filter, simplification, kwargs_dict_for_osmnx_graph_from_bbox, kwargs_dict_for_osmnx_project_graph, kwargs_dict_for_neatnet_neatify):
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

    if simplification:
        simplified_links = neatnet.neatify(ox.graph_to_gdfs(G_projected)[1], **kwargs_dict_for_neatnet_neatify)
    else:
        simplified_links = ox.graph_to_gdfs(G_projected)[1]
    
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
            if col not in ['geometry', 'link_id', 'from_node', 'to_node', 'length']:  # ジオメトリ等は別途処理するので除外
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


def _attach_osm_nodes_links(W, nodes, links, set_node_capacity, override_osm_attributes=False, maxspeed_by_road_type={}, lanes_by_road_type={}):
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

    maxspeed_dict = {
        "motorway": 100,
        "trunk": 80,
        "primary": 60,
        "secondary": 50,
        "tertiary": 40,
        "residential": 30,
        "living_street": 20,
        "service": 20,
        "others": 40
    }
    maxspeed_dict = {key:maxspeed_dict[key]/3.6 for key in maxspeed_dict.keys()}
    for key in maxspeed_by_road_type:
        maxspeed_dict[key] = maxspeed_by_road_type[key]

    lanes_dict = {
        "motorway": 3,
        "trunk": 3,
        "primary": 2,
        "secondary": 2,
        "tertiary": 2,
        "residential": 1,
        "living_street": 1,
        "service": 1,
        "others": 1
    }
    for key in lanes_by_road_type:
        lanes_dict[key] = lanes_by_road_type[key]

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

        name = links.at[i, "name"]
        try:
            if type(name) is list:
                name = name[0]
            if type(name) is not str:
                name = str(i)
        except:
            name = str(i)
        
        highway_type = links.at[i, "highway"]
        try:
            if highway_type == None:
                #estimate the type from connected links
                from_node = links.at[i, "from_node"]
                links_upstream = links[links["to_node"] == from_node]
                to_node = links.at[i, "from_node"]
                links_downstream = links[links["from_node"] == to_node]
                
                links_connected = list(links_upstream["highway"])+list(links_downstream["highway"])
                links_connected_filtered = [l for l in links_connected if l != None]

                if len(links_connected_filtered):
                    assumed_type = Counter(links_connected_filtered).most_common(1)[0][0]
                else:
                    assumed_type = "None"
                
                links.at[i, "highway"] = assumed_type
                highway_type = str(assumed_type)
                
            if type(highway_type) is list:
                highway_type = str(Counter(highway_type).most_common(1)[0][0])
                links.at[i, "highway"] = highway_type
                
        except Exception as e:
            highway_type = "None"
            warnings.warn(f"Link {name} - Could not parse highway_type: {links.at[i, 'highway']}, due to '{e}' error. Using default value of {'None'}", UserWarning)

        maxspeed = links.at[i, "maxspeed"]
        try:
            if type(maxspeed) is str and not override_osm_attributes:
                maxspeed = float(maxspeed)/3.6
            elif type(maxspeed) is list and not override_osm_attributes:
                maxspeed = np.average([float(v) for v in maxspeed])/3.6
            else:
                maxspeed = maxspeed_dict["others"]
                for key in maxspeed_dict:
                    if key in highway_type:
                        maxspeed = maxspeed_dict[key]
                        break
        except Exception as e:
            maxspeed = maxspeed_dict["others"]
            warnings.warn(f"Link {name} - Could not parse maxspeed: {links.at[i, 'maxspeed']}, due to '{e}' error. Using default value of {maxspeed_dict['others']} m/s", UserWarning)
        

        """
        東京の一般道で確認できた範囲では，oneway=Falseの時のlanesは両方向の総和で，oneway=Trueの時のlanesはその方向のみの数
        """
        oneway = links.at[i, "oneway"]
        lanes = links.at[i, "lanes"]
        try:
            if type(lanes) is str and not override_osm_attributes:
                lanes = int(lanes)
            elif type(lanes) is list and not override_osm_attributes:
                lanes = max([int(l) for l in lanes])
            else:
                lanes = lanes_dict["others"]
                for key in lanes_dict:
                    if key in highway_type:
                        if oneway:
                            lanes = lanes_dict[key]
                        else:
                            lanes = lanes_dict[key]*2
                        break
        except Exception as e:
            lanes = lanes_dict["others"]
            warnings.warn(f"Link {name} - Could not parse number of lane: {links.at[i, 'lanes']}, due to '{e}' error. Using default value of {lanes_dict['others']}", UserWarning)

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
                node.flow_capacity = max_lanes*0.8*2
                node.number_of_lanes = max_lanes*2


def osm_network_to_World(W, north=None, south=None, east=None, west=None, bbox=None, custom_filter='["highway"~"trunk|primary"]', 
                         simplification=True, override_osm_attributes=False,
                         set_node_capacity=True, 
                         maxspeed_by_road_type={}, lanes_by_road_type={}, kwargs_dict_for_osmnx_graph_from_bbox={}, kwargs_dict_for_osmnx_project_graph={}, kwargs_dict_for_neatnet_neatify={}):
    """
    Imports OpenStreetMap (OSM) network data into a World object. This function retrieves OSM road network data within the specified bounding box, simplifies the network, and extracts nodes and links using OSMnx and neatnet. Then it attaches them to the World object.

    Parameters
    ----------
    W : World
        The World object to which the OSM network will be added.
    north, south, east, west : float, optional
        The latitudes and longitudes defining the boundaries of the area to be imported.
    bbox : list, optional
        The bounding box of the area to be imported. Format is [west, south, east, north].
        This parameter takes precedence over individual north, south, east, west arguments.
    custom_filter : str, default='["highway"~"trunk|primary"]'
        The OSM filter to be used for importing the data.
        Examples:

        - '["highway"~"motorway"]': highways only
        - '["highway"~"motorway|trunk|primary|secondary|tertiary"]': highways and major/mid arterials

    simplification : bool, default=True
        If True, the network is simplified using neatnet's neatify function.
        If False, the original network is used.
    override_osm_attributes : bool, default=False
        If True, the function will override OSM attributes (maxspeed, lanes) with default values. This can be useful when OSM data is erroneous or inconsistent.
        If False, it will use the attributes from the OSM data.
    set_node_capacity : bool, default=True
        If True, automatically determines flow capacity of nodes based on connected links.
        If False, the capacity is unbounded. This simulates traffic signals in a continuous manner.
    maxspeed_by_road_type : dict, default={}
        Dictionary to override default speed limits for different road types.
    lanes_by_road_type : dict, default={}
        Dictionary to override default number of lanes for different road types.
    kwargs_dict_for_osmnx_graph_from_bbox : dict, default={}
        Additional keyword arguments to pass to OSMnx's graph.graph_from_bbox function.
    kwargs_dict_for_osmnx_project_graph : dict, default={}
        Additional keyword arguments to pass to OSMnx's project_graph function.
    kwargs_dict_for_neatnet_neatify : dict, default={}
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
    print("Started network import from OSM. It may take some time...")
    
    warnings.warn("Import from OSM is experimental and may not work as expected. It is functional but produces inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.")
    
    if bbox == None:
        bbox = [west,south,east,north]
        
    simplified_links = _obtain_simplify_osm_by_osmnx_neatnet(bbox, custom_filter, simplification,
                                                             kwargs_dict_for_osmnx_graph_from_bbox, kwargs_dict_for_osmnx_project_graph, kwargs_dict_for_neatnet_neatify)
    nodes, links = _extract_network_from_geodataframe(simplified_links)  
    _attach_osm_nodes_links(W, nodes, links, set_node_capacity, override_osm_attributes=override_osm_attributes, maxspeed_by_road_type=maxspeed_by_road_type, lanes_by_road_type=lanes_by_road_type)

    stats_count = defaultdict(lambda: 0)
    stats_length = defaultdict(lambda: 0)
    for i in links.index:
        length = links.at[i, "length"]
        highway_type = str(links.at[i, "highway"]).replace("_link", "")
        if links.at[i, "oneway"]:
            coef = 1
        else:
            coef = 2
        stats_count[highway_type] += 1*coef
        stats_length[highway_type] += length*coef

    print(f"Imported {len(nodes)} nodes and {sum([val for val in stats_count.values()])} links from OSM")
    for key in stats_count:
        print(f" '{key}' links:\t {stats_count[key]} with total length {stats_length[key]:.0f} m")

