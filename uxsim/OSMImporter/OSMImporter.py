"""
OpenStreetMap importer using OSMnx.
Work in progress. Import from OSM is experimental and may not work as expected. It is functional but may produce inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.


Examples
--------
Import highway in Tokyo:
    >>> from uxsim.OSMImporter import OSMImporter
    >>> ... #define World object W
    >>> nodes, links = OSMImporter.import_osm_data(north=35.817, south=35.570, east=139.881, west=139.583, custom_filter='["highway"~"motorway"]')
    >>> nodes, links = OSMImporter.osm_network_postprocessing(nodes, links, node_merge_threshold=0.005, node_merge_iteration=5, >>> enforce_bidirectional=True)  # merge threshold distance: 0.005 degree ~= 500 m. `enforce_bidirectional` makes all links bidirectional, so >>> that network is not fragmented (but the original network topology is not preserved rigorously).
    >>> OSMImporter.osm_network_visualize(nodes, links, show_link_name=0)
    >>> OSMImporter.osm_network_to_World(W, nodes, links, default_jam_density=0.2, coef_degree_to_meter=111000)
    >>> ... #add demand
    >>> W.exec_simulation()
"""


import matplotlib.pyplot as plt
import math
import warnings

class OSMImporter:
    """
    OpenStreetMap importer using OSMnx.
    Work in progress. Import from OSM is experimental and may not work as expected. It is functional but may produce inappropriate networks for simulation, such as too many nodes, too many deadends, fragmented networks.
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
                    if lanes < 1:
                        lanes = 1
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
                #links.append([name, e[0], e[1], 1, maxspeed]) # name, from, to, number_of_lanes, maxspeed
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
            if lname in [l.name for l in W.LINKS]:
                lname + f"_osm{i}"
            W.addLink(lname, str(link[1]), str(link[2]), length=link[5]*coef_degree_to_meter, free_flow_speed=link[4], jam_density_per_lane=default_jam_density, number_of_lanes=link[3], auto_rename=True)