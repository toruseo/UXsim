from uxsim import *
from uxsim.OSMImporter import OSMImporter

W = World(
    name="",
    deltan=5,
    tmax=7200,
    print_mode=1, save_mode=1, show_mode=0, 
    random_seed=0
)

#Tokyo highway
nodes, links = OSMImporter.import_osm_data(north=35.817, south=35.570, east=139.881, west=139.583, custom_filter='["highway"~"motorway"]')
nodes, links = OSMImporter.osm_network_postprocessing(nodes, links, node_merge_threshold=0.005, node_merge_iteration=5, enforce_bidirectional=True)  # merge threshold distance: 0.005 degree ~= 500 m. `enforce_bidirectional` makes all links bidirectional, so that network is not fragmented (but the original network topology is not preserved rigorously).
OSMImporter.osm_network_visualize(nodes, links, show_link_name=0)
OSMImporter.osm_network_to_World(W, nodes, links, default_jam_density=0.2, coef_degree_to_meter=111000)

# set demand to central Tokyo from surroundings
W.adddemand_area2area(139.70, 35.60, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)
W.adddemand_area2area(139.65, 35.70, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)
W.adddemand_area2area(139.75, 35.75, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)
W.adddemand_area2area(139.85, 35.70, 0, 139.75, 35.68, 0.05, 0, 3600, volume=5000)

W.exec_simulation()

W.analyzer.print_simple_stats()
for t in list(range(0,W.TMAX,int(W.TMAX/6))):
    W.analyzer.network(t, detailed=0, network_font_size=0, figsize=(6,6))

W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.1, interval=10, trace_length=5)

W.analyzer.output_data()