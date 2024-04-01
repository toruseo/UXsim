import uxsim as UX


# scenario
W = UX.World(
    name="",
    deltan=5,
    tmax=2000,
    print_mode=1, save_mode=0, show_mode=1,
    random_seed=None
)

W.addNode("orig", 0, 0)
W.addNode("mid", 0, 0)
W.addNode("dest", 2, 1)
link1 = W.addLink("link1", "orig", "mid", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=2)
link2 = W.addLink("link2", "mid", "dest", length=1000, free_flow_speed=20, jam_density_per_lane=0.2, number_of_lanes=1)
W.adddemand("orig", "dest", 0, 1000, flow=0.8)
W.adddemand("orig", "dest", 500, 1000, flow=0.8)
"""
With free_flow_speed=20, jam_density_per_lane=0.2 setting, the flow capacity per lane is 0.8 veh/s.
Thus, the total flow capacity of the link1 is 1.6 veh/s, whereas the total flow capacity of the link2 is 0.8 veh/s.
This means that the node "mid" is a bottleneck.
It is expected that the traffic flow will be congested at the node "mid" after the demand increases at t=500.
"""

W.exec_simulation()
df = W.analyzer.od_to_pandas()

# Print summary of simulation result
W.analyzer.print_simple_stats()

W.analyzer.time_space_diagram_traj_links([link1, link2])