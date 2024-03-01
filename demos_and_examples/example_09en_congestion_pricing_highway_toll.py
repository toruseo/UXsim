import pandas as pd
from uxsim import *
import pickle


# Simulation definition
W = World(
    name="",
    deltan=5,
    tmax=6000,
    print_mode=1, save_mode=1, show_mode=0,
    random_seed=0,
)

# Scenario definition
#automated network generation
#generate nodes to create (imax, imax) grid network
imax = 11
jmax = imax
nodes = {}
for i in range(imax):
    for j in range(jmax):
        nodes[i,j] = W.addNode(f"n{(i,j)}", i, j)

#grid-shaped arterial roads
links = {}
for i in range(imax):
    for j in range(jmax):
        if i != imax-1:
            ii = i+1
            jj = j
            links[i,j,ii,jj] = W.addLink(f"l{(i,j,ii,jj)}", nodes[i,j], nodes[ii,jj], length=1000,
                                         free_flow_speed=15, jam_density=0.2, attribute="arterial")
        if i != 0:
            ii = i-1
            jj = j
            links[i,j,ii,jj] = W.addLink(f"l{(i,j,ii,jj)}", nodes[i,j], nodes[ii,jj], length=1000,
                                         free_flow_speed=15, jam_density=0.2, attribute="arterial")
        if j != jmax-1:
            ii = i
            jj = j+1
            links[i,j,ii,jj] = W.addLink(f"l{(i,j,ii,jj)}", nodes[i,j], nodes[ii,jj], length=1000,
                                         free_flow_speed=15, jam_density=0.2, attribute="arterial")
        if j != 0:
            ii = i
            jj = j-1
            links[i,j,ii,jj] = W.addLink(f"l{(i,j,ii,jj)}", nodes[i,j], nodes[ii,jj], length=1000,
                                         free_flow_speed=15, jam_density=0.2, attribute="arterial")
#diagonal highway
for i in range(imax):
    j = i
    if i != imax-1:
        ii = i+1
        jj = j+1
        links[i,j,ii,jj] = W.addLink(f"l{(i,j,ii,jj)}", nodes[i,j], nodes[ii,jj], length=1000*np.sqrt(2),
                                     free_flow_speed=30, jam_density=0.2, attribute="highway")
    if i != 0:
        ii = i-1
        jj = j-1
        links[i,j,ii,jj] = W.addLink(f"l{(i,j,ii,jj)}", nodes[i,j], nodes[ii,jj], length=1000*np.sqrt(2),
                                     free_flow_speed=30, jam_density=0.2, attribute="highway")

#traffic demand from edge to edge in grid network
demand_flow = 0.03
demand_duration = 3600
for n1 in [(0,j) for j in range(jmax)]:
    for n2 in [(imax-1,j) for j in range(jmax)]:
        W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
        W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
for n1 in [(i,0) for i in range(imax)]:
    for n2 in [(i,jmax-1) for i in range(imax)]:
        W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
        W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

#copy scenario for re-use (copy.deepcopy does not work for some reason)
W_orig = pickle.loads(pickle.dumps(W))
W.name = "_notoll"

print("#"*80)
print("#### no toll scenario")
print("#"*80)

# Simulation execution
W.exec_simulation()

# Result visualization
W.analyzer.print_simple_stats()
W.analyzer.macroscopic_fundamental_diagram(figtitle="all")
W.analyzer.macroscopic_fundamental_diagram(figtitle="arterial", links=[link for link in W.LINKS if link.attribute == "arterial"])
W.analyzer.macroscopic_fundamental_diagram(figtitle="highway", links=[link for link in W.LINKS if link.attribute == "highway"])
#trajectories on highway
W.analyzer.time_space_diagram_traj_links([links[i,i,i+1,i+1] for i in range(0,imax-1)])
W.analyzer.time_space_diagram_traj_links([links[i,i,i-1,i-1] for i in range(imax-1,0,-1)])

W.analyzer.network_anim(animation_speed_inverse=5, timestep_skip=200, detailed=0, network_font_size=0, figsize=(4,4))

####################################################################################################################
####################################################################################################################
####################################################################################################################

W2 = pickle.loads(pickle.dumps(W_orig))
W2.name = "_withtoll"

print("#"*80)
print("#### with toll scenario")
print("#"*80)

#fixed toll for highway (toll corresponds to route_choice_penalty seconds of travel time per link)
for link in W2.LINKS:
    if link.attribute == "highway":
        link.route_choice_penalty = 70

# Simulation execution
W2.exec_simulation()

# Result visualization
W2.analyzer.print_simple_stats()
W2.analyzer.macroscopic_fundamental_diagram(figtitle="all")
W2.analyzer.macroscopic_fundamental_diagram(figtitle="arterial", links=[link for link in W2.LINKS if link.attribute == "arterial"])
W2.analyzer.macroscopic_fundamental_diagram(figtitle="highway", links=[link for link in W2.LINKS if link.attribute == "highway"])
#trajectories on highway
W2.analyzer.time_space_diagram_traj_links([links[i,i,i+1,i+1] for i in range(0,imax-1)])
W2.analyzer.time_space_diagram_traj_links([links[i,i,i-1,i-1] for i in range(imax-1,0,-1)])

W2.analyzer.network_anim(animation_speed_inverse=5, timestep_skip=200, detailed=0, network_font_size=0, figsize=(4,4))


####################################################################################################################
####################################################################################################################
####################################################################################################################

# Results comparison

print("#"*80)
print("#### results comparison")
print("#"*80)

df = W.analyzer.basic_to_pandas()
all_ave_tt = df["average_travel_time"][0]
all_ave_delay = df["average_delay"][0]

df = W.analyzer.link_to_pandas()
highway_user = (df
                [df["link"].isin([l.name for l in W.LINKS if l.attribute=="highway"])]
                ["traffic_volume"]
                .sum())
highway_ave_tt = (df
                  [df["link"].isin([l.name for l in W.LINKS if l.attribute=="highway"])]
                  ["average_travel_time"]
                  .mean())

df = W2.analyzer.basic_to_pandas()
all_ave_tt2 = df["average_travel_time"][0]
all_ave_delay2 = df["average_delay"][0]

df = W2.analyzer.link_to_pandas()
highway_user2 = (df
                 [df["link"].isin([l.name for l in W.LINKS if l.attribute=="highway"])]
                 ["traffic_volume"]
                 .sum())
highway_ave_tt2 = (df
                   [df["link"].isin([l.name for l in W.LINKS if l.attribute=="highway"])]
                   ["average_travel_time"]
                   .mean())

print(f"""no toll case
- average travel time: {all_ave_tt}
- average delay: {all_ave_delay}
- # of highway link users: {highway_user}
- average highway link travel time: {highway_ave_tt}
toll case
- average travel time: {all_ave_tt2}
- average delay: {all_ave_delay2}
- # of highway link users: {highway_user2}
- average highway link travel time: {highway_ave_tt2}
""")