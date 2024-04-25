from uxsim import *
from uxsim.TaxiHandler import *

# World definition
tmax = 7200
deltan = 5
W = World(
    name="",
    deltan=deltan,
    tmax=tmax,
    print_mode=1, save_mode=1, show_mode=1,
    random_seed=0,
)

# Scenario definition: grid network
#deploy nodes as an imax x jmax grid
imax = 6
jmax = 6
nodes = {}
for i in range(imax):
    for j in range(jmax):
        nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

#create links between neighborhood nodes
links = {}
for i in range(imax):
    for j in range(jmax):
        if i != imax-1:
            links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=20, number_of_lanes=1)
        if i != 0:
            links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=20, number_of_lanes=1)
        if j != jmax-1:
            links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=20, number_of_lanes=1)
        if j != 0:
            links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=20, number_of_lanes=1)

# Demand
Handler = TaxiHandler_nearest(W)
n_total_demand = 20000
n_car_travelers = 10000
n_taxi_travelers = n_total_demand-n_car_travelers
n_taxis = 1000

#car travelers
for i in range(int(n_car_travelers/deltan)):
    node1 = random.choice(list(nodes.values()))
    node2 = random.choice(list(nodes.values()))
    while node1 == node2:
        node2 = random.choice(list(nodes.values()))
    W.addVehicle(node1, node2, i/n_car_travelers*deltan*tmax/2)

#carsharing (taxi)
for i in range(int(n_taxis/deltan)):
    node = random.choice(list(nodes.values()))
    W.addVehicle(node, None, 0, mode="taxi")

#carsharing (taxi) travelers
for i in range(int(n_taxi_travelers/deltan)):
    node1 = random.choice(list(nodes.values()))
    node2 = random.choice(list(nodes.values()))
    while node1 == node2:
        node2 = random.choice(list(nodes.values()))
    Handler.add_trip_request(node1, node2, i/n_taxi_travelers*deltan*tmax/2)

# Run the simulation 
while W.check_simulation_ongoing():
    W.exec_simulation(duration_t = 60)
    Handler.assign_trip_request_to_taxi() # for every 60 seconds, the taxi is assgined

# Results
W.analyzer.print_simple_stats()

Handler.print_stats()
