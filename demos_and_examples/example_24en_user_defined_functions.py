import numpy as np
from uxsim import *

# define custom user_functions

def world_user_function(W):
    """
    This is a custom user function to be attached to `World` object and automatically executed on each timestep during the simulation.
    This can be specified as an optional argument `user_function` in the initialization function or by doing `W.user_function=world_user_function` after initialization.

    User functions must have only one argument: the `World` object itself.
    It can access to the properties of the `World` object via the argument.
    Users can also define custom properties of the `World` object by using the variable `World.user_attribute`; This variable can be used by the user freely.

    This simple example prints the current time and the average speed of traveling vehicles in the World during the simulation.
    Users can do more complicated things like traffic control or data collection.
    """
    if W.T%20 == 0:
        if len(W.VEHICLES_RUNNING):
            print("World state:", W.TIME, np.average([veh.v for veh in W.VEHICLES_RUNNING.values()]))
        else:
            print("World state:", W.TIME)

def node_user_function(node):
    #print the number of incoming vehicles to node. The specification is similar to world_user_function
    if node.W.T%20 == 0:
        print(f"Node {node.name} state:", len(node.incoming_vehicles))

def link_user_function(link):
    #print the number of vehicles on link and their average speed, and record the speed. The specification is similar to world_user_function
    if link.W.T%20 == 0:
        if len(link.vehicles):
            print(f"Link {link.name} state:", len(link.vehicles), np.average([veh.v for veh in link.vehicles]))
            link.user_attribute["speed_record"].append(np.average([veh.v for veh in link.vehicles]))
        else:
            print(f"Link {link.name} state:", len(link.vehicles))

def veh_user_function(veh):
    #print when the vehicle started or ended its trip. The specification is similar to world_user_function
    if veh.state != "home" and veh.user_attribute["state_old"] == "home":
        print(f"Vehicle {veh.name} started trip on time {veh.W.TIME}")        
    if veh.state in ["end", "abort"] and veh.user_attribute["state_old"] not in ["end", "abort"]:
        print(f"Vehicle {veh.name} ended trip on time {veh.W.TIME}")
    veh.user_attribute["state_old"] = veh.state

# define uxsim scenario

W = World(
    name="",
    deltan=5,
    tmax=1200, 
    print_mode=1, save_mode=1, show_mode=0,
    random_seed=0,
    user_function=world_user_function
)

W.addNode("orig1", 0, 0)
W.addNode("orig2", 0, 2)
W.addNode("merge", 1, 1, user_function=node_user_function)
W.addNode("dest", 2, 1)
W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, number_of_lanes=1, user_function=link_user_function, user_attribute={"speed_record":[]})
W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
W.adddemand("orig1", "dest", 0, 1000, 0.45)
W.adddemand("orig2", "dest", 400, 1000, 0.6)

W.VEHICLES["10"].user_attribute = {"state_old":None}
W.VEHICLES["10"].user_function = veh_user_function
W.VEHICLES["120"].user_attribute = {"state_old":None}
W.VEHICLES["120"].user_function = veh_user_function

W.exec_simulation()

W.analyzer.print_simple_stats()

# prosess data collected by user_function

link1 = W.get_link("link1")
print("average speed of link1:", np.nanmean(link1.user_attribute["speed_record"]))
