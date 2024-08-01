"""
This script verifies whether UXsim outputs reasonable solutions with taxis.
"""

import pytest
from uxsim import *
from uxsim.TaxiHandler import *
from math import sin, cos, pi

def equal_tolerance(val, check, rel_tol=0.1, abs_tol=0.0):
    if check == 0 and abs_tol == 0:
        abs_tol = 0.1
    return abs(val - check) <= abs(check*rel_tol) + abs_tol

"""
default FD:
    u = 20
    kappa = 0.2
    tau = 1
    w = 5
    k^* = 0.04
    q^* = 0.8
"""

@pytest.mark.flaky(reruns=10)
def test_taxi_small_scale():
    W = World(
        name="",  
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1, 
        random_seed=None
    )

    # Scenario definition: circular road
    n1 = W.addNode("1", sin(1/10*2*pi), cos(1/10*2*pi))
    n2 = W.addNode("2", sin(2/10*2*pi), cos(2/10*2*pi))
    n3 = W.addNode("3", sin(3/10*2*pi), cos(3/10*2*pi))
    n4 = W.addNode("4", sin(4/10*2*pi), cos(4/10*2*pi))
    n5 = W.addNode("5", sin(5/10*2*pi), cos(5/10*2*pi))
    n6 = W.addNode("6", sin(6/10*2*pi), cos(6/10*2*pi))
    n7 = W.addNode("7", sin(7/10*2*pi), cos(7/10*2*pi))
    n8 = W.addNode("8", sin(8/10*2*pi), cos(8/10*2*pi))
    n9 = W.addNode("9", sin(9/10*2*pi), cos(9/10*2*pi))
    n10 = W.addNode("10", sin(10/10*2*pi), cos(10/10*2*pi))

    for no,nd in [[n1,n2],[n2,n3],[n3,n4],[n4,n5],[n5,n6],[n6,n7],[n7,n8],[n8,n9],[n9,n10],[n10,n1]]:
        W.addLink(f"{no.name}-{nd.name}", no, nd, length=500, free_flow_speed=20)
        W.addLink(f"{nd.name}-{no.name}", nd, no, length=500, free_flow_speed=20)

    # only 2 taxis
    veh0 = W.addVehicle(n1, None, 0, mode="taxi")
    veh1 = W.addVehicle(n1, None, 0, mode="taxi")

    # only 6 trips
    Handler = TaxiHandler_nearest(W)
    Handler.add_trip_request(n1, n5, 100)
    Handler.add_trip_request(n1, n5, 101)
    Handler.add_trip_request(n1, n5, 102)
    Handler.add_trip_request(n1, n5, 103)
    Handler.add_trip_request(n1, n5, 104)
    Handler.add_trip_request(n1, n5, 105)

    # Run the simulation 
    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t = 60)
        Handler.assign_trip_request_to_taxi() # for every 60 seconds, the taxi is assgined

    # Results
    W.analyzer.print_simple_stats()

    Handler.print_stats()

    assert equal_tolerance(Handler.invehicle_times[0], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[1], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[2], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[3], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[4], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[5], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.travel_times[0], 150, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[1], 150, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[2], 400, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[3], 400, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[4], 600, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[5], 600, rel_tol=0.3)

@pytest.mark.flaky(reruns=10)
def test_taxi_small_scale_mixed_mode():        
    # World definition
    W = World(
        name="",  
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1, 
        random_seed=None
    )

    # Scenario definition: circular road
    n1 = W.addNode("1", sin(1/10*2*pi), cos(1/10*2*pi))
    n2 = W.addNode("2", sin(2/10*2*pi), cos(2/10*2*pi))
    n3 = W.addNode("3", sin(3/10*2*pi), cos(3/10*2*pi))
    n4 = W.addNode("4", sin(4/10*2*pi), cos(4/10*2*pi))
    n5 = W.addNode("5", sin(5/10*2*pi), cos(5/10*2*pi))
    n6 = W.addNode("6", sin(6/10*2*pi), cos(6/10*2*pi))
    n7 = W.addNode("7", sin(7/10*2*pi), cos(7/10*2*pi))
    n8 = W.addNode("8", sin(8/10*2*pi), cos(8/10*2*pi))
    n9 = W.addNode("9", sin(9/10*2*pi), cos(9/10*2*pi))
    n10 = W.addNode("10", sin(10/10*2*pi), cos(10/10*2*pi))

    for no,nd in [[n1,n2],[n2,n3],[n3,n4],[n4,n5],[n5,n6],[n6,n7],[n7,n8],[n8,n9],[n9,n10],[n10,n1]]:
        W.addLink(f"{no.name}-{nd.name}", no, nd, length=500, free_flow_speed=20)
        W.addLink(f"{nd.name}-{no.name}", nd, no, length=500, free_flow_speed=20)

    # only 2 taxis
    veh0 = W.addVehicle(n1, None, 0, mode="taxi")
    veh1 = W.addVehicle(n1, None, 0, mode="taxi")

    # only 6 trips
    Handler = TaxiHandler_nearest(W)
    Handler.add_trip_request(n1, n5, 100)
    Handler.add_trip_request(n1, n5, 101)
    Handler.add_trip_request(n1, n5, 102)
    Handler.add_trip_request(n1, n5, 103)
    Handler.add_trip_request(n1, n5, 104)
    Handler.add_trip_request(n1, n5, 105)

    W.adddemand(n1, n5, 0, 600, volume=20)
    W.adddemand(n2, n5, 0, 600, volume=20)

    # Run the simulation 
    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t = 60)
        Handler.assign_trip_request_to_taxi() # for every 60 seconds, the taxi is assgined

    # Results
    W.analyzer.print_simple_stats()
    df = W.analyzer.basic_to_pandas()

    assert equal_tolerance(df["total_trips"][0], 40)
    assert equal_tolerance(df["completed_trips"][0], 40)
    assert equal_tolerance(df["average_travel_time"][0], 85)

    Handler.print_stats()
    
    assert equal_tolerance(Handler.invehicle_times[0], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[1], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[2], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[3], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[4], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.invehicle_times[5], 100, rel_tol=0.1)
    assert equal_tolerance(Handler.travel_times[0], 150, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[1], 150, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[2], 400, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[3], 400, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[4], 600, rel_tol=0.3)
    assert equal_tolerance(Handler.travel_times[5], 600, rel_tol=0.3)

@pytest.mark.flaky(reruns=10)
def test_taxi_large_scale_mixed_mode_noncongested():

    # World definition
    tmax = 7200
    deltan = 5
    W = World(
        name="",
        deltan=deltan,
        tmax=tmax,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=None,
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

    # taxis and passengers
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

    #taxis
    for i in range(int(n_taxis/deltan)):
        node = random.choice(list(nodes.values()))
        W.addVehicle(node, None, 0, mode="taxi")

    #taxi travelers
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

    df_vehi = W.analyzer.basic_to_pandas()

    df_taxi = Handler.basic_to_pandas()
    
    df_trips = Handler.trips_to_pandas()

    assert equal_tolerance(df_vehi["average_travel_time"][0], df_taxi["average_invehicle_time"][0])
    assert equal_tolerance(df_vehi["completed_trips"][0], 10000)
    assert equal_tolerance(df_taxi["completed_trips"][0], 10000)
    assert equal_tolerance(df_taxi["average_travel_time"][0], 300, rel_tol=0.2)


def test_taxi_small_scale_mixed_mode_random_just_run():        
    # World definition
    W = World(
        name="",  
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1, 
        random_seed=None
    )

    # Scenario definition: circular road
    n1 = W.addNode("1", sin(1/10*2*pi), cos(1/10*2*pi))
    n2 = W.addNode("2", sin(2/10*2*pi), cos(2/10*2*pi))
    n3 = W.addNode("3", sin(3/10*2*pi), cos(3/10*2*pi))
    n4 = W.addNode("4", sin(4/10*2*pi), cos(4/10*2*pi))
    n5 = W.addNode("5", sin(5/10*2*pi), cos(5/10*2*pi))
    n6 = W.addNode("6", sin(6/10*2*pi), cos(6/10*2*pi))
    n7 = W.addNode("7", sin(7/10*2*pi), cos(7/10*2*pi))
    n8 = W.addNode("8", sin(8/10*2*pi), cos(8/10*2*pi))
    n9 = W.addNode("9", sin(9/10*2*pi), cos(9/10*2*pi))
    n10 = W.addNode("10", sin(10/10*2*pi), cos(10/10*2*pi))

    for no,nd in [[n1,n2],[n2,n3],[n3,n4],[n4,n5],[n5,n6],[n6,n7],[n7,n8],[n8,n9],[n9,n10],[n10,n1]]:
        W.addLink(f"{no.name}-{nd.name}", no, nd, length=500, free_flow_speed=20)
        W.addLink(f"{nd.name}-{no.name}", nd, no, length=500, free_flow_speed=20)

    # only 2 taxis
    veh0 = W.addVehicle(n1, None, 0, mode="taxi")
    veh1 = W.addVehicle(n1, None, 0, mode="taxi")

    # only 6 trips
    Handler = TaxiHandler_random(W)
    Handler.add_trip_request(n1, n5, 100)
    Handler.add_trip_request(n1, n5, 101)
    Handler.add_trip_request(n1, n5, 102)
    Handler.add_trip_request(n1, n5, 103)
    Handler.add_trip_request(n1, n5, 104)
    Handler.add_trip_request(n1, n5, 105)

    W.adddemand(n1, n5, 0, 600, volume=20)
    W.adddemand(n2, n5, 0, 600, volume=20)

    # Run the simulation 
    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t = 60)
        Handler.assign_trip_request_to_taxi() # for every 60 seconds, the taxi is assgined

    # Results
    W.analyzer.print_simple_stats()

    assert True
    
def test_taxi_small_scale_mixed_mode_matching_radious_just_run():        
    # World definition
    W = World(
        name="",  
        deltan=5,
        tmax=1200,
        print_mode=1, save_mode=1, show_mode=1, 
        random_seed=None
    )

    # Scenario definition: circular road
    n1 = W.addNode("1", sin(1/10*2*pi), cos(1/10*2*pi))
    n2 = W.addNode("2", sin(2/10*2*pi), cos(2/10*2*pi))
    n3 = W.addNode("3", sin(3/10*2*pi), cos(3/10*2*pi))
    n4 = W.addNode("4", sin(4/10*2*pi), cos(4/10*2*pi))
    n5 = W.addNode("5", sin(5/10*2*pi), cos(5/10*2*pi))
    n6 = W.addNode("6", sin(6/10*2*pi), cos(6/10*2*pi))
    n7 = W.addNode("7", sin(7/10*2*pi), cos(7/10*2*pi))
    n8 = W.addNode("8", sin(8/10*2*pi), cos(8/10*2*pi))
    n9 = W.addNode("9", sin(9/10*2*pi), cos(9/10*2*pi))
    n10 = W.addNode("10", sin(10/10*2*pi), cos(10/10*2*pi))

    for no,nd in [[n1,n2],[n2,n3],[n3,n4],[n4,n5],[n5,n6],[n6,n7],[n7,n8],[n8,n9],[n9,n10],[n10,n1]]:
        W.addLink(f"{no.name}-{nd.name}", no, nd, length=500, free_flow_speed=20)
        W.addLink(f"{nd.name}-{no.name}", nd, no, length=500, free_flow_speed=20)

    # only 2 taxis
    veh0 = W.addVehicle(n1, None, 0, mode="taxi")
    veh1 = W.addVehicle(n1, None, 0, mode="taxi")

    # only 6 trips
    Handler = TaxiHandler_nearest_matching_radious(W, 0.5)
    Handler.add_trip_request(n1, n5, 100)
    Handler.add_trip_request(n1, n5, 101)
    Handler.add_trip_request(n1, n5, 102)
    Handler.add_trip_request(n1, n5, 103)
    Handler.add_trip_request(n1, n5, 104)
    Handler.add_trip_request(n1, n5, 105)

    W.adddemand(n1, n5, 0, 600, volume=20)
    W.adddemand(n2, n5, 0, 600, volume=20)

    # Run the simulation 
    while W.check_simulation_ongoing():
        W.exec_simulation(duration_t = 60)
        Handler.assign_trip_request_to_taxi() # for every 60 seconds, the taxi is assgined

    # Results
    W.analyzer.print_simple_stats()

    assert True