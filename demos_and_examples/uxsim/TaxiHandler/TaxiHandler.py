"""
Submodule for handling taxis
"""

import random, math

class TravelRequest:
    """
    A class representing a travel request (or travelers, passengers, cargo, etc.)
    """
    def __init__(s, W, orig, dest, depart_time, attribute=None):
        """
        Initializes a travel request.

        Parameters
        ----------
        W : World
            The world object.
        orig : str | Node
            The origin node of the travel request.
        dest : str | Node
            The destination node of the travel request.
        depart_time : float
            The time at which the travel request departs.
        attribute : any
            An optional attribute of the travel request. This can be used to store any information by users' need.
        """
        
        s.W = W
        s.orig = s.W.get_node(orig)
        s.dest = s.W.get_node(dest)
        s.depart_time = depart_time
        s.attribute = attribute
        s.get_taxi_time = None
        s.arrival_time = None
        s.taxi = None

    def __repr__(s):
        return f"TravelRequest({s.orig}, {s.dest}, {s.depart_time})"

    def get_on_taxi(s):
        """
        The event of getting on a taxi. This is called via `Vehicle.event_node`
        """
        #print(f"{s.W.TIME}: {s} is getting on a taxi {s.taxi}")        
        s.taxi.add_dest(s.dest)
        s.taxi.node_event[s.dest] = s.arrive_at_dest

        s.get_taxi_time = s.W.TIME
        
        del s.taxi.node_event[s.orig]
    
    def arrive_at_dest(s):
        """
        The event of arriving at the destination. This is called via `Vehicle.event_node`
        """
        #print(f"{s.W.TIME}: {s} arrived at the destination {s.dest}")
        s.arrival_time = s.W.TIME
        
        del s.taxi.node_event[s.dest]

class TaxiHandler:
    """
    A base class for handling the assignment of travel requests to taxis (or ridesourcing, ridehailing, ridesharing, robot-taxi, shared mobility, whatever).
    """
    def __init__(s, W):
        """
        Initializes a taxi handler.

        Parameters
        ----------
        W : World
            The world object.
        """
        s.travel_requests = []
        s.travel_requests_all = []
        s.W = W
        random.seed(s.W.random_seed)

    def add_travel_request(s, orig, dest, depart_time, attribute=None):
        """
        Adds a travel request to this handler.

        Parameters
        ----------
        orig : str | Node
            The origin node of the travel request.
        dest : str | Node
            The destination node of the travel request.
        depart_time : float
            The time at which the travel request departs.
        attribute : any
            An optional attribute of the travel request. This can be used to store any information by users' need.
        """
        s.travel_requests.append(TravelRequest(s.W, orig, dest, depart_time, attribute=attribute))
        s.travel_requests_all.append(s.travel_requests[-1])
    
    def assign_taxi(s, vehicle, travel_request):
        """
        Assigns a travel request to a vehicle.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to assign the travel request to.
        """
        #print(f"{s.W.TIME}: Assigning {travel_request} to {vehicle}")

        if travel_request not in s.travel_requests:
            raise ValueError("Travel request not found in the list of travel requests")

        vehicle.add_dest(travel_request.orig)
        vehicle.node_event[travel_request.orig] = travel_request.get_on_taxi
        travel_request.taxi = vehicle
        s.travel_requests.remove(travel_request)
    
    def compute_stats(s):
        s.n_total_requests = len(s.travel_requests_all)
        s.n_completed_requests = 0
        s.waiting_times = []
        s.travel_times = []
        s.invehicle_times = []
        for tr in s.travel_requests_all:
            if tr.arrival_time != None:
                s.n_completed_requests += 1
                s.invehicle_times.append(tr.arrival_time - tr.get_taxi_time)
                s.travel_times.append(tr.arrival_time - tr.depart_time)
            if tr.get_taxi_time != None:
                s.waiting_times.append(tr.get_taxi_time - tr.depart_time)


class TaxiHandler_random(TaxiHandler):
    """
    A naive taxi handler that assigns travel requests to random taxis. 
    """
    def __init__(s, W):
        super().__init__(W)
    
    def assign_travel_request_to_taxi(s):
        """
        Assigns travel request to random available taxi.
        """
        vacant_taxis = [veh for veh in s.W.VEHICLES.values() if veh.mode == "taxi" and veh.state == "run" and veh.dest == None]
        random.shuffle(vacant_taxis)
        for travel_request in s.travel_requests[:]:
            if len(vacant_taxis) == 0:
                break
            if travel_request.depart_time <= s.W.TIME:
                taxi = vacant_taxis.pop()
                s.assign_taxi(taxi, travel_request)


class TaxiHandler_nearest(TaxiHandler):
    """
    A taxi handler that assigns travel requests to nearest taxis. 
    """
    def __init__(s, W):
        super().__init__(W)
    
    def assign_travel_request_to_taxi(s):
        """
        Assigns travel request to nearest available taxi.
        """
        vacant_taxis = [veh for veh in s.W.VEHICLES.values() if veh.mode == "taxi" and veh.state == "run" and veh.dest == None]
        random.shuffle(vacant_taxis)
        for travel_request in s.travel_requests[:]:
            if len(vacant_taxis) == 0:
                break
            if travel_request.depart_time <= s.W.TIME:
                dist_tmp = float("inf")
                taxi_tmp = None
                for taxi in vacant_taxis[:]:
                    x0, y0 = taxi.get_xy_coords(-1)
                    x1, y1 = travel_request.orig.x, travel_request.orig.y
                    dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)
                    if dist <= dist_tmp:
                        dist_tmp = dist
                        taxi_tmp = taxi
                if taxi_tmp != None:
                    vacant_taxis.remove(taxi_tmp)
                    s.assign_taxi(taxi_tmp, travel_request)


class TaxiHandler_nearest_matching_radious(TaxiHandler):
    """
    A taxi handler that assigns travel requests to nearest taxis that are within a certain radious of the origin node. 
    """
    def __init__(s, W, matching_radious):
        super().__init__(W)
        s.matching_radious = matching_radious
    
    def assign_travel_request_to_taxi(s):
        """
        Assigns travel request to nearest available taxi that is within the radious of the origin node.
        """
        vacant_taxis = [veh for veh in s.W.VEHICLES.values() if veh.mode == "taxi" and veh.state == "run" and veh.dest == None]
        random.shuffle(vacant_taxis)
        for travel_request in s.travel_requests[:]:
            if len(vacant_taxis) == 0:
                break
            if travel_request.depart_time <= s.W.TIME:
                dist_tmp = float("inf")
                taxi_tmp = None
                for taxi in vacant_taxis[:]:
                    x0, y0 = taxi.get_xy_coords(-1)
                    x1, y1 = travel_request.orig.x, travel_request.orig.y
                    dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)
                    if dist <= s.matching_radious and dist <= dist_tmp:
                        dist_tmp = dist
                        taxi_tmp = taxi
                if taxi_tmp != None:
                    vacant_taxis.remove(taxi_tmp)
                    s.assign_taxi(taxi_tmp, travel_request)