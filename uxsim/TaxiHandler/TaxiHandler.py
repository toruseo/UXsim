"""
Submodule for handling taxis
"""

import random, math
import pandas as pd

class TripRequest:
    """
    A class representing a trip request (or travelers, passengers, cargo, etc.)
    """
    def __init__(s, W, H, orig, dest, depart_time, name=None, attribute=None):
        """
        Initializes a trip request.

        Parameters
        ----------
        W : World
            The world object.
        H : TaxiHandler
            The TaxiHandler object.
        orig : str | Node
            The origin node of the trip request.
        dest : str | Node
            The destination node of the trip request.
        depart_time : float
            The time at which the trip request departs.
        name : any, optional
            The name of the trip request. Default is None.
        attribute : any
            An optional attribute of the trip request. This can be used to store any information by users' need.
        """
        
        s.W = W
        s.H = H
        s.orig = s.W.get_node(orig)
        s.dest = s.W.get_node(dest)
        s.depart_time = depart_time
        s.attribute = attribute
        s.name = name

        s.get_taxi_time = None
        s.arrival_time = None
        s.taxi = None

    def __repr__(s):
        return f"TravelRequest({s.name}: {s.orig}, {s.dest}, {s.depart_time})"

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
    A base class for handling the assignment of trip requests to taxis (or ridesourcing, ridehailing, ridesharing, robot-taxi, shared mobility, whatever).
    """
    def __init__(s, W):
        """
        Initializes a taxi handler.

        Parameters
        ----------
        W : World
            The world object.
        """
        s.trip_requests = []
        s.trip_requests_all = []
        s.W = W
        random.seed(s.W.random_seed)

    def add_trip_request(s, orig, dest, depart_time, name=None, attribute=None):
        """
        Adds a trip request to this handler.

        Parameters
        ----------
        orig : str | Node
            The origin node of the trip request.
        dest : str | Node
            The destination node of the trip request.
        depart_time : float
            The time at which the trip request departs.
        name : any, optional
            The name of the trip request. Default is None.
        attribute : any
            An optional attribute of the trip request. This can be used to store any information by users' need.
        """
        if name == None:
            name = len(s.trip_requests)
        s.trip_requests.append(TripRequest(s.W, s, orig, dest, depart_time, name, attribute=attribute))
        s.trip_requests_all.append(s.trip_requests[-1])
    
    def assign_taxi(s, vehicle, trip_request):
        """
        Assigns a trip request to a vehicle.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to assign the trip request to.
        """
        #print(f"{s.W.TIME}: Assigning {trip_request} to {vehicle}")

        if trip_request not in s.trip_requests:
            raise ValueError("Travel request not found in the list of trip requests")

        vehicle.add_dest(trip_request.orig)
        vehicle.node_event[trip_request.orig] = trip_request.get_on_taxi
        trip_request.taxi = vehicle
        s.trip_requests.remove(trip_request)
    
    def get_trip(s, tr):
        """
        Get a TripRequest instance by name or object.

        Parameters
        ----------
        tr : any | TripRequest
            The name or object of the trip request.
        """
        if type(tr) is TripRequest:
            if tr in s.trip_requests_all:
                return tr
        else:
            for trr in s.trip_requests_all:
                if tr == trr.name:
                    return trr
        raise Exception(f"'{tr}' is not TripRequest of this TaxiHandler")


    def compute_stats(s):
        """
        Computes basic statistics of the taxi travels.
        """

        s.n_total_requests = len(s.trip_requests_all)*s.W.DELTAN
        s.n_completed_requests = 0
        s.waiting_times = []
        s.travel_times = []
        s.invehicle_times = []
        s.number_of_taxis = sum([1 for veh in s.W.VEHICLES.values() if veh.mode == "taxi"])*s.W.DELTAN
        for tr in s.trip_requests_all:
            if tr.arrival_time != None:
                s.n_completed_requests += s.W.DELTAN
                s.invehicle_times.append(tr.arrival_time - tr.get_taxi_time)
                s.travel_times.append(tr.arrival_time - tr.depart_time)
            if tr.get_taxi_time != None:
                s.waiting_times.append(tr.get_taxi_time - tr.depart_time)
    
    def basic_to_pandas(s):
        """
        Converts the basic statistics on taxi travels to a pandas DataFrame.
        """
        s.compute_stats()
        data = {
            "total_trips": [s.n_total_requests],
            "completed_trips": [s.n_completed_requests],
            "average_waiting_time": [sum(s.waiting_times)/len(s.waiting_times) if len(s.waiting_times) > 0 else 0],
            "average_invehicle_time": [sum(s.invehicle_times)/len(s.invehicle_times) if len(s.invehicle_times) > 0 else 0],
            "average_travel_time": [sum(s.travel_times)/len(s.travel_times) if len(s.travel_times) > 0 else 0],
        }
        return pd.DataFrame(data)
    
    def print_stats(s):
        """
        Prints the basic statistics.
        """
        s.compute_stats()
        print("results for taxi transportation:")
        print(f" total trip rquests: {s.n_total_requests}")
        print(f" completed trip requests: {s.n_completed_requests}")
        print(f" completed trip requests ratio: {s.n_completed_requests/s.n_total_requests if s.n_total_requests > 0 else 0: .2f}")
        print(f" average number of completed requests per taxi: {s.n_completed_requests/s.number_of_taxis: .2f}")
        print(f" average waiting time: {sum(s.waiting_times)/len(s.waiting_times) if len(s.waiting_times) > 0 else 0: .1f}")
        print(f" average in-vehicle time: {sum(s.invehicle_times)/len(s.invehicle_times) if len(s.invehicle_times) > 0 else 0: .1f}")
        print(f" average trip time: {sum(s.travel_times)/len(s.travel_times) if len(s.travel_times) > 0 else 0: .1f}")
    
    def trips_to_pandas(s):
        """
        Converts the trips and their travel records to a pandas DataFrame.
        """
        s.compute_stats()
        data = {
            "orig": [tr.orig.name for tr in s.trip_requests_all],
            "dest": [tr.dest.name for tr in s.trip_requests_all],
            "depart_time": [tr.depart_time for tr in s.trip_requests_all],
            "get_taxi_time": [tr.get_taxi_time for tr in s.trip_requests_all],
            "arrival_time": [tr.arrival_time for tr in s.trip_requests_all],
            "travel_time": [tr.arrival_time - tr.depart_time if tr.arrival_time != None else None for tr in s.trip_requests_all],
            "waiting_time": [tr.get_taxi_time - tr.depart_time if tr.get_taxi_time != None else None for tr in s.trip_requests_all],
            "used_taxi": [tr.taxi.name if tr.taxi != None else None for tr in s.trip_requests_all],
        }
        return pd.DataFrame(data)


class TaxiHandler_random(TaxiHandler):
    """
    A naive taxi handler that assigns trip requests to random taxis. 
    """
    def __init__(s, W):
        super().__init__(W)
    
    def assign_trip_request_to_taxi(s):
        """
        Assigns trip request to random available taxi.
        """
        vacant_taxis = [veh for veh in s.W.VEHICLES.values() if veh.mode == "taxi" and veh.state == "run" and veh.dest == None]
        random.shuffle(vacant_taxis)
        for trip_request in s.trip_requests[:]:
            if len(vacant_taxis) == 0:
                break
            if trip_request.depart_time <= s.W.TIME:
                taxi = vacant_taxis.pop()
                s.assign_taxi(taxi, trip_request)


class TaxiHandler_nearest(TaxiHandler):
    """
    A taxi handler that assigns trip requests to nearest taxis (based on Euclidean distance). 
    """
    def __init__(s, W):
        super().__init__(W)
    
    def assign_trip_request_to_taxi(s):
        """
        Assigns trip request to nearest available taxi.
        """
        vacant_taxis = [veh for veh in s.W.VEHICLES.values() if veh.mode == "taxi" and veh.state == "run" and veh.dest == None]
        for trip_request in s.trip_requests[:]:
            if len(vacant_taxis) == 0:
                break
            if trip_request.depart_time <= s.W.TIME:
                dist_tmp = float("inf")
                taxi_tmp = None
                for taxi in vacant_taxis[:]:
                    x0, y0 = taxi.get_xy_coords()
                    x1, y1 = trip_request.orig.x, trip_request.orig.y
                    dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)
                    if dist <= dist_tmp:
                        dist_tmp = dist
                        taxi_tmp = taxi
                if taxi_tmp != None:
                    vacant_taxis.remove(taxi_tmp)
                    s.assign_taxi(taxi_tmp, trip_request)


class TaxiHandler_nearest_matching_radious(TaxiHandler):
    """
    A taxi handler that assigns trip requests to nearest taxis that are within a certain radious of the origin node (based on Euclidean distance. 
    """
    def __init__(s, W, matching_radious):
        super().__init__(W)
        s.matching_radious = matching_radious
    
    def assign_trip_request_to_taxi(s):
        """
        Assigns trip request to nearest available taxi that is within the radious of the origin node.
        """
        vacant_taxis = [veh for veh in s.W.VEHICLES.values() if veh.mode == "taxi" and veh.state == "run" and veh.dest == None]
        for trip_request in s.trip_requests[:]:
            if len(vacant_taxis) == 0:
                break
            if trip_request.depart_time <= s.W.TIME:
                dist_tmp = float("inf")
                taxi_tmp = None
                for taxi in vacant_taxis[:]:
                    x0, y0 = taxi.get_xy_coords()
                    x1, y1 = trip_request.orig.x, trip_request.orig.y
                    dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)
                    if dist <= s.matching_radious and dist <= dist_tmp:
                        dist_tmp = dist
                        taxi_tmp = taxi
                if taxi_tmp != None:
                    vacant_taxis.remove(taxi_tmp)
                    s.assign_taxi(taxi_tmp, trip_request)


class TaxiHandler_nearest_network_distance(TaxiHandler):
    """
    A taxi handler that assigns trip requests to nearest taxis based on network distance considering travel time as well. 
    This would be useful, but not implemented.
    """
    pass