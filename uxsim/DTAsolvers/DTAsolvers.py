"""
Submodule for Dynamic Traffic Assginment solvers
"""
import random
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from ..Utilities import enumerate_k_shortest_routes, enumerate_k_random_routes
from ..utils import *

import warnings

class SolverDUE:
    def __init__(s, func_World):
        """
        Solve quasi Dynamic User Equilibrium (DUE) problem using day-to-day dynamics. WIP

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
            
        Notes
        -----
            This function computes a near dynamic user equilibirum state as a steady state of day-to-day dynamical routing game.

            Specifically, on day `i`, vehicles choose their route based on actual travel time on day `i-1` with the same departure time.
            If there are shorter travel time route, they will change with probability `swap_prob`.
            This process is repeated until `max_iter` day.
            It is expected that this process eventually reach a steady state.
            Due to the problem complexity, it does not necessarily reach or converge to Nash equilibrium or any other stationary points.
            However, in the literature, it is argued that the steady state can be considered as a reasonable proxy for Nash equilibrium or dynamic equilibrium state.
            There are some theoretical background for it; but intuitively speaking, the steady state can be considered as a realistic state that people's rational behavior will reach.

            This method is based on the following literature:
            Ishihara, M., & Iryo, T. (2015). Dynamic Traffic Assignment by Markov Chain. Journal of Japan Society of Civil Engineers, Ser. D3 (Infrastructure Planning and Management), 71(5), I_503-I_509. (in Japanese). https://doi.org/10.2208/jscejipm.71.I_503
            Iryo, T., Urata, J., & Kawase, R. (2024). Traffic Flow Simulator and Travel Demand Simulators for Assessing Congestion on Roads After a Major Earthquake. In APPLICATION OF HIGH-PERFORMANCE COMPUTING TO EARTHQUAKE-RELATED PROBLEMS (pp. 413-447). https://doi.org/10.1142/9781800614635_0007
            Iryo, T., Watling, D., & Hazelton, M. (2024). Estimating Markov Chain Mixing Times: Convergence Rate Towards Equilibrium of a Stochastic Process Traffic Assignment Model. Transportation Science. https://doi.org/10.1287/trsc.2024.0523
        """
        s.func_World = func_World
        s.W_sol = None
        s.dfs_link = []
    
    def solve(s, max_iter, n_routes_per_od=4, swap_prob=0.05, print_info=True):
        """
        Solve quasi Dynamic User Equilibrium (DUE) problem using day-to-day dynamics. WIP

        Parameters
        ----------
        max_iter : int
            maximum number of iterations
        n_routes_per_od : int
            number of routes to enumerate for each OD pair
        swap_prob : float
            probability of route swap
        print_info : bool
            whether to print the information

        Returns
        -------
        W : World
            World object with quasi DUE solution (if properly converged)
        """

        W_orig = s.func_World()
        if print_info:
            W_orig.print_scenario_stats()

        # enumerate routes for each OD pair
        n_routes_per_od = n_routes_per_od

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W_orig.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        # dict_od_to_routes = {}
        # for o,d in dict_od_to_vehid.keys():
        #     routes = enumerate_k_shortest_routes(W_orig, o, d, k=n_routes_per_od)
        #     dict_od_to_routes[o,d] = routes

        if W_orig.finalized == False:
            W_orig.finalize_scenario()
        dict_od_to_routes = enumerate_k_random_routes(W_orig, k=n_routes_per_od)

        if print_info:
            print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

        # day-to-day dynamics
        s.ttts = []
        s.n_swaps = []
        s.potential_swaps = []
        s.total_t_gaps = []
        s.route_log = []
        s.cost_log = []
        swap_prob = swap_prob
        max_iter = max_iter #50 or 100 is better

        if print_info:
            print("solving DUE...")
        for i in range(max_iter):
            W = s.func_World()
            if i != max_iter-1:
                W.vehicle_logging_timestep_interval = -1

            if i != 0:
                for key in W.VEHICLES:
                    if key in routes_specified:
                        W.VEHICLES[key].enforce_route(routes_specified[key])
            
            route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
            for o,d in dict_od_to_vehid.keys():
                for r in dict_od_to_routes[o,d]:
                    route_set[o,d].append(W.defRoute(r))

            # simulation
            W.exec_simulation()

            # results
            W.analyzer.print_simple_stats()
            #W.analyzer.network_average()

            s.dfs_link.append(W.analyzer.link_to_pandas())

            # route swap
            routes_specified = {}
            route_actual = {}
            cost_actual = {}
            n_swap = 0
            total_t_gap = 0
            potential_n_swap = 0
            for key,veh in W.VEHICLES.items():
                o = veh.orig.name
                d = veh.dest.name
                r, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]
                
                route_actual[key] = [rr.name for rr in r]
                cost_actual[key] = travel_time

                if veh.state != "end":
                    continue

                flag_route_changed = False
                route_changed = None
                t_gap = 0
                
                for alt_route in route_set[o,d]:
                    if alt_route.actual_travel_time(ts[0]) < r.actual_travel_time(ts[0]):
                        if flag_route_changed == False or (alt_route.actual_travel_time(ts[0]) < route_changed.actual_travel_time(ts[0])):
                            t_gap = r.actual_travel_time(ts[0]) - alt_route.actual_travel_time(ts[0])
                            potential_n_swap += 1 #there may be double count; but not critical issue
                            if random.random() < swap_prob:
                                flag_route_changed = True
                                route_changed = alt_route

                
                total_t_gap += t_gap
                routes_specified[key] = r
                if flag_route_changed:
                    n_swap += 1
                    routes_specified[key] = route_changed

            if print_info:
                print(f' iter {i}: time gap: {total_t_gap:.1f}, potential route change: {potential_n_swap}, route change: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)

            s.ttts.append(int(W.analyzer.total_travel_time))
            s.n_swaps.append(n_swap)
            s.potential_swaps.append(potential_n_swap)
            s.total_t_gaps.append(total_t_gap)

            # if i == 0:
            #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_0init.gif")
            # if i == int(max_iter/2):
            #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_1mid.gif")
            # if i == max_iter-1:
            #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_2last.gif")

        s.W_sol = W
        return s.W_sol

    def plot_convergence(s):
        # iteration plot
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(s.ttts)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of route change")
        plt.plot(s.n_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of potential route change")
        plt.plot(s.potential_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("time gap for potential route change")
        plt.plot(s.total_t_gaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("time gap per potential route change")
        plt.plot(np.array(s.total_t_gaps)/np.array(s.potential_swaps))
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.show()

        # plt.figure()
        # plot_multiple_y(ys=[s.ttts, s.n_swaps, s.potential_swaps, s.total_t_gaps, np.array(s.total_t_gaps)/np.array(s.potential_swaps)], labels=["total travel time", "number of route change", "number of potential route change", "time gap for potential route change", "time gap per potential route change"])
        # plt.xlabel("iter")
        # plt.show()

    def plot_link_stats(s):
        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        ave_TT = []
        std_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                length = len(s.route_log)
                ts = [s.cost_log[day][vehid] for day in range(int(length/2), length)]
                ave_TT.append(np.average(ts))
                std_TT.append(np.std(ts))
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.errorbar(x=depature_time, y=ave_TT, yerr=std_TT, 
                fmt='bx', ecolor="#aaaaff", capsize=0, label="travel time (mean $\pm$ std)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()

class SolverDSO:
    def __init__(s, func_World):
        """
        Solve Dynamic System Optimum (DSO) problem. WIP

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
            
        Notes
        -----
            This is based on an algorithm whose stochastic convergence is theoretically guaranteed. However, the convergence is very slow. It is difficult to obtain accurate solution within a practical amount of time if the network was not small. 
            
            This method is based on the following literature:
            Satsukawa, K., Wada, K., & Watling, D. (2022). Dynamic system optimal traffic assignment with atomic users: Convergence and stability. Transportation Research Part B: Methodological, 155, 188-209. https://doi.org/10.1016/j.trb.2021.11.001
        """
        s.func_World = func_World
        s.W_sol = None
        s.dfs_link = []
    
    def solve(s, max_iter, n_routes_per_od=4, beta_coef=100, beta_coef2=5000, print_info=True, print_info_detailed=False, plot_res=False, initial_solution_World=None):
        """
        Solve Dynamic System Optimum (DSO) problem. WIP

        Parameters
        ----------
        max_iter : int
            maximum number of iterations
        n_routes_per_od : int
            number of routes to enumerate for each OD pair
        beta_coef : float
            coefficient for logit response dynamics
        beta_coef2 : float
            coefficient for logit response dynamics
        print_info : bool
            whether to print the information
        print_info_detailed : bool
            whether to print the detailed information
        plot_res : bool
            whether to plot the results
        initial_solution_World : World
            initial solution (starting point) for the optimization algorithm. It must have the same structure as the output of func_World, and its simulation has been already executed. Recommended example: `W_init = solve_DUE(func_World); W = solve_DSO(func_World, initial_solution_World=W_init)`

        Returns
        -------
        W : World
            World object with near DSO solution (if properly converged)
        
        Notes
        -----
            This is based on an algorithm whose stochastic convergence is theoretically guaranteed. However, the convergence is very slow. It is difficult to obtain accurate solution within a practical amount of time if the network was not small. 
            
            This method is based on the following literature:
            Satsukawa, K., Wada, K., & Watling, D. (2022). Dynamic system optimal traffic assignment with atomic users: Convergence and stability. Transportation Research Part B: Methodological, 155, 188-209. https://doi.org/10.1016/j.trb.2021.11.001
        """

        W = s.func_World()
        if print_info:
            W.print_scenario_stats()

        # enumerate routes for each OD pair
        n_routes_per_od = n_routes_per_od

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        # dict_od_to_routes = {}
        # for o,d in dict_od_to_vehid.keys():
        #     routes = enumerate_k_shortest_routes(W, o, d, k=n_routes_per_od)
        #     dict_od_to_routes[o,d] = routes
        #     #print(o, d, routes)
        
        if W.finalized == False:
            W.finalize_scenario()
        dict_od_to_routes = enumerate_k_random_routes(W, k=n_routes_per_od)

        if print_info:
            print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

        # day-to-day dynamics
        s.ttts = []
        s.route_log = []
        s.cost_log = []
        max_iter = max_iter

        s.dfs_link = []

        if print_info:
            print("solving DSO...")
        for i in range(max_iter):

            W = s.func_World()
            if i != max_iter-1:
                W.vehicle_logging_timestep_interval = -1

            if i != 0:
                for key in W.VEHICLES:
                    if key in routes_specified:
                        W.VEHICLES[key].enforce_route(routes_specified[key])
            
            # simulation
            # if initial_solution_DUE and i==0:
            #     if print_info:
            #         print(" pre-solving DUE...")
            #     W = solve_DUE(func_World, max_iter=initial_solution_DUE_max_iter)
            # else:
            #     W.exec_simulation()

            if i == 0 and initial_solution_World != None:
                if print_info:
                    print(" using pre-solved World...")
                W = initial_solution_World
            else:
                W.exec_simulation()

            route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
            for o,d in dict_od_to_vehid.keys():
                for r in dict_od_to_routes[o,d]:
                    route_set[o,d].append(W.defRoute(r))
            
            # results
            W.analyzer.print_simple_stats()
            s.ttts.append(W.analyzer.total_travel_time)
            s.dfs_link.append(W.analyzer.link_to_pandas())

            if i == max_iter-1:
                break

            # DSO game

            # select vehicle
            tmp_counter=0
            while True: #select vehicle with multiple route options
                vehid = random.choice(list(W.VEHICLES.keys()))
                
                o = W.VEHICLES[vehid].orig.name
                d = W.VEHICLES[vehid].dest.name
                if len(route_set[o,d]) > 1:
                    break

                tmp_counter += 1
                if tmp_counter>10000:
                    raise Exception("DSO error: No alternative routes.")

            routes_specified = {}
            route_actual = {}
            cost_actual = {}
            for key,veh in W.VEHICLES.items():
                if veh.state != "end":
                    continue

                route, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]
                
                route_actual[key] = [rr.name for rr in route]
                cost_actual[key] = travel_time

                routes_specified[key] = route
            
            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)
            
            beta = (i/beta_coef+1)/beta_coef2
            game_route = []
            game_self_cost = []
            game_system_cost = []

            W_without_i = s.func_World()
            for key in W.VEHICLES:
                if key in routes_specified:
                    W_without_i.VEHICLES[key].enforce_route(routes_specified[key])
                if key == vehid:
                    W_without_i.VEHICLES[key].state = "end"
            W_without_i.exec_simulation()
            system_cost_without_i = W_without_i.analyzer.total_travel_time

            for alt_route in dict_od_to_routes[o,d]:
                W_alt = s.func_World()
                W_alt.vehicle_logging_timestep_interval = -1
                routes_specified[vehid] = alt_route
                for key in W.VEHICLES:
                    if key in routes_specified:
                        W_alt.VEHICLES[key].enforce_route(routes_specified[key])
                W_alt.exec_simulation()

                game_route.append(alt_route)
                game_self_cost.append(W_alt.VEHICLES[vehid].travel_time)
                game_system_cost.append(W_alt.analyzer.total_travel_time-system_cost_without_i)
            
            game_utility = -np.array(game_self_cost)-np.array(game_system_cost)
            game_prob = np.exp(beta*game_utility)/sum(np.exp(beta*game_utility))

            game_prob[np.isnan(game_prob)] = 0.5 #TODO: 確率計算を正確にする
            route_index = np.random.choice([i for i in range(len(game_prob))], p=game_prob/sum(game_prob))
            routes_specified[vehid] = dict_od_to_routes[o,d][route_index]

            if print_info:
                print(f"iter {i}, ttt:{s.ttts[-1]}")
            #uxsim.print_columns(game_route, game_self_cost, game_system_cost,game_utility,game_prob)
            if print_info and print_info_detailed:
                print(f"i: {vehid}, system_cost_without_i:{system_cost_without_i}, selected route:{route_index}={routes_specified[vehid]}")
                print_columns(game_self_cost, game_system_cost,game_utility,game_prob)
        
        # final result
        #W.analyzer.print_simple_stats(force_print=True)
        
        s.W_sol = W
        return s.W_sol
    
    def plot_convergence(s):
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(s.ttts)
        plt.xlabel("iter")
        plt.show()

    def plot_link_stats(s):
        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        ave_TT = []
        std_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                length = len(s.route_log)
                ts = [s.cost_log[day][vehid] for day in range(int(length/2), length)]
                ave_TT.append(np.average(ts))
                std_TT.append(np.std(ts))
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.errorbar(x=depature_time, y=ave_TT, yerr=std_TT, 
                fmt='bx', ecolor="#aaaaff", capsize=0, label="travel time (mean $\pm$ std)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()


class SolverDSO_GA:
    def __init__(s, func_World):
        """
        Solve Dynamic System Optimum (DSO) problem using genetic algorithm. WIP

        Parameters
        ----------
        func_World : function
            function that returns a World object with nodes, links, and demand specifications
        
        Notes
        -----
            This is based on the standard genetic algorithm. For some scenarios, this method works well.
        """
        s.func_World = func_World
        s.W_sol = None
        s.dfs_link = []
    
    def solve(s, max_iter, n_routes=4, pop_size=50, elite_size=2, mutation_occur_rate=0.1, mutation_gene_rate=0.1, n_crossover_points=2, initial_solution_World=None):
        """
        Solve Dynamic System Optimum (DSO) problem using genetic algorithm. WIP

        Parameters
        ----------
        n_routes : int
            number of routes to enumerate for each OD pair
        pop_size : int
            population size
        max_iter : int
            number of generations
        mutation_occur_rate : float
            mutation rate for individual
        mutation_gene_rate : float
            mutation rate for gene
        n_crossover_points : int
            number of crossover points

        Returns
        -------
        W : World
            World object with near DSO solution (if properly converged)
        
        Notes
        -----
            This is based on the standard genetic algorithm. For some scenarios, this method works well.

            GA part is initially written by ChatGPT o1-preview, and reviewed and overwritten by human.
        """
        
        def initialize_population(pop_size, length, genome_maxs, W_init):
            """Initialize a population with random genomes."""
            population = []
            #default pop is random
            for _ in range(pop_size):
                genome = [random.randint(0, genome_maxs[i]-1) for i in range(length)]
                population.append(genome)
            
            #one pop is the initial solution
            for i,veh in enumerate(W_init.VEHICLES.values()):   
                route_id = 0
                for j,r in enumerate(routes[(veh.orig.name, veh.dest.name)]):
                    if W_init.defRoute(r) == veh.traveled_route()[0]:
                        route_id = j
                population[0][i] = route_id
            #1/4 of the pop is mutation of the initial solution
            for k in range(1, int(pop_size/4)):
                population[k] = mutate(population[0], mutation_gene_rate, genome_maxs)
            #1/4 of the pop is heavier mutation of the initial solution
            for k in range(int(pop_size/4), int(pop_size/2)):
                population[k] = mutate(population[0], 0.3, genome_maxs)

            return population

        def tournament_selection(population, fitnesses, k=3):
            """Select individuals using tournament selection."""
            selected = []
            for _ in range(len(population)):
                aspirants = random.sample(list(zip(population, fitnesses)), k)
                selected.append(max(aspirants, key=lambda x: x[1])[0])
            return selected

        def crossover(parent1, parent2, n_points=1):
            """Perform n-point crossover between two parents."""
            if n_points >= len(parent1):
                raise ValueError("Number of crossover points must be less than genome length.")
            
            points = sorted(random.sample(range(1, len(parent1)), n_points))
            child1, child2 = [], []
            last_point = 0
            for i, point in enumerate(points + [len(parent1)]):
                if i % 2 == 0:
                    child1.extend(parent1[last_point:point])
                    child2.extend(parent2[last_point:point])
                else:
                    child1.extend(parent2[last_point:point])
                    child2.extend(parent1[last_point:point])
                last_point = point
            return child1, child2

        def mutate(genome, mutation_rate, genome_maxs):
            """Mutate a genome with a given mutation rate."""
            mutated_genome = []
            for i,gene in enumerate(genome):
                if random.random() < mutation_rate:
                    mutated_gene = random.randint(0, genome_maxs[i]-1)
                    mutated_genome.append(mutated_gene)
                else:
                    mutated_genome.append(gene)
            return mutated_genome

            
        ##############################################################
        # initial solution
        if initial_solution_World == None:
            print("####"*20)
            print("Deriving DUO")
            
            W = s.func_World()
            W.exec_simulation()
            print(W.analyzer.basic_to_pandas())
        else:
            print("####"*20)
            print("using initial solution")
            W = initial_solution_World

        ##############################################################
        # enumerate some routes between each OD pair

        dict_od_to_vehid = defaultdict(lambda: [])
        for key, veh in W.VEHICLES.items():
            o = veh.orig.name
            d = veh.dest.name
            dict_od_to_vehid[o,d].append(key)

        # routes = {}
        # n_routes = n_routes
        # for od_pair in dict_od_to_vehid.keys():
        #     routes[od_pair] = enumerate_k_shortest_routes(W, od_pair[0], od_pair[1], n_routes)

        routes = enumerate_k_random_routes(W, k=n_routes)

        # print("available routes for each OD pair")
        # for key in routes:
        #    for route in routes[key]:
        #        print(key, route)

        ##############################################################
        # Prepare genetic algorithm

        # specify routing based on genome
        def specify_routes(W, genome):
            veh_list = list(W.VEHICLES.values())
            for i, value in enumerate(genome):
                veh = veh_list[i]
                if value < len(routes[(veh.orig.name, veh.dest.name)]):
                    veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][value])
                else:
                    veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][-1])

        pop_size = pop_size     # Population size
        max_iter = max_iter     # Number of generations
        elite_size = elite_size  # Number of elites that survive
        length = len(W.VEHICLES.values())        # Genome length
        n_max = n_routes         # Maximum value for each gene
        mutation_occur_rate = mutation_occur_rate  # Mutation rate for individual
        mutation_gene_rate = mutation_gene_rate  # Mutation rate for gene
        n_points = n_crossover_points    # Number of crossovers

        genome_maxs = []
        for i, veh in enumerate(W.VEHICLES.values()):
            actual_n_route = len(routes[(veh.orig.name, veh.dest.name)])
            genome_maxs.append(actual_n_route)
        
        s.ttts = []
        s.route_log = []
        s.cost_log = []

        print("solving DSO by GA...")

        population = initialize_population(pop_size, length, genome_maxs, W)

        for gen in range(max_iter):
            print(f"Generation {gen}")
            print(" total travel times: ", end="")
            fitnesses = []
            fitness_best = None
            for genome in population:
                W = s.func_World()
                if gen != max_iter-1:
                    W.vehicle_logging_timestep_interval = -1
                specify_routes(W, genome)
                W.exec_simulation()

                print(W.analyzer.total_travel_time, end=" ")
                fitnesses.append(- W.analyzer.total_travel_time)
                if fitness_best == None or fitnesses[-1] > fitness_best:
                    fitness_best = fitnesses[-1]
                    W_best = W
            
            route_actual = {}
            cost_actual = {}
            for key,veh in W_best.VEHICLES.items():
                if veh.state != "end":
                    continue

                route, ts = veh.traveled_route()
                travel_time = ts[-1]-ts[0]
                
                route_actual[key] = [rr.name for rr in route]
                cost_actual[key] = travel_time

            s.route_log.append(route_actual)
            s.cost_log.append(cost_actual)
            s.ttts.append(int(W_best.analyzer.total_travel_time))

            # Print the best fitness in the current generation
            print(f"\n Best fitness = {max(fitnesses)}, TTT = {W_best.analyzer.total_travel_time}, completed trips: {W_best.analyzer.trip_completed}")
            
            s.dfs_link.append(W_best.analyzer.link_to_pandas())

            # Selection
            selected = tournament_selection(population, fitnesses)

            # Generate next generation
            next_generation = []
            # Elite
            for i in sorted(range(len(fitnesses)), key=lambda i: fitnesses[i], reverse=True)[:elite_size]:
                next_generation.append(population[i])
            # Children
            for i in range(int(pop_size/2)):
                parent1 = random.choice(selected)
                parent2 = random.choice(selected)
                # Crossover
                child1, child2 = crossover(parent1, parent2, n_points=n_points)
                # Mutation
                if random.random() < mutation_occur_rate:
                    child1 = mutate(child1, mutation_gene_rate, genome_maxs)
                    child2 = mutate(child2, mutation_gene_rate, genome_maxs)
                next_generation.extend([child1, child2])

            population = next_generation[:pop_size]
        
        s.W_sol = W
        return s.W_sol
    
    def plot_convergence(s):
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(s.ttts)
        plt.xlabel("iter")
        plt.show()

    def plot_link_stats(s):
        plt.figure()
        plt.title("traffic volume")
        for i in range(len(s.dfs_link[0])):
            vols = [df["traffic_volume"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time")
        for i in range(len(s.dfs_link[0])):
            vols = [df["average_travel_time"][i] for df in s.dfs_link]
            plt.plot(vols, label=s.dfs_link[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.show()
    
    def plot_vehicle_stats(s, orig=None, dest=None):
        ave_TT = []
        std_TT = []
        depature_time = []
        for vehid in s.route_log[0].keys():
            if (s.W_sol.VEHICLES[vehid].orig.name == orig or orig == None) and (s.W_sol.VEHICLES[vehid].dest.name == dest or dest == None):
                length = len(s.route_log)
                ts = [s.cost_log[day][vehid] for day in range(int(length/2), length)]
                ave_TT.append(np.average(ts))
                std_TT.append(np.std(ts))
                depature_time.append(s.W_sol.VEHICLES[vehid].departure_time_in_second)

        plt.figure()
        orig_ = orig
        if orig == None:
            orig_ = "any"
        dest_ = dest
        if dest == None:
            dest_ = "any"
        plt.title(f"orig: {orig_}, dest: {dest_}")
        plt.errorbar(x=depature_time, y=ave_TT, yerr=std_TT, 
                fmt='bx', ecolor="#aaaaff", capsize=0, label="travel time (mean $\pm$ std)")
        plt.xlabel("departure time of vehicle")
        plt.ylabel("travel time")
        plt.legend()
        plt.show()



def solve_DUE(func_World, max_iter, n_routes_per_od=4, swap_prob=0.05, print_info=True, plot_res=False, return_link_history=False, trace_plot=False):
    """
    Solve quasi Dynamic User Equilibrium (DUE) problem using day-to-day dynamics. WIP

    Parameters
    ----------
    func_World : function
        function that returns a World object with nodes, links, and demand specifications
    max_iter : int
        maximum number of iterations
    n_routes_per_od : int
        number of routes to enumerate for each OD pair
    swap_prob : float
        probability of route swap
    print_info : bool
        whether to print the information
    plot_res : bool
        whether to plot the results

    Returns
    -------
    W : World
        World object with quasi DUE solution (if properly converged)
    
    Notes
    -----
        This function computes a near dynamic user equilibirum state as a steady state of day-to-day dynamical routing game.

        Specifically, on day `i`, vehicles choose their route based on actual travel time on day `i-1` with the same departure time.
        If there are shorter travel time route, they will change with probability `swap_prob`.
        This process is repeated until `max_iter` day.
        It is expected that this process eventually reach a steady state.
        Due to the problem complexity, it does not necessarily reach or converge to Nash equilibrium or any other stationary points.
        However, in the literature, it is argued that the steady state can be considered as a reasonable proxy for Nash equilibrium or dynamic equilibrium state.
        There are some theoretical background for it; but intuitively speaking, the steady state can be considered as a realistic state that people's rational behavior will reach.

        This method is based on the following literature:
        Ishihara, M., & Iryo, T. (2015). Dynamic Traffic Assignment by Markov Chain. Journal of Japan Society of Civil Engineers, Ser. D3 (Infrastructure Planning and Management), 71(5), I_503-I_509. (in Japanese). https://doi.org/10.2208/jscejipm.71.I_503
        Iryo, T., Urata, J., & Kawase, R. (2024). Traffic Flow Simulator and Travel Demand Simulators for Assessing Congestion on Roads After a Major Earthquake. In APPLICATION OF HIGH-PERFORMANCE COMPUTING TO EARTHQUAKE-RELATED PROBLEMS (pp. 413-447). https://doi.org/10.1142/9781800614635_0007
    """
    warnings.warn("This function is deprecated", DeprecationWarning, stacklevel=2)

    W_orig = func_World()
    if print_info:
        W_orig.print_scenario_stats()

    # enumerate routes for each OD pair
    n_routes_per_od = n_routes_per_od

    dict_od_to_vehid = defaultdict(lambda: [])
    for key, veh in W_orig.VEHICLES.items():
        o = veh.orig.name
        d = veh.dest.name
        dict_od_to_vehid[o,d].append(key)

    dict_od_to_routes = {}
    for o,d in dict_od_to_vehid.keys():
        routes = enumerate_k_shortest_routes(W_orig, o, d, k=n_routes_per_od) #TODO: to be replaced with many-to-many shortest path search
        dict_od_to_routes[o,d] = routes

    if print_info:
        print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

    # day-to-day dynamics
    ttts = []
    n_swaps = []
    potential_swaps = []
    total_t_gaps = []
    swap_prob = swap_prob
    max_iter = max_iter #50 or 100 is better

    dfs = []

    if print_info:
        print("solving DUE...")
    for i in range(max_iter):
        W = func_World()
        if i != max_iter-1:
            W.vehicle_logging_timestep_interval = -1

        if i != 0:
            for key in W.VEHICLES:
                if key in routes_specified:
                    W.VEHICLES[key].enforce_route(routes_specified[key])
        
        route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
        for o,d in dict_od_to_routes.keys():
            for r in dict_od_to_routes[o,d]:
                route_set[o,d].append(W.defRoute(r))

        # simulation
        W.exec_simulation()

        # results
        W.analyzer.print_simple_stats()
        #W.analyzer.network_average()

        if return_link_history or trace_plot:
            dfs.append(W.analyzer.link_to_pandas())

        # route swap
        routes_specified = {}
        n_swap = 0
        total_t_gap = 0
        potential_n_swap = 0
        for key,veh in W.VEHICLES.items():
            if veh.state != "end":
                continue

            o = veh.orig.name
            d = veh.dest.name
            r, ts = veh.traveled_route()
            travel_time = ts[-1]-ts[0]

            flag_route_changed = False
            route_changed = None
            t_gap = 0
            
            for alt_route in route_set[o,d]:
                if alt_route.actual_travel_time(ts[0]) < r.actual_travel_time(ts[0]):
                    if flag_route_changed == False or (alt_route.actual_travel_time(ts[0]) < route_changed.actual_travel_time(ts[0])):
                        t_gap = r.actual_travel_time(ts[0]) - alt_route.actual_travel_time(ts[0])
                        potential_n_swap += 1 #there may be double count; but not critical issue
                        if random.random() < swap_prob:
                            flag_route_changed = True
                            route_changed = alt_route
            
            total_t_gap += t_gap
            routes_specified[key] = r
            if flag_route_changed:
                n_swap += 1
                routes_specified[key] = route_changed

        if print_info:
            print(f' iter {i}: time gap: {total_t_gap:.1f}, potential route change: {potential_n_swap}, route change: {n_swap}, total travel time: {W.analyzer.total_travel_time: .1f}, delay ratio: {W.analyzer.average_delay/W.analyzer.average_travel_time: .3f}')

        ttts.append(int(W.analyzer.total_travel_time))
        n_swaps.append(n_swap)
        potential_swaps.append(potential_n_swap)
        total_t_gaps.append(total_t_gap)

        # if i == 0:
        #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_0init.gif")
        # if i == int(max_iter/2):
        #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_1mid.gif")
        # if i == max_iter-1:
        #     W.analyzer.network_anim(animation_speed_inverse=15, figsize=(6,6), detailed=0, network_font_size=0, file_name="out/due_anim_2last.gif")

    if plot_res:
        # iteration plot
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(ttts)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of route change")
        plt.plot(n_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("number of potential route change")
        plt.plot(potential_swaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("time gap for potential route change")
        plt.plot(total_t_gaps)
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.figure(figsize=(6,2))
        plt.title("time gap per potential route change")
        plt.plot(np.array(total_t_gaps)/np.array(potential_swaps))
        plt.ylim(0,None)
        plt.xlabel("iter")

        plt.show()

    if trace_plot:
        plt.figure()
        plt.title("traffic volume trace")
        for i in range(len(dfs[0])):
            vols = [df["traffic_volume"][i] for df in dfs]
            plt.plot(vols, label=dfs[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time trace")
        for i in range(len(dfs[0])):
            vols = [df["average_travel_time"][i] for df in dfs]
            plt.plot(vols, label=dfs[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    if return_link_history:
        return W, dfs
    else:
        return W    

def solve_DSO(func_World, max_iter, n_routes_per_od=4, beta_coef=100, beta_coef2=5000, print_info=True, print_info_detailed=False, plot_res=False, initial_solution_World=None, return_link_history=False, trace_plot=False):
    """
    Solve Dynamic System Optimum (DSO) problem. WIP

    Parameters
    ----------
    func_World : function
        function that returns a World object with nodes, links, and demand specifications
    max_iter : int
        maximum number of iterations
    n_routes_per_od : int
        number of routes to enumerate for each OD pair
    beta_coef : float
        coefficient for logit response dynamics
    beta_coef2 : float
        coefficient for logit response dynamics
    print_info : bool
        whether to print the information
    print_info_detailed : bool
        whether to print the detailed information
    plot_res : bool
        whether to plot the results
    initial_solution_World : World
        initial solution (starting point) for the optimization algorithm. It must have the same structure as the output of func_World, and its simulation has been already executed. Recommended example: `W_init = solve_DUE(func_World); W = solve_DSO(func_World, initial_solution_World=W_init)`

    Returns
    -------
    W : World
        World object with near DSO solution (if properly converged)
    
    Notes
    -----
        This is based on an algorithm whose stochastic convergence is theoretically guaranteed. However, the convergence is very slow. It is difficult to obtain accurate solution within a practical amount of time if the network was not small. 
        
        This method is based on the following literature:
        Satsukawa, K., Wada, K., & Watling, D. (2022). Dynamic system optimal traffic assignment with atomic users: Convergence and stability. Transportation Research Part B: Methodological, 155, 188-209. https://doi.org/10.1016/j.trb.2021.11.001
    """
    warnings.warn("This function is deprecated", DeprecationWarning, stacklevel=2)

    W = func_World()
    if print_info:
        W.print_scenario_stats()

    # enumerate routes for each OD pair
    n_routes_per_od = n_routes_per_od

    dict_od_to_vehid = defaultdict(lambda: [])
    for key, veh in W.VEHICLES.items():
        o = veh.orig.name
        d = veh.dest.name
        dict_od_to_vehid[o,d].append(key)

    # dict_od_to_routes = {}
    # for o,d in dict_od_to_vehid.keys():
    #     routes = enumerate_k_shortest_routes(W, o, d, k=n_routes_per_od)
    #     dict_od_to_routes[o,d] = routes
    #     #print(o, d, routes)
    
    if W.finalized == False:
        W.finalize_scenario()
    dict_od_to_routes = enumerate_k_random_routes(W, k=n_routes_per_od)

    if print_info:
        print(f"number of OD pairs: {len(dict_od_to_routes.keys())}, number of routes: {sum([len(val) for val in dict_od_to_routes.values()])}")

    # day-to-day dynamics
    ttts = []
    max_iter = max_iter

    dfs = []

    if print_info:
        print("solving DSO...")
    for i in range(max_iter):

        W = func_World()
        if i != max_iter-1:
            W.vehicle_logging_timestep_interval = -1

        if i != 0:
            for key in W.VEHICLES:
                if key in routes_specified:
                    W.VEHICLES[key].enforce_route(routes_specified[key])
        
        # simulation
        # if initial_solution_DUE and i==0:
        #     if print_info:
        #         print(" pre-solving DUE...")
        #     W = solve_DUE(func_World, max_iter=initial_solution_DUE_max_iter)
        # else:
        #     W.exec_simulation()

        if i == 0 and initial_solution_World != None:
            if print_info:
                print(" using pre-solved World...")
            W = initial_solution_World
        else:
            W.exec_simulation()

        route_set = defaultdict(lambda: []) #routes[o,d] = [Route, Route, ...]
        for o,d in dict_od_to_vehid.keys():
            for r in dict_od_to_routes[o,d]:
                route_set[o,d].append(W.defRoute(r))
        
        # results
        W.analyzer.print_simple_stats()
        ttts.append(W.analyzer.total_travel_time)
        if trace_plot or return_link_history:
            dfs.append(W.analyzer.link_to_pandas())

        if i == max_iter-1:
            break

        # DSO game

        # select vehicle
        tmp_counter=0
        while True: #select vehicle with multiple route options
            vehid = random.choice(list(W.VEHICLES.keys()))
            
            o = W.VEHICLES[vehid].orig.name
            d = W.VEHICLES[vehid].dest.name
            if len(route_set[o,d]) > 1:
                break

            tmp_counter += 1
            if tmp_counter>10000:
                raise Exception("DSO error: No alternative routes.")

        routes_specified = {}
        for key,veh in W.VEHICLES.items():
            if veh.state != "end":
                continue

            route, ts = veh.traveled_route()
            
            routes_specified[key] = route
        
        beta = (i/beta_coef+1)/beta_coef2
        game_route = []
        game_self_cost = []
        game_system_cost = []

        W_without_i = func_World()
        for key in W.VEHICLES:
            if key in routes_specified:
                W_without_i.VEHICLES[key].enforce_route(routes_specified[key])
            if key == vehid:
                W_without_i.VEHICLES[key].state = "end"
        W_without_i.exec_simulation()
        system_cost_without_i = W_without_i.analyzer.total_travel_time

        for alt_route in dict_od_to_routes[o,d]:
            W_alt = func_World()
            W_alt.vehicle_logging_timestep_interval = -1
            routes_specified[vehid] = alt_route
            for key in W.VEHICLES:
                if key in routes_specified:
                    W_alt.VEHICLES[key].enforce_route(routes_specified[key])
            W_alt.exec_simulation()

            game_route.append(alt_route)
            game_self_cost.append(W_alt.VEHICLES[vehid].travel_time)
            game_system_cost.append(W_alt.analyzer.total_travel_time-system_cost_without_i)
        
        game_utility = -np.array(game_self_cost)-np.array(game_system_cost)
        game_prob = np.exp(beta*game_utility)/sum(np.exp(beta*game_utility))

        game_prob[np.isnan(game_prob)] = 0.5
        route_index = np.random.choice([i for i in range(len(game_prob))], p=game_prob/sum(game_prob))
        routes_specified[vehid] = dict_od_to_routes[o,d][route_index]

        if print_info:
            print(f"iter {i}, ttt:{ttts[-1]}")
        #uxsim.print_columns(game_route, game_self_cost, game_system_cost,game_utility,game_prob)
        if print_info and print_info_detailed:
            print(f"i: {vehid}, system_cost_without_i:{system_cost_without_i}, selected route:{route_index}={routes_specified[vehid]}")
            print_columns(game_self_cost, game_system_cost,game_utility,game_prob)
    
    # final result
    W.analyzer.print_simple_stats(force_print=True)

    # iteration plot
    if plot_res:
        plt.figure(figsize=(6,2))
        plt.title("total travel time")
        plt.plot(ttts)
        plt.xlabel("iter")
        plt.show()

    if trace_plot:
        plt.figure()
        plt.title("traffic volume trace")
        for i in range(len(dfs[0])):
            vols = [df["traffic_volume"][i] for df in dfs]
            plt.plot(vols, label=dfs[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time trace")
        for i in range(len(dfs[0])):
            vols = [df["average_travel_time"][i] for df in dfs]
            plt.plot(vols, label=dfs[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    if return_link_history:
        return W, dfs
    else:
        return W    

def solve_DSO_genetic_algorithm(func_World, max_iter, n_routes=4, pop_size=50, elite_size=2, mutation_occur_rate=0.1, mutation_gene_rate=0.1, n_crossover_points=2, initial_solution_World=None, return_link_history=False, trace_plot=False):
    """
    Solve Dynamic System Optimum (DSO) problem using genetic algorithm. WIP

    Parameters
    ----------
    func_World : function
        function that returns a World object with nodes, links, and demand specifications
    n_routes : int
        number of routes to enumerate for each OD pair
    pop_size : int
        population size
    max_iter : int
        number of generations
    mutation_occur_rate : float
        mutation rate for individual
    mutation_gene_rate : float
        mutation rate for gene
    n_crossover_points : int
        number of crossover points

    Returns
    -------
    W : World
        World object with near DSO solution (if properly converged)
    
    Notes
    -----
        This is based on the standard genetic algorithm. For some scenarios, this method works well.

        GA part is initially written by ChatGPT o1-preview, and reviewed and overwritten by human.
    """
    warnings.warn("This function is deprecated", DeprecationWarning, stacklevel=2)

    def initialize_population(pop_size, length, genome_maxs, W_init):
        """Initialize a population with random genomes."""
        population = []
        #default pop is random
        for _ in range(pop_size):
            genome = [random.randint(0, genome_maxs[i]-1) for i in range(length)]
            population.append(genome)
        
        #one pop is the initial solution
        for i,veh in enumerate(W_init.VEHICLES.values()):   
            route_id = 0
            for j,r in enumerate(routes[(veh.orig.name, veh.dest.name)]):
                if W_init.defRoute(r) == veh.traveled_route()[0]:
                    route_id = j
            population[0][i] = route_id
        #1/4 of the pop is mutation of the initial solution
        for k in range(1, int(pop_size/4)):
            population[k] = mutate(population[0], mutation_gene_rate, genome_maxs)
        #1/4 of the pop is heavier mutation of the initial solution
        for k in range(int(pop_size/4), int(pop_size/2)):
            population[k] = mutate(population[0], 0.3, genome_maxs)

        return population

    def tournament_selection(population, fitnesses, k=3):
        """Select individuals using tournament selection."""
        selected = []
        for _ in range(len(population)):
            aspirants = random.sample(list(zip(population, fitnesses)), k)
            selected.append(max(aspirants, key=lambda x: x[1])[0])
        return selected

    def crossover(parent1, parent2, n_points=1):
        """Perform n-point crossover between two parents."""
        if n_points >= len(parent1):
            raise ValueError("Number of crossover points must be less than genome length.")
        
        points = sorted(random.sample(range(1, len(parent1)), n_points))
        child1, child2 = [], []
        last_point = 0
        for i, point in enumerate(points + [len(parent1)]):
            if i % 2 == 0:
                child1.extend(parent1[last_point:point])
                child2.extend(parent2[last_point:point])
            else:
                child1.extend(parent2[last_point:point])
                child2.extend(parent1[last_point:point])
            last_point = point
        return child1, child2

    def mutate(genome, mutation_rate, genome_maxs):
        """Mutate a genome with a given mutation rate."""
        mutated_genome = []
        for i,gene in enumerate(genome):
            if random.random() < mutation_rate:
                mutated_gene = random.randint(0, genome_maxs[i]-1)
                mutated_genome.append(mutated_gene)
            else:
                mutated_genome.append(gene)
        return mutated_genome

        
    ##############################################################
    # initial solution
    if initial_solution_World == None:
        print("####"*20)
        print("Deriving DUO")
        
        W = func_World()
        W.exec_simulation()
        print(W.analyzer.basic_to_pandas())
    else:
        print("####"*20)
        print("using initial solution")
        W = initial_solution_World

    ##############################################################
    # enumerate some routes between each OD pair

    dict_od_to_vehid = defaultdict(lambda: [])
    for key, veh in W.VEHICLES.items():
        o = veh.orig.name
        d = veh.dest.name
        dict_od_to_vehid[o,d].append(key)

    # routes = {}
    # n_routes = n_routes
    # for od_pair in dict_od_to_vehid.keys():
    #     routes[od_pair] = enumerate_k_shortest_routes(W, od_pair[0], od_pair[1], n_routes)

    routes = enumerate_k_random_routes(W, k=n_routes)

    # print("available routes for each OD pair")
    # for key in routes:
    #    for route in routes[key]:
    #        print(key, route)

    ##############################################################
    # Prepare genetic algorithm

    # specify routing based on genome
    def specify_routes(W, genome):
        veh_list = list(W.VEHICLES.values())
        for i, value in enumerate(genome):
            veh = veh_list[i]
            if value < len(routes[(veh.orig.name, veh.dest.name)]):
                veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][value])
            else:
                veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][-1])

    pop_size = pop_size     # Population size
    max_iter = max_iter     # Number of generations
    elite_size = elite_size  # Number of elites that survive
    length = len(W.VEHICLES.values())        # Genome length
    n_max = n_routes         # Maximum value for each gene
    mutation_occur_rate = mutation_occur_rate  # Mutation rate for individual
    mutation_gene_rate = mutation_gene_rate  # Mutation rate for gene
    n_points = n_crossover_points    # Number of crossovers

    genome_maxs = []
    for i, veh in enumerate(W.VEHICLES.values()):
        actual_n_route = len(routes[(veh.orig.name, veh.dest.name)])
        genome_maxs.append(actual_n_route)
    
    dfs = []

    print("solving DSO by GA...")

    population = initialize_population(pop_size, length, genome_maxs, W)

    for gen in range(max_iter):
        print(f"Generation {gen}")
        print(" total travel times: ", end="")
        fitnesses = []
        fitness_best = None
        for genome in population:
            W = func_World()
            if gen != max_iter-1:
                W.vehicle_logging_timestep_interval = -1
            specify_routes(W, genome)
            W.exec_simulation()
            print(W.analyzer.total_travel_time, end=" ")
            fitnesses.append(- W.analyzer.total_travel_time)
            if fitness_best == None or fitnesses[-1] > fitness_best:
                fitness_best = fitnesses[-1]
                W_best = W
        
        # Print the best fitness in the current generation
        print(f"\n Best TTT = {-max(fitnesses)} = {W_best.analyzer.total_travel_time}, completed trips: {W_best.analyzer.trip_completed}")
        
        if trace_plot or return_link_history:
            dfs.append(W_best.analyzer.link_to_pandas())

        # Selection
        selected = tournament_selection(population, fitnesses)

        # Generate next generation
        next_generation = []
        # Elite
        for i in sorted(range(len(fitnesses)), key=lambda i: fitnesses[i], reverse=True)[:elite_size]:
            next_generation.append(population[i])
        # Children
        for i in range(int(pop_size/2)):
            parent1 = random.choice(selected)
            parent2 = random.choice(selected)
            # Crossover
            child1, child2 = crossover(parent1, parent2, n_points=n_points)
            # Mutation
            if random.random() < mutation_occur_rate:
                child1 = mutate(child1, mutation_gene_rate, genome_maxs)
                child2 = mutate(child2, mutation_gene_rate, genome_maxs)
            next_generation.extend([child1, child2])

        population = next_generation[:pop_size]
    
    if trace_plot:
        plt.figure()
        plt.title("traffic volume trace")
        for i in range(len(dfs[0])):
            vols = [df["traffic_volume"][i] for df in dfs]
            plt.plot(vols, label=dfs[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("volume (veh)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        plt.figure()
        plt.title("average travel time trace")
        for i in range(len(dfs[0])):
            vols = [df["average_travel_time"][i] for df in dfs]
            plt.plot(vols, label=dfs[0]["link"][i])
        plt.xlabel("iteration")
        plt.ylabel("time (s)")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    if return_link_history:
        return W_best, dfs
    else:
        return W_best

