from pylab import *
import uxsim
from uxsim.Utilities import enumerate_k_shortest_routes, generate_grid_network
import random
from collections import defaultdict
import random

def initialize_population(pop_size, length, n_max):
    """Initialize a population with random genomes."""
    population = []
    for _ in range(pop_size):
        genome = [random.randint(0, n_max) for _ in range(length)]
        population.append(genome)
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

def mutate(genome, mutation_rate, n_max):
    """Mutate a genome with a given mutation rate."""
    mutated_genome = []
    for gene in genome:
        if random.random() < mutation_rate:
            mutated_gene = random.randint(0, n_max)
            mutated_genome.append(mutated_gene)
        else:
            mutated_genome.append(gene)
    return mutated_genome



##############################################################
# Define UXsim World
def create_World():
    W = uxsim.World(
        name="", 
        deltan=5, 
        tmax=1200,
        print_mode=0, save_mode=1, show_mode=1, #print is off, otherwise it will be very verbose during the genetic algorithm
        duo_update_time=60, 
        vehicle_logging_timestep_interval=-1, 
        random_seed=42
    )

    ## generate grid network
    generate_grid_network(W, 3, 3, length=1000)
    # W.show_network()

    ## set demand
    od_pairs = [
        ("n(0, 0)", "n(2, 2)"),
        ("n(2, 0)", "n(0, 2)"),
        ("n(0, 2)", "n(2, 0)"),
        ("n(2, 2)", "n(0, 0)"),
    ]
    for od_pair in od_pairs:
        W.adddemand(od_pair[0], od_pair[1], 0, 500, 1)

    return W

W = create_World()
       
##############################################################
# Compute DUO as a reference
print("####"*20)
print("Deriving DUO")
W.exec_simulation()
print(W.analyzer.basic_to_pandas())

W_orig= W.copy()

##############################################################
# enumerate some routes between each OD pair

dict_od_to_vehid = defaultdict(lambda: [])
for key, veh in W_orig.VEHICLES.items():
    o = veh.orig.name
    d = veh.dest.name
    dict_od_to_vehid[o,d].append(key)

routes = {}
n_routes = 6
for od_pair in dict_od_to_vehid.keys():
   routes[od_pair] = enumerate_k_shortest_routes(W, od_pair[0], od_pair[1], n_routes)

# print("available routes for each OD pair")
# for key in routes:
#    for route in routes[key]:
#        print(key, route)

##############################################################
# Prepare genetic algorithm
# evaluate fitness by total travel time
def evaluate_by_total_travel_time(W):
    W.exec_simulation()
    print(W.analyzer.total_travel_time, end=" ")
    return - W.analyzer.total_travel_time,

# specify routing based on genome
def specify_routes(W, genome):
    veh_list = list(W.VEHICLES.values())
    for i, value in enumerate(genome):
        veh = veh_list[i]
        if value < len(routes[(veh.orig.name, veh.dest.name)]):
            veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][value])
        else:
            veh.set_links_prefer(routes[(veh.orig.name, veh.dest.name)][-1])

pop_size = 20     # Population size
generations = 10     # Number of generations
elite_size = 1  # Number of elites
length = len(W.VEHICLES.values())        # Genome length
n_max = n_routes         # Maximum value for each gene
mutation_occur_rate = 0.1  # Mutation rate for individual
mutation_gene_rate = 0.1  # Mutation rate for gene
n_points = 2    # Number of crossovers

print("####"*20)
print("Deriving DSO")

W_best = W
population = initialize_population(pop_size, length, n_max)
for gen in range(generations):
    print(f"\nGeneration {gen}")
    print("total travel times: ", end="")
    fitnesses = []
    fitness_best = None
    for genome in population:
        W = create_World()
        specify_routes(W, genome)
        W.exec_simulation()
        print(W.analyzer.total_travel_time, end=" ")
        fitnesses.append(- W.analyzer.total_travel_time)
        if fitness_best == None or fitnesses[-1] > fitness_best:
            fitness_best = fitnesses[-1]
            W_best = W
        
    #fitnesses = [fitness(genome) for genome in population]
    # Print the best fitness in the current generation
    print(f"\nBest Fitness = {max(fitnesses)}")
    print(W_best.analyzer.basic_to_pandas())

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
            child1 = mutate(child1, mutation_gene_rate, n_max)
            child2 = mutate(child2, mutation_gene_rate, n_max)
        next_generation.extend([child1, child2])

    population = next_generation[:pop_size]

