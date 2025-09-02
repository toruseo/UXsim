"""
The Nguyen--Dupuis network

The parameters are based on 
Satsukawa, K., Wada, K., & Watling, D. (2022). Dynamic system optimal traffic assignment with atomic users: Convergence and stability. Transportation Research Part B: Methodological, 155, 188-209.
https://doi.org/10.1016/j.trb.2021.11.001

However, this parameter setting looks too sparse. It is recommended to increase the demand.
The other args can be also adjustable.
"""

import uxsim

def create_World_ND(demand_rate=0.5, n_lanes=6, coef_length=1):
    # simulation world
    W = uxsim.World(
        name="ND network",
        deltan=5,
        tmax=4800,
        duo_update_time=300,
        print_mode=1, save_mode=0, show_mode=1,
        vehicle_logging_timestep_interval=-1,
        random_seed=42,
    )
    
    nodes = [ 
        [1,1,3], #id, x, y
        [2,4,1],
        [3,3,0],
        [4,0,2],
        [5,1,2],
        [6,2,2],
        [7,3,2],
        [8,4,2],
        [9,1,1],
        [10,2,1],
        [11,3,1],
        [12,2,3],
        [13,2,0]
    ]
    for n in nodes:
        W.addNode(f"n{n[0]}", n[1], n[2])

    links=[
        [(1,5), 42, 1.25, ], #(node_start, node_end), free-flow travel time, bottleneck capacity, saturation flow
        [(1,12), 54, 1.25, ],
        [(4,5), 54, 1.25, ],
        [(4,9), 90, 0.83, ],
        [(5,6), 36, 1.42, ],
        [(5,9), 54, 1.67, ],
        [(6,7), 24, 1.25, ],
        [(6,10), 78, 0.83, ],
        [(7,8), 48, 0.83, ],
        [(7,11), 66, 0.92, ],
        [(8,2), 72, 1.25, ],
        [(9,10), 60, 2.25, ],
        [(9,13), 54, 0.83, ],
        [(10,11), 18, 1.67, ],
        [(11,2), 54, 1.25, ],
        [(11,3), 42, 1.25, ],
        [(12,6), 30, 0.67, ],
        [(12,8), 84, 1.25, ],
        [(13,3), 66, 0.83, ]
    ]

    for l in links:
        free_flow_speed = 10
        length = free_flow_speed*l[1]*coef_length
        capacity_out = l[2]
        W.addLink(f"l{l[0]}", f"n{l[0][0]}", f"n{l[0][1]}", length=length, free_flow_speed=free_flow_speed, number_of_lanes=n_lanes, capacity_out=capacity_out)
    
    demand_rate = demand_rate
    t0 = 0
    t1 = 2000
    for i,j in [[1,2], [1,3], [4,2], [4,3]]:
        W.adddemand(f"n{i}", f"n{j}", t0, t1, demand_rate)
    
    return W

W = create_World_ND(0.5)
W.print_scenario_stats()
W.show_network()

W.exec_simulation()
W.analyzer.print_simple_stats()
W.analyzer.network_average(network_font_size=0)