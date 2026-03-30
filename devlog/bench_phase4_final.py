"""
Phase 4 Final DTA Solver Benchmark
9x9 grid scenario from demo_notebook_09en.
SolverDUE + SolverDSO_D2D, Python vs C++, 3 runs each.
"""
import time
import sys
import statistics
sys.path.insert(0, ".")

import uxsim
from uxsim.DTAsolvers import SolverDUE, SolverDSO_D2D


def create_world_9x9(cpp=False):
    """9x9 grid network from demo_notebook_09en: 81 nodes, 288 links"""
    W = uxsim.World(
        name="",
        deltan=10,
        tmax=4800,
        duo_update_time=300,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=42,
        cpp=cpp,
    )

    imax = 9
    jmax = 9
    id_center = 4
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)

    for i in range(imax):
        for j in range(jmax):
            if i != imax-1:
                spd = 20 if j == id_center else 10
                W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j],
                          length=1000, free_flow_speed=spd)
            if i != 0:
                spd = 20 if j == id_center else 10
                W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j],
                          length=1000, free_flow_speed=spd)
            if j != jmax-1:
                spd = 20 if i == id_center else 10
                W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1],
                          length=1000, free_flow_speed=spd)
            if j != 0:
                spd = 20 if i == id_center else 10
                W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1],
                          length=1000, free_flow_speed=spd)

    demand_flow = 0.08
    demand_duration = 2400
    outer_ids = 3
    for n1 in [(0,j) for j in range(outer_ids, jmax-outer_ids)]:
        for n2 in [(imax-1,j) for j in range(outer_ids, jmax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(i,jmax-1) for i in range(outer_ids, imax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i,0) for i in range(outer_ids, imax-outer_ids)]:
        for n2 in [(i,jmax-1) for i in range(outer_ids, imax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(imax-1,j) for j in range(outer_ids, jmax-outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    return W


def run_solver(solver_class, cpp, max_iter, n_runs):
    results = []
    for _ in range(n_runs):
        def factory(cpp_flag=cpp):
            return create_world_9x9(cpp=cpp_flag)

        t0 = time.perf_counter()
        solver = solver_class(factory)
        solver.solve(max_iter=max_iter, print_progress=False)
        t_solve = time.perf_counter() - t0

        W_sol = solver.W_sol
        df = W_sol.analyzer.basic_to_pandas()
        completed = int(df["completed_trips"].values[0])
        total_trips = int(df["total_trips"].values[0])
        ttt = float(df["total_travel_time"].values[0])

        results.append({
            "solve_time": t_solve,
            "completed": completed,
            "total_trips": total_trips,
            "ttt": ttt,
        })
    return results


def median_result(results):
    idx = sorted(range(len(results)), key=lambda i: results[i]["solve_time"])[len(results)//2]
    r = results[idx]
    return {
        "solve_time": statistics.median([x["solve_time"] for x in results]),
        "completed": r["completed"],
        "total_trips": r["total_trips"],
        "ttt": r["ttt"],
    }


if __name__ == "__main__":
    MAX_ITER = 100
    N_RUNS = 1

    print(f"Phase 4 Final DTA Benchmark")
    print(f"Network: 9x9 grid (81 nodes, 288 links, 18 OD pairs)")
    print(f"max_iter={MAX_ITER}, {N_RUNS} runs, median reported")
    print()

    configs = [
        ("SolverDUE",      SolverDUE,      False),
        ("SolverDUE",      SolverDUE,      True),
        ("SolverDSO_D2D",  SolverDSO_D2D,  False),
        ("SolverDSO_D2D",  SolverDSO_D2D,  True),
    ]

    all_results = {}
    for solver_name, solver_class, cpp in configs:
        mode = "C++" if cpp else "Python"
        label = f"{solver_name} ({mode})"
        print(f"Running {label}...")
        results = run_solver(solver_class, cpp, MAX_ITER, N_RUNS)
        med = median_result(results)
        all_results[label] = med
        raw_times = [f"{r['solve_time']:.2f}" for r in results]
        print(f"  -> {med['solve_time']:.2f}s (raw: {raw_times})")

    # Print table
    print()
    print(f"{'='*80}")
    print(f"{'Solver + Mode':<30} {'Time(s)':>8} {'Trips':>12} {'TTT':>14}")
    print(f"{'-'*80}")
    for label, r in all_results.items():
        trips_str = f"{r['completed']}/{r['total_trips']}"
        print(f"{label:<30} {r['solve_time']:>8.2f} {trips_str:>12} {r['ttt']:>14.0f}")
    print(f"{'='*80}")

    # Speedups
    print()
    for solver_name in ["SolverDUE", "SolverDSO_D2D"]:
        py_key = f"{solver_name} (Python)"
        cpp_key = f"{solver_name} (C++)"
        if py_key in all_results and cpp_key in all_results:
            py_t = all_results[py_key]["solve_time"]
            cpp_t = all_results[cpp_key]["solve_time"]
            speedup = py_t / cpp_t if cpp_t > 0 else float("inf")
            print(f"{solver_name}: Python {py_t:.2f}s / C++ {cpp_t:.2f}s = {speedup:.2f}x speedup")
