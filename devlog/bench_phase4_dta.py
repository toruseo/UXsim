"""
Phase 4 DTA Solver Performance Benchmark
Uses the large-scale 9x9 grid scenario from demo_notebook_09en.
Compares Python vs C++ mode for SolverDUE.
Measures: (a) create_World+finalize (b) exec_simulation (c) solver.solve total
Also profiles C++ mode with cProfile.
"""
import time
import sys
import statistics
import cProfile
import pstats
import io
sys.path.insert(0, ".")

import uxsim
from uxsim.DTAsolvers import SolverDUE


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
            free_flow_speed = 10
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


def bench_mode(cpp, max_iter=10, n_runs=3):
    """Benchmark a single mode, returning timing breakdown per run."""
    results = []
    for run in range(n_runs):
        # (a) create_World + finalize
        t0 = time.perf_counter()
        W_test = create_world_9x9(cpp=cpp)
        W_test.finalize_scenario()
        t_create = time.perf_counter() - t0
        del W_test

        # (b) single exec_simulation
        t0 = time.perf_counter()
        W_single = create_world_9x9(cpp=cpp)
        W_single.exec_simulation()
        t_sim = time.perf_counter() - t0

        # (c) full solver.solve
        def factory(cpp_flag=cpp):
            return create_world_9x9(cpp=cpp_flag)

        t0 = time.perf_counter()
        solver = SolverDUE(factory)
        solver.solve(max_iter=max_iter, print_progress=False)
        t_solve = time.perf_counter() - t0

        t_overhead = t_solve - t_sim * max_iter

        results.append({
            "create_finalize": t_create,
            "single_sim": t_sim,
            "total_solve": t_solve,
            "per_iter_avg": t_solve / max_iter,
            "route_overhead_est": max(0, t_overhead),
        })
    return results


def print_results(label, results):
    print(f"\n{'='*60}")
    print(f" {label}")
    print(f"{'='*60}")
    for key in ["create_finalize", "single_sim", "total_solve", "per_iter_avg", "route_overhead_est"]:
        vals = [r[key] for r in results]
        med = statistics.median(vals)
        print(f" {key:22s}: {med:7.3f}s (median of {len(results)})")
    totals = [r["total_solve"] for r in results]
    if len(results) > 1:
        print(f" {'total_solve stdev':22s}: {statistics.stdev(totals):7.3f}s")
    print(f" raw totals: {[f'{t:.3f}' for t in totals]}")


def profile_cpp(max_iter=10):
    """cProfile the C++ mode solver, print top 20 functions."""
    def factory():
        return create_world_9x9(cpp=True)

    pr = cProfile.Profile()
    pr.enable()
    solver = SolverDUE(factory)
    solver.solve(max_iter=max_iter, print_progress=False)
    pr.disable()

    s = io.StringIO()
    ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
    ps.print_stats(20)
    print(f"\n{'='*60}")
    print(f" cProfile top 20 (C++ mode, {max_iter} iters)")
    print(f"{'='*60}")
    print(s.getvalue())


if __name__ == "__main__":
    MAX_ITER = 10
    N_RUNS = 3

    print(f"DTA Solver Benchmark: 9x9 grid (demo_notebook_09en scenario)")
    print(f"Network: 81 nodes, 288 links, 18 OD pairs")
    print(f"SolverDUE max_iter={MAX_ITER}, {N_RUNS} runs each")

    print("\n--- Python mode ---")
    py_results = bench_mode(cpp=False, max_iter=MAX_ITER, n_runs=N_RUNS)
    print_results("Python mode", py_results)

    print("\n--- C++ mode ---")
    cpp_results = bench_mode(cpp=True, max_iter=MAX_ITER, n_runs=N_RUNS)
    print_results("C++ mode", cpp_results)

    py_med = statistics.median([r["total_solve"] for r in py_results])
    cpp_med = statistics.median([r["total_solve"] for r in cpp_results])
    speedup = py_med / cpp_med if cpp_med > 0 else float("inf")

    print(f"\n{'='*60}")
    print(f" SPEEDUP: {speedup:.2f}x (Python {py_med:.3f}s / C++ {cpp_med:.3f}s)")
    print(f"{'='*60}")

    # Breakdown speedups
    for key in ["create_finalize", "single_sim"]:
        py_v = statistics.median([r[key] for r in py_results])
        cpp_v = statistics.median([r[key] for r in cpp_results])
        sp = py_v / cpp_v if cpp_v > 0 else float("inf")
        print(f" {key}: Python {py_v:.3f}s / C++ {cpp_v:.3f}s = {sp:.2f}x")

    print("\n--- cProfile C++ mode ---")
    profile_cpp(max_iter=MAX_ITER)
