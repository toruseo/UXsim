"""
UXsim C++ mode profiling benchmark - 11x11 Grid Network (based on example 04)
Re-usable A/B baseline benchmark. Random seed fixed for reproducibility.

Network: 11x11 grid, 121 nodes, 440 links
Demand:  boundary-to-boundary, flow=0.035 veh/s, duration=3600s
Params:  deltan=3, tmax=7200, no_cyclic_routing=True, random_seed=0
"""
import time
import cProfile
import pstats
import io
import numpy as np

from uxsim import World

DELTAN = 3
TMAX = 7200
IMAX = 11
JMAX = 11
DEMAND_FLOW = 0.035
DEMAND_DURATION = 3600
RANDOM_SEED = 0
N_RUNS = 10


def build_and_run(cpp=True):
    """Build 11x11 grid network and run simulation. Returns World object."""
    W = World(
        cpp=cpp,
        name="",
        deltan=DELTAN,
        tmax=TMAX,
        no_cyclic_routing=True,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=RANDOM_SEED,
    )

    # nodes
    nodes = {}
    for i in range(IMAX):
        for j in range(JMAX):
            nodes[i, j] = W.addNode(f"n{(i,j)}", i, j)

    # links
    for i in range(IMAX):
        for j in range(JMAX):
            if i != IMAX - 1:
                W.addLink(f"l{(i,j,i+1,j)}", nodes[i, j], nodes[i + 1, j],
                          length=1000, free_flow_speed=20, jam_density=0.2)
            if i != 0:
                W.addLink(f"l{(i,j,i-1,j)}", nodes[i, j], nodes[i - 1, j],
                          length=1000, free_flow_speed=20, jam_density=0.2)
            if j != JMAX - 1:
                W.addLink(f"l{(i,j,i,j+1)}", nodes[i, j], nodes[i, j + 1],
                          length=1000, free_flow_speed=20, jam_density=0.2)
            if j != 0:
                W.addLink(f"l{(i,j,i,j-1)}", nodes[i, j], nodes[i, j - 1],
                          length=1000, free_flow_speed=20, jam_density=0.2)

    # demand: boundary-to-boundary
    for n1 in [(0, j) for j in range(JMAX)]:
        for n2 in [(IMAX - 1, j) for j in range(JMAX)]:
            W.adddemand(nodes[n2], nodes[n1], 0, DEMAND_DURATION, DEMAND_FLOW)
            W.adddemand(nodes[n1], nodes[n2], 0, DEMAND_DURATION, DEMAND_FLOW)
    for n1 in [(i, 0) for i in range(IMAX)]:
        for n2 in [(i, JMAX - 1) for i in range(IMAX)]:
            W.adddemand(nodes[n2], nodes[n1], 0, DEMAND_DURATION, DEMAND_FLOW)
            W.adddemand(nodes[n1], nodes[n2], 0, DEMAND_DURATION, DEMAND_FLOW)

    W.exec_simulation()
    W.analyzer.print_simple_stats()
    return W


def main():
    # === 1. cProfile (C++ mode, 1 run) ===
    print("=" * 70)
    print(f"cProfile: C++ mode, deltan={DELTAN} (cumtime top 30)")
    print("=" * 70)
    pr = cProfile.Profile()
    pr.enable()
    build_and_run(cpp=True)
    pr.disable()

    s = io.StringIO()
    ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
    ps.print_stats(30)
    print(s.getvalue())

    # === 2. 10-run benchmark (C++ mode) ===
    print("=" * 70)
    print(f"Benchmark: C++ mode x{N_RUNS}, deltan={DELTAN}")
    print("=" * 70)
    cpp_times = []
    for i in range(N_RUNS):
        t0 = time.perf_counter()
        build_and_run(cpp=True)
        elapsed = time.perf_counter() - t0
        cpp_times.append(elapsed)
        print(f"  Run {i + 1:2d}: {elapsed:.4f}s")

    cpp_times = np.array(cpp_times)
    print(f"\n  Median: {np.median(cpp_times):.4f}s")
    print(f"  Std:    {np.std(cpp_times):.4f}s")
    print(f"  Min:    {np.min(cpp_times):.4f}s")
    print(f"  Max:    {np.max(cpp_times):.4f}s")

    # === 3. Python mode (1 run) ===
    print("\n" + "=" * 70)
    print(f"Benchmark: Python mode x1, deltan={DELTAN}")
    print("=" * 70)
    t0 = time.perf_counter()
    build_and_run(cpp=False)
    py_time = time.perf_counter() - t0
    print(f"  Time: {py_time:.4f}s")

    # === Summary ===
    print("\n" + "=" * 70)
    print("Summary")
    print("=" * 70)
    print(f"  Network:     {IMAX}x{JMAX} grid ({IMAX*JMAX} nodes, {440} links)")
    print(f"  deltan:      {DELTAN}")
    print(f"  tmax:        {TMAX}")
    print(f"  C++ median:  {np.median(cpp_times):.4f}s  (std={np.std(cpp_times):.4f})")
    print(f"  Python:      {py_time:.4f}s")
    print(f"  Speedup:     {py_time / np.median(cpp_times):.2f}x")


if __name__ == "__main__":
    main()
