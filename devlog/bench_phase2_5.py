"""
Phase 2.5 Baseline Performance Benchmark
Measures simulation time for simple network and Sioux Falls network
in both Python and C++ modes.
"""
import time
import sys
sys.path.insert(0, ".")

from uxsim import World
import numpy as np

def bench_simple_network(cpp=False, n_runs=3):
    """Simple 3-link network with moderate demand"""
    times = []
    for _ in range(n_runs):
        W = World(
            name="bench_simple",
            deltan=5,
            tmax=2000,
            print_mode=0,
            save_mode=0,
            random_seed=42,
            cpp=cpp,
        )
        n1 = W.addNode("n1", 0, 0)
        n2 = W.addNode("n2", 1, 0)
        n3 = W.addNode("n3", 2, 0)
        n4 = W.addNode("n4", 3, 0)
        W.addLink("l1", n1, n2, length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("l2", n2, n3, length=1000, free_flow_speed=20, jam_density=0.2)
        W.addLink("l3", n3, n4, length=1000, free_flow_speed=20, jam_density=0.2)
        W.adddemand(n1, n4, 0, 1500, volume=2400)

        t0 = time.perf_counter()
        W.exec_simulation()
        t1 = time.perf_counter()
        times.append(t1 - t0)
    return times

def bench_sioux_falls(cpp=False, n_runs=3):
    """Sioux Falls network benchmark"""
    times = []
    for _ in range(n_runs):
        W = World(
            name="bench_sioux_falls",
            deltan=5,
            tmax=7200,
            print_mode=0,
            save_mode=0,
            random_seed=42,
            cpp=cpp,
        )
        # Sioux Falls network nodes
        nodes_data = [
            ("1", 0, 4), ("2", 1, 4), ("3", 2, 4.5), ("4", 3, 4),
            ("5", 0, 3), ("6", 1, 3), ("7", 2, 3), ("8", 3, 3),
            ("9", 0, 2), ("10", 1, 2), ("11", 2, 2), ("12", 3, 2),
            ("13", 0, 1), ("14", 1, 1), ("15", 2, 1), ("16", 3, 1),
            ("17", 0, 0), ("18", 1, 0), ("19", 2, 0), ("20", 3, 0),
            ("21", 1, -1), ("22", 2, -1), ("23", 1.5, 4.5), ("24", 2.5, 4.5),
        ]
        for name, x, y in nodes_data:
            W.addNode(name, x, y)

        # Sioux Falls links (simplified - key links)
        links_data = [
            ("1", "2"), ("1", "5"), ("2", "1"), ("2", "3"), ("2", "6"),
            ("3", "2"), ("3", "4"), ("3", "7"), ("3", "23"), ("4", "3"),
            ("4", "8"), ("4", "24"), ("5", "1"), ("5", "6"), ("5", "9"),
            ("6", "2"), ("6", "5"), ("6", "7"), ("6", "10"), ("7", "3"),
            ("7", "6"), ("7", "8"), ("7", "11"), ("8", "4"), ("8", "7"),
            ("8", "12"), ("9", "5"), ("9", "10"), ("9", "13"), ("10", "6"),
            ("10", "9"), ("10", "11"), ("10", "14"), ("11", "7"), ("11", "10"),
            ("11", "12"), ("11", "15"), ("12", "8"), ("12", "11"), ("12", "16"),
            ("13", "9"), ("13", "14"), ("13", "17"), ("14", "10"), ("14", "13"),
            ("14", "15"), ("14", "18"), ("15", "11"), ("15", "14"), ("15", "16"),
            ("15", "19"), ("16", "12"), ("16", "15"), ("16", "20"), ("17", "13"),
            ("17", "18"), ("18", "14"), ("18", "17"), ("18", "19"), ("18", "21"),
            ("19", "15"), ("19", "18"), ("19", "20"), ("19", "22"), ("20", "16"),
            ("20", "19"), ("21", "18"), ("21", "22"), ("22", "19"), ("22", "21"),
            ("23", "3"), ("23", "24"), ("24", "4"), ("24", "23"),
        ]
        for o, d in links_data:
            W.addLink(f"{o}-{d}", o, d, length=1000, free_flow_speed=20,
                      jam_density=0.2, number_of_lanes=2)

        # Add demand between several OD pairs
        od_pairs = [
            ("1", "20"), ("5", "16"), ("9", "12"), ("13", "4"),
            ("17", "8"), ("2", "19"), ("6", "15"), ("10", "24"),
            ("21", "3"), ("22", "7"),
        ]
        for o, d in od_pairs:
            W.adddemand(o, d, 0, 5400, volume=1800)

        t0 = time.perf_counter()
        W.exec_simulation()
        t1 = time.perf_counter()
        times.append(t1 - t0)
    return times

if __name__ == "__main__":
    print("=" * 60)
    print("Phase 2.5 Baseline Performance Benchmark")
    print("=" * 60)

    n_runs = 3

    # Simple network
    print("\n--- Simple Network (3 links, 2000s, 2400 veh) ---")
    py_simple = bench_simple_network(cpp=False, n_runs=n_runs)
    print(f"  Python: {py_simple} => mean={np.mean(py_simple):.4f}s")
    cpp_simple = bench_simple_network(cpp=True, n_runs=n_runs)
    print(f"  C++:    {cpp_simple} => mean={np.mean(cpp_simple):.4f}s")
    if np.mean(cpp_simple) > 0:
        print(f"  Speedup: {np.mean(py_simple)/np.mean(cpp_simple):.2f}x")

    # Sioux Falls
    print("\n--- Sioux Falls Network (76 links, 7200s, 18000 veh) ---")
    py_sf = bench_sioux_falls(cpp=False, n_runs=n_runs)
    print(f"  Python: {py_sf} => mean={np.mean(py_sf):.4f}s")
    cpp_sf = bench_sioux_falls(cpp=True, n_runs=n_runs)
    print(f"  C++:    {cpp_sf} => mean={np.mean(cpp_sf):.4f}s")
    if np.mean(cpp_sf) > 0:
        print(f"  Speedup: {np.mean(py_sf)/np.mean(cpp_sf):.2f}x")

    print("\n" + "=" * 60)
    print("Benchmark complete!")
