"""
UXsim benchmark: cpp=False vs cpp=True across multiple random seeds.

3 scenarios (Small-scale, Large-scale, DUE/DSO) x 2 modes (Python, C++) x N seeds.
Collects wall-clock time and key simulation metrics, then prints summary statistics.
"""

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm

import uxsim
from uxsim import World
from uxsim.DTAsolvers import SolverDUE, SolverDSO_D2D

N_SEEDS = 2
SEEDS = list(range(N_SEEDS))
MAX_ITER_DUE_DSO = 50


# ============================================================
# Scenario builders
# ============================================================

def run_small(seed, cpp, threads=1):
    """Small-scale merge scenario."""
    W = World(
        name="", deltan=5, tmax=1200,
        no_cyclic_routing=True,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=seed, cpp=cpp, threads=threads,
    )
    W.addNode("orig1", 0, 0)
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
    W.adddemand("orig1", "dest", 0, 1000, 0.45)
    W.adddemand("orig2", "dest", 400, 1000, 0.6)

    t0 = time.perf_counter()
    W.exec_simulation()
    elapsed = time.perf_counter() - t0

    df = W.analyzer.basic_to_pandas()
    link_df = W.analyzer.link_to_pandas()
    return {
        "elapsed": elapsed,
        "total_travel_time": df["total_travel_time"].sum(),
        "average_delay": df["average_delay"].mean(),
        "completed_trips": df["completed_trips"].sum(),
        "_link_df": link_df,
    }


def run_large(seed, cpp, threads=1):
    """Large-scale 11x11 grid scenario."""
    W = World(
        name="", deltan=5, tmax=7200,
        no_cyclic_routing=True,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=seed, cpp=cpp, threads=threads,
    )
    imax, jmax = 11, 11
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i, j] = W.addNode(f"n{(i, j)}", i, j)
    for i in range(imax):
        for j in range(jmax):
            if i != imax - 1:
                W.addLink(f"l{(i, j, i+1, j)}", nodes[i, j], nodes[i+1, j],
                           length=1000, free_flow_speed=20, jam_density=0.2)
            if i != 0:
                W.addLink(f"l{(i, j, i-1, j)}", nodes[i, j], nodes[i-1, j],
                           length=1000, free_flow_speed=20, jam_density=0.2)
            if j != jmax - 1:
                W.addLink(f"l{(i, j, i, j+1)}", nodes[i, j], nodes[i, j+1],
                           length=1000, free_flow_speed=20, jam_density=0.2)
            if j != 0:
                W.addLink(f"l{(i, j, i, j-1)}", nodes[i, j], nodes[i, j-1],
                           length=1000, free_flow_speed=20, jam_density=0.2)

    demand_flow = 0.035
    demand_duration = 3600
    for n1 in [(0, j) for j in range(jmax)]:
        for n2 in [(imax - 1, j) for j in range(jmax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i, 0) for i in range(imax)]:
        for n2 in [(i, jmax - 1) for i in range(imax)]:
            W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    t0 = time.perf_counter()
    W.exec_simulation()
    elapsed = time.perf_counter() - t0

    df = W.analyzer.basic_to_pandas()
    link_df = W.analyzer.link_to_pandas()
    return {
        "elapsed": elapsed,
        "total_travel_time": df["total_travel_time"].sum(),
        "average_delay": df["average_delay"].mean(),
        "completed_trips": df["completed_trips"].sum(),
        "_link_df": link_df,
    }


def _create_due_world(seed, cpp):
    """Helper: build the 9x9 DUE/DSO scenario world."""
    W = uxsim.World(
        name="", deltan=10, tmax=4800,
        duo_update_time=300,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=seed, cpp=cpp,
    )
    imax, jmax = 9, 9
    id_center = 4
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i, j] = W.addNode(f"n{(i, j)}", i, j, flow_capacity=1.6)
    for i in range(imax):
        for j in range(jmax):
            free_flow_speed = 10
            if i != imax - 1:
                if j == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i+1, j)}", nodes[i, j], nodes[i+1, j],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if i != 0:
                if j == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i-1, j)}", nodes[i, j], nodes[i-1, j],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if j != jmax - 1:
                if i == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i, j+1)}", nodes[i, j], nodes[i, j+1],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if j != 0:
                if i == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i, j-1)}", nodes[i, j], nodes[i, j-1],
                           length=1000, free_flow_speed=free_flow_speed)

    demand_flow = 0.08
    demand_duration = 2400
    outer_ids = 3
    for n1 in [(0, j) for j in range(outer_ids, jmax - outer_ids)]:
        for n2 in [(imax - 1, j) for j in range(outer_ids, jmax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(i, jmax - 1) for i in range(outer_ids, imax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i, 0) for i in range(outer_ids, imax - outer_ids)]:
        for n2 in [(i, jmax - 1) for i in range(outer_ids, imax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(imax - 1, j) for j in range(outer_ids, jmax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    return W


def _avg_last_n_link_dfs(dfs_link, n=20):
    """Average traffic_volume and delay_ratio over the last *n* iteration link DataFrames."""
    tail = dfs_link[-n:]
    base = tail[0][["link"]].copy()
    for col in ["traffic_volume", "delay_ratio"]:
        stacked = np.column_stack([df[col].values for df in tail])
        base[col] = stacked.mean(axis=1)
    return base


def run_due_dso(seed, cpp):
    """DUE/DSO scenario: runs DUO, DUE, DSO and returns metrics for each."""
    create_world = lambda: _create_due_world(seed, cpp)

    results = {}

    # DUO
    W = create_world()
    t0 = time.perf_counter()
    W.exec_simulation()
    elapsed_duo = time.perf_counter() - t0
    df_duo = W.analyzer.basic_to_pandas()
    results["duo_elapsed"] = elapsed_duo
    results["duo_ttt"] = df_duo["total_travel_time"].sum()
    results["_duo_link_df"] = W.analyzer.link_to_pandas()

    # DUE
    solver_DUE = SolverDUE(create_world, cpp=cpp)
    t0 = time.perf_counter()
    solver_DUE.solve(max_iter=MAX_ITER_DUE_DSO)
    elapsed_due = time.perf_counter() - t0
    df_due = solver_DUE.W_sol.analyzer.basic_to_pandas()
    results["due_elapsed"] = elapsed_due
    results["due_ttt"] = df_due["total_travel_time"].sum()
    results["_due_link_df"] = _avg_last_n_link_dfs(solver_DUE.dfs_link, n=20)

    # DSO
    solver_DSO = SolverDSO_D2D(create_world, cpp=cpp)
    t0 = time.perf_counter()
    solver_DSO.solve(max_iter=MAX_ITER_DUE_DSO)
    elapsed_dso = time.perf_counter() - t0
    df_dso = solver_DSO.W_sol.analyzer.basic_to_pandas()
    results["dso_elapsed"] = elapsed_dso
    results["dso_ttt"] = min(solver_DSO.ttts)
    best_idx = int(np.argmin(solver_DSO.ttts))
    results["_dso_link_df"] = solver_DSO.dfs_link[best_idx]

    results["total_elapsed"] = elapsed_duo + elapsed_due + elapsed_dso
    return results


# ============================================================
# Main
# ============================================================

def run_scenario(name, run_fn, seeds):
    """Run a scenario function for both cpp=False and cpp=True across all seeds.
    Returns (summary_df, link_data_dict).
    link_data_dict: {seed: {"Python": ..., "C++": ...}} with link DataFrames.
    """
    rows = []
    link_data = {seed: {} for seed in seeds}
    for cpp in [False, True]:
        mode = "C++" if cpp else "Python"
        desc = f"{name} ({mode})"
        for seed in tqdm(seeds, desc=desc):
            res = run_fn(seed, cpp)
            # Extract link DataFrames before building the summary row
            link_dfs = {}
            keys_to_remove = [k for k in res if k.startswith("_")]
            for k in keys_to_remove:
                link_dfs[k] = res.pop(k)
            link_data[seed][mode] = link_dfs
            res["scenario"] = name
            res["mode"] = mode
            res["seed"] = seed
            rows.append(res)
    return pd.DataFrame(rows), link_data


def summarize(df, metric_cols):
    """Print mean +/- std for each scenario x mode combination."""
    group = df.groupby(["scenario", "mode"])
    stats = group[metric_cols].agg(["mean", "std"])
    return stats


def compute_link_accuracy(link_data, seeds, link_df_key="_link_df"):
    """Compare Python vs C++ link-level results for each seed.

    For traffic_volume and delay_ratio, compute:
      - Pearson correlation coefficient
      - MAE (Mean Absolute Error)

    Returns a DataFrame with one row per seed.
    """
    rows = []
    for seed in seeds:
        py_link = link_data[seed]["Python"][link_df_key]
        cpp_link = link_data[seed]["C++"][link_df_key]
        # Merge on link name to ensure alignment
        merged = py_link.merge(cpp_link, on="link", suffixes=("_py", "_cpp"))

        for col in ["traffic_volume", "delay_ratio"]:
            py_vals = merged[f"{col}_py"].values.astype(float)
            cpp_vals = merged[f"{col}_cpp"].values.astype(float)

            # Correlation
            if np.std(py_vals) > 0 and np.std(cpp_vals) > 0:
                corr = np.corrcoef(py_vals, cpp_vals)[0, 1]
            else:
                corr = np.nan

            # MAE
            mae = np.mean(np.abs(py_vals - cpp_vals))

            rows.append({
                "seed": seed,
                "metric": col,
                "corr": corr,
                "MAE": mae,
            })
    return pd.DataFrame(rows)


def compute_link_accuracy_mean(link_data, seeds, link_df_key="_link_df"):
    """Compare Python vs C++ using seed-averaged link values.

    Average traffic_volume and delay_ratio across all seeds for each mode,
    then compute correlation and MAE on the averaged values.
    """
    py_dfs = [link_data[s]["Python"][link_df_key] for s in seeds]
    cpp_dfs = [link_data[s]["C++"][link_df_key] for s in seeds]

    base = py_dfs[0][["link"]].copy()
    rows = []
    for col in ["traffic_volume", "delay_ratio"]:
        py_mean = np.column_stack([df[col].values.astype(float) for df in py_dfs]).mean(axis=1)
        cpp_mean = np.column_stack([df[col].values.astype(float) for df in cpp_dfs]).mean(axis=1)

        if np.std(py_mean) > 0 and np.std(cpp_mean) > 0:
            corr = np.corrcoef(py_mean, cpp_mean)[0, 1]
        else:
            corr = np.nan
        mae = np.mean(np.abs(py_mean - cpp_mean))
        rows.append({"metric": col, "corr": corr, "MAE": mae})
    return pd.DataFrame(rows)


if __name__ == "__main__":
    all_dfs = []
    all_link_data = {}

    # --- Small-scale ---
    print("=" * 60)
    print("Small-scale scenario")
    print("=" * 60)
    df_small, link_small = run_scenario("small", run_small, SEEDS)
    all_dfs.append(df_small)
    all_link_data["small"] = link_small

    # --- Large-scale ---
    print("=" * 60)
    print("Large-scale scenario")
    print("=" * 60)
    df_large, link_large = run_scenario("large", run_large, SEEDS)
    all_dfs.append(df_large)
    all_link_data["large"] = link_large

    # --- Small & Large with threads=8 (C++ only) ---
    print("=" * 60)
    print("Small & Large scenarios (C++, threads=8)")
    print("=" * 60)
    rows_threads8 = []
    for name, run_fn in [("small", run_small), ("large", run_large)]:
        for seed in tqdm(SEEDS, desc=f"{name} (C++, threads=8)"):
            res = run_fn(seed, True, threads=8)
            res = {k: v for k, v in res.items() if not k.startswith("_")}
            res["scenario"] = name
            res["seed"] = seed
            rows_threads8.append(res)
    df_threads8 = pd.DataFrame(rows_threads8)

    # --- DUE/DSO ---
    print("=" * 60)
    print("DUE/DSO scenario")
    print("=" * 60)
    df_due, link_due = run_scenario("due_dso", run_due_dso, SEEDS)
    all_dfs.append(df_due)
    all_link_data["due_dso"] = link_due

    # ============================================================
    # Summary statistics
    # ============================================================
    print("\n" + "=" * 60)
    print("SUMMARY STATISTICS (mean ± std over {} seeds)".format(N_SEEDS))
    print("=" * 60)

    # Small / Large
    df_sl = pd.concat([df_small, df_large], ignore_index=True)
    stats_sl = summarize(df_sl, ["elapsed", "total_travel_time", "average_delay", "completed_trips"])
    print("\n--- Small & Large scenarios ---")
    print(stats_sl.to_string())

    # DUE/DSO
    due_metrics = ["total_elapsed", "duo_elapsed", "due_elapsed", "dso_elapsed",
                   "duo_ttt", "due_ttt", "dso_ttt"]
    stats_due = summarize(df_due, due_metrics)
    print("\n--- DUE/DSO scenario ---")
    print(stats_due.to_string())

    # Speedup ratios
    print("\n--- Speedup (Python time / C++ time) ---")
    for scenario in ["small", "large"]:
        sub = df_sl[df_sl["scenario"] == scenario]
        py_times = sub[sub["mode"] == "Python"]["elapsed"].values
        cpp_times = sub[sub["mode"] == "C++"]["elapsed"].values
        ratio = py_times.mean() / cpp_times.mean()
        print(f"  {scenario}: {ratio:.2f}x")

    sub = df_due
    py_times = sub[sub["mode"] == "Python"]["total_elapsed"].values
    cpp_times = sub[sub["mode"] == "C++"]["total_elapsed"].values
    ratio = py_times.mean() / cpp_times.mean()
    print(f"  due_dso (total): {ratio:.2f}x")

    # ============================================================
    # Link-level accuracy: Python vs C++
    # ============================================================
    print("\n" + "=" * 60)
    print("LINK-LEVEL ACCURACY: Python vs C++ (mean ± std over {} seeds)".format(N_SEEDS))
    print("=" * 60)

    # Small & Large (single link_df per run)
    for scenario_name, link_data in [("small", link_small), ("large", link_large)]:
        acc = compute_link_accuracy(link_data, SEEDS, link_df_key="_link_df")
        summary = acc.groupby("metric")[["corr", "MAE"]].agg(["mean", "std"])
        print(f"\n--- {scenario_name} ---")
        print(summary.to_string())

    # DUE/DSO (separate link_df for DUO, DUE, DSO)
    for solver_name, key in [("DUO", "_duo_link_df"), ("DUE", "_due_link_df"), ("DSO", "_dso_link_df")]:
        acc = compute_link_accuracy(link_due, SEEDS, link_df_key=key)
        summary = acc.groupby("metric")[["corr", "MAE"]].agg(["mean", "std"])
        print(f"\n--- due_dso / {solver_name} ---")
        print(summary.to_string())

    # Seed-averaged link-level accuracy
    print("\n" + "=" * 60)
    print("LINK-LEVEL ACCURACY (seed-averaged): Python vs C++")
    print("=" * 60)

    for scenario_name, link_data, key in [
        ("small", link_small, "_link_df"),
        ("large", link_large, "_link_df"),
        ("DUO", link_due, "_duo_link_df"),
        ("DUE", link_due, "_due_link_df"),
        ("DSO", link_due, "_dso_link_df"),
    ]:
        acc_mean = compute_link_accuracy_mean(link_data, SEEDS, link_df_key=key)
        print(f"\n--- {scenario_name} ---")
        print(acc_mean.to_string(index=False))

    # ============================================================
    # Scatter plots: Python vs C++ (seed-averaged)
    # ============================================================
#    scatter_configs = [
#        ("small", link_small, "_link_df"),
#        ("large", link_large, "_link_df"),
#        ("DUO", link_due, "_duo_link_df"),
#        ("DUE", link_due, "_due_link_df"),
#        ("DSO", link_due, "_dso_link_df"),
#    ]
#    metrics_plot = ["traffic_volume", "delay_ratio"]
#
#    fig, axes = plt.subplots(len(metrics_plot), len(scatter_configs), figsize=(20, 8))
#    for j, (label, ld, key) in enumerate(scatter_configs):
#        py_dfs = [ld[s]["Python"][key] for s in SEEDS]
#        cpp_dfs = [ld[s]["C++"][key] for s in SEEDS]
#        links = py_dfs[0]["link"]
#        avg_data = {"link": links}
#        for col in metrics_plot:
#            avg_data[f"{col}_py"] = np.column_stack(
#                [df[col].values.astype(float) for df in py_dfs]).mean(axis=1)
#            avg_data[f"{col}_cpp"] = np.column_stack(
#                [df[col].values.astype(float) for df in cpp_dfs]).mean(axis=1)
#        merged = pd.DataFrame(avg_data)
#
#        for i, metric in enumerate(metrics_plot):
#            ax = axes[i, j]
#            x = merged[f"{metric}_py"].values
#            y = merged[f"{metric}_cpp"].values
#            ax.scatter(x, y, s=10, alpha=0.5)
#            lo, hi = min(x.min(), y.min()), max(x.max(), y.max())
#            margin = (hi - lo) * 0.05
#            ax.plot([lo - margin, hi + margin], [lo - margin, hi + margin],
#                    "r--", linewidth=0.8)
#            ax.set_xlim(lo - margin, hi + margin)
#            ax.set_ylim(lo - margin, hi + margin)
#            ax.set_aspect("equal")
#            if np.std(x) > 0 and np.std(y) > 0:
#                corr = np.corrcoef(x, y)[0, 1]
#                ax.set_title(f"{label} / {metric}\nr={corr:.3f}", fontsize=10)
#            else:
#                ax.set_title(f"{label} / {metric}", fontsize=10)
#            if j == 0:
#                ax.set_ylabel("C++")
#            if i == len(metrics_plot) - 1:
#                ax.set_xlabel("Python")
#
#    fig.suptitle(f"Link-level: Python vs C++ (averaged over {N_SEEDS} seeds)", fontsize=13)
#    fig.tight_layout()
#    fig.savefig("scatter_link_accuracy.png", dpi=150)
#    print("\nScatter plot saved to scatter_link_accuracy.png")

    # ============================================================
    # Markdown summary output
    # ============================================================
    md = []

    md.append(f"### Small & Large scenarios (mean ± std, {N_SEEDS} seeds)\n")
    md.append("| Scenario | Mode | Elapsed (s) | Total Travel Time | Average Delay | Completed Trips |")
    md.append("| --- | --- | ---:| ---:| ---:| ---:|")
    for scenario in ["small", "large"]:
        for mode in ["Python", "C++"]:
            sub = df_sl[(df_sl["scenario"] == scenario) & (df_sl["mode"] == mode)]
            md.append(
                f"| **{scenario}** | {mode} "
                f"| {sub['elapsed'].mean():.3f} ± {sub['elapsed'].std():.3f} "
                f"| {sub['total_travel_time'].mean():,.0f} ± {sub['total_travel_time'].std():,.0f} "
                f"| {sub['average_delay'].mean():.1f} ± {sub['average_delay'].std():.1f} "
                f"| {sub['completed_trips'].mean():,.0f} ± {sub['completed_trips'].std():,.0f} |"
            )

    md.append(f"\n### DUE/DSO scenario (mean ± std, {N_SEEDS} seeds)\n")
    md.append("| Mode | Total Elapsed (s) | DUO TTT | DUE TTT | DSO TTT |")
    md.append("| --- | ---:| ---:| ---:| ---:|")
    for mode in ["Python", "C++"]:
        sub = df_due[df_due["mode"] == mode]
        md.append(
            f"| {mode} | {sub['total_elapsed'].mean():.1f} ± {sub['total_elapsed'].std():.1f} "
            f"| {sub['duo_ttt'].mean():,.0f} ± {sub['duo_ttt'].std():,.0f} "
            f"| {sub['due_ttt'].mean():,.0f} ± {sub['due_ttt'].std():,.0f} "
            f"| {sub['dso_ttt'].mean():,.0f} ± {sub['dso_ttt'].std():,.0f} |"
        )

    md.append("\n### Speedup (Python / C++)\n")
    for scenario in ["small", "large"]:
        sub = df_sl[df_sl["scenario"] == scenario]
        ratio = sub[sub["mode"] == "Python"]["elapsed"].mean() / sub[sub["mode"] == "C++"]["elapsed"].mean()
        md.append(f"- **{scenario}**: {ratio:.2f}x")
    ratio = df_due[df_due["mode"] == "Python"]["total_elapsed"].mean() / df_due[df_due["mode"] == "C++"]["total_elapsed"].mean()
    md.append(f"- **due_dso**: {ratio:.2f}x")

    md.append(f"\n### C++ threads comparison: threads=1 (default) vs threads=8 (mean ± std, {N_SEEDS} seeds)\n")
    md.append("| Scenario | Threads | Elapsed (s) | Total Travel Time | Speedup vs threads=1 |")
    md.append("| --- | --- | ---:| ---:| ---:|")
    for scenario in ["small", "large"]:
        sub1 = df_sl[(df_sl["scenario"] == scenario) & (df_sl["mode"] == "C++")]
        sub8 = df_threads8[df_threads8["scenario"] == scenario]
        for threads, sub in [(1, sub1), (8, sub8)]:
            speedup = f"{sub1['elapsed'].mean() / sub['elapsed'].mean():.2f}x" if threads != 1 else "-"
            md.append(
                f"| **{scenario}** | {threads} "
                f"| {sub['elapsed'].mean():.3f} ± {sub['elapsed'].std():.3f} "
                f"| {sub['total_travel_time'].mean():,.0f} ± {sub['total_travel_time'].std():,.0f} "
                f"| {speedup} |"
            )

    md.append("\n### Link-level accuracy: Python vs C++\n")
    md.append("DUE metrics are based on average link values over the last 20 iterations. DSO metrics are based on the iteration with the minimum total travel time.\n")

    acc_configs = [
        ("small", link_small, "_link_df"),
        ("large", link_large, "_link_df"),
        ("DUO", link_due, "_duo_link_df"),
        ("DUE", link_due, "_due_link_df"),
        ("DSO", link_due, "_dso_link_df"),
    ]

    md.append(f"#### Per-seed (mean ± std, {N_SEEDS} seeds)\n")
    md.append("| Scenario | Metric | Corr | MAE |")
    md.append("| --- | --- | ---:| ---:|")
    for label, ld, key in acc_configs:
        acc = compute_link_accuracy(ld, SEEDS, link_df_key=key)
        for metric in ["traffic_volume", "delay_ratio"]:
            sub = acc[acc["metric"] == metric]
            md.append(f"| **{label}** | {metric} | {sub['corr'].mean():.3f} ± {sub['corr'].std():.3f} | {sub['MAE'].mean():.2f} ± {sub['MAE'].std():.2f} |")

    md.append(f"\n#### Seed-averaged ({N_SEEDS}-seed mean vs {N_SEEDS}-seed mean)\n")
    md.append("| Scenario | Metric | Corr | MAE |")
    md.append("| --- | --- | ---:| ---:|")
    for label, ld, key in acc_configs:
        acc_mean = compute_link_accuracy_mean(ld, SEEDS, link_df_key=key)
        for _, row in acc_mean.iterrows():
            md.append(f"| **{label}** | {row['metric']} | {row['corr']:.3f} | {row['MAE']:.2f} |")

    md_filename = f"benchmark_summary_v{uxsim.__version__}.md"
    with open(md_filename, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print(f"Markdown summary saved to {md_filename}")
