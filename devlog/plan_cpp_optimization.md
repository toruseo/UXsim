# C++ Engine & Wrapper Optimization Plan

## Goal
Identify and eliminate performance bottlenecks in the C++ engine and Python wrapper to maximize simulation speed.

## Benchmark Setup
- **Scenario**: 11x11 grid network (example 04 based), 121 nodes, 440 links
- **Params**: deltan=3, tmax=7200, no_cyclic_routing=True, random_seed=0
- **Demand**: boundary-to-boundary, flow=0.035 veh/s, duration=3600s, ~19,844 vehicles
- **Measurement**: 10 runs, median + std (1 thread)
- **Script**: `devlog/bench_heavy.py`

## Agent Assignment
- **Alice (%1)**: C++ engine internals (traffi.h, traffi.cpp)
- **Bob (%2)**: Python wrapper (uxsim_cpp_wrapper.py, bindings.cpp)
- **Carol (%3)**: Profiling, benchmarking, testing (test_cpp_mode.py)

---

## Round 1: Initial Profiling & Quick Wins

### Phase 1: Baseline Profiling (Carol) — DONE
- [x] cProfile benchmark on grid network (deltan=3)
- [x] Identify top bottleneck functions (cumtime sort)
- [x] Measure baseline: 10 runs, median+std

**Baseline Results:**
| Metric | Value |
|--------|-------|
| C++ median | 4.97s (std=0.82s) |
| Python mode | 108.3s |
| Speedup | 21.8x |

**cProfile Breakdown (Baseline):**
| Function | Time | % |
|----------|------|---|
| exec_simulation (total) | 5.449s | 100% |
| C++ computation (tottime) | 3.487s | 64% |
| simulation_terminated | 1.818s | 33% |
| _build_vehicles_enter_log | 1.581s | 29% |
| log_t_link (39688 calls) | 1.419s | 26% |
| _build_all_vehicle_log_caches | 0.935s | 17% |
| adddemand | 0.224s | 4% |

### Phase 2: Round 1 Optimizations (Alice & Bob) — DONE
- [x] Alice: const-ref linkset parameter, pass-by-reference optimizations in traffi.cpp
- [x] Bob: bulk build_enter_log_data() C++ API to replace per-vehicle Python loop in _build_vehicles_enter_log

### Phase 3: Round 1 Results (Carol) — DONE
- [x] Precision benchmark (no resource contention)

**Round 1 Optimized Results:**
| Metric | Baseline | Round 1 | Change |
|--------|----------|---------|--------|
| C++ median | 4.97s | 4.66s | **-6.2%** |
| C++ std | 0.82s | 0.10s | Stability improved |
| Python mode | 108.3s | 102.3s | (variance) |
| Speedup | 21.8x | 21.9x | |

**cProfile Breakdown (Round 1):**
| Function | Baseline | Round 1 | Change |
|----------|----------|---------|--------|
| C++ computation (tottime) | 3.487s | 3.046s | -12.6% |
| _build_vehicles_enter_log | 1.581s | 0.241s | **-84.8%** |
| log_t_link calls | 39688 | 19844 | **-50%** |
| _build_all_vehicle_log_caches | 0.935s | 0.703s | -24.8% |

**Key improvement**: _build_vehicles_enter_log was reduced from 1.58s to 0.24s by using bulk C++ API (build_enter_log_data) instead of per-vehicle Python log_t_link access.

**Note on od_analysis apparent regression (0.24s→1.33s)**: This is a measurement artifact, not a real regression. The log_t_link cache construction cost shifted from _build_vehicles_enter_log to od_analysis (which now triggers the first log_t_link access). Total time unchanged.

---

## Round 2: C++ Engine Internal Optimization (PLANNED)

### C++ main_loop Internal Profiling (Carol) — DONE

Instrumented main_loop with std::chrono timers. Results (4.41s total, 2400 timesteps, 19844 vehicles):

| Phase | Time | % | Notes |
|-------|------|---|-------|
| link_update | 0.08s | 1.9% | |
| node_generate | 0.05s | 1.1% | |
| node_transfer | 0.49s | 11.0% | |
| car_follow | 0.44s | 9.9% | |
| **veh_update** | **3.34s** | **75.8%** | **Dominant bottleneck** |
| ├ log_data | 1.77s | 40.2% | 6x push_back per vehicle per step |
| ├ route_next_link_choice | 0.66s | 14.9% | no_cyclic_routing log_link scan |
| └ other_logic | 0.91s | 20.7% | state transitions, end_trip, etc. |
| route_choice (DUO) | 0.02s | 0.4% | |

**With vehicle_log_mode=0**: veh_update drops from 2.23s to 0.60s — log_data accounts for 73% of veh_update.

### no_cyclic_routing log_link Scan Analysis (Carol) — DONE

Problem: route_next_link_choice scans entire log_link vector to build traveled_nodes set on **every call**.

| Metric | Value |
|--------|-------|
| log_link avg size | 465 elements (max=2000) |
| route_next_link_choice calls | 281,620 |
| Avg scan length per call | 274 elements |
| **Total scan iterations** | **77,042,283** |

### Round 2 Optimization Targets

#### Target 1: log_data optimization (40% of C++ time)
- **Problem**: 6x vector push_back per vehicle per timestep (~19844 vehicles × 2400 steps)
- **Options**:
  - (a) Pre-reserve vector capacity (estimated vehicle lifetime × entry count)
  - (b) SoA (Structure of Arrays) — single contiguous buffer with stride
  - (c) Reduce logging frequency (every N steps instead of every step)
  - (d) Conditional logging (only log state changes)

#### Target 2: no_cyclic_routing optimization (15% of C++ time)
- **Problem**: O(log_link_size) scan per route_next_link_choice call, 77M total iterations
- **Solution**: Incremental `visited_nodes` member on Vehicle
  - `vector<bool> visited_nodes(num_nodes)` set at link entry
  - route_next_link_choice checks `visited_nodes[end_node->id]` in O(1)
  - **Expected reduction**: 77M scans → 281K O(1) lookups (99.6% reduction)
- See `devlog/plan_no_cyclic_routing.md` for detailed plan

#### Target 3: Active vehicle indexing (reduces iteration overhead)
- **Problem**: veh_update iterates all 19844 vehicles, but many are HOME/END/ABORT
- **Solution**: Maintain active vehicle list (HOME+WAIT+RUN only)
- **Expected benefit**: Skip ~30-50% of vehicles in later timesteps

#### Target 4: Python wrapper post-processing (~1.7s)
- _build_all_vehicle_log_caches: 0.70s
- od_analysis vehicle loop: 0.23s
- Potential: bulk numpy operations instead of per-vehicle Python loops

### Round 2 Priority Order
1. **no_cyclic_routing** — Clearest win, well-analyzed, low risk (Alice)
2. **log_data reserve** — Low-hanging fruit, minimal code change (Alice)
3. **Active vehicle indexing** — Moderate complexity, good payoff (Alice)
4. **Wrapper post-processing** — Python-side, independent work (Bob)

---

## Progress Log

| Date | Round | Agent | Action | Result |
|------|-------|-------|--------|--------|
| 2026-04-01 | R1 | Carol | Baseline profiling (Sioux Falls + grid) | C++ 4.97s, Python 108.3s, 21.8x |
| 2026-04-01 | R1 | Alice | const-ref linkset, pass-by-ref fixes | Minor C++ improvement |
| 2026-04-01 | R1 | Bob | Bulk build_enter_log_data API | _build_veh_enter_log 1.58s→0.24s |
| 2026-04-01 | R1 | Carol | Precision benchmark (Round 1) | C++ 4.66s (-6.2%), std 0.10s |
| 2026-04-01 | R2 | Carol | C++ internal profiling (std::chrono) | veh_update=75.8%, log_data=40% |
| 2026-04-01 | R2 | Carol | no_cyclic_routing analysis | 77M scan iterations, O(1) fix possible |
