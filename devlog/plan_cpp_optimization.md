# C++ Engine & Wrapper Optimization Plan

## Goal
Identify and eliminate performance bottlenecks in the C++ engine and Python wrapper to maximize simulation speed.

## Phase 1: Baseline & Profiling
- [ ] Run cProfile benchmark on a standard scenario (Sioux Falls or grid network)
- [ ] Identify top 10 bottleneck functions (cumtime sort)
- [ ] Measure baseline: 1-thread, 10 runs, median+std

## Phase 2: Optimization Rounds
Each round:
1. Identify top bottleneck
2. Implement fix
3. Run tests (pytest --reruns 5)
4. Measure improvement (A/B benchmark)
5. Commit & push if tests pass and improvement confirmed

### Potential optimization areas:
- Wrapper overhead (property access, Python↔C++ boundary)
- C++ engine internals (hot loops, memory allocation)
- Data structure choices (vector vs flat array, SoA vs AoS)
- pybind11 binding overhead (numpy returns, string handling)

## Phase 3: Validation
- Full test suite pass (Python + C++ modes)
- Final benchmark comparison vs baseline

## Agent Assignment
- **Alice (%1)**: C++ engine internals (traffi.h, traffi.cpp)
- **Bob (%2)**: Python wrapper (uxsim_cpp_wrapper.py)  
- **Carol (%3)**: Profiling, benchmarking, testing (test_cpp_mode.py)

## Progress Log
(updated during work)
