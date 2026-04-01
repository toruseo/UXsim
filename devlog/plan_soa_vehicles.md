# Plan: SoA化 of Vehicle Hot-Path Variables

## Concept
Convert Vehicle's frequently-updated variables from AoS to SoA for better cache locality.

## Target Variables (6)
- `x` (double) — position
- `x_next` (double) — next position (car-follow result)
- `v` (double) — velocity
- `move_remain` (double) — remaining movement
- `state` (int) — vehicle state (0-4)
- `link` (Link*) — current link pointer

## Architecture
- World holds parallel arrays: `veh_x[]`, `veh_x_next[]`, etc.
- Vehicle.idx indexes into these arrays
- Vehicle.x becomes inline accessor: `w->veh_x[idx]`
- Backward compatible: Vehicle.x still works in C++ and Python

## Implementation Steps

### Phase 1: Add SoA arrays to World (Alice - traffi.h/cpp)
- Add `vector<double> veh_x, veh_x_next, veh_v, veh_move_remain`
- Add `vector<int> veh_state`
- Add `vector<Link*> veh_link`
- Add `int idx` to Vehicle
- Vehicle constructor assigns idx = vehicles.size() and resizes arrays
- Replace Vehicle member access with array access in hot-path functions

### Phase 2: Update bindings (Alice - bindings.cpp)
- Vehicle.x etc. exposed as properties reading from World arrays
- Or keep direct binding if Vehicle stores reference/pointer

### Phase 3: Test (Carol)
- All 172 tests must pass
- Benchmark comparison

## Risk
- leader->x random access pattern remains (limits car_follow improvement)
- Main expected benefit: state array scan + update loop cache locality
- If no measurable improvement, revert

## Status: IN PROGRESS
