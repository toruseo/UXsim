# Analyzer Compatibility Specification with C++ Changes Allowed

## Purpose

This document specifies the preferred architecture for making the C++ engine compatible with existing Python-side UXsim consumers, especially `uxsim/analyzer.py`.

This version assumes:

- C++ side changes are allowed
- pybind/binding layer changes are allowed
- a small Python compatibility layer is still allowed when necessary

The goal is to maximize runtime efficiency while preserving analyzer compatibility with minimal analyzer changes.

## Final Design Decision

Adopt a **C++-first compatibility architecture**:

- simulation state remains owned by C++
- compatibility-critical data shaping should be implemented in C++ or bindings when practical
- Python should keep only a thin facade for object identity, dict-like containers, and analyzer-writable scratch fields

This is preferred over a Python-heavy adapter design if C++ modifications are permitted.

## Why This Is More Efficient

If compatibility conversion is done only in Python:

- vehicle logs must be repeatedly transformed from C++ layouts into analyzer-facing layouts
- link ID to Link object conversion happens on the Python side
- state-int to state-string conversion happens on the Python side
- transition log reconstruction happens on the Python side

If compatibility conversion is pushed into C++ or bindings:

- bulk log construction can be done once in C++
- intermediate Python allocations can be reduced
- repeated cross-language calls can be reduced
- compact internal representations can still be kept until export time

Therefore, with C++ changes allowed, the most efficient design is not "raw C++ only" and not "Python-only adaptation", but a hybrid where:

- heavy compatibility shaping is done in C++/bindings
- Python preserves UXsim object semantics and analyzer scratch mutability

## Scope

This specification covers:

- C++ engine data exposure for analyzer compatibility
- binding-layer compatibility APIs
- minimal Python facade requirements
- ownership of analyzer scratch data
- performance constraints

This specification does not require:

- large-scale `analyzer.py` rewrites
- moving analyzer algorithms themselves to C++
- eliminating all Python facade objects

## Architectural Principles

### 1. C++ owns canonical runtime state

The following belong in C++:

- node/link/vehicle canonical state
- cumulative curves
- route choice state
- trajectory primitive data
- vehicle logs
- bulk log export

### 2. Compatibility shaping should be pushed down

If analyzer expects a specific structure and that structure is expensive to reconstruct in Python, the preferred implementation is:

- first: C++ core
- second: pybind layer
- last resort: Python proxy

### 3. Python remains responsible for dynamic scratch state

Fields created and mutated by analyzer itself should remain on Python-side objects.

Typical examples:

- `tss`
- `xss`
- `ls`
- `cs`
- `names`
- `k_mat`
- `q_mat`
- `v_mat`
- `tn_mat`
- `dn_mat`
- `an`

These are analyzer working buffers, not core simulation state.

### 4. Object identity must remain stable in Python

Even if C++ generates the data, Python-facing objects must preserve UXsim-like identity semantics where analyzer expects object references.

Examples:

- `veh.log_link[i]` should resolve to the same Python Link object as `W.LINKS[j]`
- `veh.orig` and `veh.dest` should resolve to stable Python Node objects

## Recommended Division of Responsibilities

## C++ Core Responsibilities

Implement or maintain in C++:

- compact internal log storage
- batch full-log construction for all vehicles
- abort-aware effective state computation
- transition-log generation
- travel time and cumulative curves
- ID-stable node/link/vehicle indexing

Preferred additions on the C++ side:

- helper APIs that export analyzer-ready log payloads in batch
- helper APIs that expose effective states already normalized to UXsim semantics
- helper APIs that expose transition logs without requiring Python to rescan full logs

## Binding Layer Responsibilities

Bindings should expose efficient compatibility-oriented APIs such as:

- `build_all_vehicle_logs()`
- `build_full_log()`
- state-string conversion helpers when useful
- transition-log builders
- direct reference accessors for node/link/vehicle by name and ID

Bindings should prefer:

- batch export over many per-element calls
- arrays or compact structures over piecemeal property access
- zero-copy or near-zero-copy transfer where safe

## Python Facade Responsibilities

Python should keep only what is hard or inappropriate to store in raw C++ objects:

- `W.VEHICLES` dict-like facade with `.values()`
- stable Python `Node`/`Link`/`Vehicle` compatibility objects
- name lookup dictionaries
- analyzer scratch fields
- optional cached compatibility views built from batch C++ exports

Python should not:

- mirror all simulation state eagerly
- recompute full logs repeatedly
- become a second simulation state store

## Required Compatibility Surface

### World

The world object seen by `analyzer.py` must provide:

- `NODES`
- `LINKS`
- `VEHICLES`
- `ADJ_MAT`
- `DELTAT`
- `DELTAN`
- `TMAX`
- `TSIZE`
- `EULAR_DT`
- `analyzer`
- `get_node()`
- `get_link()`

Compatibility rules:

- `VEHICLES` must remain dict-like for `.values()`
- `LINKS[i]` and `NODES[i]` must remain index-stable
- link IDs used in logs must match Python `W.LINKS` ordering

### Node

Node-compatible objects must expose at least:

- `id`
- `name`
- `x`
- `y`
- `inlinks`
- `outlinks`

Direct raw C++ exposure is acceptable if Python identity mapping is stable.

### Link

Link-compatible objects must expose:

- `id`
- `name`
- `start_node`
- `end_node`
- `length`
- `u`
- `capacity_in`
- `capacity_out`
- `cum_arrival`
- `cum_departure`
- `traveltime_instant`
- `traveltime_actual`
- `speed`
- `density`
- `flow`
- `num_vehicles`
- `num_vehicles_queue`
- `arrival_count(t)`
- `departure_count(t)`
- `actual_travel_time(t)`
- `average_travel_time_between(t0, t1)`
- `average_speed(t)`
- `average_density(t)`
- `average_flow(t)`

Link-compatible Python objects must also allow analyzer scratch fields:

- `tss`
- `xss`
- `ls`
- `cs`
- `names`
- `edie_dx`
- `edie_dt`
- `k_mat`
- `q_mat`
- `v_mat`
- `tn_mat`
- `dn_mat`
- `an`

### Vehicle

Vehicle-compatible objects must expose:

- `id`
- `name`
- `orig`
- `dest`
- `state`
- `departure_time`
- `departure_time_in_second`
- `arrival_time`
- `travel_time`
- `x`
- `v`
- `lane`
- `link`
- `distance_traveled`
- `color`
- `log_t`
- `log_state`
- `log_link`
- `log_x`
- `log_s`
- `log_v`
- `log_lane`
- `log_t_link`

Vehicle semantics must match UXsim expectations:

- effective state strings must be `"home"`, `"wait"`, `"run"`, `"end"`, `"abort"`
- abort cases must not silently appear as normal `"end"`
- link references in logs must resolve to Python Link objects, not plain IDs

## Data Export Strategy

### Preferred strategy

Use C++ batch export APIs to produce analyzer-oriented payloads once, then map them to stable Python objects.

Preferred pipeline:

1. C++ builds full log payloads for all vehicles.
2. Bindings expose compact arrays or structured payloads.
3. Python resolves IDs to stable `Node`/`Link` objects and caches the result.
4. Analyzer reads the cached compatibility view.

### Discouraged strategy

Avoid this pattern when performance matters:

1. analyzer accesses `veh.log_link`
2. Python calls into C++ many times per element
3. Python reconstructs object/string forms repeatedly

That pattern creates unnecessary cross-language overhead.

## Adaptation Rules

### Rule 1: Simple scalar reads should stay direct

Examples:

- `link.name`
- `link.length`
- `link.u`
- `veh.name`
- `veh.x`
- `veh.v`

These should be direct C++ reads or binding-exposed properties.

### Rule 2: Compatibility-heavy transforms should move downward

If a property requires:

- full-log reconstruction
- transition reconstruction
- effective-state normalization
- batch conversion over many vehicles

then the preferred implementation location is C++ or bindings.

### Rule 3: Python should cache final compatibility views

Python may cache:

- `log_state`
- `log_link`
- `log_t_link`

but should preferably cache the result of a batch C++ export rather than recompute from raw primitive arrays one vehicle at a time.

### Rule 4: Analyzer scratch state stays Python-side

If analyzer writes to a field, that field should generally remain on a Python object.

Reason:

- raw pybind objects are not ideal as arbitrary mutable scratch pads
- analyzer scratch data is not canonical engine state

### Rule 5: Collection shape compatibility is mandatory

Even if C++ internals use vectors, Python-facing world collections must preserve UXsim-compatible interfaces where analyzer depends on them.

Most importantly:

- `W.VEHICLES` must remain dict-like

## Performance Requirements

### Required

- no full eager duplication of simulation state in Python
- no repeated per-access reconstruction of already built compatibility logs
- batch log export from C++ should be the default path
- link ID -> Python Link resolution must be O(1)
- name -> Node/Link resolution must be O(1)
- repeated analyzer passes over logs must reuse cached compatibility views

### Strongly Recommended

- expose analyzer-oriented helper APIs from bindings
- keep log storage compact in C++
- reconstruct `log_t_link` in C++, not by rescanning from Python
- normalize effective abort/end state in C++ helper APIs
- minimize per-element Python/C++ boundary crossings

### Not Required

- pure zero-copy everywhere
- elimination of all Python proxy objects

The target is end-to-end efficiency with analyzer compatibility, not ideological binding minimalism.

## Data Ownership

Ownership rules:

- canonical simulation state: C++
- analyzer scratch matrices/lists: Python
- compatibility-export payloads: produced by C++, cached by Python

Python must not become an authoritative mirror of the engine.

## Recommended Implementation Shape

### World facade

Maintain in Python:

- `NODES`: stable Python-facing node objects
- `LINKS`: stable Python-facing link objects
- `VEHICLES`: ordered dict-like mapping to vehicle objects
- lookup dictionaries by name

Use C++ for:

- bulk state and log data
- by-ID / by-name resolution helpers

### Link object

Use C++/bindings for:

- scalar properties
- cumulative curves
- travel-time arrays
- dynamic state queries

Use Python object storage for:

- analyzer scratch buffers

### Vehicle object

Use C++/bindings for:

- core scalar state
- effective state normalization helpers
- full log export
- transition log export

Use Python for:

- stable object identity
- cached compatibility view
- references to world-owned `Node`/`Link` objects
- analyzer-only fields such as `color` and `distance_traveled` when not canonical in C++

## Acceptance Criteria

The design is acceptable if:

1. `analyzer.py` runs with no large redesign.
2. `W.analyzer.basic_to_pandas()` works in C++ mode.
3. `W.analyzer.link_to_pandas()` works in C++ mode.
4. `W.analyzer.od_to_pandas()` works in C++ mode.
5. `W.analyzer.compute_edie_state()` works in C++ mode.
6. Vehicle compatibility logs are built from batch C++ export or equivalent efficient path.
7. Repeated analyzer access does not repeatedly rebuild log compatibility data.
8. Python does not store a full duplicated mirror of all canonical engine state.

## Recommended Tests

Minimum tests:

- `basic_to_pandas()` on a one-link scenario
- `link_to_pandas()` cumulative curves correctness
- `od_to_pandas()` travel time and distance correctness
- `compute_edie_state()` trajectory reconstruction correctness
- `network()` and plotting smoke tests
- stable identity test: `veh.log_link[i] is W.get_link(veh.log_link[i].name)` for valid entries
- cache reuse test: repeated `veh.log_link` access does not rebuild compatibility logs
- batch-vs-per-vehicle test: batch export path is actually used in analyzer-heavy workloads

## Explicit Conclusion

If C++ changes are allowed, the preferred specification is:

- **push compatibility-heavy work into C++ and bindings**
- **keep Python as a thin UXsim-compatible facade**
- **retain Python-side analyzer scratch mutability**
- **avoid both raw-binding-only design and Python-heavy full adaptation**

This is the most efficient architecture while keeping `analyzer.py` largely intact.
