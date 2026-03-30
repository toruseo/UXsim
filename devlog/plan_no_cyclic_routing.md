# Plan: Port no_cyclic_routing to C++ Engine

## Task
Port `no_cyclic_routing` feature from Python (uxsim.py) to the C++ engine.

## Changes Required

### 1. traffi.h
- Add `bool no_cyclic_routing;` to World struct
- Add to World constructor parameters

### 2. traffi.cpp
- World constructor: initialize `no_cyclic_routing`
- `Vehicle::route_next_link_choice()`: After links_avoid filter, add no_cyclic_routing logic:
  - Collect start_nodes of all previously traveled links (from log_link)
  - Filter outlinks to exclude those whose end_node is in traveled_nodes
  - Only apply if filtered result is non-empty

### 3. bindings.cpp
- Add `no_cyclic_routing` parameter to World binding constructor
- Expose `no_cyclic_routing` as a readwrite attribute

### 4. uxsim_cpp_wrapper.py
- Add `no_cyclic_routing` parameter to CppWorld.__init__
- Pass it to C++ world via `_ensure_cpp_world`

### 5. test_cpp_mode.py
- Copy `test_route_choice_no_cyclic_routing` from test_verification_route_choice.py
- Replace `World(` with `World(cpp=True, `

## Owner Assignment
- Alice (%2): traffi.h + traffi.cpp
- Bob (%1): bindings.cpp + uxsim_cpp_wrapper.py
- Carol (%3): test_cpp_mode.py

## Status: IN PROGRESS
