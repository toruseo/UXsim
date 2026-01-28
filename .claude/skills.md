# UXsim: Network Traffic Flow Simulator

UXsim is a free, open-source macroscopic and mesoscopic network traffic flow simulator written in pure Python. It simulates the movements of car travelers and traffic congestion in road networks, suitable for city-scale traffic phenomena simulation.

## Quick Start

### Installation
```bash
pip install uxsim
```

### Minimal Example
```python
from uxsim import *

# Create simulation world
W = World(
    name="",        # Scenario name
    deltan=5,       # Platoon size (aggregation unit)
    tmax=1200,      # Total simulation time in seconds
    print_mode=1,   # Print progress (0=silent, 1=verbose)
    save_mode=1,    # Save results (0=no, 1=yes)
    show_mode=0,    # Show plots (0=no, 1=yes)
    random_seed=0   # Random seed for reproducibility
)

# Create nodes (intersections)
W.addNode("origin", 0, 0)      # name, x, y coordinates
W.addNode("destination", 1000, 0)

# Create links (roads)
W.addLink("road1", "origin", "destination", 
          length=1000,           # meters
          free_flow_speed=20,    # m/s
          number_of_lanes=1)

# Add traffic demand (vehicles traveling from origin to destination)
W.adddemand("origin", "destination", 
            t_start=0,    # demand start time (s)
            t_end=600,    # demand end time (s)
            flow=0.5)     # vehicles per second

# Run simulation
W.exec_simulation()

# Print results
W.analyzer.print_simple_stats()
```

## Core Concepts

### World
The `World` class is the main simulation container. All nodes, links, and vehicles exist within a World.

**Key Parameters:**
- `deltan`: Platoon size for mesoscopic simulation (default 5). Smaller = more accurate but slower
- `tmax`: Total simulation duration in seconds
- `print_mode`: 0 for silent, 1 for progress output
- `save_mode`: 1 to save visualizations to `out{name}/` directory
- `show_mode`: 1 to display plots (useful in Jupyter notebooks)
- `random_seed`: Set for reproducible results

### Node
Nodes represent intersections or endpoints in the network.

```python
# Basic node
W.addNode("node_name", x=0, y=0)

# Node with traffic signal
W.addNode("signalized", x=100, y=0, 
          signal=[60, 40])  # [phase0_duration, phase1_duration, ...]

# Node with flow capacity constraint
W.addNode("constrained", x=200, y=0, flow_capacity=0.8)  # vehicles/second
```

### Link
Links represent road segments connecting nodes.

```python
# Basic link
W.addLink("link_name", "start_node", "end_node",
          length=1000,           # meters
          free_flow_speed=20,    # m/s (default)
          jam_density=0.2,       # vehicles/m (default)
          number_of_lanes=1)     # lanes (default)

# Link with signal group (for signalized intersections)
W.addLink("link1", "orig", "signalized",
          length=500,
          signal_group=0)  # Green during signal phase 0

W.addLink("link2", "orig2", "signalized", 
          length=500,
          signal_group=1)  # Green during signal phase 1

# Link with merge priority
W.addLink("main_road", "A", "B", length=500, merge_priority=2)
W.addLink("ramp", "C", "B", length=200, merge_priority=1)

# Link with capacity constraints
W.addLink("bottleneck", "X", "Y",
          length=500,
          capacity_in=0.5,   # inflow capacity (veh/s)
          capacity_out=0.5)  # outflow capacity (veh/s)
```

### Demand
Traffic demand specifies how many vehicles travel between origin-destination pairs.

```python
# Constant demand
W.adddemand("origin", "destination", 
            t_start=0,      # start time (s)
            t_end=1000,     # end time (s)
            flow=0.4)       # vehicles per second

# Multiple OD pairs
W.adddemand("A", "B", 0, 500, 0.3)
W.adddemand("A", "C", 0, 500, 0.2)
W.adddemand("B", "C", 200, 800, 0.5)
```

### Vehicle
Vehicles are automatically created from demand, but can also be added manually.

```python
# Add a single vehicle
W.addVehicle("origin_node", "destination_node", departure_time=100)

# Add taxi/shared mobility vehicle (no fixed destination)
W.addVehicle("start_node", None, departure_time=0, mode="taxi")
```

## Simulation Execution

### Basic Execution
```python
# Run entire simulation at once
W.exec_simulation()
```

### Step-by-Step Execution (for intervention/control)
```python
while W.check_simulation_ongoing():
    W.exec_simulation(duration_t=100)  # Run for 100 seconds
    
    # Access current state and make interventions here
    for link in W.LINKS:
        print(f"{link.name}: {link.num_vehicles} vehicles")
```

## Analysis and Visualization

### Basic Statistics
```python
W.analyzer.print_simple_stats()
```

Output includes:
- Number of vehicles
- Average speed
- Trip completion rate
- Average travel time and delay

### Export to pandas DataFrames
```python
# Basic statistics
df_basic = W.analyzer.basic_to_pandas()

# OD pair statistics
df_od = W.analyzer.od_to_pandas()

# Link statistics
df_link = W.analyzer.link_to_pandas()

# Link traffic states over time
df_link_state = W.analyzer.link_traffic_state_to_pandas()

# Individual vehicle data
df_vehicles = W.analyzer.vehicles_to_pandas()

# Macroscopic Fundamental Diagram data
df_mfd = W.analyzer.mfd_to_pandas()
```

### Export to CSV
```python
W.analyzer.output_data()  # Saves all data to out{name}/ directory
```

### Visualizations
```python
# Network state at specific time
W.analyzer.network(t=600, detailed=1)

# Time-space diagrams
W.analyzer.time_space_diagram_traj()      # Vehicle trajectories
W.analyzer.time_space_diagram_density()   # Density plot

# Time-space diagram for specific route
W.analyzer.time_space_diagram_traj_links([["link1", "link2", "link3"]])

# Cumulative curves (arrivals/departures)
W.analyzer.cumulative_curves()

# Macroscopic Fundamental Diagram
W.analyzer.macroscopic_fundamental_diagram()

# Animations (saved to out{name}/ directory)
W.analyzer.network_anim(detailed=0)         # Simple animation
W.analyzer.network_anim(detailed=1)         # Detailed animation
W.analyzer.network_fancy(sample_ratio=0.3)  # Vehicle trajectory animation
```

## Common Patterns

### Grid Network
```python
# Create grid of nodes
nodes = {}
for i in range(10):
    for j in range(10):
        nodes[i,j] = W.addNode(f"n{i}_{j}", i*1000, j*1000)

# Create bidirectional links
for i in range(10):
    for j in range(10):
        if i < 9:
            W.addLink(f"l{i}_{j}_E", nodes[i,j], nodes[i+1,j], length=1000)
            W.addLink(f"l{i+1}_{j}_W", nodes[i+1,j], nodes[i,j], length=1000)
        if j < 9:
            W.addLink(f"l{i}_{j}_N", nodes[i,j], nodes[i,j+1], length=1000)
            W.addLink(f"l{i}_{j+1}_S", nodes[i,j+1], nodes[i,j], length=1000)
```

### Signal Control
```python
# Create signalized intersection
W.addNode("intersection", 100, 100, signal=[60, 40])

# Links with signal groups
W.addLink("north_approach", "north", "intersection", 500, signal_group=0)
W.addLink("south_approach", "south", "intersection", 500, signal_group=0)
W.addLink("east_approach", "east", "intersection", 500, signal_group=1)
W.addLink("west_approach", "west", "intersection", 500, signal_group=1)
```

### Dynamic Signal Control
```python
while W.check_simulation_ongoing():
    W.exec_simulation(duration_t=60)
    
    # Get intersection node
    intersection = W.get_node("intersection")
    
    # Modify signal timing based on queue length
    queue_ns = sum(len(l.vehicles) for l in intersection.inlinks.values() 
                   if 0 in l.signal_group)
    queue_ew = sum(len(l.vehicles) for l in intersection.inlinks.values() 
                   if 1 in l.signal_group)
    
    if queue_ns > queue_ew * 1.5:
        intersection.signal = [80, 40]  # More green for N-S
    elif queue_ew > queue_ns * 1.5:
        intersection.signal = [40, 80]  # More green for E-W
```

### Save and Load Scenarios
```python
# Save scenario to file
W.save_scenario("my_scenario.uxsim_scenario")

# Load scenario in new simulation
W2 = World(name="", deltan=5, tmax=7200)
W2.load_scenario("my_scenario.uxsim_scenario")
W2.exec_simulation()
```

### Accessing Simulation State
```python
# Get specific node/link
node = W.get_node("node_name")
link = W.get_link("link_name")

# All nodes and links
for node in W.NODES:
    print(node.name, len(node.incoming_vehicles))

for link in W.LINKS:
    print(link.name, link.num_vehicles, link.speed)

# Running vehicles
for veh_name, veh in W.VEHICLES_RUNNING.items():
    print(veh.name, veh.link.name, veh.x)
```

## Advanced Features

### Import from OpenStreetMap
```python
from uxsim.OSMImporter import OSMImporter

# Import network from OSM
nodes, links = OSMImporter.import_osm_data(
    "path/to/map.osm",
    default_number_of_lanes=2,
    default_free_flow_speed=15
)

# Add to World
W = World(name="osm_network", deltan=5, tmax=3600)
for node_data in nodes:
    W.addNode(**node_data)
for link_data in links:
    W.addLink(**link_data)
```

### Taxi/Shared Mobility
```python
from uxsim.TaxiHandler import TaxiHandler_nearest

# Create handler
handler = TaxiHandler_nearest(W)

# Add taxis (vehicles with no fixed destination)
for i in range(100):
    W.addVehicle("depot", None, 0, mode="taxi")

# Add trip requests
handler.add_trip_request("pickup_node", "dropoff_node", request_time=100)

# Simulation with taxi assignment
while W.check_simulation_ongoing():
    W.exec_simulation(duration_t=60)
    handler.assign_trip_request_to_taxi()

handler.print_stats()
```

### User-Defined Functions
```python
# Custom function called every timestep for a link
def monitor_link(link):
    if link.num_vehicles > 50:
        print(f"Warning: {link.name} is congested!")

W.addLink("monitored_link", "A", "B", length=1000, 
          user_function=monitor_link)
```

### Congestion Pricing
```python
def toll_function(t):
    """Dynamic toll: higher during peak hours"""
    if 7*3600 <= t <= 9*3600:  # 7-9 AM
        return 5.0  # $5 toll
    return 0.0

W.addLink("toll_road", "A", "B", length=2000,
          congestion_pricing=toll_function)
```

## Units and Conventions

- **Distance**: meters (m)
- **Time**: seconds (s)
- **Speed**: meters per second (m/s)
- **Density**: vehicles per meter (veh/m)
- **Flow**: vehicles per second (veh/s)
- **Coordinates (x, y)**: For visualization only, no physical meaning

## Common Parameters Reference

### World
| Parameter | Default | Description |
|-----------|---------|-------------|
| deltan | 5 | Platoon size for mesoscopic simulation |
| tmax | 3600 | Simulation duration (s) |
| print_mode | 1 | Print progress (0/1) |
| save_mode | 1 | Save results (0/1) |
| show_mode | 0 | Display plots (0/1) |
| random_seed | None | Random seed for reproducibility |

### Node
| Parameter | Default | Description |
|-----------|---------|-------------|
| signal | [0] | Signal phases duration list. [0] means no signal |
| signal_offset | 0 | Signal cycle offset (s) |
| flow_capacity | None | Max throughput (veh/s) |

### Link
| Parameter | Default | Description |
|-----------|---------|-------------|
| length | - | Link length (m) |
| free_flow_speed | 20 | Free flow speed (m/s) |
| jam_density | 0.2 | Jam density (veh/m) |
| number_of_lanes | 1 | Number of lanes |
| merge_priority | 1 | Priority at downstream merge |
| signal_group | [0] | Active signal phases |
| capacity_in | None | Inflow capacity (veh/s) |
| capacity_out | None | Outflow capacity (veh/s) |

## Performance Tips

1. **Increase deltan** for faster but less detailed simulation
2. **Use print_mode=0** when running many simulations
3. **Use save_mode=0 and show_mode=0** when not needing visualizations
4. **Avoid creating visualizations during simulation loops** - they are slow

## Typical Workflow

1. Create `World` with appropriate parameters
2. Add `Nodes` (intersections, origins, destinations)
3. Add `Links` (roads connecting nodes)
4. Add `demand` (traffic between OD pairs)
5. Run `exec_simulation()`
6. Analyze with `W.analyzer` methods
7. Export data or create visualizations
