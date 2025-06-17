import streamlit as st
from uxsim import *
import matplotlib.pyplot as plt

st.set_page_config(page_title="Coordinated Signal Control", layout="wide")
st.title("Coordinated Signal Control")
st.write(
"""
This is an interactive simulation that shows how traffic conditions change depending on signal control parameters.
Traffic demand flows from one end of a single road to the other.
There is no entry or exit along the way.
The traffic flow simulator [UXsim](https://github.com/toruseo/UXsim) is used.
""")

# Simulation model (accepts parameters)
class UXsim_model:
    def __init__(self, demand_flow=0.3, green_split=0.5, offset1=0, offset2=0, offset3=0):
        self.demand_flow = demand_flow
        self.green_split = green_split
        self.offsets = [0, offset1, offset2, offset3, 0]
        self.n_nodes = 5
        self.cycle = 120
        self.W = None
        
    def create_world(self):
        self.W = World(
            name="signal_series",
            deltan=5,
            tmax=1500,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=0
        )
        
        # Create nodes
        for i in range(self.n_nodes):
            if i != 0 and i != self.n_nodes - 1:
                if i == 2:
                    signal = [self.cycle * self.green_split, self.cycle * (1 - self.green_split)]
                else:
                    signal = [self.cycle * 0.5, self.cycle * 0.5]
            else:
                signal = [0]
            self.W.addNode(f"node{i}", 0, i, signal=signal, signal_offset=self.offsets[i])
        
        # Create links
        for i in range(self.n_nodes - 1):
            j = i + 1
            self.W.addLink(f"link{i}-{j}", f"node{i}", f"node{j}", length=300, free_flow_speed=10, signal_group=0)
            self.W.addLink(f"link{j}-{i}", f"node{j}", f"node{i}", length=300, free_flow_speed=10, signal_group=0)
        
        # Add demand
        self.W.adddemand(orig="node0", dest=f"node{self.n_nodes - 1}", t_start=0, t_end=1000, flow=self.demand_flow, attribute="group1")
    
    def show_network(self):
        """Display network diagram"""
        if self.W is None:
            self.create_world()
        self.W.show_network(show_id=0, figsize=(3,3), network_font_size=6)
        return plt.gcf()
    
    def run_simulation(self):
        """Run simulation"""
        if self.W is None:
            self.create_world()
        self.W.exec_simulation()
    
    def get_stats(self):
        """Get statistics"""
        if len(self.W.VEHICLES) > 0:
            avg_travel_time = self.W.analyzer.total_travel_time / len(self.W.VEHICLES)/self.W.DELTAN
        else:
            avg_travel_time = 0
        
        return {
            'total_travel_time': self.W.analyzer.total_travel_time,
            'avg_travel_time': avg_travel_time
        }
    
    def show_time_space_diagram(self):
        """Display time-space trajectory diagram"""
        self.W.analyzer.time_space_diagram_traj_links([f"link{i}-{i + 1}" for i in range(self.n_nodes - 1)])
        return plt.gcf()

# Initial simulation for display (default parameters)
initial_sim = UXsim_model()

# Road network structure
st.subheader("Road Network Structure")
fig1 = initial_sim.show_network()
st.pyplot(fig1, use_container_width=False)

# Sliders
st.subheader("Scenario Parameter Settings")
demand_flow = st.slider("Traffic Demand", min_value=0.0, max_value=1.0, value=0.3, step=0.05)
green_split = st.slider("Green Time Ratio at Signal 2", min_value=0.0, max_value=1.0, value=0.5, step=0.05)
o3 = st.slider("Offset at Signal 3", min_value=0, max_value=120, value=0, step=5)
o2 = st.slider("Offset at Signal 2", min_value=0, max_value=120, value=0, step=5)
o1 = st.slider("Offset at Signal 1", min_value=0, max_value=120, value=0, step=5)

# Run simulation with parameters
sim = UXsim_model(demand_flow, green_split, o1, o2, o3)
sim.run_simulation()

# Show statistics
st.subheader("Statistics")
stats = sim.get_stats()
st.write("Total Travel Time:", f"{stats['total_travel_time']/60:.1f}", "vehicle-minutes")
st.write("Average Travel Time:", f"{stats['avg_travel_time']:.1f}", "seconds")

# Time-space diagram
st.subheader("Time-Space Trajectory Diagram")
fig2 = sim.show_time_space_diagram()
st.pyplot(fig2)
