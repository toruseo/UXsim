# UXsim: Network traffic flow simulator in pure Python

[![PyPi](https://img.shields.io/pypi/v/uxsim.svg)](https://pypi.python.org/pypi/uxsim)
[![arXiv](https://img.shields.io/badge/arXiv-2309.17114-b31b1b.svg)](http://dx.doi.org/10.48550/arXiv.2309.17114)
[(日本語の説明書はこちら/Japanese readme is here)](https://github.com/toruseo/UXsim/blob/main/README.jp.md)

*UXsim* is a free, open-source macroscopic and mesoscopic network traffic flow simulator developed in Python.
It is suitable for simulating large-scale (e.g., city-scale) vehicular transportation.
It computes dynamic traffic flow in a network by using traffic flow models commonly utilized by transportation research.
UXsim would be especially useful for scientific and educational purposes because of its simple, lightweight, and customizable features; but of course users are free to use UXsim for any purpose.

- [Simple demo in Jupyter Notebook](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb)
- [Technical documentation](https://toruseo.jp/UXsim/docs/index.html)
- [arXiv preprint on scientific overview](https://arxiv.org/abs/2309.17114)

## Main Features

- Dynamic network traffic simulation with a given network and time-dependent OD demand (i.e., dynamic traffic assignment). Specifically, the following models are used jointly:
	- Newell's simplified car-following model (X-Model)
	- Lagrangian Incremental Node Model
	- Dynamic User Optimum-type Route Choice Model (with inertia)
- Implementation of traffic management schemes (e.g., traffic signals, inflow control, route guidance, congestion pricing).
- Basic analysis of simulation results (e.g., trip completion rate, total travel time, delay), and their export to pandas.DataFrame and CSV files.
- Visualization of simulation results (e.g., time-space diagram, MFD, network traffic animation).
- Can be flexibly customized by users thanks to pure Python implementation.
	- Can also be directly integrated with other Python-based frameworks, such as PyTorch for deep reinforcement learning traffic control.

## Main Files

- `uxsim` directory: UXsim main package
	- `uxsim/uxsim.py`: UXsim main code
	- `uxsim/utils.py`: UXsim utilities code
 	- `uxsim/utils` directory:  UXsim utilities files
- `demos_and_examples` directory: Tutorials and examples of UXsim
- `dat` directory: Sample scenario files

## Install

### Using pip

The simplest way is using pip to install from PyPI.

```
pip install uxsim
```

<details>
<summary>Alternative methods (click to see)</summary>
	
### Using pip with custom configuration

You can also use `pip` to install the GitHub version:

```
pip install -U -e git+https://github.com/toruseo/uxsim@main#egg=uxsim
```

Or any other (development) branch on this repo or your own fork:

```
pip install -U -e git+https://github.com/YOUR_FORK/uxsim@YOUR_BRANCH#egg=uxsim
```

	
### Manual install

Download the `uxsim` directory from this Github repo or [the latest release](https://github.com/toruseo/UXsim/releases/latest/download/uxsim.zip) and place it to your local directory as follows:
```
your_project_directory/
├── uxsim/ # The uxsim directory
│ ├── utils/ # Utility files of UXsim
│ ├── uxsim.py # The main code of UXsim. You can customize this as you wish
│ ├── utils.py # Utility funcsions of UXsim
│ └── ...
├── your_simulation_code01.py # Your code 1
├── your_simulation_code02.py # Your code 2
├── ...
```
In this way, you can flexibly customize UXsim by your own.

</details>

## Usage

Import the module using:
```python
from uxsim import *
```
and then define your simulation scenario.

<details>
<summary>A simplest example (click to see)</summary>
	
```python
from uxsim import *
import pandas as pd

if __name__ == "__main__":

    # Define the main simulation
    # Units are standardized to seconds (s) and meters (m)
    W = World(
        name="",    # Scenario name. Can be blank. Used as the folder name for saving results.
        deltan=5,   # Simulation aggregation unit delta n. Defines how many vehicles are grouped together (i.e., platoon size) for computation. Computation cost is generally inversely proportional to deltan^2.
        tmax=1200,  # Total simulation time (s)
        print_mode=1, save_mode=1, show_mode=0,    # Various options. print_mode determines whether to print information. Usually set to 1, but recommended 0 when running multiple simulations automatically. save_mode determines if visualization results are saved. show_mode determines if visualization results are displayed. It's good to set show_mode=1 on Jupyter Notebook, otherwise recommended 0.
        random_seed=0    # Set the random seed. Specify if you want repeatable experiments. If not, set to None. On Jupyter Notebook, randomness might not always be consistent (requires a fix).
    )

    # Define the scenario
    # Merge network: Example of hard-coded definition
    W.addNode("orig1", 0, 0) # Create a node. Parameters: node name, visualization x-coordinate, visualization y-coordinate
    node_orig2 = W.addNode("orig2", 0, 2) # W.addNode returns the created node instance, so it can be assigned to a variable
    W.addNode("merge", 1, 1)
    W.addNode("dest", 2, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=0.5) # Create a link. Parameters: link name, start node, end node, length, free_flow_speed, jam_density, merge_priority during merging
    W.addLink("link2", node_orig2, "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2) # Nodes can be specified using the variable name instead of the string name
    link3 = W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2) # W.addLink also returns the created link instance
    W.adddemand("orig1", "dest", 0, 1000, 0.4) # Create OD traffic demand. Parameters: origin node, destination node, start time, end time, demand flow rate
    W.adddemand("orig2", "dest", 500, 1000, 0.6)

    # Execute the simulation
    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()
```

This code will simulate traffic flow in a Y-shaped network.
</details>

The [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) summarizes the basic usage and features.
For the further details, please see [demos_and_examples](https://github.com/toruseo/UXsim/tree/main/demos_and_examples) and [UXsim technical documentation](https://toruseo.jp/UXsim/docs/index.html).

## Simulation Examples

### Large-scale scenario

Belows are simulation result where approximately 60000 vehicles pass through a 10km x 10km grid network in 2 hours. The computation time was about 30 seconds on a standard desktop PC.

Visualization of link traffic states (thicker lines mean more vehicles, darker colors mean slower speeds) and some vehicle trajectories:
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

Vehicle trajectory diagram on a corridor of the above network:
<img src="https://github.com/toruseo/UXsim/blob/images/tsd_traj_links_grid.png" width="600">

### Deep reinforcement learning signal control using PyTorch

Traffic signal controller is trained by deep reinforcement learning (DRL) of [PyTorch](https://pytorch.org/).
The left is no control scenario with fixed signal timing; the traffic demand exceeds the network capacity with naive signal setting, and a gridlock occurs.
The right is with DRL control scenario, where traffic signal can be changed by observing queue length; although the demand level is the same, traffic is smoothly flowing.
[Jupyter Notebook of this example](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_03en_pytorch.ipynb) is available.

<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_nocontrol.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_DQL.gif" width="400"/>
</p>


## Future Plans

- day-to-day dynamics
- taxi and shared mobility (i.e., vehicles travel through a network by passing through specific nodes that are dynamically updated)
- network import from OSMnx
- basemap for visualization
- modern packaging

## Terms of Use & License

UXsim is released under the MIT License. You are free to use it as long as the source is acknowledged.

When publishing works based on from UXsim, please cite:

- Toru Seo. Macroscopic Traffic Flow Simulation: Fundamental Mathematical Theory and Python Implementation. Corona Publishing Co., Ltd., 2023.
- Toru Seo. UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview. arXiv preprint arXiv: 2309.17114, 2023

## Related Links

- [Toru Seo (Author)](https://toruseo.jp/)
- [Collection of related simulators by Seo](https://toruseo.jp/uxsim/index_en.html)
- Japanese book "[Macroscopic Traffic Simulation: Fundamental Mathematical Theory and Python Implementation](https://www.coronasha.co.jp/np/isbn/9784339052794/)" (Author: [Toru Seo](https://toruseo.jp/), Publisher: [Corona Publishing Co., Ltd.](https://www.coronasha.co.jp/)): UXsim is a significant expansion of the traffic flow simulator *UroborosX* described in this book.
- [Seo Laboratory, Tokyo Institute of Technology](http://seo.cv.ens.titech.ac.jp/)
- [Interactive Traffic Flow Simulator that Runs on a Web Browser](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck.html): Play with the same link traffic flow model used in this simulator interactively, and learn the basics of traffic flow and its simulation.
