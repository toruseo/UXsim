# UXsim: Network traffic flow simulator in pure Python

[![PyPi](https://img.shields.io/pypi/v/uxsim.svg)](https://pypi.python.org/pypi/uxsim)
[![](https://tokei.rs/b1/github/toruseo/UXsim?style=flat&category=code&color=dddd22)](https://github.com/toruseo/UXsim)
[![](https://tokei.rs/b1/github/toruseo/UXsim?category=comments&style=flat&color=44cc44)](https://github.com/toruseo/UXsim/)
[![Demo in Colab](https://colab.research.google.com/assets/colab-badge.svg)](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)
[![arXiv](https://img.shields.io/badge/arXiv-2309.17114-b31b1b.svg)](http://dx.doi.org/10.48550/arXiv.2309.17114)
[![Static Badge](https://img.shields.io/badge/readme-English%20%F0%9F%87%BA%F0%9F%87%B8%20-%20darkblue)](https://github.com/toruseo/UXsim/blob/main/README.md)
[![Static Badge](https://img.shields.io/badge/readme-%E6%97%A5%E6%9C%AC%E8%AA%9E%20%F0%9F%87%AF%F0%9F%87%B5%20-pink)](https://github.com/toruseo/UXsim/blob/main/README.jp.md)

*UXsim* is a free, open-source macroscopic and mesoscopic network traffic flow simulator written in Python.
It simulates the movements of car travelers and traffic congestion in road networks.
It is suitable for simulating large-scale (e.g. city-scale) traffic phenomena.
UXsim would be especially useful for scientific and educational purposes because of its simple, lightweight, and customizable features, but of course users are free to use UXsim for any purpose.

If you are interested in, please see

- [Jupyter Notebook](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) or [Google Colab](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb): Interactive demonstrations and tutorials
- [Technical Documentation](https://toruseo.jp/UXsim/docs/index.html): Detailed documents on tutorials, simulation mechanism, and specifications of modules/functions

## Main Features

- Simple, lightweight, and easy-to-use Python implementation of the modern standard models of dynamic network traffic flow.
- Macroscopic traffic simulation: Simulating over 60000 vehicles in 30 seconds in a city.
- Dynamic traffic assignment: Traffic flow simulation with a given network and time-dependent OD demand.
- Implementation of traffic control/management schemes such as traffic signals and road pricing.
- Basic analysis of simulation results and their export to pandas.DataFrame and CSV files.
- Visualization of simulation results using matplotlib. Interactive GUI is available.
- Flexible and customizable thanks to pure Python implementation. Can also be directly integrated with other Python-based frameworks, such as PyTorch for deep reinforcement learning traffic control.

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
The left (or upper) is no control scenario with fixed signal timing; the traffic demand exceeds the network capacity with naive signal setting, and a gridlock occurs.
The right (or bottom) is with DRL control scenario, where traffic signal can be changed by observing queue length; although the demand level is the same, traffic is smoothly flowing.
[Jupyter Notebook of this example](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_03en_pytorch.ipynb) is available.

<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_nocontrol.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_DQL.gif" width="400"/>
</p>

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
├── uxsim/ 	# The uxsim directory
│ ├── uxsim.py 	# The main code of UXsim. You can customize this as you wish
│ └── ... 	# Other files and directories in uxsim
├── your_simulation_code.py 		# Your code if nessesary
├── your_simulation_notebook.ipynb 	# Your Jupyter notebook if nessesary
├── ... 	# Other files if nessesary
```
In this way, you can flexibly customize UXsim by your own.

</details>

## Usage

Import the module using:
```python
from uxsim import *
```
and then define your simulation scenario.

The [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) summarizes the basic usage and features.
You can also test [Google Colab demo](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb).
For the further details, please see [demos_and_examples](https://github.com/toruseo/UXsim/tree/main/demos_and_examples) and [UXsim technical documentation](https://toruseo.jp/UXsim/docs/index.html).

As a simple example, the following code will simulate traffic flow in a Y-shaped network. 
```python
from uxsim import *

# Define the main simulation
# Units are standardized to seconds (s) and meters (m)
W = World(
    name="",    # Scenario name
    deltan=5,   # Simulation aggregation unit delta n
    tmax=1200,  # Total simulation time (s)
    print_mode=1, save_mode=1, show_mode=0,    # Various options
    random_seed=0    # Set the random seed
)

# Define the scenario
W.addNode("orig1", 0, 0) # Create a node
W.addNode("orig2", 0, 2)
W.addNode("merge", 1, 1)
W.addNode("dest", 2, 1)
W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=0.5) # Create a link
W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2)
W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
W.adddemand("orig1", "dest", 0, 1000, 0.4) # Create OD traffic demand
W.adddemand("orig2", "dest", 500, 1000, 0.6)

# Run the simulation to the end
W.exec_simulation()

# Print summary of simulation result
W.analyzer.print_simple_stats()

# Visualize snapshots of network traffic state for several timesteps
W.analyzer.network(0, detailed=1, network_font_size=0)
W.analyzer.network(500, detailed=1, network_font_size=0)
W.analyzer.network(1000, detailed=1, network_font_size=0)
```

It would output text to the terminal and images to `out` directory like below:
```
simulation setting:
 scenario name:
 simulation duration:    1200 s
 number of vehicles:     700 veh
 total road length:      3000 m
 time discret. width:    5 s
 platoon size:           5 veh
 number of timesteps:    240
 number of platoons:     140
 number of links:        3
 number of nodes:        4
 setup time:             0.00 s
simulating...
      time| # of vehicles| ave speed| computation time
       0 s|        0 vehs|   0.0 m/s|     0.00 s
     600 s|      100 vehs|  17.5 m/s|     0.03 s
    1195 s|       25 vehs|  20.0 m/s|     0.05 s
 simulation finished
results:
 average speed:  13.8 m/s
 number of completed trips:      675 / 700
 average travel time of trips:   142.7 s
 average delay of trips:         42.7 s
 delay ratio:                    0.299
```
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/simple_example_network1_1000.png" width="400"/>
</p>

## Main Files

- `uxsim` directory: UXsim main package
	- `uxsim/uxsim.py`: UXsim main code
	- `uxsim/utils.py`: UXsim utilities code
	- `uxsim/ResultGUIViewer/ResultGUIViewer.py`: GUI for visualizing simulation results
	- `uxsim/OSMImporter/OSMImporter.py`: Road network importer from OpenStreetMap (experimental)
 	- `uxsim/files` directory:  UXsim utilities files
- `demos_and_examples` directory: Tutorials and examples of UXsim
- `dat` directory: Sample scenario files
- `tests`, `.github` directories: Development-related files

## Further Reading

If you want to know the details of UXsim, please see

- [Simple demo in Jupyter Notebook](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) or [Google Colab](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb): Interactive demonstrations
- [UXsim Technical Documentation](https://toruseo.jp/UXsim/docs/index.html): Detailed documents on tutorials, simulation mechanism, and specifications of modules/functions
- [Demos and examples](https://github.com/toruseo/UXsim/tree/main/demos_and_examples): Various examples using Jupyter Notebooks and Python codes
- [arXiv preprint](https://arxiv.org/abs/2309.17114): Scientific overview

## Terms of Use & License

UXsim is released under the MIT License. You are free to use it as long as the source is acknowledged.

When publishing works based on from UXsim, please cite:

- Toru Seo. Macroscopic Traffic Flow Simulation: Fundamental Mathematical Theory and Python Implementation. Corona Publishing Co., Ltd., 2023.
- Toru Seo. UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview. arXiv preprint arXiv: 2309.17114, 2023

## Contributing and Discussion

Contribution is welcome!
For minor changes including bug fixes, please submit a pull request.
Please make sure that your codes pass the automatic tests in Github Action.
If you want a major change, please start a discussion at [Issues](https://github.com/toruseo/UXsim/issues) page first.

If you have any questions or suggestions, please start a discussion at [Issues](https://github.com/toruseo/UXsim/issues) page (in English or Japanese).

I (Toru Seo) work on this project in my spare time. Please understand that my response may be delayed.

## Acknowledgments

UXsim is based on various works in traffic flow theory and related fields. We would like to acknowledge the contributions of the research community in advancing this field.
Especially, UXsim directly uses the following works:

- [Newell's simplified car-following model](https://doi.org/10.1016/S0191-2615(00)00044-8) and its extention [X-model](https://doi.org/10.1016/j.trb.2013.02.008)
- [Incremental Node Model](https://doi.org/10.1016/j.trb.2011.04.001) and its [mesoscopic version](https://ubiquitypress.com/site/chapters/e/10.5334/baw.50/)
- [Dynamic User Optimum](https://doi.org/10.1016/S0191-2615(00)00005-9)-type Route Choice Model

## Related Links

- [Toru Seo (Author)](https://toruseo.jp/)
- [Collection of related simulators by Seo](https://toruseo.jp/uxsim/index_en.html)
- Japanese book "[Macroscopic Traffic Simulation: Fundamental Mathematical Theory and Python Implementation](https://www.coronasha.co.jp/np/isbn/9784339052794/)" (Author: [Toru Seo](https://toruseo.jp/), Publisher: [Corona Publishing Co., Ltd.](https://www.coronasha.co.jp/)): UXsim is a significant expansion of the traffic flow simulator *UroborosX* described in this book.
- [Seo Laboratory, Tokyo Institute of Technology](http://seo.cv.ens.titech.ac.jp/)
- [Interactive Traffic Flow Simulator that Runs on a Web Browser](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck.html): Play with the same link traffic flow model used in this simulator interactively, and learn the basics of traffic flow and its simulation.
