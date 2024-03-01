# UXsim: Network traffic flow simulator in pure Python

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

## Main files

- `uxsim` directory: UXsim main package
	- `uxsim/uxsim.py`: UXsim main code
	- `uxsim/utils.py`: UXsim utilities code
 	- `uxsim/utils` directory:  UXsim utilities files
- `demos_and_examples` directory: Tutorials and examples of UXsim
- `dat` directory: Sample scenario files

## Usage

First, install UXsim package using pip:
```
pip install uxsim
```

You can also use `pip` to install the GitHub version:

```
pip install -U -e git+https://github.com/toruseo/uxsim@main#egg=uxsim
```

Or any other (development) branch on this repo or your own fork:

```
pip install -U -e git+https://github.com/YOUR_FORK/uxsim@YOUR_BRANCH#egg=uxsim
```

Then import the module using:
```python
from uxsim import *
```

The [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) summarizes the basic usage and features.
For the further details, please see [demos_and_examples](https://github.com/toruseo/UXsim/tree/main/demos_and_examples) and [UXsim technical documentation](https://toruseo.jp/UXsim/docs/index.html).

## Simulation Example

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
