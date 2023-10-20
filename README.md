# UXsim: Network traffic flow simulator in pure Python

[(日本語の説明書はこちら)](https://github.com/toruseo/UXsim/blob/main/README.jp.md)

This repository introduces *UXsim*, a free, open-source macroscopic and mesoscopic network traffic flow simulator developed in Python. 
It is suitable for simulating large-scale (e.g., city-scale) vehicular transportation.
It computes dynamic traffic flow in a network by using traffic flow models commonly utilized by transportation research.
UXsim would be especially useful for scientific and educational purposes because of its simple, lightweight, and customizable features.

Simple example is summarized in [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb).


## Main Features

- Dynamic network traffic simulation with a given network and time-dependent OD demand (i.e., dynamic traffic assignment). Specifically, the following models are used jointly:
	- Newell's simplified car-following model (X-Model)
	- Lagrangian Incremental Node Model
	- Dynamic User Optimum-type Route Choice Model (with inertia)
- Implementation of traffic management schemes (e.g., traffic signals, inflow control, route guidance, congestion pricing).
- Basic analysis of simulation results (e.g., trip completion rate, total travel time, delay), and their export to pandas.DataFrame and CSV files.
- Visualization of simulation results (e.g., time-space diagram, MFD, network traffic animation).
- Can be flexibly customized by users thanks to pure Python implementation.

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
or download the `uxsim` directory from this Github repo and place it to your local directory. 
The former is required if you want to customize UXsim’s internal logic.

Then import the module using:
```python
from uxsim import *
```

The [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) summarizes the basic usage and features.
For the further details, please see [UXsim technical documentation](https://toruseo.jp/UXsim/docs/index.html).

## Simulation Example

Approximately 60,000 vehicles pass through a 10km x 10km grid network in 2 hours. The computation time was about 30 seconds on a standard desktop PC. Visualization of link traffic states (thicker lines mean more vehicles, darker colors mean slower speeds) and some vehicle trajectories:
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

Vehicle trajectory diagram on a corridor of the above network:

<img src="https://github.com/toruseo/UXsim/blob/images/tsd_traj_links_grid.png" width="600">

## Detailed documents

- [UXsim technical documentation](https://toruseo.jp/UXsim/docs/index.html)
- [arXiv preprint on overview](https://arxiv.org/abs/2309.17114)
- [Japanese textbook on fundamental theory](https://www.coronasha.co.jp/np/isbn/9784339052794/)

## Terms of Use & License

This code is released under the MIT License. You are free to use it as long as the source is acknowledged.

When publishing results obtained from this code, please cite:

- Toru Seo. Macroscopic Traffic Simulation: Fundamental Mathematical Theory and Python Implementation. Corona Publishing Co., Ltd., 2023.
- Toru Seo. UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview. arXiv preprint arXiv: 2309.17114, 2023

## Related Links

- [Toru Seo (Author)](https://toruseo.jp/)
- [Collection of related simulators by Seo](https://toruseo.jp/uxsim/)
- Japanese book "[Macroscopic Traffic Simulation: Fundamental Mathematical Theory and Python Implementation](https://www.coronasha.co.jp/np/isbn/9784339052794/)" (Author: [Toru Seo](https://toruseo.jp/), Publisher: [Corona Publishing Co., Ltd.](https://www.coronasha.co.jp/))
	- UXsim is a significant expansion of the traffic flow simulator *UroborosX* described in this book.
- [Tokyo Institute of Technology Seo Laboratory](http://seo.cv.ens.titech.ac.jp/)
- [Interactive Traffic Flow Simulator that Runs on a Web Browser](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck_jp.html): Operate the same link traffic flow model used in this simulator interactively, and learn the basics of traffic flow and its simulation.
