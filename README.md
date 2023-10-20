# UXsim: An open source macroscopic and mesoscopic traffic simulator in Python

This repository introduces UXsim, a free, open-source macroscopic and mesoscopic network traffic flow simulator developed in Python. 
It is suitable for simulating large-scale (e.g., city-scale) vehicular transportation.
It computes dynamic traffic flow in a network by using traffic flow models commonly utilized by transportation research.

Simple example is summarized in [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb).

Note that UXsim is a significant expansion of the traffic flow simulator UroborosX, which is included in the book "[Macroscopic Traffic Simulation: Fundamental Mathematical Theory and Python Implementation](https://www.coronasha.co.jp/np/isbn/9784339052794/)" (Author: [Toru Seo](https://toruseo.jp/), Publisher: [Corona Publishing Co., Ltd.](https://www.coronasha.co.jp/)). The basic operating principles remain the same; for details, please refer to the book.

## Main Features

- Dynamic network traffic simulation with a given network and time-dependent OD demand (i.e., dynamic traffic assignment). Specifically, the following models are used jointly:
	- Newell's simplified car-following model (X-Model)
	- Lagrangian Incremental Node Model
	- Dynamic User Optimum-type Route Choice Model (with inertia)
- Implementation of traffic management schemes (e.g., traffic signals, inflow control, route guidance, congestion pricing).
- Basic analysis of simulation results (e.g., trip completion rate, total travel time, delay), and their export to pandas.DataFrame and CSV files.
- Visualization of simulation results (e.g., time-space diagram, MFD, network traffic animation).

## Contents

- `uxsim.py`: UXsim main code
- directory `utils`: Necessary files and modules for UXsim
- directory `dat`: Scenario files
- directory `demos_and_examples`: Tutorials and examples of UXsim
- `README.md` and others: Files for explanations etc. Not necessary for simulation

## Usage

The [Jupyter Notebook Demo](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) summarizes the basic usage and features.

To use, place your Python code or Jupyter Notebook in the same folder as the `uxsim.py` file and the `utils` directory and use `from uxsim import *`. For more details, refer to the examples in the `demos_and_examples` directory.

Formal deployment as a Python module will be addressed in the future.

### Execution Environment

Uses Python version 3. Additionally, please install the following modules in advance:

- NumPy
- Matplotlib
- Pillow
- tqdm
- SciPy
- pandas

## Calculation Example

Approximately 60,000 vehicles pass through a 10km x 10km grid network in 2 hours. The computation time is about 40 seconds on a standard desktop PC. Visualization of link traffic conditions (thicker lines mean more vehicles, darker colors mean slower speeds) and some vehicle trajectories:
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

Vehicle trajectory diagram on a corridor of the above network:

<img src="https://github.com/toruseo/UXsim/blob/images/tsd_traj_links_grid.png" width="600">

## Internal Structure and Calculation Flow

This simulator is purely Python and allows users to flexibly customize it. 
The details are explained by the following documents:

- Overview: [arXiv preprint](https://arxiv.org/abs/2309.17114)
- Specifications of classes and functions: [Technical reference](https://toruseo.jp/UXsim/docs/index.html)
- Fundamental theories: [Japanese textbook](https://www.coronasha.co.jp/np/isbn/9784339052794/)

Below is an overview of the structure and calculation flow.

### Class Diagram
<img src="https://github.com/toruseo/UXsim/blob/images/class_diagram.png" width="500">

### Activity Diagram of the Entire Simulator
<img src="https://github.com/toruseo/UXsim/blob/images/activity_diagram.png" width="600">

### Activity Diagram of a Single Vehicle
<img src="https://github.com/toruseo/UXsim/blob/images/activity_diagram_veh.png" width="400">

## Terms of Use & License

This code is released under the MIT License. You are free to use it as long as the source is acknowledged.

When publishing results obtained from this code, please cite:

- Toru Seo. Macroscopic Traffic Simulation: Fundamental Mathematical Theory and Python Implementation. Corona Publishing Co., Ltd., 2023.
- Toru Seo. UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview. arXiv preprint arXiv: 2309.17114, 2023

## Related Links

- [Toru Seo (Author)](https://toruseo.jp/)
- [Collection of related simulators by Seo](https://toruseo.jp/uxsim/)
- [Book page on Corona Publishing Co., Ltd.](https://www.coronasha.co.jp/np/isbn/9784339052794/)
- [Tokyo Institute of Technology Seo Laboratory](http://seo.cv.ens.titech.ac.jp/)
- [Interactive Traffic Flow Simulator that Runs on a Web Browser](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck_jp.html): Operate the same link traffic flow model used in this simulator interactively, and learn the basics of traffic flow and its simulation.
