# UXsim: Traffic flow simulator in pure Python

[![PyPi](https://img.shields.io/pypi/v/uxsim.svg)](https://pypi.python.org/pypi/uxsim)
[![Conda Version](https://img.shields.io/conda/vn/conda-forge/uxsim.svg)](https://anaconda.org/conda-forge/uxsim)<!-- ![PyPI - Python Version](https://img.shields.io/pypi/pyversions/uxsim)-->
[![Demo in Colab](https://colab.research.google.com/assets/colab-badge.svg)](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)
[![codecov](https://codecov.io/gh/toruseo/UXsim/graph/badge.svg?token=DK77Y1Y5AT)](https://codecov.io/gh/toruseo/UXsim)
![PyPI - Downloads](https://img.shields.io/pypi/dm/uxsim)
[![arXiv](https://img.shields.io/badge/arXiv-2309.17114-b31b1b.svg)](http://dx.doi.org/10.48550/arXiv.2309.17114)
[![DOI](https://joss.theoj.org/papers/10.21105/joss.07617/status.svg)](https://doi.org/10.21105/joss.07617)
[![Static Badge](https://img.shields.io/badge/readme-%E6%97%A5%E6%9C%AC%E8%AA%9E%20%F0%9F%87%AF%F0%9F%87%B5%20-pink)](https://github.com/toruseo/UXsim/blob/main/README.jp.md)
[![Static Badge](https://img.shields.io/badge/docs-%F0%9F%93%96-lightblue)](https://toruseo.jp/UXsim/docs/)

*UXsim* is a lightweight traffic flow simulator in pure Python.
It enables users to model various traffic scenarios, from small toy networks to large metropolitan areas, without external dependencies.
Especially ideal for academic research and educational use.

## Quick Start
### Install

For Python 3.10 or later

```bash
pip install uxsim
```

### Sample code

```python
from uxsim import World

# Units are standardized to seconds (s) and meters (m)
W = World(name="simple", tmax=2000)

W.addNode("start", x=0, y=0)
W.addNode("bottleneck", x=5000, y=0, flow_capacity=0.4)
W.addNode("goal", x=7500, y=0)

W.addLink("road1", start_node="start", end_node="bottleneck", length=5000)
W.addLink("road2", start_node="bottleneck", end_node="goal", length=2500)

W.adddemand(orig="start", dest="goal", t_start=0, t_end=600, flow=0.8)

W.exec_simulation()

W.analyzer.network_fancy()
```

### Result

Traffic congestion caused by the bottleneck

![anim_network_fancy](https://github.com/toruseo/UXsim/blob/images/anim_network_fancy_simple.gif)

## Simulation Examples

### Large-scale scenario

Approximately 60000 vehicles pass through a 10km x 10km grid network in 2 hours. The computation time was about 30 seconds on a standard desktop PC.
In the first animation, thicker lines mean more vehicles and darker colors mean slower speeds.

<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

<details>
<summary>Code (click to see)</summary>

```python
from uxsim import World

# Define the main simulation
# Units are standardized to seconds (s) and meters (m)
W = World(
    name="grid",
    deltan=5,
    tmax=7200,
    print_mode=1, save_mode=1, show_mode=0,
    random_seed=0,
)

# Scenario
# automated network generation, deploy nodes as an imax x jmax grid
imax = 11
jmax = 11
nodes = {}
for i in range(imax):
    for j in range(jmax):
        nodes[i,j] = W.addNode(f"n{(i,j)}", i, j)

# create links between neighborhood nodes
links = {}
for i in range(imax):
    for j in range(jmax):
        if i != imax-1:
            links[i,j,i+1,j] = W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=20, jam_density=0.2)
        if i != 0:
            links[i,j,i-1,j] = W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=20, jam_density=0.2)
        if j != jmax-1:
            links[i,j,i,j+1] = W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=20, jam_density=0.2)
        if j != 0:
            links[i,j,i,j-1] = W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=20, jam_density=0.2)

# generate traffic demand between the boundary nodes
demand_flow = 0.035
demand_duration = 3600
for n1 in [(0,j) for j in range(jmax)]:
    for n2 in [(imax-1,j) for j in range(jmax)]:
        W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
        W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
for n1 in [(i,0) for i in range(imax)]:
    for n2 in [(i,jmax-1) for i in range(imax)]:
        W.adddemand(nodes[n2], nodes[n1], 0, demand_duration, demand_flow)
        W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

# execute simulation
W.exec_simulation()

# result
W.analyzer.print_simple_stats()
W.analyzer.network_anim(animation_speed_inverse=15, detailed=0, network_font_size=0)
W.analyzer.network_fancy()
```
</details>

### AI-based traffic signal control using PyTorch

A traffic signal controller is trained by AI (deep reinforcement learning) using [PyTorch](https://pytorch.org/).
The first result shows no control with fixed signal timing; a gridlock occurs.
The second result shows AI-based control; although the demand level is the same, traffic flows smoothly.
A [Jupyter Notebook of this example](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_03en_pytorch.ipynb) is available.

<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_nocontrol.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_DQL.gif" width="400"/>
</p>


## Main Features

- Lightweight, dependency-free, and fast mesoscopic simulation
- Simulate large-scale traffic networks with 1 million vehicles in a minute
- Based on theoretically valid models used in transportation research
- Apply traffic control mechanisms such as signals, tolls, and shared mobility
- Fully written in Python (core code ~1200 lines), easy to read and modify
- Supports approximate solvers for Dynamic User Equilibrium and Dynamic System Optimum
- Built-in analysis tool using `matplotlib` and `pandas`

## Further Reading

To learn more about UXsim, please see:

- [UXsim Technical Documentation](https://toruseo.jp/UXsim/docs/index.html): Detailed documents on tutorials, simulation mechanism, and specifications of modules/functions
- [Simple demo in Jupyter Notebook](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01en.ipynb) or [Google Colab](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb): Interactive demonstrations
- [Demos and examples](https://github.com/toruseo/UXsim/tree/main/demos_and_examples): Various examples using Jupyter Notebooks and Python codes
- [Scientific overview](https://doi.org/10.21105/joss.07617): Peer-reviewed article in Journal of Open Source Software

## Terms of Use & License

UXsim is released under the MIT License. You are free to use it as long as the source is acknowledged.

When publishing works based on UXsim, please cite:
- Toru Seo. [UXsim: lightweight mesoscopic traffic flow simulator in pure Python](https://doi.org/10.21105/joss.07617). Journal of Open Source Software, Vol. 10, No. 106, p. 7617, 2025.

```bibtex
@Article{seo2025uxsim,
  author    = {Toru Seo},
  journal   = {Journal of Open Source Software},
  title     = {{UXsim}: lightweight mesoscopic traffic flow simulator in pure {Python}},
  year      = {2025},
  number    = {106},
  pages     = {7617},
  volume    = {10},
  doi       = {10.21105/joss.07617},
  publisher = {The Open Journal},
  url       = {https://doi.org/10.21105/joss.07617},
}
```
Works using UXsim are summarized on the [Github Wiki page](https://github.com/toruseo/UXsim/wiki). Please feel free to edit.

## Contributing and Discussion

Contributions are welcome!
Please see the [Contributing Guideline](https://github.com/toruseo/UXsim/blob/main/.github/CONTRIBUTING.md).

If you have any questions or suggestions, please post them to the [Issues](https://github.com/toruseo/UXsim/issues) or [Discussions](https://github.com/toruseo/UXsim/discussions) (in English or Japanese).

## Author

[Toru Seo, Institute of Science Tokyo](https://toruseo.jp/index.html)
