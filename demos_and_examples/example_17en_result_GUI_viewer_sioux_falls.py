import uxsim as UX
from uxsim.ResultGUIViewer import ResultGUIViewer

#SF network
W = UX.World(
    name="",
    deltan=5,
    tmax=7200,
    print_mode=1, save_mode=1, show_mode=0,
    random_seed=0
)

W.load_scenario_from_csv("dat/siouxfalls_nodes.csv", "dat/siouxfalls_links.csv", "dat/siouxfalls_demand.csv")

# Run the simulation to the end
W.exec_simulation()

# Print summary of simulation result
W.analyzer.print_simple_stats()

# Launch the GUI viewer
ResultGUIViewer.launch_World_viewer(W)