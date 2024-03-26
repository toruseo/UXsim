"""
This script tests ResultGUIViewer.
This is for local test only. I don't know how to test it in Github Actions properly.
"""

import pytest

def test_result_gui_viewer():
    import uxsim as UX
    from uxsim.ResultGUIViewer import ResultGUIViewer

    #simple network
    W = UX.World(
        name="",    # Scenario name
        deltan=5,   # Simulation aggregation unit delta n
        tmax=1200,  # Total simulation time (s)
        print_mode=0, save_mode=0, show_mode=0,    # Various options
        random_seed=None    # Set the random seed
    )

    # Define the scenario
    W.addNode("orig1", 0, 0) # Create a node
    W.addNode("orig2", 0, 2)
    W.addNode("merge", 0.5, 1)
    W.addNode("dest", 1, 1)
    W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=0.5) # Create a link
    W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, jam_density=0.2, merge_priority=2)
    W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, jam_density=0.2)
    W.adddemand("orig1", "dest", 0, 1000, 0.4) # Create OD traffic demand
    W.adddemand("orig2", "dest", 500, 1000, 0.6)


    # Run the simulation to the end
    W.exec_simulation()

    # Print summary of simulation result
    W.analyzer.print_simple_stats()

    app, window = ResultGUIViewer.launch_World_viewer(W, return_app_window=True)

    # Shut down 5 sec later
    from PyQt5.QtCore import QTimer
    import sys
    timer = QTimer()
    timer.timeout.connect(window.close)
    timer.start(1000*5)
    assert True
    with pytest.raises(SystemExit):
        sys.exit(app.exec_())