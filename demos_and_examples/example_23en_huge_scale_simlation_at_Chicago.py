from uxsim import *

W = World(
    name="",
    deltan=30, #for huge scale simulation (e.g., 100000+ vehicles or 1000+ links), large deltan is recommended. 
    tmax=10000,
    print_mode=1, save_mode=1, show_mode=1,
    random_seed=42,
)

W.load_scenario("dat/chicago_sketch.uxsim_scenario")

W.exec_simulation()
W.analyzer.print_simple_stats()

W.analyzer.network_fancy(animation_speed_inverse=15, sample_ratio=0.2, interval=5, trace_length=10,  figsize=6, antialiasing=False)