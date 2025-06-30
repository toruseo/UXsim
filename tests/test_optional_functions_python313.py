"""
This script tests optional functions in UXsim available for Python 3.10 or later (the requirement for UXsim is Python 3.9).
"""

import pytest
from numpy import *
from uxsim import *

@pytest.mark.filterwarnings("ignore::UserWarning")
def test_osm_import2():
    """
    Requires 3.11 or later.
    This test is affected by OSM update.
    """

    from uxsim.OSMImporter2 import OSMImporter2

    W = World(
        name="", 
        deltan=5,
        tmax=3600,
        print_mode=1, save_mode=1, show_mode=1,
        random_seed=0
    )

    # bbox=(139.583, 35.570, 139.881, 35.817)
    # custom_filter='["highway"~"motorway"]' #高速道路

    bbox=(139.638, 35.621, 139.700, 35.600)
    custom_filter='["highway"~"motorway|trunk|primary|secondary|tertiary"]'

    OSMImporter2.osm_network_to_World(W, bbox=bbox, custom_filter=custom_filter)

    W.adddemand_nodes2nodes2(W.NODES, W.NODES, 0, 2000, volume=2.0*len(W.NODES)**2)
    W.exec_simulation()
    W.analyzer.print_simple_stats()

    #W.analyzer.network_average(network_font_size=0)

    assert equal_tolerance(W.analyzer.basic_to_pandas()["average_travel_time"][0], 327, rel_tol=2)