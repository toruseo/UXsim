"""
This script tests whether the example scripts can run without errors.
"""

import pytest
import subprocess
import os

# Directory containing example scripts
examples_dir = 'demos_and_examples'
# Ignore list
nontest_files = [
    "example_14en_multiple_signals_Deep_Reinforcement_Learning_pytorch.py", #takes too much time. DQN can be tested by example_12en_* as well.
    "example_17en_result_GUI_viewer_sioux_falls.py", #GUI viewer is not testable in Github Actions
    "example_18en_result_GUI_viewer_grid.py", #GUI viewer is not testable in Github Actions
]

# Dynamically generate test cases for each example script
def pytest_generate_tests(metafunc):
    # List all .py files in the examples_dir
    example_scripts = [f for f in os.listdir(examples_dir) if f.endswith('.py') and not f in nontest_files]
    # If the test function expects an "example_script" argument, parametrize it
    if 'example_script' in metafunc.fixturenames:
        metafunc.parametrize('example_script', example_scripts)

def test_example_runs(example_script):
    """Test that a Python example script runs successfully."""
    # Build the script path
    script_path = os.path.join(examples_dir, example_script)
    # Run the script as a separate process
    result = subprocess.run(['python', script_path], capture_output=True, text=True)
    # Assert that the script ran successfully
    assert result.returncode == 0, f"Script {example_script} failed with output:\n{result.stdout}\n{result.stderr}"
