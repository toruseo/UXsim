import pytest
import subprocess
import os

# Directory containing example scripts
examples_dir = 'demos_and_examples'

# Dynamically generate test cases for each example script
def pytest_generate_tests(metafunc):
    # List all .py files in the examples_dir
    example_scripts = [f for f in os.listdir(examples_dir) if f.endswith('.py')]
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
