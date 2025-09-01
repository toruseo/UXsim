# UXsim: Network Traffic Flow Simulator

UXsim is a pure Python macroscopic and mesoscopic network traffic flow simulator for large-scale (city-scale) traffic phenomena simulation. It is designed for scientific and educational purposes with simple, lightweight, and customizable features.

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

## Working Effectively

### Bootstrap and Setup
- Install Python 3.9 or higher (Python 3.12+ recommended)
- Bootstrap the repository:
  ```bash
  cd /home/runner/work/UXsim/UXsim
  python -m pip install --upgrade pip
  pip install .
  ```
  - **NEVER CANCEL**: Installation takes 2-4 minutes. Set timeout to 300+ seconds.

### Install Test Dependencies
- For running tests, install additional dependencies:
  ```bash
  pip install pytest pytest-rerunfailures pytest-xdist setuptools
  ```
- For advanced examples (optional):
  ```bash
  pip install osmnx torch streamlit gymnasium deap geopandas shapely
  ```

### Quick Validation
- Verify installation works:
  ```bash
  python -c "import uxsim; print('UXsim import successful')"
  ```
- Run simple example to validate basic functionality (1-2 seconds):
  ```bash
  python demos_and_examples/example_00en_simple.py
  ```

## Testing and Validation

### Example Tests
- **NEVER CANCEL**: Full example test suite takes 12+ minutes. Set timeout to 20+ minutes:
  ```bash
  pytest tests/test_examples.py --durations=10 -v
  ```
- Test specific examples (faster, 1-60 seconds each):
  ```bash
  pytest tests/test_examples.py -v -k "example_00en_simple"
  pytest tests/test_examples.py -v -k "example_01en_basic" 
  ```

### Verification Tests  
- **NEVER CANCEL**: Each verification test file takes 30-120 seconds. Set timeout to 300+ seconds:
  ```bash
  pytest tests/test_verification_straight_road.py -v --durations=5
  pytest tests/test_verification_sioux_falls.py -v --durations=5
  pytest tests/test_verification_exceptional.py -v --durations=5
  ```

### Function Tests
- Test core functionality (30-60 seconds):
  ```bash
  pytest tests/test_other_functions.py -v --durations=5
  ```

### Parallel Testing
- Use parallel testing to speed up test suites:
  ```bash
  pytest -n auto tests/test_examples.py --durations=0 -v
  ```

## Manual Validation Scenarios

### Basic Simulation Scenario
Always validate changes by running a complete simulation scenario:

1. **Simple Linear Network**:
   ```bash
   python demos_and_examples/example_00en_simple.py
   ```
   - Expected: Completes in 1-2 seconds
   - Validates: Basic simulation, vehicle movement, statistics output

2. **Basic Network with Visualization**:
   ```bash
   python demos_and_examples/example_01en_basic.py
   ```
   - Expected: Completes in 15-20 seconds 
   - Validates: Network creation, demand generation, simulation, analysis, visualization

3. **Large-Scale Scenario**:
   ```bash
   python demos_and_examples/example_23en_huge_scale_simlation_at_Chicago.py
   ```
   - **NEVER CANCEL**: Takes 40-60 seconds. Set timeout to 120+ seconds.
   - Validates: Large network handling, performance optimization

### Interactive Testing
For changes to analysis or visualization:
```bash
python -c "
from uxsim import *
W = World(name='test', deltan=5, tmax=600, print_mode=1, save_mode=0, show_mode=0)
W.addNode('orig', 0, 0)
W.addNode('dest', 1000, 0) 
W.addLink('link1', 'orig', 'dest', length=1000, free_flow_speed=20)
W.adddemand('orig', 'dest', 0, 300, 0.5)
W.exec_simulation()
W.analyzer.print_simple_stats()
print('Manual validation successful')
"
```

## Build Timings and Expectations

### Installation
- `pip install .`: 2-4 minutes
- `pip install pytest pytest-xdist`: 1-2 minutes  
- `pip install osmnx torch streamlit`: 3-5 minutes (optional advanced dependencies)

### Test Suite Timings
- **Simple examples** (example_00en, example_01en): 1-20 seconds each
- **Medium examples** (bottleneck, signal control): 20-60 seconds each  
- **Complex examples** (gridlock, large networks): 60-120 seconds each
- **Full example test suite**: 12+ minutes total
- **Verification tests**: 30-120 seconds per test file
- **Function tests**: 30-60 seconds total

### Individual Test Warnings
- **NEVER CANCEL** any test taking longer than expected
- Tests involving large networks or complex scenarios may take several minutes
- Chicago large-scale example specifically takes 40-60 seconds

## Common Issues and Dependencies

### Optional Dependencies
Some examples require additional packages:

- **OSM Import Examples**: 
  ```bash
  pip install osmnx geopandas shapely
  ```
  - `example_16en_import_from_OpenStreetMap.py` fails without osmnx

- **Deep Learning Examples**:
  ```bash  
  pip install torch gymnasium deap
  ```
  - `example_12en_*_pytorch*.py` and `example_14en_*_pytorch.py` need pytorch

- **Interactive Examples**:
  ```bash
  pip install streamlit
  ```
  - `example_27en_interactive_simulation_by_streamlit.py` needs streamlit

### GUI Limitations
- GUI viewer examples (`example_17en_result_GUI_viewer_*.py`, `example_18en_result_GUI_viewer_*.py`) require X11 display
- These examples are not testable in headless GitHub Actions environment
- They are excluded from automated tests but can be run locally with display

### Test Exclusions
The following examples are excluded from automated testing:
- `example_14en_*_Deep_Reinforcement_Learning_pytorch.py` (takes too much time)
- `example_17en_result_GUI_viewer_*.py` (requires GUI)
- `example_18en_result_GUI_viewer_*.py` (requires GUI)

## Key Project Structure

### Core Files
```
uxsim/
├── __init__.py           # Package initialization, version info
├── uxsim.py             # Main simulation engine (~2300 lines)
├── analyzer.py          # Results analysis and visualization  
├── utils.py             # Utility functions
├── scenario_reader_writer.py  # I/O for scenarios
└── [subdirectories]/    # Specialized modules (OSM, GUI, etc.)
```

### Important Directories
```
demos_and_examples/      # 40+ example scripts and Jupyter notebooks
├── demo_notebook_*.ipynb    # Interactive tutorial notebooks
├── example_00en_simple.py   # Simplest example (start here)  
├── example_01en_basic.py    # Basic example with visualization
├── example_*en_*.py         # English examples by topic
└── example_*jp_*.py         # Japanese examples

tests/                   # Comprehensive test suite
├── test_examples.py          # Tests all example scripts 
├── test_verification_*.py   # Mathematical model verification
└── test_other_functions.py  # Core functionality tests
```

### Configuration Files
- `pyproject.toml`: Package configuration, dependencies, build settings
- `.github/workflows/`: GitHub Actions CI/CD pipelines
- `MANIFEST.in`: Additional files to include in package distribution

## Validation Checklist

When making changes to UXsim, always run:

1. **Quick validation** (30 seconds):
   ```bash
   python -c "import uxsim; print('Import OK')"
   python demos_and_examples/example_00en_simple.py
   ```

2. **Core functionality** (2 minutes):  
   ```bash
   pytest tests/test_other_functions.py -v
   ```

3. **Example validation** (12+ minutes - **NEVER CANCEL**):
   ```bash
   pytest tests/test_examples.py -v --durations=10
   ```

4. **Mathematical verification** (5+ minutes):
   ```bash
   pytest tests/test_verification_straight_road.py -v
   pytest tests/test_verification_sioux_falls.py -v  
   ```

## Development Notes

### Code Organization
- `uxsim.py` contains the core simulation engine and must only contain essential simulation code
- Additional utilities should be added as submodules (like existing `Utilities/`, `OSMImporter/`, etc.)
- Keep the main simulation code lightweight and focused

### Performance Expectations  
- UXsim can simulate 60,000+ vehicles in city-scale networks in 30 seconds
- Large metropolitan networks (1M+ vehicles) simulate in 40+ seconds
- Performance scales roughly with vehicle count and network complexity

### Memory Requirements
- Typical simulations require minimal memory
- Large-scale simulations (Chicago example) may use several hundred MB
- Memory usage scales with vehicle count and simulation duration

Always follow these instructions to ensure consistent, reliable development and testing of UXsim changes.