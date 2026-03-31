# Plan: Coverage for Wrapper & C++ Code

## Current State
- measure-coverage.yml runs Python tests only (test_verification_*, test_other_functions)
- --cov=uxsim covers Python files including uxsim_cpp_wrapper.py
- But C++ mode tests (test_cpp_mode.py) are NOT run → wrapper coverage is near 0
- C++ code (trafficpp/) has no coverage instrumentation

## Tasks

### 1. Python/Wrapper coverage (Alice)
- Add test_cpp_mode.py to the test targets in measure-coverage.yml
- Ensure uxsim_cpp_wrapper.py gets covered

### 2. C++ coverage (Bob)
- Add gcov flags to CMakeLists.txt (conditional, e.g. -DCOVERAGE=ON)
- --coverage / -fprofile-arcs / -ftest-coverage on compile + link
- After tests, run lcov/gcov to generate coverage report
- Upload C++ coverage XML to Codecov alongside Python coverage

### 3. Workflow integration (Carol)
- Update measure-coverage.yml to:
  - Build C++ extension with coverage
  - Install lcov
  - Run both Python and C++ mode tests
  - Collect gcov data → lcov → cobertura XML
  - Upload both coverage reports to Codecov

## Status: IN PROGRESS
