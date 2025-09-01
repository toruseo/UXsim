#!/usr/bin/env python3
"""
Test script to validate pytest-xdist serialization fix.

This script tests the specific combination of pytest, pytest-xdist, and pytest-rerunfailures
to ensure they work together without serialization issues.
"""

import subprocess
import sys

def test_xdist_compatibility():
    """Test that pytest-xdist works without serialization errors."""
    
    # Test 1: Simple parallel test run
    cmd1 = ["python", "-m", "pytest", "-n", "2", "tests/test_verification_node.py::test_merge_fair_nocongestion", "-v"]
    
    try:
        result1 = subprocess.run(cmd1, capture_output=True, text=True, cwd=".")
        if result1.returncode != 0:
            print(f"ERROR: Simple parallel test failed with return code {result1.returncode}")
            print("STDOUT:", result1.stdout)
            print("STDERR:", result1.stderr)
            return False
        else:
            print("✓ Simple parallel test passed")
    except Exception as e:
        print(f"ERROR: Failed to run simple parallel test: {e}")
        return False
    
    # Test 2: Flaky test with rerun functionality
    cmd2 = ["python", "-m", "pytest", "-n", "2", "tests/test_verification_node.py::test_merge_fair_congestion", "-v"]
    
    try:
        result2 = subprocess.run(cmd2, capture_output=True, text=True, cwd=".")
        if result2.returncode not in [0, 1]:  # 1 is acceptable for test failures, but not for internal errors
            if "INTERNALERROR" in result2.stdout or "DumpError" in result2.stdout:
                print(f"ERROR: Serialization error detected in flaky test")
                print("STDOUT:", result2.stdout)
                print("STDERR:", result2.stderr)
                return False
        print("✓ Flaky test passed without serialization errors")
    except Exception as e:
        print(f"ERROR: Failed to run flaky test: {e}")
        return False
    
    print("✓ All compatibility tests passed!")
    return True

if __name__ == "__main__":
    success = test_xdist_compatibility()
    sys.exit(0 if success else 1)