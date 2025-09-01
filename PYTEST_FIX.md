# pytest-xdist Serialization Issue Fix

This document explains the fix for the pytest-xdist serialization issue that was causing CI tests to fail.

## Problem Description

The CI tests were failing with the following error:
```
INTERNALERROR> execnet.gateway_base.DumpError: can't serialize <class '_pytest._code.code.ExceptionInfo'>
```

This error occurred when pytest-xdist tried to serialize test results (particularly exception information) to send them from worker processes to the main process.

## Root Cause

The issue was caused by an incompatibility between the latest versions of:
- pytest (8.4.1)
- pytest-xdist (3.8.0)  
- pytest-rerunfailures (16.0)

When pytest-rerunfailures tries to serialize `ExceptionInfo` objects for communication between pytest-xdist workers, the objects can't be serialized by execnet, causing the entire test run to fail with an internal error.

## Solution

The fix implements version constraints for the pytest ecosystem packages to use compatible versions that work together without serialization issues:

### 1. Version Constraints

Updated the GitHub workflows to use these specific version ranges:
- `pytest>=7.4.0,<8.0.0` - Use stable pytest 7.x series
- `pytest-rerunfailures>=11.1,<12.0` - Use compatible rerunfailures version
- `pytest-xdist>=3.3.0,<4.0.0` - Use stable xdist 3.x series

### 2. pytest Configuration

Added `pytest.ini` with optimized settings:
- Registered the `flaky` marker to avoid warnings
- Short tracebacks (`--tb=short`) to reduce serialization complexity
- Strict markers to catch configuration issues early

### 3. Validation Script

Created `test_pytest_compatibility.py` to validate that the fix works correctly and detect any future regressions.

## Benefits of this Solution

1. **Re-enables parallel testing** - Tests can run with `-n auto` again for better CI performance
2. **Preserves retry functionality** - Flaky tests still get automatic retries via `@pytest.mark.flaky(reruns=X)`
3. **Stable and reliable** - Uses proven version combinations that are known to work together
4. **Future-proof** - Version constraints prevent automatic upgrades that could break compatibility

## Alternative Solutions Considered

1. **Remove pytest-rerunfailures** - Would lose retry functionality for flaky tests
2. **Remove pytest-xdist** - Would lose parallel testing performance benefits
3. **Custom serialization workaround** - Would be complex and fragile

The version constraint approach is the most robust solution that maintains all functionality while fixing the underlying compatibility issue.