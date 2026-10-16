"""
pytest configuration for UXsim C++ mode testing.

Usage:
    pytest tests/                  # Run with pure-Python engine (default)
    pytest tests/ --cpp            # Run with C++ engine
"""

import pytest


def pytest_addoption(parser):
    parser.addoption("--cpp", action="store_true", default=False,
                     help="Run tests using the C++ simulation engine (cpp=True)")


@pytest.fixture(autouse=True)
def _patch_world_cpp_default(request):
    """When --cpp is passed, monkey-patch World so that cpp defaults to True."""
    if not request.config.getoption("--cpp"):
        yield
        return

    import uxsim.uxsim as _mod
    _OrigWorld = _mod.World
    _orig_new = _OrigWorld.__new__
    _orig_init = _OrigWorld.__init__

    # Patch __new__: flip cpp default to True
    def _patched_new(cls, *args, cpp=True, **kwargs):
        return _orig_new(cls, *args, cpp=cpp, **kwargs)

    # Patch __init__: flip cpp default to True
    def _patched_init(self, *args, cpp=True, **kwargs):
        return _orig_init(self, *args, cpp=cpp, **kwargs)

    _OrigWorld.__new__ = _patched_new
    _OrigWorld.__init__ = _patched_init

    yield

    # Restore originals
    _OrigWorld.__new__ = _orig_new
    _OrigWorld.__init__ = _orig_init

# Skip tests that failed due to external service issue, especially OSM and Overpass API
_NETWORK_ERROR_NAMES = {
    "ResponseStatusCodeError",     # osmnx: 502/503/504 
    "InsufficientResponseError",   # osmnx: empty
    "ConnectionError",             # requests
    "ReadTimeout",                 # requests
    "ConnectTimeout",              # requests
}

def _find_network_error(exc):
    """Return "name: message" of the first network error in the exception chain of `exc`, or None.
    Notebook tests re-raise a kernel-side error as nbclient's CellExecutionError (and then pytest.fail), which carries the original exception name and message in `ename` and `evalue`.
    """
    seen = set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        if type(exc).__name__ in _NETWORK_ERROR_NAMES:
            return f"{type(exc).__name__}: {exc}"
        if getattr(exc, "ename", None) in _NETWORK_ERROR_NAMES:
            return f"{exc.ename}: {exc.evalue}"
        exc = exc.__cause__ or exc.__context__
    return None

@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    if call.when == "call" and report.failed and call.excinfo is not None:
        found = _find_network_error(call.excinfo.value)
        if found:
            report.outcome = "skipped"
            report.longrepr = (str(item.fspath), item.location[1],
                               f"Skipped: external service unavailable ({found})")