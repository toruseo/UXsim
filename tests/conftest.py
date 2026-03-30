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
