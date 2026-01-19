Test scripts. 

For development purposes only. Users can ignore these files, but they may be informative as use cases of UXsim.

`test_examples.py` tests whether the example scripts in `demos_and_examples` can run without errors.

`test_verification_*.py` tests theoretical soundness of UXsim as a traffic flow simulator, namely, whether it can output reasonable solutions of the underlying mathematical models (e.g., KW theory, DUO).

Other `test_*.py` tests other functionalities of UXsim.

If a user would like to run these tests locally for development purposes, first install UXsim with the development dependencies:
```bash
pip install .[dev]
```

For tests that require additional optional packages (e.g., `osmnx`, `torch`, `gymnasium`), install the extra dependencies:
```bash
pip install .[dev,extra]
```

For tests that require advanced features (e.g., `neatnet`, `geopandas`), install the advanced dependencies:
```bash
pip install .[dev,advanced,extra]
```

Then run the tests:
```bash
pytest test_spam.py --durations=0 -v
```

These dependency groups are defined in `pyproject.toml`. For more details, please see the yml files in https://github.com/toruseo/UXsim/tree/main/.github/workflows.