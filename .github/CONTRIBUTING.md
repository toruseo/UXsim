# Contributing Guideline

For minor changes including bug fixes, please submit a pull request (like [this good example](https://github.com/toruseo/UXsim/pull/29)).
If you want a major change, please start a discussion on the [Issues](https://github.com/toruseo/UXsim/issues) or [Discussions](https://github.com/toruseo/UXsim/discussions) pages first.

To be merged, your code must pass [test scripts](https://github.com/toruseo/UXsim/tree/main/tests) that are automatically run by the Github Action.
But feel free to send pull requests that fail the tests. 
The other people may be able to help.

`uxsim.py` is the core of the traffic flow simulation.
It must contain codes that are essential for simulation only.
If you have developed useful utilities, please consider to add them as a submodule like the current `Utilities`.
