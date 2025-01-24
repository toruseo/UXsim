Test scripts. 

For development purposes only. Users can ignore these files, but they may be informative as use cases of UXsim.

`test_examples.py` tests whether the example scripts in `demos_and_examples` can run without errors.

`test_verification_*.py` tests theoretical soundness of UXsim as a traffic flow simulator, namely, whether it can output reasonable solutions of the underlying mathematical models (e.g., KW theory, DUO).

Other `test_*.py` tests other functionalities of UXsim.

If a user would like to run these tests locally for development purposes, please organize the code in the following directory structure:
```
your_project_directory/
├── uxsim/ 	# The uxsim directory
│ ├── uxsim.py 	# The main code of UXsim. 
│ └── ... 	# Other files and directories in uxsim
├── test_spam.py 		# test code
├── ... 	# Other files or test codes if necessary
```
and run `pytest test_spam.py --durations=0 -v`.
Note that some of the tests require optional packages. For the details, please see yml files in https://github.com/toruseo/UXsim/tree/main/.github/workflows.