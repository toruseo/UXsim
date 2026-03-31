# Plan: Address Copilot Review Comments on PR #279

## Comment Triage

### Address (code fixes needed):
- #2: wrapper:817 - duo_noise not passed to C++ (0 fixed) → Bob
- #3: wrapper:277 - capacity_out_remain setter doesn't propagate → Bob
- #4: wrapper:801 - vehicle_logging_timestep_interval warning → Bob
- #5: utils.h:122 - contains() std::end inconsistency → Alice
- #6: wrapper:808 - seed range 2**8 too small → Bob
- #7: wrapper:178 - jam_density vs kappa inconsistency → Bob
- #8: wrapper:1006 - ADJ_MAT O(N*L) optimization → Bob
- #11: wrapper:754 - mutable default meta_data → Bob
- #12: wrapper:277 - capacity_in_remain setter doesn't propagate → Bob
- #13: wrapper:1285 - copy()/save() should raise NotImplementedError → Bob
- #14: test_cpp_build.py:27 - eq_tol print noise → Alice
- #16: utils.h:8 - missing #include <map> → Alice

### Evaluate (may not need changes):
- #1: uxsim.py:1629 - mutable default meta_data (uxsim.py is not ours to edit)
- #9: pyproject.toml:71 - C++ build as hard requirement
- #10: uxsim.py:2740 - os.makedirs without os import (check if valid)
- #15: pyproject.toml:6 - build backend change concern

## Assignment
- **Alice (%2)**: #5, #14, #16 (utils.h + test_cpp_build.py)
- **Bob (%1)**: #2, #3, #4, #6, #7, #8, #11, #12, #13 (uxsim_cpp_wrapper.py)
- **Carol (%3)**: #1, #9, #10, #15 evaluation → report back

## Status: IN PROGRESS
