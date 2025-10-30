"""
This script tests the consistency between wrapper functions and their original in UXsim.
"""

import pytest
import inspect

import uxsim

def arg_compare_wrapper(orig, wrapper):
    sig_original = inspect.signature(orig)
    sig_wrapper = inspect.signature(wrapper)

    params_original = list(sig_original.parameters.values())[2:]
    params_wrapper = list(sig_wrapper.parameters.values())[1:]
    
    assert len(params_original) == len(params_wrapper), "Number of args differ"

    for po, pw in zip(params_original, params_wrapper):
        print("checking:", po, "and", pw, end=" ... ")
        assert po.name == pw.name, f"Arg name mismatch: {po.name} != {pw.name}"
        assert po.default == pw.default, f"Default mismatch for {po.name}"
        assert po.kind == pw.kind, f"Kind mismatch for {po.name}"
        print("OK")
    
    return True

def test_addNode():
    assert arg_compare_wrapper(uxsim.Node.__init__, uxsim.World.addNode)

def test_addLink():
    assert arg_compare_wrapper(uxsim.Link.__init__, uxsim.World.addLink)

def test_defRoute():
    assert arg_compare_wrapper(uxsim.Route.__init__, uxsim.World.defRoute)