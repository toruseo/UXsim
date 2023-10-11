"""
General utilities for UXsim.
"""

import warnings
import functools
import traceback
import sys

# 汎用関数

def catch_exceptions_and_warn(warning_msg=""):
    """
    A decorator that catches exceptions in the decorated function and raises a warning with the specified message.

    Parameters
    ----------
    warning_msg : str, optional
        A custom message to be displayed along with the exception details. Default is an empty string.

    Returns
    -------
    decorator
        A decorator that wraps the decorated function to catch exceptions and raise warnings.

    Notes
    -----
    When an exception occurs in the decorated function, the decorator will raise a warning that includes:
    - The name of the function.
    - A custom message, if provided.
    - The type of the exception.
    - The exception message.
    - The filename and line number where the decorated function was called.

    This decorator is for inessential functions where random exceptions are expected (especially due to file I/O or something), but you want to avoid crashing the entire program and instead receive a warning. Mainly written by ChatGPT.
    """
    warnings.simplefilter('default', category=UserWarning)
    warnings.formatwarning = lambda message, category, filename, lineno, line: f"{category.__name__}: {message}\n"
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                #warnings.warn(f"{func.__name__}(){warning_msg}: {type(e).__name__}: {str(e)}")
                current_frame = sys._getframe()
                caller_frame = traceback.extract_stack(current_frame, limit=2)[0]
                filename = caller_frame.filename
                lineno = caller_frame.lineno
                warnings.warn(f"{filename}:{lineno}:{func.__name__}(){warning_msg}: {type(e).__name__}: {str(e)}")
        return wrapper
    return decorator

def lange(l):
    """
    Super lazy function for abbreviating range(len(l))
    
    Parameters
    ----
    l : list
    """
    return range(len(l))