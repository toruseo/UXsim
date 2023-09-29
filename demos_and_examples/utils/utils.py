import warnings
import functools
import traceback
import sys

# 汎用関数

def catch_exceptions_and_warn(warning_msg=""):
    #与えられた関数に対して例外をキャッチし、警告メッセージを表示するデコレータ．mainly written by ChatGPT
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
    """range(len(l))を略すためだけの関数
    
    Parameters
    ----
    l : list
    """
    return range(len(l))