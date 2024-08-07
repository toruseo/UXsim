"""
This script test `demo_notebook_01en.ipynb` and `demo_notebook_01jp.ipynb` can run without error.
Implementation is awkward, but it works. Other jupyter notebook demos are not tested.
"""


import pytest
import nbformat
import os
from nbconvert.preprocessors import ExecutePreprocessor
from IPython.display import display


def test_demo_notebook_01en():
    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_01en" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if "ResultGUIViewer" in cell.source or "%matplotlib" in cell.source:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

def test_demo_notebook_01jp():
    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_01jp" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if "ResultGUIViewer" in cell.source or "%matplotlib" in cell.source:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

