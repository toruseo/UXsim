"""
This script test Jupyter notebooks that are not tested by `test_notebook_demos.py`. Since it takes long time, this test is excluded from the merging checker.
Implementation is awkward, but it works. 
"""


import pytest
import nbformat
import os
from nbconvert.preprocessors import ExecutePreprocessor
from IPython.display import display


def test_demo_notebook_02en():
    exception_words = [ "ResultGUIViewer", "%matplotlib"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_02en" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

def test_demo_notebook_03en_pytorch():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "%run"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_03en_pytorch" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

def test_demo_notebook_04en_OpenStreetMap():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "osm_tokyo.png"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_04en_OpenStreetMap" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

def test_demo_notebook_06en_taxi_or_shared_mobility():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "osm_tokyo.png"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_06en_taxi_or_shared_mobility" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

def test_demo_notebook_06en_taxi_or_shared_mobility():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "osm_tokyo.png"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_06en_taxi_or_shared_mobility" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")


def test_demo_notebook_08en_chicago():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "df_tntp_flow", "res_tntp", "res_tntp_sec*res_tntp_vol", "tntp parser customized for Chicago-sketch"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_08en_chicago" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")

# def test_demo_notebook_09en_dynamic_traffic_assignment():
#     exception_words = [ "ResultGUIViewer", "%matplotlib"]

#     notebook_dir = "demos_and_examples"
#     for notebook in os.listdir(notebook_dir):
#         if notebook.endswith(".ipynb") and "demo_notebook_09en_dynamic_traffic_assignment" in notebook :
#             full_path = os.path.join(notebook_dir, notebook)
#             with open(full_path, "r", encoding="utf-8") as f:
#                 nb = nbformat.read(f, as_version=4)
            
#             for cell in nb.cells:
#                 if cell.cell_type == "code":
#                     if sum([word in cell.source for word in exception_words]) > 0:
#                         print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
#                         continue
                    
#                     print("Testing:", cell.source[:50])
#                     try:
#                         exec(cell.source)
#                     except Exception as e:
#                         pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")



def test_demo_notebook_10en_traffic_signal_tutorialo():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "df_tntp_flow", "res_tntp", "res_tntp_sec*res_tntp_vol", "tntp parser customized for Chicago-sketch"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_10en_traffic_signal_tutorial" in notebook :
            full_path = os.path.join(notebook_dir, notebook)
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            for cell in nb.cells:
                if cell.cell_type == "code":
                    if sum([word in cell.source for word in exception_words]) > 0:
                        print(f"Skipping cell in {notebook}: {cell.source[:50]}...")
                        continue
                    
                    print("Testing:", cell.source[:50])
                    try:
                        exec(cell.source)
                    except Exception as e:
                        pytest.fail(f"Error in notebook {notebook}, cell:\n{cell.source}\n\nError: {str(e)}")