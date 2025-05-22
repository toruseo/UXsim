"""
This script test Jupyter notebooks that are not tested by `test_notebook_demos.py`. Since it takes long time, this test is excluded from the merging checker.
Implementation is awkward, but it works. 
"""


import pytest
import nbformat
import os
from nbconvert.preprocessors import ExecutePreprocessor
from nbconvert.preprocessors import CellExecutionError

from IPython.display import display
import re


class CellExecuter(ExecutePreprocessor):
    """特定のキーワードを含むセルをスキップするプリプロセッサ"""
    
    def __init__(self, exception_words=None, **kwargs):
        super().__init__(**kwargs)
        self.exception_words = exception_words or []
    
    def preprocess_cell(self, cell, resources, cell_index):
        """セルを処理する前に、スキップすべきかどうかをチェック"""
        if cell.cell_type == "code":
            if any(word in cell.source for word in self.exception_words):
                print(f"Skipping cell {cell_index}: {cell.source[:50]}...")
                return cell, resources
        
        return super().preprocess_cell(cell, resources, cell_index)
    

def test_demo_notebook_02en():
    # スキップするワードのリスト
    exception_words = ["ResultGUIViewer", "%matplotlib"]
    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_02en" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=1800, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")

def test_demo_notebook_03en_pytorch():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "%run"]
    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_03en_pytorch" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=1800, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")

def test_demo_notebook_04en_OpenStreetMap():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "osm_tokyo.png"]

    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_04en_OpenStreetMap" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=1800, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")

def test_demo_notebook_06en_taxi_or_shared_mobility():
    exception_words = [ "ResultGUIViewer", "%matplotlib"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_06en_taxi_or_shared_mobility" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=1800, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")


def test_demo_notebook_08en_chicago():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "df_tntp_flow", "res_tntp", "res_tntp_sec*res_tntp_vol", "tntp parser customized for Chicago-sketch"]

    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_08en_chicago" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=1800, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")

def test_demo_notebook_09en_dynamic_traffic_assignment():
    exception_words = [ "ResultGUIViewer", "%matplotlib"]

    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_09en_dynamic_traffic_assignment" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=7200, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")



def test_demo_notebook_10en_traffic_signal_tutorial():
    exception_words = [ "ResultGUIViewer", "%matplotlib", "df_tntp_flow", "res_tntp", "res_tntp_sec*res_tntp_vol", "tntp parser customized for Chicago-sketch"]

    notebook_dir = "demos_and_examples"
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_10en_traffic_signal_tutorial" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = CellExecuter(
                exception_words=exception_words,
                timeout=1800, 
                kernel_name='python3'
            )
            
            try:
                # ノートブックを実行
                print(f"Testing notebook: {notebook}")
                ep.preprocess(nb, {'metadata': {'path': notebook_dir}})
                print(f"Successfully executed all non-skipped cells in {notebook}")
            except CellExecutionError as e:
                # エラーが発生したセルの情報を出力
                print(f"Error executing notebook {notebook}")
                print(f"Error: {str(e)}")
                pytest.fail(f"Error executing notebook {notebook}: {str(e)}")