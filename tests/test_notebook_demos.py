"""
This script test essential Jupyter notebook demos, namely `demo_notebook_01en.ipynb`, `demo_notebook_01jp.ipynb`, and `demo_notebook_05en_for_google_colab.ipynb`, can run without error.
Implementation is awkward, but it works. Other jupyter notebook demos are optionally tested by `test_notebook_demos_extra.py`.
"""


import pytest
import nbformat
import os

from nbconvert.preprocessors import ExecutePreprocessor
from nbconvert.preprocessors import CellExecutionError

from IPython.display import display


class SkipCellsPreprocessor(ExecutePreprocessor):
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

def test_demo_notebook_01en():                        
    exception_words = ["ResultGUIViewer", "%matplotlib"]
    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_01en" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = SkipCellsPreprocessor(
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

def test_demo_notebook_01jp():
    exception_words = ["ResultGUIViewer", "%matplotlib"]
    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_01jp" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = SkipCellsPreprocessor(
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

def test_demo_notebook_05en():
    exception_words = ["ResultGUIViewer", "%matplotlib", "!pip"]
    notebook_dir = "demos_and_examples"
    
    for notebook in os.listdir(notebook_dir):
        if notebook.endswith(".ipynb") and "demo_notebook_05en_for_google_colab" in notebook:
            full_path = os.path.join(notebook_dir, notebook)
            
            with open(full_path, "r", encoding="utf-8") as f:
                nb = nbformat.read(f, as_version=4)
            
            # カスタムプリプロセッサを使用
            ep = SkipCellsPreprocessor(
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