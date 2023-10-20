# UXsim: Python製のマクロ・メソ交通流シミュレータ

[(English readme is here)](https://github.com/toruseo/UXsim/blob/main/README.en.md)

このリポジトリでは，Python製のオープンソース・フリーなマクロ・メソ交通流シミュレータUXsimを公開しています．
これは，都市規模のような大局的な自動車交通を再現する交通シミュレーションであり，交通工学分野で標準的なモデルにより道路ネットワークの動的交通流を計算するものです．

簡単な使用例を[Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01.ipynb)にまとめてあります．

UXsimは書籍『[マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装](https://www.coronasha.co.jp/np/isbn/9784339052794/)』（著者：[瀬尾亨](https://toruseo.jp/)，出版社：[コロナ社](https://www.coronasha.co.jp/)）に含まれる交通流シミュレータUroborosXを大幅に拡張したものです．
基本動作原理は同じですので，詳細は当該書籍を参照ください．

## 主な機能

- ネットワーク構造と時間帯別OD需要が与えられているときに，動的なネットワーク交通流を計算（動的交通量配分）．具体的な交通モデルは以下の通り：
	- Newellの単純追従モデル（Xモデル）
	- Lagrange版Incremental Node Model
	- Dynamic User Optimum型経路選択モデル（慣性付き）
- 信号交差点，流入制御，経路誘導，混雑課金などの交通マネジメントの組み込み
- 計算結果の各種分析（トリップ完了数，総旅行時間，遅れ時間など）と，そのpandas.DataFrameやCSVへのエクスポート
- 計算結果の可視化（時空間図，MFD，ネットワーク状況アニメーションなど）

## ファイル構成

- `uxsim.py`：UXsim本体
- `utils`ディレクトリ：UXsimに必要なファイル・汎用モジュールをまとめたもの
- `dat`ディレクトリ：いくつかのシナリオファイル
- `demos_and_examples`フォルダ：UXsimのチュートリアル・使用例集
- `README.md`等その他：説明用ファイル群．シミュレーションには不要

## 使用法

使用するには，各自の作業用ディレクトリにファイル`uxsim.py`とディレクトリ`utils`を置き，そこに自分のPythonコードやJupyter Notebookを配置して`from uxsim import *`してください．
[Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01.ipynb)に基本的な使用法と機能をまとめています．
詳細はディレクトリ`demos_and_examples`内の具体例を参照ください．

モジュールとしての整備は今後対応予定です．

### 実行環境

Pythonのバージョン3を用います．
また，以下のモジュールが必要です．
事前にインストールしてください．

- NumPy
- Matplotlib
- Pillow
- tqdm
- SciPy
- pandas

## 計算例

10km x 10kmのグリッドネットワーク内を2時間で約6万台が通行する様子．計算時間は通常のデスクトップPCで約40秒．

リンク交通状況（リンクの線が太いと車両台数が多く，色が暗いと速度が遅い）と一部車両の軌跡：
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

上記ネットワークのある回廊上の車両軌跡図：

<img src="https://github.com/toruseo/UXsim/blob/images/tsd_traj_links_grid.png" width="600">

## 内部の構造と計算フロー

本シミュレータは純Python製であり，使用者が柔軟にカスタムして使用できます．
細部を以下の資料で説明しています．

- 概要：[arXivプレプリント](https://arxiv.org/abs/2309.17114)
- クラスや関数の詳細：[技術リファレンス](https://toruseo.jp/UXsim/docs/index.html)
- 基づいている理論：[書籍](https://www.coronasha.co.jp/np/isbn/9784339052794/)

大まかな構造と計算フローを以下に示します．

### クラス図
<img src="https://github.com/toruseo/UXsim/blob/images/class_diagram.png" width="500">

### シミュレータ全体のアクティビティ図
<img src="https://github.com/toruseo/UXsim/blob/images/activity_diagram.png" width="600">

### ある車両一台のアクティビティ図
<img src="https://github.com/toruseo/UXsim/blob/images/activity_diagram_veh.png" width="400">


## 使用条件・ライセンス

本コードはMIT Licenseのもとで公開しています．
出典を明記すれば自由に使用できます．

使用された成果を発表される際は，参考文献として

- 瀬尾亨. マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装. コロナ社, 2023
- Seo, T. UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview. arXiv preprint arXiv: 2309.17114, 2023

を引用してください．

## 関連リンク

- [瀬尾亨（作者）](https://toruseo.jp/)
- [瀬尾による関連シミュレータまとめサイト](https://toruseo.jp/uxsim/)
- [コロナ社の当該書籍ページ](https://www.coronasha.co.jp/np/isbn/9784339052794/)
- [東京工業大学 瀬尾研究室](http://seo.cv.ens.titech.ac.jp/)
- [Webブラウザ上で動くインタラクティブ交通流シミュレータ](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck_jp.html)：このシミュレータが用いているリンク交通流モデルと同じものをインタラクティブに動かして，交通流とそのシミュレーションの基礎を学べます．
