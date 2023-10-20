# UXsim: Python製のマクロ・メソ交通流シミュレータ

[(English readme is here)](https://github.com/toruseo/UXsim/blob/main/README.md)

このリポジトリでは，Python製のオープンソース・フリーなマクロ・メソ交通流シミュレータ*UXsim*を公開しています．
これは，都市規模のような大局的な自動車交通とその渋滞を再現する交通シミュレーションであり，交通工学分野で標準的なモデルにより道路ネットワークの動的交通流を計算するものです．
UXsimは単純，軽量，柔軟であるため，研究・教育上の目的に適することを意図していますが，それ以外の目的にも自由に使用可能です．

簡単な使用例を[Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01.ipynb)にまとめてあります．

## 主な機能・特徴

- ネットワーク構造と時間帯別OD需要が与えられているときに，動的なネットワーク交通流を計算（動的交通量配分）．具体的な交通モデルは以下の通り：
	- Newellの単純追従モデル（Xモデル）
	- Lagrange版Incremental Node Model
	- Dynamic User Optimum型経路選択モデル（慣性付き）
- 信号交差点，流入制御，経路誘導，混雑課金などの交通マネジメントの組み込み
- 計算結果の各種分析（トリップ完了数，総旅行時間，遅れ時間など）と，そのpandas.DataFrameやCSVへのエクスポート
- 計算結果の可視化（時空間図，MFD，ネットワーク状況アニメーションなど）
- 純Pythonであることを活かした高度なカスタマイズ性

## 主なファイル構成

- `uxsim`ディレクトリ: UXsimパッケージ
	- `uxsim/uxsim.py`: UXsim本体のコード
	- `uxsim/utils.py`: 関連コード
 	- `uxsim/utils`ディレクトリ: 関連ファイル
- `demos_and_examples`ディレクトリ: チュートリアルや使用例
- `dat`ディレクトリ: サンプルシナリオファイル

## 使用法

まず，pipでUXsimパッケージをインストールします．
```
pip install uxsim
```
あるいは，本レポジトリから`uxsim`ディレクトリをダウンロードし，各自の作業用ディレクトリに配置します．
UXsim自体をカスタムして使用したい場合は後者の方法を取ってください．

各自のPythonコード上では以下でインポートできます：
```python
from uxsim import *
```

[Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01.ipynb)に基本的な使用法と機能をまとめています．
さらなる詳細は`demos_and_examples`ディレクトリ内の[使用例](https://github.com/toruseo/UXsim/tree/main/demos_and_examples)や，[UXsim技術資料](https://toruseo.jp/UXsim/docs/index.html)を確認してください．

## 計算例

10km x 10kmのグリッドネットワーク内を2時間で約6万台が通行する様子．計算時間は通常のデスクトップPCで約30秒．

リンク交通状況（リンクの線が太いと車両台数が多く，色が暗いと速度が遅い）と一部車両の軌跡：
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

上記ネットワークのある回廊上の車両軌跡図：

<img src="https://github.com/toruseo/UXsim/blob/images/tsd_traj_links_grid.png" width="600">

## 詳細資料

- [UXsim技術資料](https://toruseo.jp/UXsim/docs/index.html)
- [arXivプレプリントでの概要](https://arxiv.org/abs/2309.17114)
- [交通流理論・シミュレーションの書籍](https://www.coronasha.co.jp/np/isbn/9784339052794/)


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
- 書籍『[マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装](https://www.coronasha.co.jp/np/isbn/9784339052794/)』（著者：[瀬尾亨](https://toruseo.jp/)，出版社：[コロナ社](https://www.coronasha.co.jp/)）：UXsimは本書に含まれる交通流シミュレータ*UroborosX*を大幅に拡張したものです．理論と実装の詳細については本書を参照してください．
- [東京工業大学 瀬尾研究室](http://seo.cv.ens.titech.ac.jp/)
- [Webブラウザ上で動くインタラクティブ交通流シミュレータ](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck_jp.html)：このシミュレータが用いているリンク交通流モデルと同じものをインタラクティブに動かして，交通流とそのシミュレーションの基礎を学べます．
