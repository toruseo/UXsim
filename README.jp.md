# UXsim: Python製のマクロ・メソ交通流シミュレータ

[![PyPi](https://img.shields.io/pypi/v/uxsim.svg)](https://pypi.python.org/pypi/uxsim)
[![Conda Version](https://img.shields.io/conda/vn/conda-forge/uxsim.svg)](https://anaconda.org/conda-forge/uxsim)
[![Demo in Colab](https://colab.research.google.com/assets/colab-badge.svg)](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)
[![arXiv](https://img.shields.io/badge/arXiv-2309.17114-b31b1b.svg)](http://dx.doi.org/10.48550/arXiv.2309.17114)
[![codecov](https://codecov.io/gh/toruseo/UXsim/graph/badge.svg?token=DK77Y1Y5AT)](https://codecov.io/gh/toruseo/UXsim)
[![Static Badge](https://img.shields.io/badge/readme-English%20%F0%9F%87%BA%F0%9F%87%B8%20-%20darkblue)](https://github.com/toruseo/UXsim/blob/main/README.md)

*UXsim*はPython製のオープンソース・フリーなマクロ・メソ交通流シミュレータです．
これは，都市規模のような大局的な自動車交通とその渋滞を再現する交通シミュレーションであり，交通工学分野で標準的なモデルにより道路ネットワークの動的交通流を計算するものです．
UXsimは単純，軽量，柔軟であるため，研究・教育上の目的に適することを意図していますが，それ以外の目的にも自由に使用可能です．

興味がある方は以下をご覧ください：

- [Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01jp.ipynb)または[Google Colabデモ](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)：インタラクティブなデモとチュートリアル
- [詳細ページ（チュートリアル，仕様）](https://toruseo.jp/UXsim/docs/index.html)：チュートリアル，シミュレーションメカニズム，モジュール/関数の仕様に関する詳細なドキュメント

## 主な機能・特徴

- 標準的なネットワーク交通流モデルの，単純かつ容易に使えるPython実装
- マクロ交通流シミュレーション：30秒で都市内の6万台以上の車両をシミュレート
- 動的交通量配分：ネットワーク構造と時間帯別OD需要が与えられているときに，動的なネットワーク交通流を計算
- 学術/専門的交通研究で一般的に使用される理論的に有効なモデル
- タクシー/シェアモビリティ，信号制御，道路課金などの交通制御/管理スキームの実装
- シミュレーション結果の基本的な分析とpandas.DataFrame，CSVへのエクスポート
- matplotlibを使用したシミュレーション結果の可視化．インタラクティブなGUIも利用可能
- 純Pythonであることを活かした高度なカスタマイズ性．PyTorchを用いた深層強化学習による交通制御など，他のPythonベースのフレームワークとも直接連携可能
- メインコードの`uxsim.py`は約1800行のコードのみ．ユーザーが簡単に理解しカスタマイズできます

## 計算例

### 大規模ネットワーク

10km x 10kmのグリッドネットワーク内を2時間で約6万台が通行する様子．計算時間は通常のデスクトップPCで約30秒．

リンク交通状況（リンクの線が太いと車両台数が多く，色が暗いと速度が遅い）と一部車両の軌跡：
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_macro.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/gridnetwork_fancy.gif" width="400"/>
</p>

上記ネットワークのある回廊上の車両軌跡図：
<img src="https://github.com/toruseo/UXsim/blob/images/tsd_traj_links_grid.png" width="600">

### 深層強化学習（AI）による信号制御

信号制御を[PyTorch](https://pytorch.org/)の深層強化学習によって効率化する例です．
下の左図は単純な固定時間の信号の場合で，交通需要がネットワーク容量を上回りグリッドロックが生じています．
右図は深層強化学習を適用した場合で，需要レベルが同じであるのに関わらず効率的な交通が実現しています．
このコードは[Jupyter Notebook](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_03en_pytorch.ipynb)にもまとめてあります．

<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_nocontrol.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_DQL.gif" width="400"/>
</p>

### シミュレーション結果可視化GUI

https://github.com/toruseo/UXsim/assets/34780089/ec780a33-d9ba-4068-a005-0b06127196d9

## インストール

### pipを使用

最も簡単な方法は，PyPIからpipを使用してインストールすることです．

```
pip install uxsim
```

### condaを使用

`conda-forge`チャンネルから`conda`を使用してインストールすることもできます：

```
conda install uxsim
```

詳細については[こちら](https://github.com/conda-forge/uxsim-feedstock?tab=readme-ov-file#installing-uxsim)をご覧ください．

<details>
<summary>上級ユーザー向けの代替方法（クリックして表示）</summary>
	
### カスタム設定でpipを使用

GitHubバージョンをインストールするには，`pip`も使用できます：

```
pip install -U -e git+https://github.com/toruseo/uxsim@main#egg=uxsim
```

または，このリポジトリの他の（開発中の）ブランチや自分のフォークでも：

```
pip install -U -e git+https://github.com/YOUR_FORK/uxsim@YOUR_BRANCH#egg=uxsim
```

### 手動インストール

このGithubリポジトリから`uxsim`ディレクトリをダウンロードするか，[最新リリース](https://github.com/toruseo/UXsim/releases/latest/download/uxsim.zip)からダウンロードして，以下のようにローカルディレクトリに配置します：
```
your_project_directory/
├── uxsim/ 	# uxsimディレクトリ
│ ├── uxsim.py 	# UXsimのメインコード．必要に応じてカスタマイズ可能
│ └── ... 	# uxsim内の他のファイルやディレクトリ
├── your_simulation_code.py 		# 自作コード（必要なら）
├── your_simulation_notebook.ipynb 	# 自作Jupyterノートブック（必要なら）
├── ... 	# 必要に応じて他のファイルも置ける
```
この方法で，UXsimを自分の好みに合わせて柔軟にカスタマイズできます．

</details>

## 使用法

単純な例として，Y字型の合流ネットワークのシミュレーションシナリオを以下に示します：
```python
from uxsim import *

# シミュレーション本体の定義
# 単位は全て秒とメートル
W = World(
    name="",    # シナリオ名
    deltan=5,   # 車両集計単位
    tmax=1200,  # シミュレーション時間
    print_mode=1, save_mode=1, show_mode=0,    # オプション
    random_seed=0    # ランダムシード
)

# シナリオ定義
## ノードの作成
W.addNode(name="orig1", x=0, y=0)
W.addNode("orig2", 0, 2)
W.addNode("merge", 1, 1)
W.addNode("dest", 2, 1)
## ノード間のリンクの作成
W.addLink(name="link1", start_node="orig1", end_node="merge",
          length=1000, free_flow_speed=20, number_of_lanes=1)
W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20, number_of_lanes=1)
W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20, number_of_lanes=1)
## ノード間のOD交通需要の作成
W.adddemand(orig="orig1", dest="dest", t_start=0, t_end=1000, flow=0.45)
W.adddemand("orig2", "dest", 400, 1000, 0.6)

# シミュレーション実行
W.exec_simulation()

# 結果表示
W.analyzer.print_simple_stats()

# ネットワーク交通状態のスナップショット可視化
W.analyzer.network(100, detailed=1, network_font_size=12)
W.analyzer.network(600, detailed=1, network_font_size=12)
W.analyzer.network(800, detailed=1, network_font_size=12)
```

これは以下の結果を出力します：
```
simulation setting:
 scenario name:
 simulation duration:    1200 s
 number of vehicles:     810 veh
 total road length:      3000 m
 time discret. width:    5 s
 platoon size:           5 veh
 number of timesteps:    240
 number of platoons:     162
 number of links:        3
 number of nodes:        4
 setup time:             0.00 s
simulating...
      time| # of vehicles| ave speed| computation time
       0 s|        0 vehs|   0.0 m/s|     0.00 s
     600 s|      130 vehs|  13.7 m/s|     0.03 s
    1195 s|       75 vehs|  12.3 m/s|     0.06 s
 simulation finished
results:
 average speed:  11.6 m/s
 number of completed trips:      735 / 810
 average travel time of trips:   162.6 s
 average delay of trips:         62.6 s
 delay ratio:                    0.385
```
<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/network1_100.png" width="250"/>
<img src="https://github.com/toruseo/UXsim/blob/images/network1_600.png" width="250"/>
<img src="https://github.com/toruseo/UXsim/blob/images/network1_800.png" width="250"/>
</p>

## さらなる情報

UXsimについてさらに学ぶには，以下をご覧ください：

- [Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01jp.ipynb)または[Google Colabデモ](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)：インタラクティブなデモ
- [デモと例](https://github.com/toruseo/UXsim/tree/main/demos_and_examples)：Jupyter NotebookとPythonコードを使用した様々な例
- [UXsim技術資料](https://toruseo.jp/UXsim/docs/index.html)：チュートリアル，シミュレーションメカニズム，モジュール/関数の仕様に関する詳細なドキュメント
- [arXivプレプリント](https://arxiv.org/abs/2309.17114)：科学的な概要

## 主なファイル構成

- `uxsim`ディレクトリ: UXsim本体のパッケージ
	- `uxsim/uxsim.py`: UXsimのメインコード
- `demos_and_examples`ディレクトリ: チュートリアルや使用例
- `dat`ディレクトリ: サンプルシナリオファイル

## 使用条件・ライセンス

UXsimはMIT Licenseのもとで公開しています．
出典を明記すれば自由に使用できます．

使用された成果を発表される際は，参考文献として以下を引用してください：

- 瀬尾亨. [マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装](https://www.coronasha.co.jp/np/isbn/9784339052794/). コロナ社, 2023
- Toru Seo. [UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview](http://dx.doi.org/10.48550/arXiv.2309.17114). arXiv preprint arXiv: 2309.17114, 2023

UXsimを使用した研究は[Githubのwikiページ](https://github.com/toruseo/UXsim/wiki)にまとめられています．自由に編集してください．

## 更新と質問・議論

コード更新のcontributionは大歓迎です！
[貢献ガイドライン](https://github.com/toruseo/UXsim/blob/main/.github/CONTRIBUTING.md)をご覧ください．

質問や提案がある場合は，[Issues](https://github.com/toruseo/UXsim/issues)または[Discussions](https://github.com/toruseo/UXsim/discussions)ページに投稿してください（言語は英語または日本語）．

私（瀬尾亨）は仕事の合間にこのプロジェクトに取り組んでいます．
反応が遅れる可能性があることをご了承ください．

## 謝辞

UXsimは交通流理論と関連分野の様々な研究成果に基づいています．この分野を進展させてきた研究コミュニティに感謝いたします．
特に，UXsimは以下の研究成果を直接利用しています：

- [Newellの単純追従モデル](https://doi.org/10.1016/S0191-2615(00)00044-8)とその拡張版[Xモデル](https://doi.org/10.1016/j.trb.2013.02.008)
- [Incremental Node Model](https://doi.org/10.1016/j.trb.2011.04.001)とその[メソスコピック版](https://ubiquitypress.com/site/chapters/e/10.5334/baw.50/)
- [Dynamic User Optimum](https://doi.org/10.1016/S0191-2615(00)00005-9)型経路選択モデル

## 関連リンク

- [瀬尾亨（作者）](https://toruseo.jp/)
- [東京工業大学 瀬尾研究室](http://seo.cv.ens.titech.ac.jp/)
