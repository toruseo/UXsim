# UXsim: Python製のマクロ・メソ交通流シミュレータ

[![PyPi](https://img.shields.io/pypi/v/uxsim.svg)](https://pypi.python.org/pypi/uxsim)
[![](https://tokei.rs/b1/github/toruseo/UXsim?style=flat&category=code&color=dddd22)](https://github.com/toruseo/UXsim)
[![](https://tokei.rs/b1/github/toruseo/UXsim?category=comments&style=flat&color=44cc44)](https://github.com/toruseo/UXsim/)
[![arXiv](https://img.shields.io/badge/arXiv-2309.17114-b31b1b.svg)](http://dx.doi.org/10.48550/arXiv.2309.17114)
[![Demo in Colab](https://colab.research.google.com/assets/colab-badge.svg)](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)
[![Static Badge](https://img.shields.io/badge/readme-English%20%F0%9F%87%BA%F0%9F%87%B8%20-%20darkblue)](https://github.com/toruseo/UXsim/blob/main/README.md)
[![Static Badge](https://img.shields.io/badge/readme-%E6%97%A5%E6%9C%AC%E8%AA%9E%20%F0%9F%87%AF%F0%9F%87%B5%20-pink)](https://github.com/toruseo/UXsim/blob/main/README.jp.md)



*UXsim*はPython製のオープンソース・フリーなマクロ・メソ交通流シミュレータです．
これは，都市規模のような大局的な自動車交通とその渋滞を再現する交通シミュレーションであり，交通工学分野で標準的なモデルにより道路ネットワークの動的交通流を計算するものです．
UXsimは単純，軽量，柔軟であるため，研究・教育上の目的に適することを意図していますが，それ以外の目的にも自由に使用可能です．

- [Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01jp.ipynb)
- [Google Colabデモ](http://colab.research.google.com/github/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_05en_for_google_colab.ipynb)
- [詳細ページ（チュートリアル，仕様）](https://toruseo.jp/UXsim/docs/index.html)
- [arXivプレプリント](https://arxiv.org/abs/2309.17114)
- [交通流理論・シミュレーションの専門書](https://www.coronasha.co.jp/np/isbn/9784339052794/)


## 主な機能・特徴

- 標準的なネットワーク交通流モデルの，単純かつ容易に使えるPython実装
- ネットワーク構造と時間帯別OD需要が与えられているときに，動的なネットワーク交通流を計算（動的交通量配分）．具体的な交通モデルは以下の通り：
	- Newellの単純追従モデル（Xモデル）
	- Lagrange版Incremental Node Model
	- Dynamic User Optimum型経路選択モデル（慣性付き）
- 信号交差点，流入制御，経路誘導，混雑課金などの交通マネジメントの組み込み
- 計算結果の各種分析（トリップ完了数，総旅行時間，遅れ時間など）と，そのpandas.DataFrameやCSVへのエクスポート
- 計算結果の可視化（時空間図，MFD，ネットワーク状況アニメーションなど）
- 純Pythonであることを活かした高度なカスタマイズ性
	- Python製の他のフレームワークとも直接連携可能．例：PyTorchを用いた深層強化学習による交通制御


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
このコードは[Jupyter Notebook](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_03en_pytorch.ipynb)にもまとめてあります.

<p float="left">
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_nocontrol.gif" width="400"/>
<img src="https://github.com/toruseo/UXsim/blob/images/anim_network1_0.22_DQL.gif" width="400"/>
</p>

### シミュレーション結果可視化GUI


https://github.com/toruseo/UXsim/assets/34780089/31607686-99a8-42bd-875d-13a911e6054f



## インストール

### pipを使用

最も簡単な方法は、PyPIからpipを使用してインストールすることです。

```
pip install uxsim
```

<details>
<summary>他の方法（クリックして表示）</summary>
	
### カスタム設定でpipを使用

GitHubバージョンをインストールするには、`pip`も使用できます：

```
pip install -U -e git+https://github.com/toruseo/uxsim@main#egg=uxsim
```

または、このリポジトリの他の（開発中の）ブランチや自分のフォークでも：

```
pip install -U -e git+https://github.com/YOUR_FORK/uxsim@YOUR_BRANCH#egg=uxsim
```

	
### 手動インストール

このGithubリポジトリから`uxsim`ディレクトリをダウンロードするか、[最新リリース](https://github.com/toruseo/UXsim/releases/latest/download/uxsim.zip)からダウンロードして、以下のようにローカルディレクトリに配置します：
```
your_project_directory/
├── uxsim/ # uxsimディレクトリ
│ ├── uxsim.py # UXsimのメインコード。必要に応じてカスタマイズ可能
│ └── ... # その他元からあったファイル
├── your_simulation_code.py # 自作コード（必要なら）
├── your_simulation_notebook.ipynb # 自作Jupyterノートブック（必要なら）
├── ... # 必要に応じて他のファイルも置ける
```
この方法で、UXsimを自分の好みに合わせて柔軟にカスタマイズできます。

</details>

## 使用法

各自のPythonコード上で以下のようにインポート
```python
from uxsim import *
```
し，その後に自分のシミュレーションシナリオを定義します．

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
W.addNode("orig1", 0, 0) # ノードの作成
W.addNode("orig2", 0, 2)
W.addNode("merge", 1, 1)
W.addNode("dest", 2, 1)
W.addLink("link1", "orig1", "merge", length=1000, free_flow_speed=20) # リンクの作成
W.addLink("link2", "orig2", "merge", length=1000, free_flow_speed=20)
W.addLink("link3", "merge", "dest", length=1000, free_flow_speed=20)
W.adddemand("orig1", "dest", 0, 1000, 0.45) # 交通需要の作成
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

これは結果として以下のような文字列をターミナルに出力し，`out`ディレクトリにその下のような画像を出力します：
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

 
[Jupyter Notebookデモ](https://github.com/toruseo/UXsim/blob/main/demos_and_examples/demo_notebook_01jp.ipynb)に基本的な使用法と機能をまとめています．
さらなる詳細は`demos_and_examples`ディレクトリ内の[使用例](https://github.com/toruseo/UXsim/tree/main/demos_and_examples)や，[UXsim技術資料](https://toruseo.jp/UXsim/docs/index.html)を確認してください．

## 主なファイル構成

- `uxsim`ディレクトリ: UXsimパッケージ
	- `uxsim/uxsim.py`: UXsim本体のコード
	- `uxsim/analyzer.py`: 計算結果分析用コード
	- `uxsim/utils.py`: 関連コード
	- `uxsim/ResultGUIViewer/ResultGUIViewer.py`: 計算結果可視化用GUIサブモジュール
	- `uxsim/OSMImporter/OSMImporter.py`: OpenStreetMapからのインポート用サブモジュール
 	- `uxsim/files`ディレクトリ: 関連ファイル
- `demos_and_examples`ディレクトリ: チュートリアルや使用例
- `dat`ディレクトリ: サンプルシナリオファイル
- `tests`，`.github`ディレクトリ: 開発用ファイル

## 使用条件・ライセンス

本コードはMIT Licenseのもとで公開しています．
出典を明記すれば自由に使用できます．

使用された成果を発表される際は，参考文献として

- 瀬尾亨. マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装. コロナ社, 2023
- Seo, T. UXsim: An open source macroscopic and mesoscopic traffic simulator in Python-a technical overview. arXiv preprint arXiv: 2309.17114, 2023

を引用してください．

## 更新と質問・議論

コード更新のcontributionは大歓迎です！
バグ修正などの小さな変更の場合は，pull requestを送ってください．
機能の大きな変更の場合は，まず[Issues](https://github.com/toruseo/UXsim/issues)ページでディスカッションを始めてください．

質問や提案がある場合は，[Discussions](https://github.com/toruseo/UXsim/discussions)ページに投稿してください（言語は英語または日本語）．

私（瀬尾亨）は仕事の合間にこのプロジェクトに取り組んでいます．
反応が遅れる可能性があることをご了承ください．

## 謝辞

UXsimは交通流理論に関する様々な学術的成果に基づいています．この分野を進展させてきた皆様に感謝いたします．

## 関連リンク

- [瀬尾亨（作者）](https://toruseo.jp/)
- [瀬尾による関連シミュレータまとめサイト](https://toruseo.jp/uxsim/)
- 書籍『[マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装](https://www.coronasha.co.jp/np/isbn/9784339052794/)』（著者：[瀬尾亨](https://toruseo.jp/)，出版社：[コロナ社](https://www.coronasha.co.jp/)）：UXsimは本書に含まれる交通流シミュレータ*UroborosX*を大幅に拡張したものです．理論と実装の詳細については本書を参照してください．
- [東京工業大学 瀬尾研究室](http://seo.cv.ens.titech.ac.jp/)
- [Webブラウザ上で動くインタラクティブ交通流シミュレータ](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck_jp.html)：このシミュレータが用いているリンク交通流モデルと同じものをインタラクティブに動かして，交通流とそのシミュレーションの基礎を学べます．
