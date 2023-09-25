# マクロ交通流シミュレーション：Pythonによる実装（書籍バージョン）

このリポジトリのこのバージョンは，書籍『[マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装](https://www.coronasha.co.jp/np/isbn/9784339052794/)』（著者：[瀬尾亨](https://toruseo.jp/)，出版社：[コロナ社](https://www.coronasha.co.jp/)）に含まれる交通流シミュレータのコードをオープンソースとして公開するものです．
以下のファイル等が含まれています．

- `code01_ctm.py`: 単一リンクでのCell Transmission Model
- `code02_vt.py`: 単一リンクでのVariational Theory
- `code03_uroborosx.py`: ネットワークでの交通流シミュレータUroborosX（Newellの単純追従モデル（Xモデル）＋Lagrange型Incremental Node Model＋慣性Dynamic User Optimum）
- `demo.ipynb`: 上記コードを実行した結果をJupyter Notebookにまとめたもの
- ディレクトリ`out`: `code03_uroborosx.py`の出力を格納するためのもの


## 使用法

それぞれのコードをそのままPythonで実行してください．
既に実行した結果例を`demo.ipynb`にまとめてあるので，適宜参照してください．

シミュレータの詳細については，『マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装』の第5，6章を参照してください．

### 実行環境

Pythonのバージョン3を用います．
また，以下のモジュールが必要です．
事前にインストールしてください．

- NumPy
- Matplotlib
- Pillow
- tqdm
- SciPy

## バージョンアップ版

このシミュレータUroborosXを大幅に拡張したバージョンアップ版をこのリポジトリで近日中に公開します．

## 使用条件・ライセンス

本コードはMIT Licenseのもとで公開しています．
出典を明記すれば自由に使用できます．

使用された成果を発表される際は，参考文献として

- 瀬尾亨. マクロ交通流シミュレーション：数学的基礎理論とPythonによる実装. コロナ社, 2023

を引用してください．

## 関連リンク

- [瀬尾亨（作者）](https://toruseo.jp/)
- [瀬尾による関連シミュレータまとめサイト](https://toruseo.jp/uxsim/)
- [コロナ社の当該書籍ページ](https://www.coronasha.co.jp/np/isbn/9784339052794/)
- [東京工業大学 瀬尾研究室](http://seo.cv.ens.titech.ac.jp/)
- [Webブラウザ上で動くインタラクティブ交通流シミュレータ](http://seo.cv.ens.titech.ac.jp/traffic-flow-demo/bottleneck_jp.html)：このシミュレータが用いている交通流モデルをインタラクティブに動かして，交通流とそのシミュレーションの基礎を学べます．