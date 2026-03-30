# UXsim-UXsimpp Integration Project

UXsimはPython製の交通流シミュレータです．
このディレクトリ構造はUXsimのmainブランチと同じです．

UXsimppは，C++製の交通流エンジンをUXsimと同じ感覚で使えるようにpybind11でPython連携したものです．
UXsimの以前のバージョンをもとにしており，完全に同一の機能は持ってません．
UXsimpp-mainにそのリポジトリの内容があります．

このプロジェクトの目的は，UXsim本体にUXsimppと同等のC++エンジンをオプションとして組み込むことです．

## 要件

- UXsimの現状の機能は完全に維持する
- `W = World(cpp=True)`でC++エンジンに切り替え．Pythonとほぼ完全な互換性（浮動小数点精度・乱数生成・Pythonコールバック関数のみ例外）
- 同じテストコードが`cpp=False`と`cpp=True`を切り替えるだけで動く
- uxsim.pyと同等の機能をC++で実装．UXsimppのアーキテクチャ（pybind11によるtraffi.cpp，bindings.cpp）を流用
- analyzer.py, DTAsolversなどの周辺機能は手を付けず，cpp版から使えるようにする
- pickle/copy/save_scenario/load_scenario/user_functionはcppモード互換性の対象外

## 開発順序・テスト

### フェーズ1〜3：完了
- **フェーズ1**: UXsimppからのコード移植とビルド
- **フェーズ2**: 基本テスト全通過（92/92）
- **フェーズ2.5**: ラッパー薄化・C++直接メモリアクセス化（1.63x高速化）
- **フェーズ2.6**: analyzer.py完全互換・さらなる高速化（137/137テスト通過）．仕様はdevlog/specification_for_phase2-6.md参照
- **フェーズ3**: demos_and_examples動作確認（16/27成功，DRL含む）．詳細はdevlog/phase3_report.md参照

### フェーズ4
test_verification_dta_solversはサブモジュールDTASolversを使った機能．これがC++版でも動くようラッパーとC++を改良する．

また，Python側のdynamic congestion pricingをC++に導入する．
コールバック関数はC++には実装困難なので，全ての時刻のtollをラッパーで求め，そのリストをC++に渡す形とする．

両タスクともにPythonとC++で交通流の形態が大きく変わらないことも確認する．

### フェーズ5
demos_and_examples内部のexample_*.pyおよびdemo_*.pyの対応確認
未移植機能（pickle/copy/save_scenario/load_scenario/user_function），GPUを使ったDRLを用いるもの，GUI，OSM，google colabに関するものはスコープから外す．
それ以外は全てエラーなしで動くことを確認する．
計算時間が非常に長いもの（最大で30分以上）も含まれるので，急がずゆっくり作業する．

最終目的：対象の例・デモが全てエラーなしで動くこと．編集して良いのはラッパーとC++コードのみ

### フェーズ6（対象外）
test_verification_taxi — 機能移植が多く難易度が高い．別プロジェクトで対応．
test_optional_functions_python313 -- バージョン違い問題があるため未対応

.github_oldにUXsimのテスト用ymlがある．必要に応じて.githubをつくり自動テスト化すると良い．

## 設計原則

- **C++内部実装**: ほぼ全機能はC++エンジン内部に実装．ラッパーはC++とPythonの薄い橋渡しのみ．analyzer.py等の外部モジュールのみPython側で処理
- **World.__new__によるディスパッチ**: ファクトリ関数に置き換えるのはNG（isinstance等が壊れる）
- **analyzerとの互換性はデータ構造で担保**: CppWorldが同名・同型のコンテナ（ADJ_MAT, TSIZE, Q_AREA等）を用意
- **bindings.cppはtraffi.cppを直接インクルードするsingle translation unit設計**
- **テスト実行**: `pytest --cpp`フラグでcpp=Trueモード（tests/conftest.pyで実装）

## マルチエージェント運用ルール

- エージェントは指揮官および部下3名（アリス，ボブ，キャロル）．区別のため部下には名前を付け固有の口調（語尾）で話させる
- devlogフォルダに計画書やその進捗をmdで保存・更新する
- コミットメッセージとプルリクエスト本文は日本語．コードコメントとプルリクエストタイトルは英語
- git操作は指揮官に一元化
- 同一問題に複数人が取り組まない．取り組んでいたら指揮官が止めて整理する
- 指揮官の確認ルーチン: sleep 60後に必ず (1) git diff --stat (2) 全3ペインの状態確認 (3) 待機中エージェントへの即時指示 の3ステップを実行。作業中エージェントも，同一問題に取り組んでいたり，指揮官の意図を誤解していたら介入．間隔は必ず60秒固定．理由：部下が誤った方向で作業していたら止める必要があるため．
- tmuxでエージェントに指示を送る際、ペーストされたテキストはEnterキーで送信が必要（自動送信されない場合がある）
- ファイル所有権を厳格に分離（C++/ラッパー/テストで明確に分ける）
- 指揮官は実装作業をしない（Coordinate, push, decide — delegate all coding/testing）
- tmux pane番号は新規ペーン作成時にずれる．指示前にペーン・エージェント対応を確認
- permission promptsがエージェントをサイレントにブロックすることがある．Enterで解除
- 部下の作業内容はtmux capture-pane -t %5 -p | tail -60などで読むが，tailが短いと作業内容を誤解するので注意．合理的な範囲で長くすること．
- 部下は作業が完了したら「tmux send-keys -t %0 "#アリス→指揮官：＊＊＊"」で指揮官に報告を送る．

## 開発ノウハウ（C++/Python統合プロジェクト向け）

### C++移植
- **uxsim.pyを正とする**（UXsimppは旧バージョンベースで差異あり）
- **Python→C++の1行ずつ移植**が最も確実．信号オフセット，capacity_remainの閾値など微妙な差異が結果を大きく変える
- **境界条件に注意**: `if (demand > delta_n)`と`while (demand >= delta_n)`，`>`と`>=`の差で車両数が変わる
- **capacity系フィールドの初期化にdelta_tを忘れない**．全初期化パスをgrepして単位を揃える
- **number_of_lanesのスケーリング漏れ**に注意．capacity_remain, transfer()ループ回数, leader距離など
- **乱数列の差分はC++側で無理に合わせない**．テスト側でrel_tolを設定

### pybind11
- **Python終了時GILクラッシュ**: デストラクタがPython APIを呼ぶとクラッシュ．atexitハンドラで明示的クリーンアップ
- **static py::strはsegfaultの原因**．ヒープ上にInternedStrings構造体を確保し、atexitでdeleteする
- **def_readonlyのvector<T*>は毎回Python list変換で遅い**．get_by_index()で2975倍高速化
- **数値配列は常にnumpy返却**．py::array_t<double>(n, vec.data())でmemcpy一発
- **C++内部のstring生成は積もると重い**．vector<int>で保持しbindings層で必要時のみ変換
- **小規模データではpy::castのリスト生成がnumpy割当+memcpyより軽い**．numpy→tolist()の二重変換は避ける

### ラッパー設計
- **get_link/get_nodeの`obj in list`は禁止**．O(n)線形検索→isinstance判定のO(1)に
- **バッチ先行構築はプロファイルで検証**．遅延評価の方が総合的に速い場合がある
- **numpy配列のPython API互換に注意**．`if not numpy_array:`はValueError，`list + numpy_array`はUFuncNoLoopError
- **プロパティアクセスごとのメソッド呼出しは積もると重い**．ローカル変数にキャッシュ
- **cum_arrival等の繰り返しアクセスにはシミュレーション終了後キャッシュが有効**

### テスト・QA
- **テスト失敗の3分類（対象外/flaky/対応可能）を早期に確立**する
- **raw APIとラッパー経由で切り分けデバッグ**
- **Pythonモードregressionチェックを毎回やる**
- **sedによるデモ一括テスト**: `sed 's/World(/World(cpp=True, /g' example.py | python3`
- **リビルド→テスト**: `pip install -e . && python -m pytest --cpp -q --tb=short`
- **精密ベンチマークは1スレッドで10回計測**（中央値+std）
- **cProfileでボトルネック特定**．cumtimeソート上位5関数を見る
