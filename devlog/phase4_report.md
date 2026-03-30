# フェーズ4完了レポート：DTA Solvers対応 & Dynamic Congestion Pricing

## 1. 概要

### 目的
1. `test_verification_dta_solvers`がC++モード（`--cpp`）で全テスト通過すること
2. Python側のdynamic congestion pricingをC++に導入すること
3. PythonとC++で交通流の形態が大きく変わらないことを確認すること

### 成果
- DTA solversテスト **4/4 PASS**（C++モード）
- congestion_pricing関連テスト **3/3 PASS**（test_verification_route_choice内）
- 既存テスト **137/137 PASS**（test_cpp_mode、regression無し）
- SolverDUE **2.76倍高速化**（Python 71.1s → C++ 25.8s）
- SolverDSO_D2D **3.54倍高速化**（Python 94.0s → C++ 26.6s）
- シミュレーション単体 **7.71倍高速化**
- enumerate_k_random_routes **18.2倍高速化**（C++実装への切替）

## 2. 実装した変更一覧

### uxsim/uxsim_cpp_wrapper.py

| 変更 | 内容 | 関連バグ/課題 |
|------|------|---------------|
| finalize_scenario()にtoll事前計算追加 | congestion_pricingが設定されたリンクの全タイムステップtollを計算し、C++にset_toll_timeseries()で渡す | congestion_pricingのC++対応 |
| enforce_route()クロスワールドバグ修正 | `list(route.links)`直接使用→`self.W.get_link(l)`で現在Worldから名前解決に変更 | DTAsolversのイテレーション間でルートが無効化される問題 |
| get_link()クロスワールド対応 | CppLinkが現在Worldに属さない場合、`.name`で名前解決するよう変更 | enforce_route修正後も残存したクロスワールド参照 |
| vehicles_enter_log再構成 | simulation_terminated()内で全車両のlog_t_linkからvehicles_enter_logを構築 | ALNS.pyのnp.random.choiceが空配列で落ちる問題 |
| average_travel_time_between()修正 | traveltime_actual配列スライス→vehicles_enter_logベースのPython版同一ロジックに変更 | 車両がいない時間帯で不正な値（140 vs 期待150）を返す問題 |
| actual_travel_time/instant_travel_time最適化 | len/min/maxビルトイン関数呼出しを排除、_free_flow_ttと_deltatをキャッシュ | DTA solver Pythonオーバーヘッド削減（500万回→0回） |

### uxsim/Utilities/Utilities.py

| 変更 | 内容 |
|------|------|
| enumerate_k_random_routesにC++ディスパッチ追加 | CppWorldの場合、C++版enumerate_k_random_routes_cppを呼び出す（18.2倍高速） |
| estimate_congestion_externality_link/routeにC++ディスパッチ追加 | hasattr(W, '_cpp_world')チェックでC++版に自動切替 |

### uxsim/trafficpp/ （タケシ担当）

| 変更 | 内容 |
|------|------|
| traffi.h: Link構造体にtoll_timeseries追加 | `vector<double> toll_timeseries` |
| traffi.cpp: update_adj_time_matrix()修正 | toll_timeseriesが非空なら使用、空なら従来のroute_choice_penalty |
| bindings.cpp: set_toll_timeseries()バインド | Pythonから`vector<double>`を受け渡し |
| traffi.cpp: traveltime_instantスパイク修正 | interval skipping実装 |
| traffi.cpp/bindings.cpp: enumerate_k_random_routes_cpp実装 | C++によるルート列挙（18.2倍高速） |
| traffi.cpp: estimate_congestion_externality実装 | Link単体およびWorld::route版のexternality計算 |
| bindings.cpp: estimate_congestion_externality_routeバインド | Pythonからルート（C++ Linkリスト）とdeparture_timeを渡して呼出し |

## 3. テスト結果

### C++モード（--cpp）

| テストファイル | 結果 | 備考 |
|---------------|------|------|
| test_cpp_mode.py | **137/137 PASS** | regression無し |
| test_verification_route_choice.py | **16/16 PASS** | congestion_pricing含む、2 rerun（flaky） |
| test_verification_dta_solvers.py | **4/4 PASS** | DUE, DSO_GA, DSO_ALNS, congestion_pricing |

### Pythonモード（regression確認）

| テストファイル | 結果 | 備考 |
|---------------|------|------|
| test_verification_route_choice.py | **16/16 PASS** | 1 rerun（flaky） |
| test_verification_dta_solvers.py | **4/4 PASS** | ベースライン |

## 4. 性能ベンチマーク

### DTA Solver最終結果（9x9グリッド、81ノード、288リンク、18 OD）

| Solver | max_iter | Python版 | C++版 | 高速化率 |
|--------|---------|---------|-------|---------|
| SolverDUE | 100 | 71.1s | 25.8s | **2.76倍** |
| SolverDSO_D2D | 100 | 94.0s | 26.6s | **3.54倍** |

シミュレーション単体: **7.71倍高速化**

### 最適化の推移

| 段階 | SolverDUE | SolverDSO_D2D | 備考 |
|------|-----------|---------------|------|
| 初回（最適化前） | 35.0s (2.24x) | 118.4s (0.83x) | DSO_D2DはC++が遅かった |
| キャッシュ追加後 | — | — | traveltime_actualプロパティキャッシュ |
| 全最適化後（最終） | 25.8s (2.76x) | 26.6s (3.54x) | 全Solverで大幅高速化達成 |

### 主要最適化一覧

| 最適化 | 効果 | 対象 |
|--------|------|------|
| C++ enumerate_k_random_routes | ルート列挙18.2倍高速 | SolverDUE/DSO共通 |
| traveltime_actualプロパティキャッシュ | シミュレーション後の繰返しC++→numpy変換を排除 | DSO_D2Dのexternality計算 |
| estimate_congestion_externality C++化 | Pythonループ→C++直接計算に切替 | DSO_D2D |
| hasattr→__dict__チェック置換 | プロパティキャッシュ判定のオーバーヘッド削減 | 全キャッシュ付きプロパティ |
| actual_travel_time/instant_travel_time最適化 | len/min/max排除、_free_flow_tt/_deltatキャッシュ | DTA solver全体 |

**DSO_D2Dの劇的改善（0.83x→3.54x）の主因**: externality計算がPythonループで大量のプロパティアクセスを行っていたため、(1) traveltime_actual等のキャッシュ、(2) externality計算自体のC++化、(3) hasattr→__dict__置換の3段階最適化が累積的に効いた。

## 5. Python/C++の交通流比較

### 車両完了率

| 項目 | Python版 | C++版 |
|------|---------|-------|
| 総車両数 | 6,840 | 6,840 |
| 完了車両数 | 6,840（100%） | 6,840（100%） |

### DTA収束比較（最終結果）

| Solver | Python版 TTT | C++版 TTT | 差異 |
|--------|-------------|-----------|------|
| SolverDUE | 5,892,000 | 6,062,500 | 2.9% |
| SolverDSO_D2D | 5,285,100 | 5,274,900 | 0.2% |

差異は乱数生成の違い（C++とPythonで異なる乱数列）による自然な変動の範囲。
DUEは確率的ルート選択の累積で差異がやや大きいが、DSO_D2Dは決定論的要素が強く差異は極めて小さい。
両モードとも収束し、交通流パターンの質的な差異はない。

## 6. 残存課題

### フェーズ4スコープ外
- **test_verification_taxi**: 機能移植が多く難易度が高い（フェーズ5、別プロジェクト）
- **test_examples等**: 計算時間が非常に長い（フェーズ6、後回し）

### 既知の制限
- **pickle/copy/save_scenario/load_scenario**: C++モード互換性の対象外
- **user_function（コールバック）**: C++には実装困難。congestion_pricingはtoll事前計算で回避済み
- **浮動小数点精度差**: Python/C++間でルート選択の微小な差異が生じうる（テストではrel_tolで吸収）
- **乱数列の差異**: C++とPythonで乱数生成が異なるため、確率的テストはflakyマーク付きで対応

### 将来の最適化候補
- DTAsolvers.pyのactual_travel_time二重呼出し排除（`:212-213`）
- _build_log_cacheのバッチ化再検討（numpy→tolist()のオーバーヘッドが解消されれば有効）
