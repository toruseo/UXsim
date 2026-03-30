# フェーズ4 開発計画

## 目的
1. test_verification_dta_solversがC++モード（`--cpp`）で動作するようにする
2. Python側のdynamic congestion pricingをC++に導入する
3. PythonとC++で交通流の形態が大きく変わらないことを確認する

## 設計方針

### 動的料金のC++実装
- コールバック関数はC++では実装困難
- **解決策**: ラッパーで全タイムステップのtollを事前計算し、`vector<double>`としてC++に渡す
- C++側では`toll_timeseries[timestep]`でルート選択時の料金を参照

### DTA Solvers互換性
- DTAsolversはPython側で`get_toll(t)`を呼ぶ → ラッパーのget_toll()で対応済み
- C++シミュレーション中のルート選択にも動的tollを反映する必要あり
- `specified_route`によるルート指定はC++で既に動作確認済み

## タスク分担

### タケシ（C++実装）
ファイル: `uxsim/trafficpp/traffi.h`, `traffi.cpp`, `bindings.cpp`

1. `traffi.h`: Link構造体に`vector<double> toll_timeseries`を追加
2. `traffi.cpp`: `update_adj_time_matrix()`を修正
   - `toll_timeseries`が空でなければ`toll_timeseries[timestep]`を使用
   - 空なら従来通り`route_choice_penalty`を使用（後方互換性）
3. `bindings.cpp`: `set_toll_timeseries(vector<double>)`メソッドをバインド

### ミサキ（Pythonラッパー）
ファイル: `uxsim/uxsim_cpp_wrapper.py`

1. `finalize_scenario()`内で、congestion_pricingが設定されたリンクのtollを事前計算
   - `[link.congestion_pricing(t * DELTAT) for t in range(TSIZE)]`
   - C++リンクの`set_toll_timeseries()`に渡す
2. DTA Solversの各イテレーションで再計算が必要か検討（toll関数は固定なので初回のみでOK）

### リョウ（テスト・QA）
1. まずPythonモードでtest_verification_dta_solversを実行（ベースライン）
2. C++モードで実行し、エラーを報告
3. 修正後、両モードの結果比較

## 依存関係
- タケシのC++実装 → ミサキのラッパー実装 → リョウのテスト
- ただしリョウはPythonモードのベースライン取得を先行して開始可能

## 完了基準
- `pytest tests/test_verification_dta_solvers.py --cpp -v` が全テスト通過
- Pythonモードでのregressionなし
- 交通流パターンがPython/C++で大きく乖離しないこと
