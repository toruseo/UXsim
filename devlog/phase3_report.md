# フェーズ3完了レポート：demos_and_examples動作確認

## 概要
demos_and_examplesのexample_*.pyをcpp=Trueモードで動作確認。
対象27ファイル中、**16ファイル成功**、3ファイルは既知制限、1ファイルは計算時間超過で断念。

## テスト結果

### 成功 (16ファイル)

| # | ファイル | 備考 |
|---|---------|------|
| 1 | example_00en_simple | 735/810 trips |
| 2 | example_01en_basic | シミュレーション＋可視化＋アニメーション |
| 3 | example_02en_bottleneck | 840/840 trips、軌跡描画OK |
| 4 | example_03en_Nguyen-Dupuis | 4000/4000 trips |
| 5 | example_04en_automatic_network | 60500/60500 trips |
| 6 | example_05en_gridlock_prevention | 2シナリオ両方完了 |
| 7 | example_06en_dynamic_parameter | 755/820 trips |
| 8 | example_07en_signal | 499/499 trips |
| 9 | example_08en_signal_reactive | 感応制御正常動作 |
| 10 | example_09en_congestion_pricing | pickle除外版で成功。route_choice_penalty正常動作 |
| 11 | example_10en_signal_4legged | 3220/3360 trips |
| 12 | example_11en_signal_reactive_4leg | 3220/3360 trips |
| 13 | example_12en_DRL_pytorch | **DRL学習成功！delay 24.6s→12.7s** |
| 14 | example_13en_multiple_signals | 5235/5575 trips |
| 15 | example_15jp_4phase_signal | 1040/1040 trips |
| 16 | example_19en_multilane | 1200/1200 trips |

日本語版（example_01jp, 07jp）も動作確認済み。

### 既知制限による不可 (3ファイル)

| ファイル | 原因 | CLAUDE.mdの記載 |
|---------|------|----------------|
| example_03en_data_loading_sioux_falls | load_scenario未サポート | 互換性対象外 |
| example_09en_congestion_pricing (原版) | pickle未サポート | 互換性対象外 |
| example_23en_Chicago | load_scenario未サポート | 互換性対象外 |

### 計算時間超過 (1ファイル)

| ファイル | 状況 |
|---------|------|
| example_14en_DRL_multiple_signals | deepcopy除外版を実行。200エピソード×280秒≒15.5時間の見積もり。1h超のため断念。ただしエピソード1は正常完了しており、機能としては動作する |

### 対象外 (8ファイル)

- タクシー/モビリティ: example_20, 21（フェーズ4）
- DTAソルバー: example_26（フェーズ4）
- GUI Viewer: example_17, 18
- Streamlit: example_27en, 27jp
- OSM: example_16

## コード修正

### 1. signal_phase/signal_t/signal_log setterバグ修正
- **ファイル**: uxsim/uxsim_cpp_wrapper.py
- **問題**: setterがPythonローカル変数にのみ書き込み、C++エンジンに反映されていなかった
- **修正**: `self._signal_phase = value` → `self._cpp_node.signal_phase = value` 等

### 2. num_vehicles_queueのC++ネイティブ実装
- **ファイル**: traffi.h, traffi.cpp, bindings.cpp, uxsim_cpp_wrapper.py
- **変更**: Python側のforループ（全車両イテレーション）をC++メソッド1回呼び出しに置換
- **効果**: DRL実行時のパフォーマンス向上

## Regressionチェック
- Pythonモード: 全テスト通過（test_cpp_mode除く）
- C++モード: 129 passed, 8 failed（全て既知の対象外テスト）
- setter修正・num_vehicles_queue最適化によるregressionなし

## 特筆事項
- **DRL（深層強化学習）がC++モードで完全動作**: example_12でdelay 24.6s→12.7sへの学習改善を確認
- **信号制御の完全動作**: 基本信号、感応制御、4脚交差点、複数信号、4現示日本式、全てOK
- **congestion_pricing**: route_choice_penaltyの設定が正常動作

## DRLベンチマーク（example_12en, 40エピソード, 3回計測中央値）

| モード | Run 1 | Run 2 | Run 3 | 中央値 |
|--------|-------|-------|-------|--------|
| Python | 40.60s | 44.88s | 34.02s | **40.60s** |
| C++ | 15.96s | 16.04s | 15.84s | **15.96s** |

**C++モードは2.54倍高速。** C++はばらつきも小さい（std: 0.10s vs 5.43s）。
DRLではシミュレーションを繰り返し実行するため、C++高速化の恩恵が大きい。

## 結論
フェーズ3の目標（example_*.pyのcppモード動作確認）は達成。
移植済み機能の範囲内で全exampleが正常動作することを確認。
失敗はいずれも互換性対象外の機能（pickle/load_scenario/deepcopy）または計算時間超過に起因。
