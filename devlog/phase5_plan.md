# フェーズ5開発計画：demos_and_examples動作確認

## 目的
demos_and_examples内のexample_*.pyおよびdemo_notebook_*.ipynbをcpp=Trueで動作確認。
編集して良いのはラッパーとC++コードのみ。

## 対象外（スコープ外）
以下はCLAUDE.mdの規定により対象外：
- pickle/copy/save_scenario/load_scenario/user_function使用: example_03en/jp_data_loading, example_23en_chicago, example_24en_user_defined, demo_notebook_08en_chicago
- GPU DRL: example_14en (計算時間超過、フェーズ3で機能確認済), demo_notebook_03en_pytorch
- GUI: example_17en, example_18en
- OSM: example_16en, demo_notebook_04en_OSM
- Google Colab: demo_notebook_05en_colab
- Streamlit: example_27en/jp
- タクシー/モビリティ (Phase 6): example_20en, example_21en (TaxiHandler使用), demo_notebook_06en_taxi

## フェーズ3で成功済み（16ファイル）
example_00en, 01en, 02en, 03en_Nguyen, 04en, 05en, 06en, 07en, 08en, 09en, 10en, 11en, 12en_DRL, 13en, 15jp, 19en
→ これらは再テスト不要（ただしregression確認は別途）

## フェーズ5で新規テストが必要なファイル

### Aグループ: 日本語版example（英語版成功済みの対応物）
1. example_01jp_basic.py — フェーズ3で確認済み
2. example_02jp_bottleneck.py
3. example_03jp_data_loading_sioux_falls_network.py → 対象外 (load_scenario)
4. example_04jp_automatic_network_generation.py
5. example_05jp_gridlock_and_prevention.py
6. example_06jp_dynamic_parameter_change.py
7. example_07jp_signal.py — フェーズ3で確認済み
8. example_08jp_signal_reactive_control.py

### Bグループ: DTA関連（フェーズ4完了後に対応可能に）
9. example_26en_dynamic_traffic_assignment_DUO_DUE_DSO.py

### Cグループ: ノートブック
10. demo_notebook_01en.ipynb
11. demo_notebook_01jp.ipynb
12. demo_notebook_02en.ipynb
13. demo_notebook_09en_dynamic_traffic_assignment.ipynb
14. demo_notebook_10en_signal_tutorial.ipynb

## 担当分配
- **アリス**: Aグループ（日本語版example）のテスト → エラーがあればラッパー/C++修正
- **ボブ**: Bグループ（DTA）+ Cグループの一部（notebook 01en, 02en）
- **キャロル**: Cグループの残り（notebook 01jp, 09en, 10en）

## 進捗

### Aグループ: 日本語版example — ✅ 完了（アリス）
- [x] example_02jp_bottleneck.py — 成功
- [x] example_04jp_automatic_network_generation.py — 成功
- [x] example_05jp_gridlock_and_prevention.py — 成功
- [x] example_06jp_dynamic_parameter_change.py — 成功
- [x] example_08jp_signal_reactive_control.py — 成功
→ **修正不要**

### Bグループ: DTA example — ✅ 完了（ボブ）
- [x] example_26en_DTA_DUO_DUE_DSO — 成功（DUO/DUE/DSO全て正常完了）
→ **修正不要**

### Cグループ: ノートブック — ✅ 完了（ボブ+キャロル）
- [x] demo_notebook_01en — 成功（load_scenarioのみ対象外で失敗、他全機能OK）
- [x] demo_notebook_02en — **対象外**（冒頭でpickle.dumps使用）
- [x] demo_notebook_01jp — 成功（load_scenarioのみ対象外で失敗、他全機能OK）
- [x] demo_notebook_09en_DTA — 成功（DUO/DUE/DSO全て動作）
- [x] demo_notebook_10en_signal — 成功（3シミュレーション全て動作）
→ **C++起因エラーなし。修正不要**

### Regressionチェック — ✅ 完了
- [x] C++モードpytest — 287 passed, 7 failed（全て対象外機能）
- [x] Pythonモードpytest — 287 passed, 7 failed（同一パターン、regression無し）
- [x] フェーズ3成功済みexampleの再確認 — 全17ファイル成功

## ✅ フェーズ5完了 — 修正不要、全対象ファイル動作確認済み
