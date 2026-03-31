# UXsim C++エンジンプロジェクト

## プロジェクト概要

UXsimはPython製の交通流シミュレータ．このリポジトリでは，C++製の高速シミュレーションエンジンをオプションとして組み込んでいる．

- `W = World(cpp=True)` でC++エンジンに切り替え．Pythonとほぼ完全な互換性
- 互換性の例外：taxiモード・Pythonコールバック関数（user_function）・pickle/copy/save_scenario/load_scenario．厳密には浮動小数点精度・乱数生成も完全互換ではないが，通常の使用であれば問題ないので気にする必要はない

初期統合フェーズ（フェーズ1〜5）は完了済み．詳細は `devlog/CLAUDE_uxsimpp_integration_phase.md` および他のファイルを参照．

## 現在のタスク：Python新機能のC++追従

UXsim本体（Python版）に新機能が追加されるたびに，それをC++エンジンに反映する．

### 作業フロー

1. **uxsim.pyの変更を確認** — Python版の新機能・変更点を把握
2. **C++に移植** — `uxsim/trafficpp/traffi.h`, `traffi.cpp` に実装
3. **バインディング追加** — `uxsim/trafficpp/bindings.cpp` にpybind11バインディング
4. **ラッパー対応** — `uxsim/uxsim_cpp_wrapper.py` でPython側インターフェース
5. **テスト追加** — `tests/test_cpp_mode.py` にテスト追加（後述）
6. **リグレッションチェック** — Python/C++両モードでテスト通過を確認

### 編集対象ファイル

| ファイル | 役割 |
|---------|------|
| `uxsim/trafficpp/traffi.h` | C++エンジンヘッダ |
| `uxsim/trafficpp/traffi.cpp` | C++エンジン実装 |
| `uxsim/trafficpp/bindings.cpp` | pybind11バインディング（traffi.cppを直接インクルードするsingle translation unit） |
| `uxsim/uxsim_cpp_wrapper.py` | C++とPythonの薄い橋渡しラッパー |
| `tests/test_cpp_mode.py` | C++モード専用テスト |

**編集しないファイル**: `uxsim/uxsim.py`, `uxsim/analyzer.py`, `uxsim/Utilities.py`, `uxsim/DTAsolvers.py` — これらはPython本体．C++側から使えるようにする

## アーキテクチャ

- **C++内部実装**: ほぼ全機能はC++エンジン内部に実装．ラッパーはC++とPythonの薄い橋渡しのみ
- **World.__new__によるディスパッチ**: `cpp=True`時にCppWorldを返す．ファクトリ関数に置き換えるのはNG（isinstance等が壊れる）
- **analyzerとの互換性はデータ構造で担保**: CppWorldが同名・同型のコンテナ（ADJ_MAT, TSIZE, Q_AREA等）を用意
- **UXsimpp-main/**: 元のUXsimppリポジトリ内容（参考用，直接は使わない）

## テスト

### テスト構成

`tests/test_cpp_mode.py` に全C++モードテストが集約（171テスト）：

- **インラインテスト（145件）**: Python版テストからコピーし `World(cpp=True, ...)` を直接指定
- **Notebookデモテスト（4件）**: パッチセル注入方式で実行（01en/jp, 09en_DTA, 10en_signal）
- **Exampleスクリプトテスト（22件）**: sed的にcpp=True注入してsubprocess実行

### テスト実行コマンド

```bash
# C++モードテスト（test_cpp_mode.py）
pip install -e . && python3 -m pytest tests/test_cpp_mode.py --reruns 5 -q --tb=short

# 全テスト（Python + C++）
python3 -m pytest tests/ --reruns 5 -q --tb=short

# --cppフラグで既存テストをC++モードで実行（conftest.pyで実装）
python3 -m pytest tests/ --cpp --reruns 5 -q --tb=short
```

### 新機能追加時のテスト追加方法

1. Python版テストが `tests/test_*.py` に追加されたら，そのテストコードを `tests/test_cpp_mode.py` にコピー
2. `World(` を `World(cpp=True, ` に置換
3. cppモード対象外の機能（pickle等）を使うテストは除外
4. DRLなど30分以上かかるテストも対象外

## Git運用

- このリポジトリは `toruseo/UXsim`（本家）をフォークした `toruseoagent/UXsim` の作業リポジトリ
  - リモート `origin`: `toruseo/UXsim`（本家，読み取り専用．新機能の取り込み元）
  - リモート `fork`: `toruseoagent/UXsim`（フォーク，push先）
  - push時は `git push fork <branch>`
  - 本家の最新を取り込むには `git pull origin main`
  - PRは `toruseoagent/UXsim` → `toruseo/UXsim` へ `gh pr create` で送る

## ブランチ運用

- **開発はmainブランチで行う**．随時 `git push fork main` でフォークにpush
- **本家へのPR時**：mainから `pr/<feature>` ブランチを作成し，フォーク専用ファイル（`CLAUDE.md`, `devlog/`, `user_testing.ipynb`）を除外してコミット → `git push fork pr/<feature>` → `gh pr create --repo toruseo/UXsim`
- PRブランチはorigin/mainベースで作り，mainの変更をcherry-pickしてフォーク専用ファイルをreset/除外する

## フォーク同期の運用

### 原則
- mainブランチは**origin/mainの上にフォーク専用ファイルの1コミットだけが乗った状態**を維持する
- フォーク専用ファイル: `CLAUDE.md`, `devlog/`, `user_testing.ipynb`
- コード差分はゼロに保つ（PRマージ後は本家と完全に一致させる）

### 本家との同期手順
```bash
# 1. 未コミット・未pushの作業がないことを確認（最重要）
git status                     # クリーンであること
git log origin/main..HEAD      # フォーク専用コミットのみであること

# 2. 本家の最新を取り込む
git pull origin main --rebase

# 3. コンフリクトがあれば解消

# 4. コード差分が残っていないか確認
git diff origin/main..HEAD --stat
# → CLAUDE.md, devlog/, user_testing.ipynb 以外が表示されたら異常
```

### PRマージ後のクリーンアップ
PRが本家にマージされると，フォークのmainにcherry-pick残骸やrebase時の重複コミットが残りがち．
以下で整理するが，**作業中のコード変更がないことを必ず確認してから行うこと**:
```bash
# ⚠ 安全確認: コード変更が残っていないこと
git status                     # クリーンであること
git diff origin/main..HEAD --stat  # フォーク専用ファイル以外が表示されたら
                                   # → 作業中の変更の可能性．先にコミット&pushするか確認

# クリーンアップ（フォーク専用ファイル以外の差分がない場合のみ実行）
git reset origin/main
git add CLAUDE.md devlog/ user_testing.ipynb
git commit -m "Add fork-only files (CLAUDE.md, devlog/)"
git push fork main --force-with-lease
```
これにより「N commits ahead」が実態（フォーク専用ファイルのみ）と一致する

### ⚠ 注意: `git reset origin/main` は破壊的操作
- 未コミットのコード変更はワーキングツリーに残るが，`git add`で拾わなければ消える
- コミット済みだが未pushの変更は`git reflog`でしか復旧できない
- **必ず事前にgit statusとgit logで安全を確認すること**

## コミット規約

- コミットメッセージ，コードコメント，プルリクエストタイトル・本文はすべて英語
- コミット&pushを求められたら，未コミットの変更・未追跡ファイルを**全て**含めてコミットし，pushまで行う（特に指定がない限り）
- PRは明示的な指示のある時にのみ行う（例：作業が完了したらPRしておいてください）．指示がないときはcommit&push後に待機する

## 開発ノウハウ

### マルチエージェント運用ルール

- あなたは仮想マシンのtmuxの中にいる．tmux画面分割2x2ペインで部下claudeを`claude ----dangerously-skip-permissions`コマンドで起動する．その後，初期化メッセージを送る．`-p`オプションは使わない．
- エージェントは指揮官および部下3名．区別のため部下には名前（アリス，ボブ，キャロル）を付け固有の口調（語尾）で話させる
- devlogフォルダに計画書やその進捗をmdで保存・更新する．
- git操作は指揮官に一元化．必要な場合git worktreeを使う．
- 同一問題に複数人が取り組まない．取り組んでいたら指揮官が止めて整理する
- 指揮官の確認ルーチン: sleep 60後に必ず (1) git diff --stat (2) 全3ペインの状態確認 (3) 待機中エージェントへの即時指示 の3ステップを実行。作業中エージェントも，同一問題に取り組んでいたり，指揮官の意図を誤解していたら介入．間隔は必ず60秒固定．理由：部下が誤った方向で作業していたら止める必要があるため．
- **超重要**：tmuxでエージェント・指揮官に指示・報告を送る際、ペーストされたテキストはEnterキーで送信が必要．理由：送信できず入力欄に残り，それに気づけない場合がよくある．時間がたっても反応が返ってこなければ確認すること．
- ファイル所有権を厳格に分離（C++/ラッパー/テストなどで明確に分ける）
- 指揮官は実装作業をしない（Coordinate, push, decide — delegate all coding/testing）
- tmux pane番号は新規ペーン作成時にずれる．指示前にペーン・エージェント対応を確認
- permission promptsがエージェントをサイレントにブロックすることがある．Enterで解除
- 部下の作業詳細はtmux capture-pane -t %5 -p | tail -60などで読む．tailが短いと作業内容を誤解するので注意．合理的な範囲で長くすること．
- 部下は作業が完了したら「tmux send-keys -t %0 "#アリス→指揮官：＊＊＊"」で指揮官に報告を送る．

### C++移植

- **uxsim.pyを正とする**（C++側は旧バージョンベースで差異あり）
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

### ラッパー設計

- **get_link/get_nodeの`obj in list`は禁止**．O(n)線形検索→isinstance判定のO(1)に
- **numpy配列のPython API互換に注意**．`if not numpy_array:`はValueError，`list + numpy_array`はUFuncNoLoopError
- **プロパティアクセスごとのメソッド呼出しは積もると重い**．ローカル変数にキャッシュ

### テスト・デバッグ

- **Pythonモードregressionチェックを毎回やる**
- **raw APIとラッパー経由で切り分けデバッグ**
- **flakyテストが多いのでpytest-rerunfailuresを必ずインストール**．rerunして成功すれば全く問題ない
- **リビルド→テスト**: `pip install -e . && python3 -m pytest tests/test_cpp_mode.py --reruns 5 -q --tb=short`
- **精密ベンチマークは1スレッドで10回計測**（中央値+std）．**cProfileでボトルネック特定**．cumtimeソート上位関数を見る
