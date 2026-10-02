# Refactor Notes

## 調査した元ファイル

協調認識系:

- `Run_Coop_Comm_V1.py`
- `Run_Coop_Comm_V2.py`
- `Run_Coop_Comm_V3.py`
- `Run_Coop_Comm_V4.py`
- `Run_Coop_Comm_V5.py`

Ego 単体系:

- `Run_Ego_OGM_V1.py`

補助コードと設定:

- `build_static_mask_from_hdmap.py`
- `build_static_mask_from_hdmap_V2.py`
- `build_static_mask_from_hdmap_V3.py`
- `fixed_objects.py`
- `show_spectator_pose.py`
- `scenarios.json`

## コピー元と保存先

新規プロジェクトは当初 D ドライブに作成し、その後の運用方針に合わせて C ドライブへコピーしました。

```text
C:\CARLA\PythonAPI\research_ogm_project
```

`legacy/` には、既存研究用ファイルを削除、移動、上書きせず、参照用コピーとして保存しました。

移行確認が終わるまで、旧配置の `D:\CARLA\PythonAPI\research_ogm_project` は削除していません。

`D:\CARLA\PythonAPI\examples` に存在したファイルは D 側からコピーしました。`Run_Ego_OGM_V1.py` と `build_static_mask_from_hdmap_V3.py` は D 側に存在しなかったため、`C:\CARLA\PythonAPI\examples` からコピーしました。

## 採用した最新ベース

- 協調認識系: `Run_Coop_Comm_V5.py`
- Ego 単体系: `Run_Ego_OGM_V1.py`
- 静的マスク生成: `build_static_mask_from_hdmap_V3.py`

D 側には `Run_Coop_Comm_V6.py`, `Run_Coop_Comm_V7.py`, `Run_Ego_OGM_Centered_V1.py` も存在しましたが、今回の依頼で明示された V1 から V5、Ego V1、静的マスク V1 から V3 を対象にしました。将来取り込む場合は、差分確認後に別メモへ追記してください。

## バージョン差分の要約

協調認識系は V1 から V5 にかけて、基本の Ego/RSU LiDAR OGM 生成に加えて、log-odds 減衰、RSU グリッドの量子化と zlib 圧縮、疑似通信路の遅延・損失・Byte 数集計、Ego 優先融合、静的マスクを使った RSU 送信対象削減、PNG/CSV 出力が整理されています。

Ego 単体系は協調認識とは目的が異なり、RSU、通信、融合を持たず、WORLD 基準の sparse OGM と Ego 中心の local OGM 出力に集中しています。このため実行入口は `run_coop_comm.py` と `run_ego_ogm.py` に分けたまま維持しました。

静的マスク生成は V3 を採用しました。保存先は `CARLA_DATA_ROOT\masks\static_mask.npy` で、`CARLA_DATA_ROOT` 未設定時は `D:\CARLA_DATA` です。

## 共通化した処理

以下を `src/ogm_project/` に分離しました。

- `paths.py`: プロジェクトパス、`CARLA_DATA_ROOT`、legacy スクリプト解決
- `config.py`: 実行時の既定シナリオ設定
- `scenario_loader.py`: `scenarios.json` と scenario actor 定義の読み込み
- `geometry.py`: LiDAR local から WORLD、WORLD から Ego への座標変換
- `grid_utils.py`: grid spec と Bresenham
- `logodds.py`: log-odds 減衰、確率変換、q8 圧縮、Ego 優先融合
- `communication.py`: 疑似通信路の送受信メトリクス
- `rendering.py`: label/heatmap PNG 出力補助
- `output_utils.py`: 出力ディレクトリ作成と PLY 保存
- `sparse_world_ogm.py`: chunked sparse world map の基本構造
- `legacy_runner.py`: legacy 実装を壊さず呼び出す実行ラッパー

## あえて分離したまま残した処理

初回整理ではアルゴリズム変更を避けるため、以下は legacy 実装の本体をそのまま使っています。

- 協調認識系の固定原点グリッド更新
- RSU 通信、融合、圧縮処理の実験本体
- Ego 単体系の world/local OGM 更新ループ
- CARLA actor と sensor の生成・破棄
- 実験ループ、同期モード、出力タイミング
- 既存の PNG、CSV、NPY、PLY、MP4 出力仕様

`src/ogm_project/` に置いた共通モジュールは、次回以降に legacy 本体から段階的に参照へ置き換えるための土台です。

## 命名とコメントの整理

新規ファイルでは、`pts_world`, `pts_ego`, `ego_logodds`, `rsu_local_logodds`, `rsu_received_logodds`, `static_mask` など、意味が追いやすい名前を優先しています。

既存 legacy ファイル内には、元ファイル由来のコメント文字化けや古い docstring が残っています。無変更コピーの方針を優先したため、legacy 内では修正していません。新規 README と本メモで対応関係を明記しました。

## 残っている技術的課題

- legacy 本体はまだ大きな `main()` を中心にしているため、CARLA 接続、actor 生成、sensor 生成、シミュレーションループ、出力処理を段階的に切り出す余地があります。
- 協調認識系と Ego 単体系で共通化可能な関数は抽出済みですが、legacy 本体からの実利用へ置換する作業は未実施です。
- `Run_Coop_Comm_V6.py` 以降や `Run_Ego_OGM_V2.py` 以降を採用する場合は、今回の採用ベースとの差分調査が必要です。
- CARLA サーバを使った実走確認は環境状態に依存します。サーバ未起動時は構文、help、シナリオ一覧確認までを実施してください。

## 実行入口

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
python scripts\build_static_mask.py
python scripts\show_spectator_pose.py
```

`--help` と `--list-scenarios` はラッパー側で処理するため、CARLA サーバ未起動でも確認できます。

## 2026-06-03 パス整理

研究用スクリプトは C ドライブ、生成データは D ドライブという運用に合わせて、パス管理を `src\ogm_project\paths.py` に集約しました。

- `DATA_ROOT`: `CARLA_DATA_ROOT`、未設定時は `D:\CARLA_DATA`
- `OUTPUT_ROOT`: `DATA_ROOT\outputs`
- `COOP_OUTPUT_ROOT`: `DATA_ROOT\outputs\coop_comm`
- `EGO_OUTPUT_ROOT`: `DATA_ROOT\outputs\ego_ogm`
- `MASK_ROOT`: `DATA_ROOT\masks`
- `STATIC_MASK_PATH`: `DATA_ROOT\masks\static_mask.npy`
- `LOG_ROOT`: `DATA_ROOT\logs`

`legacy/Run_Coop_Comm_V5.py` には `STATIC_MASK_PATH = "static_mask.npy"` と `OUT_DIR = "out_grids"` が残っていますが、legacy ファイルを変更しない方針を守るため、`legacy_runner.py` が `main()` 実行直前に `STATIC_MASK_PATH` と `OUT_DIR` を D ドライブ側の絶対パスへ差し替えます。

`legacy/Run_Ego_OGM_V1.py` は既に `CARLA_DATA_ROOT` 配下を使う実装でしたが、同じく `legacy_runner.py` から `OUT_DIR` を `DATA_ROOT\outputs\ego_ogm` へ差し替えます。

静的マスク生成では、`SAVE_MASK_PATH` を `DATA_ROOT\masks\static_mask.npy` へ差し替えます。
