# DT Risk Prediction V1

この V1 は、現在の CARLA 標準マップ上の交通状態をデジタルツインとして保存し、
そのスナップショットから Autopilot で数秒先まで未来シミュレーションを行い、
衝突および危険接近を CSV として保存するための独立実装です。

既存の OGM 研究コードとは独立しています。以下は含みません。

- PLATEAU
- 独自マップ
- LiDAR 実測入力
- UDP 通信
- LSTM
- PyTorch / CUDA / WSL
- OGM との直接接続

## ファイル

```text
check_dt_risk_environment.py
dt_risk_common.py
Run_DT_Risk_V1.py
README_DT_Risk_V1.md
```

## 前提

- Windows 環境
- 既存の CARLA サーバと Python API をそのまま使用
- 既定の接続先は `127.0.0.1:2000`
- 既定マップは `Town10HD_Opt`
- 既定出力先は `D:\CARLA_DATA\DT_RiskPrediction\runs`
- NumPy 以外の新規外部依存は不要

## 環境確認

```powershell
cd C:\CARLA\user_projects\dt_risk_prediction_project
python check_dt_risk_environment.py --host 127.0.0.1 --port 2000
```

表示する内容:

- Python バージョン
- `carla` モジュールの読み込み元
- CARLA Python API のバージョン表示値
- CARLA サーバ接続可否
- CARLA サーババージョン
- 現在ロードされているマップ名
- `Town10HD_Opt` が利用可能か

CARLA サーバが起動していない場合は、例外で終了し、
CARLA サーバ起動後に再実行するよう表示します。

## capture

現在 CARLA 内に存在する vehicle actor の状態を記録します。

```powershell
python Run_DT_Risk_V1.py --mode capture --host 127.0.0.1 --port 2000 --duration 10
```

出力:

```text
D:\CARLA_DATA\DT_RiskPrediction\runs\<run_id>\capture_states.csv
D:\CARLA_DATA\DT_RiskPrediction\runs\<run_id>\snapshot_states.csv
```

CSV は Excel で文字化けしにくいよう UTF-8 with BOM で保存します。

`capture_states.csv` の列:

```text
phase, carla_frame, sim_time_s, source_actor_id, logical_actor_id,
role_name, blueprint_id, x, y, z, roll, pitch, yaw,
vx, vy, vz, speed_mps, autopilot_enabled
```

`snapshot_states.csv` には指定時刻の全車両状態を保存します。
`--snapshot-at` を省略した場合は、capture 終了時刻付近を使用します。

## predict

保存済み `snapshot_states.csv` から車両を再構成し、
Autopilot で未来シミュレーションします。

```powershell
python Run_DT_Risk_V1.py --mode predict --snapshot "D:\CARLA_DATA\DT_RiskPrediction\runs\<run_id>\snapshot_states.csv" --prediction-seconds 8
```

専用 CARLA サーバ上でマップをリロードしてから再構成する場合:

```powershell
python Run_DT_Risk_V1.py --mode predict --snapshot "D:\CARLA_DATA\DT_RiskPrediction\runs\<run_id>\snapshot_states.csv" --map Town10HD_Opt --reload-world --prediction-seconds 8
```

注意:

`--reload-world` は CARLA ワールド全体をリロードします。
OGM 実験など他の実験が同じ CARLA サーバ上で動いている場合は使用しないでください。

出力:

```text
reconstructed_states.csv
prediction_states.csv
risk_events.csv
risk_regions.csv
config_used.json
summary.json
run.log
```

`prediction_states.csv` の列:

```text
prediction_frame, prediction_time_s, logical_actor_id, carla_actor_id,
x, y, z, yaw, vx, vy, vz, speed_mps, autopilot_enabled
```

`risk_events.csv` の列:

```text
prediction_id, source_snapshot_time_s, prediction_time_s, risk_type,
logical_actor_a, logical_actor_b, carla_actor_a, carla_actor_b,
risk_x, risk_y, risk_z, distance_m, relative_speed_mps,
impulse_magnitude, note
```

`risk_regions.csv` の列:

```text
risk_id, risk_type, center_x, center_y, center_z, radius_m,
start_time_s, end_time_s, logical_actor_a, logical_actor_b
```

## all

capture から predict までを一括実行します。

```powershell
python Run_DT_Risk_V1.py --mode all --duration 10 --snapshot-at 8 --prediction-seconds 8
```

## 同期モードと安全な後始末

このスクリプトは同期モードを使用します。
既定の `fixed_delta_seconds` は `0.05` 秒です。

実行前の WorldSettings は保存し、終了時に復元します。
Traffic Manager の同期設定も終了時に戻します。
このスクリプト自身が生成した actor と collision sensor は追跡し、
通常終了、例外、Ctrl+C の場合でも破棄します。

## 危険判定

### collision

再構成した各車両に `sensor.other.collision` を付与し、
衝突相手、衝突位置、時刻、衝撃量を `risk_events.csv` に保存します。

### near_miss

各 tick で車両ペアの XY 距離と接近速度を確認します。

既定値:

```text
near_miss_distance_m = 4.0
min_closing_speed_mps = 1.0
```

同一ペア・短時間の重複記録を避けるため、
`--near-miss-cooldown-s` を使用します。

## OGM 研究との将来接続

この V1 では OGM へ直接接続しません。

将来的には `risk_regions.csv` の world 座標を Ego 中心 OGM 座標へ変換し、
Risk Mask として扱うことを想定しています。

接続候補:

```text
road_mask -> ego_unknown -> rsu_known -> risk_mask
```

## 実行上の制限

- 完全に同一速度で再現できない場合があります。
- Autopilot への切り替え後は Traffic Manager の挙動に依存します。
- risk event が 0 件でも、再構成と未来シミュレーションが正常に動いたかは
  `summary.json` と `prediction_states.csv` で確認してください。
- 既存 OGM スクリプトは変更していません。
