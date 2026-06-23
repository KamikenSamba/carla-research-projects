# research_ogm_project

CARLA 0.9.16 上で動作する LiDAR ベースの占有グリッドマップ生成、および Ego/RSU の路車間協調認識実験コードを、CARLA 付属サンプルから分離して管理するための研究用プロジェクトです。

## 運用方針

- CARLA サーバ本体: C ドライブ
- 研究用スクリプト: `C:\CARLA\PythonAPI\research_ogm_project`
- シミュレーション生成データ: `D:\CARLA_DATA`

環境変数 `CARLA_DATA_ROOT` が設定されている場合は、その値をデータ保存先として優先します。未設定の場合のみ `D:\CARLA_DATA` を使用します。

一時的に PowerShell セッションで設定する場合:

```powershell
$env:CARLA_DATA_ROOT = "D:\CARLA_DATA"
```

恒久設定を行う場合:

```powershell
setx CARLA_DATA_ROOT "D:\CARLA_DATA"
```

`setx` 後は、新しく開いた PowerShell から値が反映されます。

## 出力先

C ドライブ側のプロジェクトから実行しても、生成データは D ドライブ側へ保存されます。

```text
D:\CARLA_DATA
|-- outputs
|   |-- coop_comm
|   `-- ego_ogm
|-- masks
|   `-- static_mask.npy
`-- logs
```

協調認識実験の生成物は次へ保存されます。

```text
D:\CARLA_DATA\outputs\coop_comm\<RUN_TAG>\
```

Ego OGM 実験の生成物は次へ保存されます。

```text
D:\CARLA_DATA\outputs\ego_ogm\<RUN_TAG>\
```

静的マスクは次へ保存され、協調認識実験からも同じパスを参照します。

```text
D:\CARLA_DATA\masks\static_mask.npy
```

## ディレクトリ構成

```text
research_ogm_project/
|-- README.md
|-- REFACTOR_NOTES.md
|-- legacy/
|-- configs/
|   |-- scenarios.json
|   `-- fixed_objects.py
|-- scripts/
|   |-- run_coop_comm.py
|   |-- run_ego_ogm.py
|   |-- build_static_mask.py
|   `-- show_spectator_pose.py
|-- src/
|   `-- ogm_project/
|       |-- config.py
|       |-- paths.py
|       |-- geometry.py
|       |-- grid_utils.py
|       |-- logodds.py
|       |-- rendering.py
|       |-- output_utils.py
|       |-- scenario_loader.py
|       |-- carla_actor_utils.py
|       |-- communication.py
|       |-- cooperative_runner.py
|       |-- sparse_world_ogm.py
|       `-- ego_ogm_runner.py
`-- outputs/
```

`outputs/` はプロジェクト内の作業用ディレクトリとして残していますが、実験生成データの保存先は `CARLA_DATA_ROOT` 配下へ統一しています。

## 実行

CARLA サーバを起動してから、プロジェクトルートで以下を実行します。

```powershell
cd C:\CARLA\PythonAPI\research_ogm_project
```

静的マスク生成:

```powershell
python scripts\build_static_mask.py
```

協調認識実験:

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

Ego OGM 実験:

```powershell
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

Spectator 位置確認:

```powershell
python scripts\show_spectator_pose.py
```

シナリオ一覧確認:

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --list-scenarios
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --list-scenarios
```

## 既存コードとの対応

- `legacy\Run_Coop_Comm_V5.py`: 協調認識系の最新ベースとして採用
- `legacy\Run_Ego_OGM_V1.py`: Ego 単体系の採用ベース
- `legacy\build_static_mask_from_hdmap_V3.py`: 静的マスク生成の最新ベースとして採用
- `legacy\*.py`, `legacy\scenarios.json`: 参照用の無変更コピー

`legacy/` 内のコードは変更していません。相対パスを含む既存実装は、`src\ogm_project\legacy_runner.py` から実行直前に保存先だけを差し替えて運用します。

## Spectator 追従表示

`coop` と `ego` の整理済み実行コードでは、CARLA の Spectator を Ego 車両へ追従させます。これは CARLA ウィンドウ上で交差点内の Ego、周辺車両、RSU 付近の位置関係を確認するための表示機能です。Spectator は観察用カメラであり、OGM 生成、LiDAR 入力、通信処理、評価指標、保存結果には使用しません。

既定値は上空俯瞰の `topdown` です。

- `topdown`: Ego の真上から下向きに見る俯瞰表示です。研究中の位置関係確認ではこちらを既定とします。
- `chase`: Ego の後方斜め上から追従します。車両の向きや走行の見た目を確認したい場合に使います。

整理済みコード側の既定設定:

```python
ENABLE_SPECTATOR_FOLLOW = True
SPECTATOR_VIEW_MODE = "topdown"
SPECTATOR_HEIGHT_M = 35.0
SPECTATOR_CHASE_DISTANCE_M = 20.0
SPECTATOR_CHASE_HEIGHT_M = 18.0
ENABLE_REALTIME_PREVIEW = False
```

シナリオ JSON に任意で次の `spectator` 設定を追加できます。既存シナリオに指定がない場合は上記の既定値を使用します。

```json
"spectator": {
  "enabled": true,
  "mode": "topdown",
  "height_m": 35.0,
  "realtime_preview": false
}
```

`ENABLE_REALTIME_PREVIEW=True` または `spectator.realtime_preview=true` にすると、tick ごとに `FIXED_DELTA` 分だけ実時間待機し、CARLA ウィンドウ上で挙動を観察しやすくできます。これはデバッグ・観察用途のみです。通信処理やログが壁時計時間の影響を受ける可能性があるため、評価結果を取得する本実験では `False` のまま使用してください。

## PowerShell 自動実行スクリプト

`run_experiment.ps1` は、CARLA サーバ起動、`CARLA_DATA_ROOT` 設定、データ保存先ディレクトリ作成、CARLA 接続待機、研究用 Python スクリプト実行をまとめて行うための実行補助スクリプトです。

実行例:

```powershell
cd C:\CARLA\PythonAPI\research_ogm_project
.\run_experiment.ps1 -Mode coop -Scenario scenario_A
.\run_experiment.ps1 -Mode ego -Scenario scenario_A
.\run_experiment.ps1 -Mode mask
.\run_experiment.ps1 -Mode spectator
```

モードの意味:

- `coop`: `scripts\run_coop_comm.py` を実行します。
- `ego`: `scripts\run_ego_ogm.py` を実行します。
- `mask`: `scripts\build_static_mask.py` を実行します。
- `spectator`: `scripts\show_spectator_pose.py` を実行します。

`coop` と `ego` では、`-Scenario` を省略すると `scenario_A` を使います。内部では次の引数を付けて実行します。

```powershell
--scenario-file configs\scenarios.json --scenario scenario_A
```

実行時には自動で次を設定します。

```powershell
$env:CARLA_DATA_ROOT = "D:\CARLA_DATA"
```

必要な保存先ディレクトリも自動作成します。

```text
D:\CARLA_DATA\outputs
D:\CARLA_DATA\outputs\coop_comm
D:\CARLA_DATA\outputs\ego_ogm
D:\CARLA_DATA\masks
D:\CARLA_DATA\logs
```

CARLA が `127.0.0.1:2000` で既に起動している場合は、新たに二重起動せず既存サーバを再利用します。起動していない場合は、以下の候補から見つかった実行ファイルを使い、`-carla-port=2000` で起動します。

```text
C:\CARLA\CarlaUnreal.exe
C:\CARLA\CarlaUE4.exe
```

既定では、実験終了後も CARLA サーバは起動したままです。このスクリプトが起動した CARLA だけを終了したい場合は、次のように指定します。既存サーバを再利用した場合は、このオプションを付けても終了しません。

```powershell
.\run_experiment.ps1 -Mode coop -Scenario scenario_A -StopCarlaAfterRun
```

ダブルクリックやショートカットから起動したい場合は、`run_experiment.bat` を使えます。引数なしの場合は既定で `coop` / `scenario_A` を実行します。引数を渡したい場合はコマンドプロンプトやショートカットのリンク先で指定してください。

Python 仮想環境の `python.exe` は、`run_experiment.ps1` 上部の `$PythonExecutableCandidates` にまとめています。環境に合わせて必要ならここを変更してください。

```powershell
$PythonExecutableCandidates = @(
    (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"),
    "C:\CARLA\PythonAPI\examples\venv312\Scripts\python.exe",
    "C:\CARLA\PythonAPI\venv312\Scripts\python.exe"
)
```

現在の既定では、次の Python 3.12 環境を最優先で探索します。

```text
C:\Users\<ユーザー名>\AppData\Local\Programs\Python\Python312\python.exe
```

トラブル時の確認事項:

- `C:\CARLA\CarlaUE4.exe` または `C:\CARLA\CarlaUnreal.exe` が存在するか。
- CARLA 用 Python 3.12 仮想環境の `python.exe` パスが `$PythonExecutableCandidates` に含まれているか。
- ポート `2000` が他プロセスに占有されていないか。
- PowerShell の実行ポリシーで `.ps1` 実行が止められていないか。
- `D:\CARLA_DATA` へ書き込み可能か。
- CARLA 起動後、`127.0.0.1:2000` へ接続可能になるまで十分に待てているか。
