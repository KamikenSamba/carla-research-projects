# research_ogm_project / research_ogm_project

## 日本語

### 概要

`research_ogm_project` は、CARLA上でEgo OGM、協調認識OGM、静的マスク生成、Spectator確認を実行するための研究用プロジェクトである。

このプロジェクトは、CARLA公式サンプルやCARLA本体とは分離して管理する。実験で生成される画像、CSV、PLY、MP4、マスク、ログはリポジトリ外のデータ保存先へ出力する。

### 構成

```text
research_ogm_project/
|-- README.md
|-- REFACTOR_NOTES.md
|-- configs/
|   |-- scenarios.json
|   `-- fixed_objects.py
|-- legacy/
|-- scripts/
|   |-- build_static_mask.py
|   |-- run_coop_comm.py
|   |-- run_ego_ogm.py
|   `-- show_spectator_pose.py
|-- src/
|   `-- ogm_project/
|       |-- paths.py
|       |-- scenario_loader.py
|       |-- cooperative_runner.py
|       |-- ego_ogm_runner.py
|       |-- coop_comm_compat.py
|       |-- ego_ogm_compat.py
|       `-- spectator_utils.py
|-- run_experiment.ps1
`-- run_experiment.bat
```

`legacy/` には、整理前のOGM系スクリプトを互換実行・比較のために保持する。通常は `scripts/` 配下の実行入口を使用する。

### 実行方法

CARLAサーバを起動し、プロジェクト直下で次のコマンドを実行する。

```powershell
cd research_ogm_project
```

静的マスク生成:

```powershell
python scripts\build_static_mask.py
```

協調認識OGM:

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

Ego OGM:

```powershell
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

シナリオ一覧:

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --list-scenarios
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --list-scenarios
```

PowerShell自動実行:

```powershell
.\run_experiment.ps1 -Mode coop -Scenario scenario_A
.\run_experiment.ps1 -Mode ego -Scenario scenario_A
.\run_experiment.ps1 -Mode mask
.\run_experiment.ps1 -Mode spectator
```

### 入出力

主な入力は `configs/scenarios.json` と `configs/fixed_objects.py` である。保存先のルートは `CARLA_DATA_ROOT` 環境変数で指定できる。未指定の場合は、コード側の既定値を使用する。

出力先の構成は次のように分かれる。

```text
<CARLA_DATA_ROOT>/
|-- outputs/
|   |-- coop_comm/
|   `-- ego_ogm/
|-- masks/
|   `-- static_mask.npy
`-- logs/
```

協調認識OGMは `outputs/coop_comm/<RUN_TAG>/`、Ego OGMは `outputs/ego_ogm/<RUN_TAG>/` に保存する。静的マスクは `masks/static_mask.npy` を使用する。

### 注意事項

- CARLA公式ファイル、Python仮想環境、生成データはこのリポジトリへ含めない。
- `legacy/` 内のコードは、元実装との比較・互換実行用として扱い、通常は変更しない。
- Spectator追従機能は表示確認用であり、OGM生成、通信処理、評価値、保存結果には使用しない。
- `ENABLE_REALTIME_PREVIEW` は観察・デバッグ用である。本実験の評価結果を取得する場合は無効のまま使用する。
- `run_experiment.ps1` のPython候補は環境に合わせて確認する。

---

## English

### Overview

`research_ogm_project` is a CARLA research project for Ego OGM, cooperative perception OGM, static mask generation, and Spectator-based visual checks.

This project is managed separately from the CARLA distribution and official CARLA examples. Generated images, CSV files, PLY files, MP4 files, masks, and logs should be written to a data directory outside the repository.

### Structure

```text
research_ogm_project/
|-- README.md
|-- REFACTOR_NOTES.md
|-- configs/
|   |-- scenarios.json
|   `-- fixed_objects.py
|-- legacy/
|-- scripts/
|   |-- build_static_mask.py
|   |-- run_coop_comm.py
|   |-- run_ego_ogm.py
|   `-- show_spectator_pose.py
|-- src/
|   `-- ogm_project/
|       |-- paths.py
|       |-- scenario_loader.py
|       |-- cooperative_runner.py
|       |-- ego_ogm_runner.py
|       |-- coop_comm_compat.py
|       |-- ego_ogm_compat.py
|       `-- spectator_utils.py
|-- run_experiment.ps1
`-- run_experiment.bat
```

The `legacy/` directory keeps earlier OGM scripts for compatibility execution and comparison. Normal runs should use the entry points under `scripts/`.

### How to Run

Start the CARLA server, then run commands from the project directory.

```powershell
cd research_ogm_project
```

Static mask generation:

```powershell
python scripts\build_static_mask.py
```

Cooperative perception OGM:

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

Ego OGM:

```powershell
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

Scenario listing:

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --list-scenarios
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --list-scenarios
```

PowerShell wrapper:

```powershell
.\run_experiment.ps1 -Mode coop -Scenario scenario_A
.\run_experiment.ps1 -Mode ego -Scenario scenario_A
.\run_experiment.ps1 -Mode mask
.\run_experiment.ps1 -Mode spectator
```

### Inputs and Outputs

The main inputs are `configs/scenarios.json` and `configs/fixed_objects.py`. The output root can be specified with the `CARLA_DATA_ROOT` environment variable. If it is not set, the code uses its default value.

The output layout is:

```text
<CARLA_DATA_ROOT>/
|-- outputs/
|   |-- coop_comm/
|   `-- ego_ogm/
|-- masks/
|   `-- static_mask.npy
`-- logs/
```

Cooperative perception OGM outputs are saved under `outputs/coop_comm/<RUN_TAG>/`, and Ego OGM outputs are saved under `outputs/ego_ogm/<RUN_TAG>/`. Static mask loading uses `masks/static_mask.npy`.

### Notes

- CARLA official files, Python virtual environments, and generated data are not included in this repository.
- Files under `legacy/` are kept for comparison and compatibility execution, and should normally remain unchanged.
- Spectator following is only for visual inspection. It is not used for OGM generation, communication, metrics, or saved results.
- `ENABLE_REALTIME_PREVIEW` is for observation and debugging. Keep it disabled when collecting evaluation results.
- Check the Python executable candidates in `run_experiment.ps1` for the local environment.
