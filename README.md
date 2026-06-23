# CARLA Research Projects / CARLA Research Projects

## 日本語

### 概要

このリポジトリには、著者が管理する研究用プログラムと設定ファイルのみを含める。外部から受領した参照実装、学習済みモデル、実験ログ、生成データは含めない。

現在の公開・追跡対象は、CARLA上で占有グリッドマップ、Ego OGM、協調認識、静的マスク生成を実行するための自作研究コードである。

### 構成

```text
.
|-- README.md
|-- docs/
|   `-- repository_policy.md
`-- research_ogm_project/
    |-- README.md
    |-- configs/
    |-- scripts/
    |-- src/
    `-- legacy/
```

`legacy/` には、整理済みコードの比較・互換実行に使う既存OGMスクリプトを保持する。通常の実行入口は `scripts/` と `src/` 配下である。

### 実行方法

詳細な実行方法は `research_ogm_project/README.md` を参照する。

代表的な実行入口は次の通り。

```powershell
cd research_ogm_project
python scripts\build_static_mask.py
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

### 入出力

入力設定は主に `research_ogm_project/configs/` に置く。シミュレーションにより生成されるPNG、CSV、PLY、MP4、マスク、ログなどは、リポジトリ外のデータ保存先へ出力する。

生成データ、実験ログ、学習済み重み、外部参照実装、CARLA本体、Python仮想環境はGitHubへ含めない。

### 注意事項

- `git add .` と `git add -A` は使わず、公開するファイルだけを明示的にステージする。
- 外部から受領した参照実装やモデル重みはローカルに保持しても、Gitの追跡対象にしない。
- CARLA本体、Unreal Engine、仮想環境、生成データはこのリポジトリの管理対象外である。
- 追跡対象の方針は `docs/repository_policy.md` に記録する。

---

## English

### Overview

This repository contains only research code and configuration files maintained by the author. External reference implementations, trained models, experiment logs, and generated data are excluded.

The currently tracked project is the author-maintained CARLA OGM research code for occupancy grid maps, Ego OGM, cooperative perception, and static mask generation.

### Structure

```text
.
|-- README.md
|-- docs/
|   `-- repository_policy.md
`-- research_ogm_project/
    |-- README.md
    |-- configs/
    |-- scripts/
    |-- src/
    `-- legacy/
```

The `legacy/` directory keeps existing OGM scripts for comparison and compatibility execution. Normal entry points are under `scripts/` and `src/`.

### How to Run

See `research_ogm_project/README.md` for detailed execution steps.

Typical entry points are:

```powershell
cd research_ogm_project
python scripts\build_static_mask.py
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

### Inputs and Outputs

Input configuration files are mainly stored in `research_ogm_project/configs/`. Generated PNG, CSV, PLY, MP4, mask, and log files should be written to a data directory outside this repository.

Generated data, experiment logs, trained weights, external reference implementations, the CARLA distribution, and Python virtual environments are not included on GitHub.

### Notes

- Do not use `git add .` or `git add -A`; stage only the files intended for publication.
- Reference implementations or model weights received from external sources may be kept locally, but they must not be tracked by Git.
- The CARLA distribution, Unreal Engine files, virtual environments, and generated data are outside the scope of this repository.
- The tracking policy is recorded in `docs/repository_policy.md`.
