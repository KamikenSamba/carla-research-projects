# CARLA Research Projects / CARLA Research Projects

## 日本語

### 概要

このリポジトリには、著者が管理する研究用プログラムと設定ファイルのみを含める。外部から受領した参照実装、学習済みモデル、実験ログ、生成データは含めない。

このmonorepoをCARLA研究コードの正式な正本とする。現在の公開・追跡対象は、`research_ogm_project/` の占有グリッドマップ・協調認識研究コードと、`dt_risk_prediction_project/` のDTリスク予測ツールである。

### 構成

```text
.
|-- .github/
|-- AGENTS.md
|-- README.md
|-- docs/
|   `-- repository_policy.md
|-- research_ogm_project/
    |-- README.md
    |-- configs/
    |-- docs/
    |-- legacy/
    |-- scripts/
    |-- src/
    `-- tests/
`-- dt_risk_prediction_project/
```

`research_ogm_project/legacy/` には、整理済みコードの比較・互換実行に使う既存OGMスクリプトを保持する。通常の実行入口は `research_ogm_project/scripts/` と `research_ogm_project/src/` 配下である。

OGMに関する今後のIssue、branch、Pull RequestはこのRepositoryで作成し、`research_ogm_project/` を変更する。

`carla_simulate_project/` は外部参照実装としてローカル保持のみにし、GitHubへ追加しない。

### 開発フロー

```text
Issue → branch → Codex実装 → test/構文確認 → diff確認 → commit → push → PR → CI → review → Squash merge → local main同期
```

詳細は `AGENTS.md` と `docs/WORKFLOW.md` を参照する。branchは `feature/`、`fix/`、`refactor/`、`docs/`、`chore/`、`experiment/` を使用し、`main`へ直接commitしない。

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

This monorepo is the canonical source for the CARLA research code. The tracked scope contains the OGM and cooperative-perception research code under `research_ogm_project/` and the DT risk-prediction tools under `dt_risk_prediction_project/`.

### Structure

```text
.
|-- .github/
|-- AGENTS.md
|-- README.md
|-- docs/
|   `-- repository_policy.md
|-- research_ogm_project/
    |-- README.md
    |-- configs/
    |-- docs/
    |-- legacy/
    |-- scripts/
    |-- src/
    `-- tests/
`-- dt_risk_prediction_project/
```

The `research_ogm_project/legacy/` directory keeps existing OGM scripts for comparison and compatibility execution. Normal entry points are under `research_ogm_project/scripts/` and `research_ogm_project/src/`.

Create future OGM Issues, branches, and Pull Requests in this repository and make their code changes under `research_ogm_project/`.

`carla_simulate_project/` is a local-only external reference implementation and must not be added to GitHub.

### Development workflow

Use Issue → branch → implementation → offline/local checks → diff review → commit → push → Pull Request → CI → review → Squash merge → local `main` sync. See `AGENTS.md` and `docs/WORKFLOW.md` for details.

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
