# リポジトリ管理方針 / Repository Management Policy

## 日本語

### 概要

この文書は、GitHubで追跡する対象を著者が管理する研究用プログラムに限定するための方針を示す。現在の公開対象は `research_ogm_project/` を中心とするOGM研究コードである。

外部から受領した参照実装、先輩研究コード、学習済みモデル、実験ログ、生成データ、CARLA本体、仮想環境はGitHubへ含めない。

### 構成

| 種別 | パス | 方針 |
|---|---|---|
| 自作OGM研究コード | `research_ogm_project/` | GitHubで追跡する |
| リポジトリ説明 | `README.md` | GitHubで追跡する |
| 管理方針 | `docs/repository_policy.md` | GitHubで追跡する |
| 外部参照実装 | `carla_simulate_project/` | ローカル保持のみ |
| DTリスク予測再現作業 | `dt_risk_prediction_project/`, `docs/dt_risk_prediction/` | ローカル保持のみ |
| 生成データ | データ保存先ディレクトリ | GitHubへ含めない |

### 実行方法

Git操作では、公開するファイルだけを明示的にステージする。

```powershell
git status --short --branch
git diff -- <target-files>
git add <target-file-1> <target-file-2>
git diff --cached
git commit -m "<message>"
git push origin <current-branch>
```

次のコマンドは使用しない。

```powershell
git add .
git add -A
```

### 入出力

GitHubへ含める入力・設定は、自作OGM実験に必要な最小限の設定ファイルに限定する。PNG、CSV、PLY、MP4、ログ、マスク、学習済み重み、zip、実験結果はGitHubへ含めない。

### 注意事項

- 外部参照コードをローカルに残したまま追跡解除する場合は `git rm --cached` を使う。
- 履歴書換え、force push、rebase、BFG Repo-Cleaner、git filter-repo は通常作業では使わない。
- 現在のGitHubツリーから除外しても、過去のcommit履歴にはファイルが残る場合がある。
- 履歴から完全に削除するには、影響範囲を確認したうえで別作業として判断する。

---

## English

### Overview

This document defines the repository policy for tracking only research programs maintained by the author. The current public scope is the OGM research code centered on `research_ogm_project/`.

External reference implementations, senior research code, trained models, experiment logs, generated data, the CARLA distribution, and virtual environments are excluded from GitHub.

### Structure

| Category | Path | Policy |
|---|---|---|
| Author-maintained OGM research code | `research_ogm_project/` | Tracked on GitHub |
| Repository overview | `README.md` | Tracked on GitHub |
| Management policy | `docs/repository_policy.md` | Tracked on GitHub |
| External reference implementation | `carla_simulate_project/` | Local only |
| DT risk reproduction work | `dt_risk_prediction_project/`, `docs/dt_risk_prediction/` | Local only |
| Generated data | Data output directories | Excluded from GitHub |

### How to Run

For Git operations, stage only the files intended for publication.

```powershell
git status --short --branch
git diff -- <target-files>
git add <target-file-1> <target-file-2>
git diff --cached
git commit -m "<message>"
git push origin <current-branch>
```

Do not use:

```powershell
git add .
git add -A
```

### Inputs and Outputs

Tracked inputs and configuration files are limited to the minimum files required for the author-maintained OGM experiments. PNG, CSV, PLY, MP4, log, mask, trained-weight, archive, and experiment-result files are excluded from GitHub.

### Notes

- Use `git rm --cached` when untracking external reference code while keeping it on the local machine.
- History rewriting, force push, rebase, BFG Repo-Cleaner, and git filter-repo are not part of the normal workflow.
- Files removed from the current GitHub tree may still remain in past commits.
- Complete history removal should be considered separately after checking its impact.
