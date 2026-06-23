# Repository Management Policy

This document separates self-authored work from senior research code, local
reference materials, and generated experiment data.

## 1. Categories

| Category | Path | GitHub policy |
|---|---|---|
| Self-managed OGM research code | `research_ogm_project/` | Push self-authored changes and documentation. Do not modify `legacy/` unless explicitly requested. |
| Senior research code snapshot | `carla_simulate_project/` | Treat as an external research dependency/reference. Avoid modifying or adding to it unless clearly necessary and discussed first. |
| DT risk prediction local workspace | `dt_risk_prediction_project/` | Root-level self-authored prototypes may be tracked. Local reference subdirectories are ignored. |
| Shared documentation and evidence | `docs/` | Preferred place for self-authored reports, reproduction notes, and small safe artifacts. |
| Generated data | `D:\CARLA_DATA\...` | Never commit. Keep outputs on the data drive. |

## 2. Push Scope

By default, push only:

- self-authored program files
- self-authored wrappers or scripts
- configuration files needed to reproduce self-authored workflows
- documentation written in this repository
- lightweight, non-sensitive evidence files

Do not push:

- CARLA binaries or Unreal files
- virtual environments
- raw simulation CSVs
- run logs
- model weights such as `*.pt`
- archives such as `*.zip`
- senior research material received as local reference
- files under ignored local reference directories

## 3. Senior Code Handling

`carla_simulate_project/` is currently used as the execution source for the
senior digital-twin risk prediction workflow. It should remain operational
locally, but future self-authored work should be separated from it.

When extending functionality:

1. Prefer adding self-authored wrappers, notes, or reports outside the senior
   snapshot.
2. If the senior code itself must change, record the reason and exact files
   before editing.
3. Do not add newly received senior files, raw datasets, or reference archives
   to GitHub without explicit permission.

## 4. Local Reference Handling

Local reference material under `dt_risk_prediction_project/*/` is ignored by
`.gitignore`. This keeps received files available on the machine while avoiding
accidental publication.

If a new local reference directory is created, add an explicit ignore rule before
placing private or third-party material inside it.

## 5. Git Procedure

Use explicit staging only:

```powershell
git status --short --branch
git diff -- <target-files>
git add <target-file-1> <target-file-2>
git diff --cached
git commit -m "<message>"
git push origin <current-branch>
```

Never use broad staging for this repository:

```powershell
git add .
git add -A
```

Before pushing, confirm that no senior-code bundle, generated data, raw CSV,
log, model weight, archive, virtual environment, or CARLA distribution file is
staged.

## 6. Existing Tracked Code Note

Some senior-code snapshot files are already tracked in the repository. Removing
them from the current branch can be done with an ordinary deletion commit, but
that does not erase them from Git history. History rewriting is not part of the
normal workflow and should not be done without an explicit decision.
