# CARLA Research Projects

This repository is for user-created CARLA research work and lightweight
documentation. It is not a mirror of the CARLA distribution.

CARLA official files remain outside this repository, for example:

- `C:\CARLA\PythonAPI\carla`
- `C:\CARLA\PythonAPI\examples`
- `C:\CARLA\PythonAPI\util`
- `C:\CARLA\CarlaUE4`
- `C:\CARLA\Engine`

## Directory Roles

```text
C:\CARLA\user_projects
|-- research_ogm_project        # Self-managed OGM research project
|-- carla_simulate_project      # Senior research code snapshot used as a dependency/reference
|-- dt_risk_prediction_project  # Local prototypes and reference workspace
|-- docs                        # Self-authored notes, reports, and lightweight evidence
```

## What Should Be Pushed

Push self-authored source code, wrappers, configuration, documentation, and
small evidence files that are safe to share.

Examples:

- `research_ogm_project/src/...`
- `research_ogm_project/scripts/...`
- `docs/...`
- small `meta.json` or summary CSV files that do not expose raw datasets

## What Should Not Be Pushed

Do not commit CARLA binaries, Unreal assets, virtual environments, generated
simulation outputs, raw experiment CSVs, logs, model weights, archives, or local
reference materials received from others.

Examples:

- `D:\CARLA_DATA\...`
- `*.log`
- large raw `*.csv`
- `*.pt`
- `*.zip`
- `dt_risk_prediction_project/*/`

## Current Senior-Code Boundary

`carla_simulate_project` contains a snapshot of senior research code that has
been used to reproduce the Town10HD_Opt digital-twin risk prediction pipeline.
Treat it as an external research dependency/reference. Avoid modifying it unless
there is a clear reason and the change is discussed first.

New work should normally go into self-managed project code or `docs/`, not into
the senior-code snapshot.

## Git Safety Rule

Do not use:

```powershell
git add .
git add -A
```

Stage files explicitly, review `git diff --cached`, and then commit only the
intended self-authored files.

See [docs/repository_policy.md](docs/repository_policy.md) for the detailed
management policy.
