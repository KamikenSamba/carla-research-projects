# CARLA User Projects

This directory contains only user-added CARLA projects.

CARLA official files are intentionally not copied here:

- `C:\CARLA\PythonAPI\carla`
- `C:\CARLA\PythonAPI\examples`
- `C:\CARLA\PythonAPI\util`
- `C:\CARLA\CarlaUE4`
- `C:\CARLA\Engine`
- other CARLA distribution files

## Included Projects

```text
C:\CARLA\user_projects
├─ research_ogm_project
├─ carla_simulate_project
└─ dt_risk_prediction_project
```

## Runtime Assumption

The projects are managed separately here, but they still run against the CARLA
installation at:

```text
C:\CARLA
```

The Python executable used in this environment is:

```text
C:\Users\神谷健太朗\AppData\Local\Programs\Python\Python312\python.exe
```

## Data Policy

Generated data should stay outside Git management.

For the OGM project, generated simulation data should continue to go under:

```text
D:\CARLA_DATA
```

For the CARLA simulate project, run artifacts are ignored by Git:

```text
runs\
*.csv
*.png
*.gif
*.mp4
*.log
```

## GitHub Use

If you want to create a GitHub repository, initialize Git in this directory:

```powershell
cd C:\CARLA\user_projects
git init
git add README.md .gitignore research_ogm_project carla_simulate_project
git status
```

Before committing, review `git status` and make sure no generated data or CARLA
official distribution files are included.
