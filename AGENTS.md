# AGENTS.md

## Repository scope

This repository contains author-maintained CARLA research tools. `research_ogm_project/` covers OGM experiments and `dt_risk_prediction_project/` contains the tracked DT risk-prediction tools. `carla_simulate_project/` is an excluded local reference implementation and must not be added to Git.

This repository is the canonical source for both tracked projects. Create OGM Issues, branches, and Pull Requests here, and keep OGM implementation changes under `research_ogm_project/`.

Use **one Issue = one branch = one Codex task**. Do not commit directly to `main`; use a Pull Request and Squash merge.

## Offline verification

CI intentionally avoids CARLA, GPU, GUI, simulator servers, external reference code, and research datasets. Run:

```powershell
python -m compileall -q research_ogm_project dt_risk_prediction_project
Push-Location research_ogm_project
python -m pytest -q
Get-ChildItem configs -Filter *.json | ForEach-Object {
    Get-Content -Raw -Encoding UTF8 $_.FullName | ConvertFrom-Json | Out-Null
}
Pop-Location
```

For CARLA-dependent changes, record the local CARLA version, map, scenario, seed, configuration, commit hash, output location, and any checks that could not be run. Never make CI depend on private wheels, local absolute paths, or generated data.

## Change rules

- Preserve existing research algorithms, baselines, directory structure, and output formats unless the Issue explicitly changes them.
- Do not perform unrelated refactoring, renaming, dependency upgrades, or bulk formatting.
- Do not add external or senior research code, trained models, CARLA/Unreal files, virtual environments, generated images, videos, point clouds, arrays, logs, archives, or datasets.
- Do not add credentials, tokens, personal information, or machine-specific secrets.
- Keep `carla_simulate_project/` local-only.
- Review `docs/repository_policy.md` before changing tracked scope.
- Before commit, run the relevant checks plus `git diff --check`, `git diff`, and `git status`.

## Branches

Use `feature/`, `fix/`, `refactor/`, `docs/`, `chore/`, or `experiment/` followed by a short kebab-case name.
