# 標準研究開発フロー

1. 目的、完了条件、対象外、確認方法を記したIssueを作る。実験は仮説、条件、seed、評価指標、出力先も記録する。
2. `main`を`git pull --ff-only origin main`で同期し、working treeがcleanであることを確認する。
3. `feature/`、`fix/`、`refactor/`、`docs/`、`chore/`、`experiment/` branchを作る。
4. Issue範囲内の最小変更を行う。外部参照実装や生成データを追加しない。
5. オフライン確認を実行し、CARLA/GPU/GUIが必要な確認とは分けて記録する。
6. `git diff --check`、`git diff`、`git status`で変更、秘密情報、生成物を確認する。
7. commitしてbranchをpushし、IssueをlinkしたPull Requestを作る。
8. CIとreviewを確認し、成功後にSquash mergeする。
9. localの`main`へ戻り、`git pull --ff-only origin main`で同期する。

CIの基準は次です。

```powershell
python -m compileall -q research_ogm_project dt_risk_prediction_project
```

CARLA実走を行った場合は、CARLA version、map、scenario、seed、設定、結果保存先、commit hashをPull Requestまたは実験記録へ残します。
