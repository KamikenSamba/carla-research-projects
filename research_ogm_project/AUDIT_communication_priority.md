# Phase 2 監査・実装報告

実施日: 2026-09-27。実ファイル・ローカルCARLA API定義・保存済みデータを根拠に確認しました。
合成検証と実CARLA統合検証を区別しています。

## A. 現行Coop OGM監査結果

### 標準経路

```text
C:\CARLA\PythonAPI\research_ogm_project\scripts\run_coop_comm.py
→ src/ogm_project/cooperative_runner.py
→ src/ogm_project/coop_comm_compat.py
```

entrypointのimport、runnerの`_patch_runtime`と`main()`呼出しを確認しました。
V番号による選択はしていません。`legacy_runner.run_coop_comm`も現在は標準runnerへ委譲します。
Static Mask生成は別経路で、`scripts/build_static_mask.py → legacy_runner.run_static_mask → legacy/build_static_mask_from_hdmap_V3.py`です。

### 変更前に確認した14項目

| 項目 | 実コード上の状態 |
|---|---|
| entrypoint | scripts/run_coop_comm.py。既定scenarioはconfig.pyのscenario_A |
| Ego OGM生成 | main内on_ego: decay → WORLD座標変換 → 高さfilter → update_from_points(logodds_ego) |
| RSU OGM生成 | main内on_rsu: decay → WORLD座標変換 → 高さfilter → update_from_points_with_origin(logodds_rsu_local) |
| resolution | RES=0.2 m |
| shape | ny=nx=300、60 m × 60 m |
| origin/center | 手動指定またはEgo spawnでORIGINを決定し、WORLD固定patch。現在は±30 m対称なのでORIGIN=center |
| world_to_grid | int((xw-ORIGIN_X-X_MIN)/RES)、Yも同様。別関数in_boundsでindex検査 |
| 配列axis | 更新はtarget_logodds[cy,cx]。axis0=WORLD Y、axis1=WORLD X |
| Ego Known/Unknown | p=1/(1+exp(-L))、Known=(p>=.60)|(p<=.48)、Unknown=~Known |
| RSU Known | 同じ閾値。既存融合・画像では主に受信後RSU、今回Priorityは送信前RSU localを使用 |
| Static Mask | paths.pyのD:\CARLA_DATA\masks\static_mask.npy。load_static_maskはshapeしか確認しない |
| fusion | fuse_logodds_prefer_ego: Ego Knownを保持し、Ego Unknown & RSU Knownだけ補完 |
| 送信 | 全Gridを8-bit量子化、zlib level6で圧縮、SimChannelへ。USE_STATIC_MASK時はコピーのstaticセルをL0にする |
| 外部保存点 | 既存snapshot/終了処理はPNG・集計CSV、最後に点群NPY/PLY。log-oddsやGrid metadataは未保存 |

根拠となる本体の関数: `world_to_grid`、`load_static_mask`、`update_from_points`、
`render_grid`、`fuse_logodds_prefer_ego`、`compute_label_counts`、`encode_grid_q8`、`main`。
Risk側は現在の `risk_grid.py` / `build_future_risk_heatmap.py` / JSON例 / README を再読し、
frozen GridSpec・絶対WORLD境界・[iy,ix]・セル中心評価を確認しました。

### scenario別Grid設定

`configs/scenarios.json`と本体の±30 m設定から確定した値です。
実行時mapはCoopが接続したworldに依存し、scenario JSONにはmap指定がありません。

| scenario | origin=center | WORLD X範囲 | WORLD Y範囲 |
|---|---|---|---|
| baseline / scenario_A | (42,56) | [12,72) | [26,86) |
| scenario_B | (-50,70) | [-80,-20) | [40,100) |
| scenario_C | (-46,20) | [-76,-16) | [-10,50) |

WORLD原点(0,0)と、このローカルpatchのanchor ORIGINは異なります。
一般には center = ORIGIN + (相対min+相対max)/2 であり、非対称境界ではORIGINとcenterは一致しません。

### Road Maskの実際の意味と問題

生成コードはセル中心のWORLD座標で`get_waypoint(loc, project_to_road=False)`を呼び、NoneならTrue。
コードコメントは任意laneを示唆していますが、ローカルの`carla.Map.get_waypoint.__doc__`は既定lane_type=Drivingと示します。
したがって、確認した生成仕様に対するTrueは非Driving側、Priority用道路は反転した値です。

現存maskは300×300で、metadataはありません。生成当時のmapやoriginを証明できません。
標準loaderはbaseline/A/B/Cで同じパスを読み、shapeだけで受理するため、別originの誤用が可能です。
今回は従来loaderやlegacy生成処理を変更せず、Priority側を必須sidecar・hash・Grid一致検査にしました。
既存NPYに推測metadataを追加していません。

## B. Grid整合設計

`GridContract`は次を検査・比較します。

```text
schema_version="1.0", map_name, frame="CARLA_WORLD",
resolution_m, nx, ny, x_min_m, x_max_m, y_min_m, y_max_m,
center_x, center_y, array_axis_0="WORLD_Y", array_axis_1="WORLD_X",
cell_coordinate="center"
```

- Risk v1 metadataは既知の軸・境界・サンプリング文字列を検証して変換します。既存Riskコードは未変更。
- Ego / RSU snapshotは同じ実行時Gridからmetadataを自動生成。actual map短名、origin、相対境界も記録。
- origin+相対境界=WORLD境界、中心=境界中点、幅/高さ=resolution×サイズを検証。
- Roadにも同じ空間metadataを必須とし、NPYのSHA-256とmaskの意味・provenanceを検査。
- 空間値の数値許容差は絶対1e-8、相対0。他の項目は完全一致。
- 不一致・不足は`GridMismatchError`。shapeだけの受理、転置、平行移動、再投影、全道路fallbackはなし。
- `[iy,ix]`とセル中心規約は全出力で明示。PNGは同一WORLD extent、origin=lower、aspect=equal。

標準OGMのint切り捨てによる「下限のわずか外側の点をindex0へ入れる」制約は変更していません。
Grid契約はセルの空間的対応を保証するもので、元OGM観測の境界誤割当を修復しません。

### Experiment/Scenario整合

別判定としてscenario_id、scenarioファイルhash、Ego初期姿勢、Target、共通時計ID、予測起点/観測時点を照合します。
明示的不一致は停止。情報不足は既定でnull/unverifiedを記録し、`--require-experiment-match`では停止します。
Gridが一致しただけで研究上のscenario一致とは判定しません。

## C. 作成・変更ファイル

新規:

```text
src/ogm_project/grid_contract.py
src/ogm_project/risk_priority.py
src/ogm_project/priority_snapshot.py
scripts/build_communication_priority_map.py
tests/test_risk_priority.py
README_communication_priority.md
AUDIT_communication_priority.md
```

変更:

```text
src/ogm_project/coop_comm_compat.py
```

変更内容は、既存logodds.pyの同値閾値を共通参照、任意debug引数、debug時のsnapshot取得hookのみ。
flag無しではdebug lock・コピー・snapshot保存を行いません。
debug時には両LiDAR callbackのframe/timestamp一致と更新完了を確認し、配列をコピーして保存します。
失敗callbackは有効な観測として扱いません。保存は一度だけです。

未変更:

```text
cooperative_runner.py / run_coop_comm.py
Run_DT_Risk_V1.py / dt_risk_common.py
risk_grid.py / build_future_risk_heatmap.py
sources/ / legacy/ / referenceコピー
logodds.py / communication.py
既存mask / scenario設定
```

legacy配下12ファイルのSHA-256一致を確認しました。変更前ソースを一時保存し、以下のAST一致も確認しています。

```text
SimChannel、encode_grid_q8、decode_grid_q8、send分岐、receive分岐
fuse_logodds_prefer_ego、world_to_grid、load_static_mask
update_from_points、on_ego、on_rsu、render_grid、compute_label_counts
```

実CARLA再実行によるdefault動作比較は未実施です。コードの不変性とdebug無効時callback経路をテストしています。
debugを有効にすると同期・保存の負荷は増えるため、その実行を通信遅延評価とは扱いません。

## D. Priority式

実装は指定どおりの単純積です。

```python
priority = (
    risk.astype(np.float32)
    * ego_unknown.astype(np.float32)
    * rsu_known.astype(np.float32)
    * road_mask.astype(np.float32)
)
```

閾値は既存Coopと同じ `.60/.48`。独自閾値・追加重みはありません。
Riskはfiniteな2D float32・[0,1]、maskは同shapeのboolを要求し、暗黙broadcastやNaN/Infは拒否します。
baselineは`ego_unknown & rsu_known & road_mask`。保存maskとlog-oddsから再計算したmaskも照合します。

## E. テスト結果

実行場所: `C:\CARLA\PythonAPI\research_ogm_project`

```text
python -m pytest tests/test_risk_priority.py -q
50 passed in 2.07s
pass: 50
fail: 0
```

既存Heatmap回帰確認（`C:\CARLA\user_projects\dt_risk_prediction_project`）:

```text
python -m pytest tests/test_risk_grid.py -q
53 passed in 2.71s
pass: 53
fail: 0
```

追加確認:

```text
python -m compileall -q src/ogm_project/grid_contract.py src/ogm_project/risk_priority.py src/ogm_project/priority_snapshot.py src/ogm_project/coop_comm_compat.py scripts/build_communication_priority_map.py
python scripts/run_coop_comm.py --list-scenarios
```

両方成功。標準entrypointはbaseline / scenario_A / scenario_B / scenario_Cを返しました。
Grid全項目、不一致shape・同shape別origin、Risk v1変換、実験条件、閾値境界、単純積・各gate、型・非有限値、mask反転・hash、
snapshot同期・コピー・失敗callback、default debug OFF、CLI E2E、通信関数不変を検証しました。

## F. 合成E2E

300×300の実NPY・metadataを作成し、snapshot exporterと独立CLIを通して検証しました。

| 項目 | 結果 |
|---|---|
| 空間 | 中心(42,56)、WORLD X=[12,72)、Y=[26,86)、resolution=.2 |
| Risk | 中央20×20、正値400セル、Score=1 |
| Ego Unknown | 中央30×30、900セル |
| RSU Known | 右半分、45,000セル |
| Road | 横10行、3,000セル |
| Baseline candidate | 150セル |
| Priority positive | 100セル、[iy145:155, ix150:160]のみ |
| 候補セル数削減率 | 0.33333333333333337（約33.33%） |
| shape / dtype | (300,300) / float32 |
| min / max | 0 / 1 |
| NaN / Inf | 0 / 0 |
| Grid / synthetic experiment | 両方一致 |
| 出力 | 指定5ファイル＋baseline_candidate_mask.npyの計6ファイル |
| 画像確認 | 6パネル同一WORLD extentで4条件の交差だけが残ることを目視確認 |

候補セル数削減率であり、送信量削減効果ではありません。
baseline=0では削減率null、正値なしではpriority_mean_positive=0です。

保存先:

```text
D:\CARLA_DATA\DT_RiskPrediction\04_evidence\communication_priority_audit_20260927\
  verification.json
  synthetic\
    source_mask.npy / source_mask.json
    ogm\ (7ファイル)
    risk\ (合成Riskとmetadata)
    priority_map\ (6出力)
    command.txt
```

`synthetic`以下はすべて合成データです。再実行コマンド:

```powershell
python scripts/build_communication_priority_map.py --ogm-dir D:\CARLA_DATA\DT_RiskPrediction\04_evidence\communication_priority_audit_20260927\synthetic\ogm --risk-dir D:\CARLA_DATA\DT_RiskPrediction\04_evidence\communication_priority_audit_20260927\synthetic\risk --output-dir D:\CARLA_DATA\DT_RiskPrediction\04_evidence\communication_priority_audit_20260927\synthetic\priority_map --require-experiment-match
```

## G. 実CARLA統合検証

**Grid mismatchのため実CARLA統合検証未実施。**

照合した実Riskは `20260624_062109` のHeatmapです。
中心=(-52.3,4.8)、X=[-82.3,-22.3)、Y=[-25.2,34.8)。
標準Coop scenario設定のいずれの範囲とも一致せず、検証関数は次を返しました。

```text
baseline / scenario_A: GridMismatchError x_min_m: -82.3 != 12.0
scenario_B:            GridMismatchError x_min_m: -82.3 != -80.0
scenario_C:            GridMismatchError x_min_m: -82.3 != -76.0
```

これはscenario設定から算出した空間との比較です。実Coop runのGrid metadataによる照合ではありません。
同じmapを仮定してもWORLD範囲で不一致になることを確認したもので、実runのmap一致を主張していません。

不足しているもの:

- 既存Coop run 4件にはPNG/集計/蓄積点群しかなく、同時刻Ego/RSU log-oddsとGrid metadataがない。
- 現存Static Maskのmap・origin・意味を裏付ける生成metadataがない。
- 既存Risk v1にはscenario、Ego初期位置、Target、共通時計を照合するexperiment metadataがない。
- 調査時、ローカル127.0.0.1:2000は接続不可で、CARLAプロセスも確認できなかった。

別originのRiskを平行移動したり、既存maskへ推測のmetadataを付けたりしていません。
今後は同じscenario・Grid・Target・予測起点のRiskとCoop snapshot、検証済みRoad maskを揃えて実統合検証してください。
手順はREADMEに記載しています。

## H. 現在の研究ステータス

```text
OGM Grid Alignment: READY
Communication Priority Map: READY
Risk-based Selective Transmission: NOT IMPLEMENTED
```

READYはこの Phase 2 監査で指定された14条件（コード監査・Grid検証・mask定義・単純積・baseline・保存・可視化・集計・テスト・合成E2E・通信未変更）の成立を指します。
この監査時点では実CARLAでのscenario整合を含む統合実験は未検証でした。
現行通信は全Gridを量子化・圧縮して送ります。Risk Priority Mapは生成できますが、Risk-based selective transmissionは未実装です。
Top-K、cell index送信、payload変更、追加重み、bandwidth/latency/drop評価、ns-3、V2Vには進んでいません。

## 2026-09-27 実 CARLA 統合追記

同一 CARLA 実験・同一基準 frame・同一 WORLD Grid による統合検証を完了しました。`REAL_CARLA_INTEGRATION_REPORT.md` にシナリオ、契約照合、数値、画像、Evidence、再現コマンドを記録しています。`Real CARLA Integrated Priority: READY` です。Risk-based Selective Transmission は引き続き `NOT IMPLEMENTED` です。
