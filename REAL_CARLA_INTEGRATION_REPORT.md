# Future Risk × Cooperative OGM 実 CARLA 統合検証

実施日: 2026-09-27  
Evidence: `D:\CARLA_DATA\DT_RiskPrediction\04_evidence\communication_priority_real_integration_20260927\final_run`

## 結論

同一 CARLA 実験の基準 frame から、Ego/RSU の LiDAR OGM と 6 秒先までの Future Risk を生成し、同一 WORLD Grid 上で次式を評価した。

```text
Priority = Risk × EgoUnknown × RSUKnown × Road
BaselineCandidate = EgoUnknown & RSUKnown & Road
```

Grid Contract と Experiment Contract はともに PASS した。Priority は 992 セルで正となり、保存配列からの独立再計算とも全セルで一致した。

## A. 使用シナリオ

| 項目 | 値 |
|---|---|
| scenario | `priority_crossing_v1` |
| experiment_id | `priority_crossing_town10_v1_seed42` |
| CARLA map | `Town10HD_Opt` |
| Ego 初期状態 | `(x,y,z)=(14.13,69.71,0.60) m`, yaw `0.1°`, speed `8.0 m/s` |
| Target 初期状態 | `(40.39,41.95,0.60) m`, yaw `89.2°`, speed `2.81 m/s` |
| RSU pose | `(40.4,49.0,3.0) m`, yaw `0.0°` |
| Grid center | `(42.0,56.0) m` |
| 制御 | Ego は東進後に南進、Target は北進する決定論的運動 |
| seed | random `42`, TM `42` |

Future Risk は基準時刻の実 CARLA actor 状態を初期値とし、上記の決定論的運動を CARLA frame ごとに進め、既存 `Run_DT_Risk_V1.detect_near_misses` で検出した。OGM は同じ基準 frame における実 LiDAR 観測から生成した。

## B. Experiment 一致

| 項目 | 結果 |
|---|---|
| Risk experiment_id | `priority_crossing_town10_v1_seed42` |
| OGM experiment_id | `priority_crossing_town10_v1_seed42` |
| Scenario 一致 | True |
| 初期条件一致 | True |
| 時間基準一致 | True |
| reference frame | `94571` |
| reference simulation time | `586.0788553245366 s` |
| time_reference_id | `priority_crossing_town10_v1_seed42:94571` |

Risk、OGM、実験ルートに保存された `experiment_metadata.json` は同じ JSON オブジェクトで、SHA-256 はすべて `f0d32f5a...77f0161` だった。18 項目の Experiment Contract 比較結果は `matched` である。

## C. Grid 一致

| 項目 | 値 |
|---|---|
| map | `Town10HD_Opt` |
| frame | `CARLA_WORLD` |
| resolution | `0.2 m/cell` |
| shape | `ny=300`, `nx=300` |
| WORLD X bounds | `[12.0,72.0) m` |
| WORLD Y bounds | `[26.0,86.0) m` |
| center | `(42.0,56.0) m` |
| array axis 0 | `WORLD_Y` |
| array axis 1 | `WORLD_X` |
| cell convention | cell center |
| Grid Compatible | True |

Road Mask はこの実験用 Grid の全セル中心を CARLA map に照会し、`project_to_road=False`、`lane_type=Driving` で再生成した。既存の metadata がない mask は再利用していない。

## D. Risk 結果

| 項目 | 値 |
|---|---:|
| prediction horizon | `6.0 s` |
| prediction dt | `0.05 s` |
| Risk event | 1 |
| Near miss | 1 |
| Collision | 0 |
| 最初の Risk | `4.65 s` |
| 最後の Risk | `4.65 s` |
| Risk max | `0.9988721013` |
| Risk positive cells | 2,824 |

## E. OGM 結果

| 項目 | 値 |
|---|---:|
| Ego Unknown cells | 69,155 |
| RSU Known cells | 26,667 |
| Road cells | 23,877 |
| Target t0: Ego Unknown | True |
| Target t0: RSU Known | True |
| Target t0: Road | True |

Target の t0 代表セルは WORLD `(40.3,41.9) m`、Grid `(ix,iy)=(141,79)` である。

## F. Priority 結果

| 項目 | 値 |
|---|---:|
| baseline candidate cells | 5,142 |
| priority positive cells | 992 |
| candidate cell reduction | `80.70789576%` |
| priority max | `0.9986224771` |
| priority mean over positive cells | `0.2519095540` |
| Risk ∩ EgoUnknown | 1,571 |
| Risk ∩ RSUKnown | 2,756 |
| Risk ∩ Road | 1,981 |
| Risk ∩ all conditions | 992 |

代表正セル:

| 項目 | 値 |
|---|---|
| WORLD | `(40.3,56.7) m` |
| Grid | `(ix,iy)=(141,153)` |
| prediction time | `4.65 s` |
| Risk | `0.9939772487` |
| EgoUnknown | True |
| RSUKnown | True |
| Road | True |
| Priority | `0.9939772487` |

RiskRegion 中心セル `(141,154)` は Ego Known だったため Priority は 0 となった。その 0.2 m 隣のセル `(141,153)` では全条件が成立した。これは mask の境界に起因し、Priority の式どおりの結果である。

## G. 可視化

- `priority/priority_components_real_carla.png`: Ego OGM、RSU OGM、Risk、Ego Unknown、RSU Known、Road、Baseline、Priority の 8 パネル
- `risk/risk_map/risk_with_trajectories.png`: Ego/Target の予測軌跡と RiskRegion 中心
- `priority/priority_map.png`: Communication Priority Map

全画像は同じ CARLA WORLD extent で描画している。

## H. 変更ファイル

今回の実 CARLA 統合で新規作成:

- `configs/priority_crossing_v1.json`
- `scripts/run_real_priority_integration.py`
- `REAL_CARLA_INTEGRATION_REPORT.md`

今回の実 CARLA 統合で変更:

- `C:\CARLA\user_projects\dt_risk_prediction_project\build_future_risk_heatmap.py`: 実験 metadata の受け渡し
- `src/ogm_project/risk_priority.py`: 現行 Experiment Contract、8 パネル画像、集計項目
- `src/ogm_project/priority_snapshot.py`: frame/time と実験 metadata の保存
- `tests/test_risk_priority.py`: 現行 contract と成果物の回帰検証

変更していない主要ファイル・領域:

- `src/ogm_project/coop_comm_compat.py`
- `src/ogm_project/grid_contract.py`
- `C:\CARLA\user_projects\dt_risk_prediction_project\Run_DT_Risk_V1.py`
- `C:\CARLA\user_projects\dt_risk_prediction_project\risk_grid.py`
- `legacy/`
- `sources/`

`SimChannel`、`encode_grid_q8`、`decode_grid_q8`、`fuse_logodds_prefer_ego`、`update_from_points`、`world_to_grid` は保存済み AST 指紋と一致することをテストで確認した。

## I. テスト

```text
cd C:\CARLA\PythonAPI\research_ogm_project
python -m pytest tests/test_risk_priority.py -q
51 passed

cd C:\CARLA\user_projects\dt_risk_prediction_project
python -m pytest tests/test_risk_grid.py -q
53 passed

python -m compileall -q ...
PASS
```

実データ配列についても shape、dtype、有限値、範囲を確認し、次を独立再計算した。

```text
priority_map == risk_aggregate * ego_unknown * rsu_known * road_mask
baseline_candidate_mask == ego_unknown & rsu_known & road_mask
```

どちらも全 90,000 セルで完全一致した。

## J. 最終研究ステータス

| 項目 | 判定 |
|---|---|
| Future Risk Heatmap | READY |
| OGM Grid Alignment | READY |
| Experiment Alignment | READY |
| Communication Priority Map | READY |
| Real CARLA Integrated Priority | READY |
| Risk-based Selective Transmission | NOT IMPLEMENTED |

現行通信は引き続き全 Grid を q8 量子化し、zlib 圧縮して送る。今回の `80.71%` は候補セル数の削減率であり、通信量削減率ではない。

## 再実行コマンド

CARLA server を起動した状態で、未使用の出力ディレクトリを指定する。

```powershell
cd C:\CARLA\PythonAPI\research_ogm_project
python scripts/run_real_priority_integration.py `
  --config configs/priority_crossing_v1.json `
  --output-dir "D:\CARLA_DATA\DT_RiskPrediction\04_evidence\communication_priority_real_integration_YYYYMMDD\run_name"
```
