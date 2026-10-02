# Phase 1b.7 Grounded Scenario Baseline

実施日: 2026-09-29  
正式scenario: `priority_crossing_v2_grounded`  
正式artifact: `D:\CARLA_DATA\outputs\priority_crossing_v2_grounded_20260929_r2`

## 判定

- All vehicle grounding audit: **READY**
- Grounded scenario: **READY**
- Experiment Contract: **READY**
- Common Engine regression: **PASS**
- Ego floating scenario artifact removed: **YES**
- Official grounded baseline: **READY**

旧 `priority_crossing_v1` はSHA-256 `369d6350fc65e7a735d9aa4cd45ec4141cb10cf0d8381148d77fdc7a879e2b19` のまま保存し、historical/floating regression専用とする。

## A. Scenario audit

configに登場するvehicle actorはEgoとTargetの2台のみ。固定vehicle、NPC、obstacle vehicleは存在しない。RSUはvehicleではなくsettle対象外。

| Role | Blueprint | 旧transform `(x,y,z,yaw)` | physics | 旧clearance |
|---|---|---|---|---:|
| Ego | `vehicle.tesla.model3` | `(14.13,69.71,0.6,0.1°)` | OFF固定 | +0.591831 m |
| Target | `vehicle.audi.tt` | `(40.39,41.95,0.6,89.2°)` | OFF固定 | +0.599801 m |

両actorとも旧scenarioではvelocity/angular velocity 0、throttle/brake 0、hand brake OFF。gravityはAPI既定ONだがphysics OFFのため落下しない。EgoもTargetと同様に明確に浮いていた。

## B. Grounding結果

共通手順はphysics ON、gravity ON、linear/angular velocity 0、brake 1、hand brake ON、同期tick、収束後physics OFF固定。判定はPhase 1b.5と同じ `|vz|<0.01 m/s` かつ `|Δz|<0.001 m` を20 frame連続、上限10秒。

| Role | settled `(x,y,z,yaw)` | clearance before | clearance after | frames | time |
|---|---|---:|---:|---:|---:|
| Ego | `(14.130000,69.709999,0.0015126,0.100000°)` | +0.591831 | -0.006657 | 42 | 2.1 s |
| Target | `(40.389999,41.950001,-0.0112870,89.199982°)` | +0.599801 | -0.011486 | 42 | 2.1 s |

同じv2 configで2回初期化し、両actorのx/y/z/yaw差はすべて0だった。OBB下面を手作業で道路面へ合わせていない。

## C. 新Scenario

- name: `priority_crossing_v2_grounded`
- version: `2.0`
- config: `configs/priority_crossing_v2_grounded.json`
- config SHA-256: `97713a71aeae0a76ad59ae4e3b67bd9b80c7bb48188cbbba0aea688760433683`
- calibration SHA-256: `5c99bfda8b44c87d02fadb98834162ef5b93ad419cff628b7f6f97d9fb56a3ce`

configのzは2回のphysics calibrationによるsettled transform由来。CARLAは道路collisionと重なる直接spawnを拒否するため、metadataに記録した旧drop poseからspawnし、毎run同じphysics settleで正式poseを再取得する。

## D. Experiment Contract

`schema_version=2.0`、scenario name/version、`grounded=true`、各actorのblueprint、旧spawn、settled transform、road surface、before/after clearance、閾値、frames/time、settle/measurement時physicsを保存した。

root、Risk、Risk map、OGM snapshotのExperiment ContractはJSON完全一致。Priorityのexperiment alignmentもcompatible。Grid Contractは旧scenarioと同一。

## E. Common Engine regression

Grounded 20 measurements/センサで以下を確認。

- Ego Phase0: bit-exact true
- Ego Phase1a: bit-exact true
- RSU Phase0: bit-exact true
- RSU Phase1a: bit-exact true
- production Common vs Legacy Phase0: Ego/RSUともtrue
- payload、fusion、EgoUnknown、RSUKnown、Priority: 全てidentical true

## F. Grounded Ego

| 指標 | Phase0 | Phase1a |
|---|---:|---:|
| Known Free | 19,117 | 25,611 |
| Unknown | 69,889 | 63,405 |
| Occupied | 994 | 984 |
| Road Known Free | 4,577 | 9,163 |
| Road Unknown | 19,236 | 14,651 |
| Road Occupied | 64 | 63 |

- Unknown→Free: 6,485
- Road Unknown→Free: 4,585
- Occupied→Unknown: 1
- Occupied→Free: 9
- Target Phase0 False Free: 16
- Target Phase1a False Free: 16
- **Target new False Free: 0**

## G. Grounded RSU

| 指標 | Phase0 | Phase1a |
|---|---:|---:|
| Known Free | 24,686 | 33,156 |
| Unknown | 63,288 | 54,818 |
| Occupied | 2,026 | 2,026 |
| Road Known Free | 8,025 | 12,116 |
| Road Unknown | 15,758 | 11,667 |
| Road Occupied | 94 | 94 |

- Unknown→Free: 8,470
- Road Unknown→Free: 4,091
- Occupied→Unknown / Free: 0 / 0
- Target Phase0 False Free: 42
- Target Phase1a False Free: 153
- **Target new False Free: 111**

RSU False Freeは記録のみで、Phase 1c対策は実装していない。

## H. Floating vs Grounded

| 指標 | Floating v1 | Grounded v2 |
|---|---:|---:|
| Ego new Target False Free | 46 | **0** |
| RSU new Target False Free | 60 | **111** |
| Ego Road Unknown→Free | 4,641 | **4,585** |
| RSU Road Unknown→Free | 3,713 | **4,091** |

Phase 1b.5 SETTLED_STATICの0 / 111を再現。Road U→FはPhase 1b.5のTarget-only settle（Ego 4,501、RSU 4,102）に近い。正式v2ではEgo自身も約0.5985 m下がり、Ego LiDARのWORLD poseも正しく下がるため完全一致は期待しない。EgoのRoad改善はfloatingの98.8%残る。

## I–J. Risk / Priority

| 指標 | Floating v1 | Grounded v2 |
|---|---:|---:|
| Risk event count | 1 | 1 |
| Risk positive cells | 2,824 | 2,824 |
| Risk max | 0.998872 | 0.998872 |
| Ego Unknown cells | 69,155 | 69,889 |
| RSU Known cells | 26,667 | 26,712 |
| Baseline candidate cells | 5,142 | 5,194 |
| Priority positive cells | 992 | 1,029 |
| Priority max | 0.998622 | 0.998622 |

RiskはGrounded actor stateから再生成し、旧floating metadataとは混在させていない。

## K. Communication

通信は引き続きfull-grid q8 + zlib + SimChannel。Selective Transmissionは未実装であり、Priority cell数を通信量削減とは解釈しない。

## L. 主成果物

- `grounding_metadata.json`
- `grounded_baseline_audit.json`
- `floating_vs_grounded_baseline.json`
- `ego_phase0.npy`, `ego_phase1a.npy`, `rsu_phase0.npy`, `rsu_phase1a.npy`
- `paired_comparison.json`, `false_free_summary.json`
- `future_risk.npy`, `priority.npy`
- `all_vehicle_grounding.png`
- `grounded_ogm_phase0_phase1a.png`
- `phase1a_paired/*_phase0_vs_phase1a_paired.png`
- `priority/priority_components_real_carla.png`
- `phase1b_analysis/*_target_false_free_analysis.png`

## M–O. テストと次Phase

`python -m pytest -q`: **136 passed / 0 failed**。Grounded artifact auditも全項目PASSしたため、本baselineをPhase 1c以降の正式入力とする。次は **Phase 1c: Grounded baselineで残るRSU False Freeへの対策** を提案する。

