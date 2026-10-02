# Phase 1b.5 接地・collision geometry診断結果

判定: **READY**。CURRENT Targetは道路面から約0.600 m浮いており、Egoの新規False Free 46セルは自然接地poseで0セルになった。RSUには別のgeometry・Grid評価問題が残る。Phase 1cは未実装。

実走・成果物: `D:\CARLA_DATA\outputs\phase1b5_priority_crossing_20260928_r1`

## A. Target spawn監査

Production runnerの`spawn_vehicle()`はconfigの `(x, y, z, yaw) = (40.39, 41.95, 0.6, 89.2°)` を直接spawn transformに使い、直後に`set_simulate_physics(False)`を呼ぶ。

- OGM warmupは1.0秒、20 measurement。warmup中にTarget transformを再設定しない。
- physics OFFなのでgravityの既定ONは位置に作用しない。gravityを明示的にOFFにはしていない。
- OGM取得前にTarget velocity、angular velocity、throttle、brake、hand brakeを設定しない。physics OFFで固定される。
- OGM取得後のRisk予測ループだけがTarget transformを毎tick更新する。今回のOGMには影響しない。
- `actor z=0.6` はCARLAが接地させた結果ではなく、configの固定値。

診断runnerはproduction runnerを変更せず、専用sceneで3 variantを実行した。

## B. Ground contact

road referenceは2種類を分けて保存した。

- `road_reference_z`: `Map.get_waypoint(...).transform.location.z`。道路中心線の参照値でありcollision surfaceではない。
- `road_surface_z`: 上方から下方へ`World.cast_ray`し、`Roads` labelとなったhitのz。collision-queryが返す道路表面の診断値。

中心値（周辺9点でも差は約1 µm以内）:

| Variant | actor origin z | OBB bottom z | OBB top z | waypoint z | Roads hit z | clearance |
|---|---:|---:|---:|---:|---:|---:|
| CURRENT | 0.600000 | 0.600000 | 1.985296 | 0.000000 | 0.000199 | **+0.599801** |
| PHYSICS_SETTLE | -0.011287 | -0.011287 | 1.374009 | 0.000000 | 0.000199 | **-0.011486** |
| SETTLED_STATIC | -0.011287 | -0.011287 | 1.374009 | 0.000000 | 0.000199 | **-0.011486** |

接地後にOBB下面が道路面へ約11.5 mm入り込むこと自体も、OBBとcollision shapeが一致しない証拠である。

## C. Settle結果

- physics ON、gravity ON、初期velocity / angular velocity 0、brake 1、hand brake ON。
- 判定: `|vz| < 0.01 m/s` かつ `|Δz| < 0.001 m` を20 frame連続。最低20 frame、上限10秒。
- 42 frame、2.1秒で収束。
- settled transform: `(40.389999, 41.950001, -0.011287)`, pitch 0°, yaw 89.199982°, roll 0°。
- final vertical velocity: 0 m/s。
- PHYSICS_SETTLEとSETTLED_STATICのEgo/RSU × Phase 0/Phase 1a配列は4枚すべて`np.array_equal=True`。測定時の動的physicsではなく接地poseが差を決めた。

## D. Ego False Free比較

| Variant | new False Free | XY_ONLY_BELOW | 3D_INTERSECTION | Road U→F | O→U | O→F |
|---|---:|---:|---:|---:|---:|---:|
| CURRENT | 46 | 37 | 9 | 4,641 | 6 | 28 |
| PHYSICS_SETTLE | **0** | **0** | **0** | 4,501 | 6 | 22 |
| SETTLED_STATIC | **0** | **0** | **0** | 4,501 | 6 | 22 |

接地後もEgoのRoad Unknown→Freeは4,501セル、CURRENTの97.0%残った。CURRENTでTargetセルを更新した577 ray-cellは全てOBB下面より下だったが、接地後は新規Target False Freeを作るrayが0になった。

## E. RSU False Free比較

| Variant | new False Free | XY_ONLY_ABOVE | 3D_INTERSECTION | Raster edge | Road U→F | O→U / O→F |
|---|---:|---:|---:|---:|---:|---:|
| CURRENT | 60 | 0 | 58 | 2 | 3,713 | 0 / 0 |
| PHYSICS_SETTLE | 111 | 46 | 63 | 2 | 4,102 | 0 / 0 |
| SETTLED_STATIC | 111 | 46 | 63 | 2 | 4,102 | 0 / 0 |

接地でEgo問題は消えたが、RSU問題は消えず増加した。OBB上端が1.985 mから1.374 mへ下がり、同じ下降rayの多くがTarget上方扱いになった。これはRSUのFalse Free評価が接地だけでは解決しないことを示す。

## F. RSU collision geometry

CARLA 0.9.16の`World.cast_ray`と`World.project_point`を同じLiDAR measurement frame中に実行した。`LabelledPoint`にはactor IDがないため、Target判定は「Car labelのhit位置が、scene内でTargetだけのOBBに入る」場合の推定である。

CURRENTのPhase 1b suspicious OBB ray 554本:

| query分類 | ray数 | 割合 |
|---|---:|---:|
| Target first hit推定 | 284 | 51.26% |
| OBB-only候補 | 270 | 48.74% |
| query内未分類 | 0 | 0% |

全554本でquery前後snapshot frameとmeasurement frameが一致。Target first hit推定284本は、hit距離がraw LOW endpointより手前で、矛盾候補としてCSVへ保存した。

ただし、この分類は標準LiDARと完全等価ではない。

- `project_point`: collision channel 2、既定query parameters。
- standard / semantic LiDAR: channel 2、complex trace。
- `cast_ray`: overlap channel 3。

したがって270本は**OBB-only候補**であり、標準LiDARの実collision meshで確定したOBB-onlyではない。標準LiDARと同一trace条件の公開Python APIがないため、厳密な意味では554本すべてにquery-equivalenceの制約が残る。

CURRENTの60 new False Freeセルをquery evidenceで集約:

| 主分類 | セル数 |
|---|---:|
| OBB_AND_TARGET_GEOMETRY（推定） | 37 |
| BOUNDARY_RASTERIZATION | 22 |
| OBB_ONLY候補 | 1 |

この結果では、RSUの原因をOBB過大評価だけでは説明できない。Target先行hitを示すrayとGrid境界影響の両方が大きい。

## G. Semantic LiDAR

利用可能。channels 32、FOV +10/-30°、horizontal FOV 360°、range 500 m、rotation 20 Hz、200,000 points/s、sensor tick 0.05 sを標準LiDARに合わせた。

Semantic LiDARには標準LiDARのdrop-off、intensity drop-off、noise attributesがない。標準LiDARのnoiseは0だがdrop-offは有効。このため一対一対応には使っていない。

| Variant | matched frames | Target object_idx hit |
|---|---:|---:|
| CURRENT | 20 | 3,460 |
| PHYSICS_SETTLE | 20 | 3,840 |
| SETTLED_STATIC | 20 | 3,840 |

RSU方向からTarget collision surfaceが可視である補助証拠。standard suspicious rayとの対応数ではない。

## H. Scenario artifact判定

- Target float: **YES**。CURRENT clearanceは約0.5998 m。
- Ego False Freeへの寄与: **LARGE**。接地後46→0セル、XY_ONLY_BELOW 37→0セル。
- Road改善: 接地後もEgoで4,501セル残る。

したがって、CURRENT Targetに合わせたheight band調整は不適切。

## I. RSU OBB GT判定

- OBBがcollision geometryを過大評価: **YESという証拠あり。ただし確定度は中程度**。
- queryでは554本中270本がOBB-only候補。
- 一方、284本はTarget先行hit推定であり、OBB差だけでは説明できない。
- Grid footprint全体をOccupied GTとする評価も22/60セルへ直接影響した。

RSU False Freeの主因は単一ではなく、Target先行hitを示すray、2D raster GT境界、OBB/collision差候補の混合。

## J. 成果物

- `phase1b5_ground_contact_analysis.json`
- `phase1b5_rsu_geometry_analysis.json`
- `phase1b5_artifact_audit.json`
- `target_ground_clearance.csv`
- `phase1b5_false_free_by_variant.csv`
- `rsu_collision_geometry_rays.csv`
- `rsu_false_free_geometry_cells.csv`
- `target_ground_contact_current_vs_settled.png`
- `ego_false_free_current_vs_settled.png`
- `rsu_false_free_current_vs_settled.png`
- `rsu_collision_geometry_examples.png`
- 各variant配下のpaired結果、ray provenance、semantic hit、settle trace。

## K. テストと不変条件

`python -m pytest -q`: **116 passed / 0 failed**。

- clearance、settle連続判定、transform記録、variant集計。
- query available/unavailable、Target推定、OBB-only候補、unresolved。
- JSON/CSV schema、NaN/Infなし。
- same measurement / same decay / Phase 0 shadow = production。
- 保存measurementのPhase 0 / Phase 1a replay bit-exact。
- PHYSICS_SETTLE / SETTLED_STATIC配列一致。
- production OGM、Phase 0 / Phase 1a algorithm、height slab、weight、threshold、通信、Risk / Priorityは未変更。

## L. 結論と次の1手

**次はscenario接地修正を行う。**

`priority_crossing_v1`のTargetを現実的な接地poseへ直し、その条件を新しい評価baselineとして固定する。今回のsettled zを盲目的に定数化するのではなく、spawn方法・mapごとの接地手順をExperiment Contractへ記録できる形にする。

理由:

1. CURRENTの0.6 mの隙間は明確なscenario artifact。
2. 接地だけでEgo new False Freeが46→0になり、Road改善の97%が残った。
3. 浮いたTargetを基準にheight bandを決めると誤った最適化になる。
4. RSUには接地後も別問題が残るため、接地baseline確定後にGT定義とocclusion-aware carvingを再評価すべき。

Phase 1c、height band、遮蔽制約、GT定義変更は今回実装していない。
