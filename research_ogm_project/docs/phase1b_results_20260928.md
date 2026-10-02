# Phase 1b 結果（2026-09-28）

判定: **Phase 1b 診断 READY**。Phase 1a 採用は引き続き NOT READY。
Free-space algorithm の変更、Phase 1c の実装、Phase 2 への移行は行っていません。

実走先: `D:\CARLA_DATA\outputs\phase1b_priority_crossing_20260928_r1`
成果物: 同ディレクトリの `phase1b_analysis`、再解析用データ: `phase1b_capture`。

## A. 実験条件

| 項目 | 条件 |
|---|---|
| Scenario / Map | priority_crossing_v1 / Town10HD_Opt |
| seed / TM seed | 42 / 42 |
| Ego | Tesla Model 3、(14.13, 69.71, 0.6)、yaw 0.1° |
| Target | Audi TT、(40.39, 41.95, 0.6)、yaw 89.2° |
| RSU | (40.4, 49.0, 3.0)、pitch / yaw 0° |
| Ego LiDAR | relative (10, 0, 2.2)、実測WORLD高さ2.8 m |
| Grid | 300×300、0.2 m、中心(42, 56) |
| fixed dt | 0.05 s |
| measurement数 | Ego 20、RSU 20 |
| t0 | frame 6002、simulation time 59.586726 s |
| z slab | 0.1 < z < 2.0 m |
| Target GT | 242セル、OBB下端0.600000 m、上端1.985296 m |

前回configのSHA256と一致。frame / absolute simulation timeは前回と異なるが、測定列から得た最終4配列は前回とbit-exact。
Actorは既存runnerの設定どおりphysics OFFでwarmup中静止。Target pose変化は0。
サーバー起動直後の初回接続はtimeoutとなり、起動完了後にr1へ再実行した。初回出力先には解析データはない。

## B. new Target False Free再現

定義: `target_gt & (~phase0_known_free) & phase1a_known_free`。

| OGM | 期待値 | 実測 | Target False Free Phase 0 → 1a |
|---|---:|---:|---:|
| Ego | 46 | 46 | 13 → 59 |
| RSU | 60 | 60 | 51 → 111 |

前回のEgo/RSU × Phase0/Phase1aの4配列と `np.array_equal=True`。
保存measurementから元の更新関数で再生した4配列も実走配列と一致。
LOW/HIGH別更新数合計、Target全provenanceの件数もPhase 1aの更新カウントに一致。

## C. Cause分類

1セルに複数分類のrayがあれば、更新ray数が最多の分類を主分類とした。全分類のray数はCSVに保存。

| 主分類 | Egoセル（割合） | RSUセル（割合） |
|---|---:|---:|
| 3D_INTERSECTION | 9 (19.57%) | 58 (96.67%) |
| XY_ONLY_ABOVE | 0 | 0 |
| XY_ONLY_BELOW | 37 (80.43%) | 0 |
| RASTERIZATION_OR_EDGE | 0 | 2 (3.33%) |
| UNRESOLVED | 0 | 0 |

RSUの主分類が0でも、ABOVE/BELOW ray自体が0という意味ではない。RSUにはABOVE 73本、BELOW 53本、XY直接交差なし256本が存在する。
これらは3D交差rayと同じセルへ更新することがある。

GT rasterizationの独立指標として、新規セル中心が連続座標のfootprint外となるセルはEgo 5、RSU 22。
主分類とこの指標は重複する。主分類のRASTERIZATION_OR_EDGEだけでartifact総量を表してはいない。

## D. Endpoint分類

| OGM | LOW由来セル | LOW原因ray | HIGH由来セル | HIGH原因ray |
|---|---:|---:|---:|---:|
| Ego | 46 | 61 | 0 | 0 |
| RSU | 60 | 936 | 0 | 0 |

rayは `sensor:frame:raw_point_index` で重複排除。全endpointはWORLD z ≈ 0.0002 m。
hit actor / material はraw LiDARから取得できないため、ground returnとは断定していない。

LOW × 原因のunique ray / touched cell:

| OGM | 3D交差 | XY-only上方 | XY-only下方 | XY直接交差なし |
|---|---:|---:|---:|---:|
| Ego | 11 / 9 | 0 / 0 | 50 / 37 | 0 / 0 |
| RSU | 554 / 60 | 73 / 10 | 53 / 7 | 256 / 5 |

HIGHの全クロス集計は0。touched cellは分類間で重複する。

## E. 3D交差・occlusion ordering

| OGM | OBB交差ray | endpointがentryより遠い | endpointがexitより遠い |
|---|---:|---:|---:|
| Ego | 11 | 11 | 11 |
| RSU | 554 | 554 | 554 |

有限線分のOBB交差は確認できた。ただしOBBは車体を囲む箱でありcollision meshではないため、実車体の透過は未証明。

- Ego交差rayのbox entry zは0.606882 m、exit zは0.600000 m。箱の底辺付近をかすめ、箱表面からの最大内側距離は約6.34 mm。
- RSU交差rayのentry z中央値は1.824308 m、exit z中央値は1.487989 m。最大内側距離の中央値は0.112202 m、最大0.270931 m。
- OBB再構成頂点とCARLA verticesの最大差は約1.86 µm。
- 座標変換・poseの不一致は今回確認されていない。OBBとcollision meshの差は、このデータだけでは分離できない。

## F. Targetセル付近のray高さ

cell centerへXY距離が最小となる有限ray parameterで計算。単位m、ray × cellごとの分布。

| OGM / cause | min | median | p95 | max |
|---|---:|---:|---:|---:|
| Ego全体 | 0.283805 | 0.422038 | 0.545539 | 0.556096 |
| Ego 3D交差 | 0.352382 | 0.417779 | 0.556096 | 0.556096 |
| Ego下方 | 0.283805 | 0.427403 | 0.534081 | 0.554379 |
| RSU全体 | 0.056153 | 1.489981 | 1.954126 | 2.019361 |
| RSU 3D交差 | 0.127003 | 1.526359 | 1.904505 | 2.014079 |
| RSU上方 | 1.908870 | 1.978887 | 2.011354 | 2.011354 |
| RSU下方 | 0.056153 | 0.191988 | 0.344930 | 0.344930 |
| RSU境界等 | 0.090701 | 0.969710 | 1.938474 | 2.019361 |

3D_INTERSECTIONはray全体の判定であり、更新された各セルでbox内を通ることを意味しない。
Egoの577 ray-cell更新はすべてTarget下端0.6 mより下。交差rayも、箱底をかすめた後に下側セルを更新している。
現在のTarget配置z=0.6、physics OFFと、slab下端0.1の組合せにより、車両下方空間がFree対象になることが主要なscenario依存要因。

セル中心への最近点はslab区間に制限していないため、RSUの一部のz値はslab外になる。これは更新algorithmの高さ変更ではない。
連続slab線分が更新セル矩形に交差しないray-cell行はEgo 43、RSU 465。Bresenham離散化の差として記録した。

## G. Evidence強度

| 指標 | OGM | min | median | p95 | max |
|---|---|---:|---:|---:|---:|
| extra ray / update count | Ego | 10 | 12 | 15 | 15 |
| extra ray / update count | RSU | 8 | 33 | 245.35 | 275 |
| extra log-odds総和 | Ego | -3.010060 | -2.408048 | -2.006707 | -2.006707 |
| extra log-odds総和 | RSU | -8.277666 | -0.993320 | -0.328097 | -0.240805 |

log-odds総和は入力evidenceの和。decay / clipping後の差分ではない。
単発rayのみではなく繰り返し更新。幾何形状を座標小数点5桁で同一視するとEgoは5種類、RSUは85種類なので、単純な観測本数の条件だけでは独立した証拠を保証しない。

## H. LOW / HIGH trade-off

同一valid-return、decay、clippingを使ったLOW-only/HIGH-only診断再生結果。

| OGM / endpoint | extra ray数 | extra unique cells | Road Unknown→Free | Target新規False Free |
|---|---:|---:|---:|---:|
| Ego LOW | 39,736 | 16,891 | 4,641 | 46 |
| Ego HIGH | 0 | 0 | 0 | 0 |
| RSU LOW | 38,778 | 20,841 | 3,713 | 60 |
| RSU HIGH | 0 | 0 | 0 | 0 |

HIGH return自体はEgo 34,052、RSU 34,324ある。しかしsensor高さ2.8/3.0とHIGH endpointはいずれもslab上端2.0以上なので、線分はslab内部に入らない。
したがってLOW限定・HIGH除外は今回の問題にも改善量にも効果がない。
EgoのRoad Unknown純減は4,640。Unknown→Free 4,641にOccupied→Unknown 1が加わるため差が1ある。

## I. Sensor timing

全40測定でmeasurement frame = world snapshot frame。
使用pose = measurement.transform、最大並進差0 m、行列差0。
measurement poseによる再計算でOBB判定が変わるrayは0。
Targetも静止しているため、今回の原因をtiming差で説明する証拠はない。移動中の別scenarioへ一般化はしない。

## J. 成果物

`phase1b_analysis/`:

- `phase1b_false_free_analysis.json`: 原因・LOW/HIGH・高さ・更新強度・再生一致。
- `phase1b_artifact_audit.json`: 前回一致、境界深さ、GT境界、有限値検証。
- `target_false_free_cells.csv`: 106セル。
- `extra_free_rays_target.csv`: 4,365 ray-cell行。
- `occlusion_ordering.csv`: OBB交差565 ray。
- `sensor_pose_timing.csv`: 40測定。
- `ego_target_false_free_analysis.png`, `rsu_target_false_free_analysis.png`。
- `ego_representative_ray_sections.png`, `rsu_representative_ray_sections.png`。
- 新規False Free mask、LOW/HIGH別log-odds / 更新カウント配列。

代表断面はrayを含む鉛直平面による実際のOBB切断。箱全体の投影で見かけ上交差を作らない。

## K. テストとコード

コマンド: `python -m pytest -q`。最終結果: **97 passed / 0 failed**（12.01秒）。
OBB通常交差、上下、非交差、有限終端、actor/box回転・offset、CARLA頂点照合、LOW/HIGH、集合定義、集計、NaN/Inf、ON/OFF配列一致、保存データからの全解析、断面形状をテスト。

新規: `phase1b_geometry.py`, `analyze_phase1b.py`, `audit_phase1b_artifacts.py`, Phase1bテスト2本、運用説明・本報告。
変更: `run_real_priority_integration.py` に診断flagとmeasurement保存のみ追加。
Phase 0 / Phase 1a更新関数、既存height slab、閾値、weight、decay、通信、Risk / Priority、契約は変更なし。

## L. 結論

1. Egoは車両下方rayの2D投影が主因。箱に触れるrayも底面の非常に浅い交差であり、更新セル付近はすべて箱下端より下。
2. RSUはOBBを通過するLOW rayと、footprintの境界をGridセルへ広げる評価の影響が大きい。実車体透過か、boxと実形状の差かは未確定。
3. False FreeもRoad Unknown改善もLOW由来。HIGH除外では変化しない。今回のpose timing差は0。

Phase 1bの要求する再現・provenance・3D/XY区別・LOW/HIGH trade-off・OGM非変更を確認した。

## M. 次の修正候補（未実装）

最小変更の次の実験として **C: extra Free用height bandの縮小** を推奨する。
legacy valid-returnとOccupied slabは維持し、追加Freeだけに独立した高さ帯を設ける案。
下限で車両下方の投影を抑え、上限で高い位置を通るrayの寄与を抑える可能性を調べる。

Egoの原因ray-cell高さは全て0.5561 m以下、RSU中央値は1.4900 mであり、LOW/HIGHより高さに診断上の分離余地がある。
ただしRSUの全False Freeを除去できる証拠も、Road改善を十分保てる証拠も現時点ではない。上下限は未決定であり、特定TargetのGTをオンライン制約には使わない。
次段階では保存measurementによるpaired比較でTarget False Free、Road Unknown→Free、境界影響を同時に検証する。
複数ray要求は既に反復観測が多く、LOW/HIGH除外は効果0なので優先しない。3D保持後の2D投影は本質的な改善候補だが変更規模が大きい。
