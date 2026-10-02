# Phase 1c-A: Grounded RSU extra-Free height-band offline sweep

## 結論

正式 Grounded baseline の保存 measurement だけを用い、production OGMを変更せずに
RSUの z-rejected return 由来 extra Free height bandを比較した。
0.10–0.85 m は Road benefitを91.42%維持しながら new Target False Freeを
111から31へ72.07%削減し、0.10–0.65 m は82.96%維持で19へ82.88%削減した。
一方、探索範囲内の最小False Freeは12であり、0および10以下は達成できなかった。
残存12セル中10セルは3D_INTERSECTIONであるため、height band制限だけでは
False Free問題を完全には解決できない。

本結果は候補探索であり、production bandは0.10–2.00 m相当のまま変更していない。
CARLA実走、Priority再計算、GTを用いたonline制約も実施していない。

## A. Baseline reproduction

| 項目 | 期待値 | 実測値 |
|---|---:|---:|
| official Phase 1a log-odds array | bit-exact | True |
| Road Unknown → Free | 4,091 | 4,091 |
| Target new False Free | 111 | 111 |
| Occupied → Unknown | 0 | 0 |
| Occupied → Free | 0 | 0 |

同一candidateの2回replayで、log-odds、extra update count、classification、metricsは完全一致した。

## B. Sweep条件

- 入力: `D:\CARLA_DATA\outputs\priority_crossing_v2_grounded_20260929_r2`
- Scenario: `priority_crossing_v2_grounded` version 2.0
- Config SHA-256: `97713a71aeae0a76ad59ae4e3b67bd9b80c7bb48188cbbba0aea688760433683`
- RSU measurement: 保存済み20フレーム、CARLA再実行なし
- Coarse: z_min=0.10–1.50 m、z_max=0.50–2.00 m、0.10 m刻み、z_min<z_max（174候補）
- Fine: coarse Pareto/category anchorの±0.05 m、0.025 m刻み（50追加候補）
- Fine anchor: (0.10,0.60), (0.10,0.90), (0.10,2.00), (0.40,0.50)
- 重複除去後のcandidate総数: 224
- 不変値: free_scale=0.15、decay_rate=0.3、valid-return/Common Engine/threshold/clipping/Bresenham/Grid Contract

Grounded Target OBBは bottom=-0.011287 m、top=1.374009 m。OBBは評価・物理解釈だけに使い、
ray carvingや候補生成には使用していない。

## C. Pareto frontier

| band z_min–z_max [m] | Road U→F | retention | Target new FF | FF reduction |
|---|---:|---:|---:|---:|
| 0.45–0.50 | 889 | 21.73% | 12 | 89.19% |
| 0.35–0.50 | 1,619 | 39.57% | 14 | 87.39% |
| 0.35–0.525 | 1,774 | 43.36% | 15 | 86.49% |
| 0.30–0.50 | 1,919 | 46.91% | 16 | 85.59% |
| 0.20–0.50 | 2,534 | 61.94% | 17 | 84.68% |
| 0.15–0.65 | 3,176 | 77.63% | 18 | 83.78% |
| 0.10–0.65 | 3,394 | 82.96% | 19 | 82.88% |
| 0.10–0.70 | 3,470 | 84.82% | 20 | 81.98% |
| 0.10–0.80 | 3,616 | 88.39% | 28 | 74.77% |
| 0.10–0.85 | 3,740 | 91.42% | 31 | 72.07% |
| 0.10–0.925 | 3,750 | 91.66% | 35 | 68.47% |
| 0.10–0.95 | 3,814 | 93.23% | 37 | 66.67% |
| 0.10–1.10 | 3,902 | 95.38% | 57 | 48.65% |
| 0.10–1.20 | 3,943 | 96.38% | 73 | 34.23% |
| 0.10–1.30 | 3,970 | 97.04% | 85 | 23.42% |
| 0.10–1.40 | 4,019 | 98.24% | 103 | 7.21% |
| 0.10–2.00 | 4,091 | 100.00% | 111 | 0.00% |

Frontierは、FF=12から19の領域ではRoad benefitが889から3,394へ急増し、その後は
benefit追加量に対してFF増加が大きくなる形である。特に0.10–0.65 m付近が屈曲点に近い。

## D. Maximum safety candidate

FF=0の候補はない。全224候補での最小値は次のとおり。

| band | Road U→F | retention | Target new FF |
|---|---:|---:|---:|
| 0.45–0.50 m | 889 | 21.73% | 12 |

## E. High safety candidate

FF≤10を満たす候補はない。したがって該当candidateなし。

## F. 90% benefit

retention≥90%でFF最小は0.10–0.85 m。Road U→F=3,740（91.42%）、
Target new FF=31（72.07%削減）。

## G. 80% benefit

retention≥80%でFF最小は0.10–0.65 m。Road U→F=3,394（82.96%）、
Target new FF=19（82.88%削減）。

## Threshold summary

| 最低retention | 最小FF | band | Road U→F | 実retention |
|---|---:|---|---:|---:|
| 100% | 111 | 0.10–2.00 | 4,091 | 100.00% |
| 95% | 57 | 0.10–1.10 | 3,902 | 95.38% |
| 90% | 31 | 0.10–0.85 | 3,740 | 91.42% |
| 80% | 19 | 0.10–0.65 | 3,394 | 82.96% |
| 70% | 18 | 0.15–0.65 | 3,176 | 77.63% |
| 50% | 17 | 0.20–0.50 | 2,534 | 61.94% |

| FF上限 | 最大Road U→F | band | retention |
|---|---:|---|---:|
| 0 | なし | — | — |
| 5 | なし | — | — |
| 10 | なし | — | — |
| 25 | 3,470 | 0.10–0.70 | 84.82% |
| 50 | 3,814 | 0.10–0.95 | 93.23% |

閾値は「以上」で評価するため、70%条件の選択結果が77.63%、50%条件が61.94%となる。

## H. Cause分類

| candidate | new FF | 3D_INTERSECTION | XY_ONLY_ABOVE | XY_ONLY_BELOW | RASTERIZATION_OR_EDGE |
|---|---:|---:|---:|---:|---:|
| current 0.10–2.00 | 111 | 59 | 50 | 0 | 2 |
| high benefit 0.10–0.85 | 31 | 29 | 0 | 0 | 2 |
| balanced 0.10–0.65 | 19 | 17 | 0 | 0 | 2 |
| minimum FF 0.45–0.50 | 12 | 10 | 0 | 0 | 2 |

上限を下げることでXY_ONLY_ABOVEは50から0へ抑制できる。一方、残存FFはほぼ
3D_INTERSECTIONであり、狭いbandでも残る。

## I. Occupied safety

全224候補で Occupied→Unknown=0、Occupied→Free=0、Phase0 Free→candidate Unknown=0。
主要candidateの `candidate_logodds - phase0_logodds` は全セルで0以下で、
正方向deltaは0セルだった。NaN/Infもない。

valid Free update数は全候補1,835,443、Occupied endpoint update数は全候補31,779で一致し、
height bandがvalid-return処理へ影響していない。

## J. 可視化と保存物

出力先:
`D:\CARLA_DATA\outputs\phase1c_height_band_sweep_grounded_20260929`

- `phase1c_height_band_sweep.csv/json`: 224候補の全指標
- `phase1c_pareto_frontier.csv`: Pareto 17候補
- `phase1c_candidate_summary.json`: baseline、A–D、threshold、原因分類、OBB関係
- `road_unknown_to_free_heatmap.png`
- `target_false_free_heatmap.png`
- `road_benefit_retention_heatmap.png`
- `false_free_reduction_heatmap.png`
- `phase1c_pareto_plot.png`
- current / high_benefit / balanced / minimum_false_free のlog-odds NPYとPhase0差分NPY

## K. Tests

`python -m pytest -q`: **143 passed, 0 failed**。

追加testは、公式0.10–2.00 bit-exact、反復determinism、valid/Occupied update invariant、
Target/Road mask定義、NaN/Inf、delta非正、band validation、normalized metrics、
Pareto dominance/frontier、threshold summaryを検証する。既存testはheight slab内外、
production Common Engine、communication、fusion、Priorityの不変性を引き続き検証する。

## L. Production status

- Production band変更: **NO**
- Production OGM: **UNCHANGED**
- Grounded config SHA-256: 指定値と一致
- CARLA Phase 1c-B run: **未実施**

実装は `src/ogm_project/phase1c_height_band.py` と
`scripts/sweep_phase1c_height_band.py` のoffline専用経路に限定した。

## M. Phase 1c-A判定

- Offline height-band sweep: **READY**
- Pareto analysis: **READY**
- Promising candidates: **YES**

有望候補は、risk削減寄りの0.10–0.65 mとbenefit維持寄りの0.10–0.85 m。
ただし単一winnerには決定しない。

## N. 次Phase提案

Phase 1c-Bとして、Grounded real CARLA runで0.10–0.65 mと0.10–0.85 mを
現行0.10–2.00 m controlとpaired validationすることを提案する。ただし本Phaseでは
採用・実走しない。

同時に、FF=0/≤10をheight bandだけで達成できず、残存原因が3D_INTERSECTION主体であるため、
Phase 1c-B後の別方式候補としてocclusion-aware carvingを検討すべきである。

## 最終質問への回答

1. 大幅削減と維持を両立するbandは存在する。0.10–0.85 mはbenefit 91.42%でFF 72.07%削減、
   0.10–0.65 mはbenefit 82.96%でFF 82.88%削減。
2. FF=0は達成不能。最小は12、Road improvementは889。
3. benefit≥90%の最小FFは31（0.10–0.85 m）。
4. benefit≥80%の最小FFは19（0.10–0.65 m）。
5. FF≤10を満たす候補はない。
6. Pareto frontierは17点で、FF 12–19でbenefitが急増し、その後はFFコストが増える。
7. height bandはXY_ONLY_ABOVEを抑えるには有効だが、3D_INTERSECTIONを残すため単独解決には不十分。
