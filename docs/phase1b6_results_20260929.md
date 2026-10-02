# Phase 1b.6 Common OGM Engine 結果

実施日: 2026-09-29  
scenario: `priority_crossing_v1`（CURRENT、Target/Egoの接地変更なし）

## 判定

- Ego / RSU common implementation: **READY**
- Numerical invariance: **PASS**
- Legacy regression: **PASS**
- Common Engine production switch: **YES**

## Shadow migration

旧productionのまま実施したpre-switch run:

`D:\CARLA_DATA\outputs\phase1b6_shadow_20260929_pre`

Common productionへ切替後のpost-switch run:

`D:\CARLA_DATA\outputs\phase1b6_common_20260929_post`

両runともEgo 20 measurement、RSU 20 measurement。post-switchの全shadow checkはtrueである。

| Check | Ego | RSU |
|---|---:|---:|
| Phase0 grid bit-exact | true | true |
| Phase1a grid bit-exact | true | true |
| Phase0 labels bit-exact | true | true |
| Phase1a labels bit-exact | true | true |
| Phase0 counts identical | true | true |
| Phase1a counts identical | true | true |
| production Phase0 vs Legacy | true | true |

pre-switchとpost-switchの保存Phase0/Phase1a配列も4組すべて `np.array_equal == True` である。

## Intermediate counts（20 measurements合計）

各セルは `Legacy = Common`。

| Sensor / Phase | raw | z pass | z reject | Free updates | Occupied updates | extra Free |
|---|---:|---:|---:|---:|---:|---:|
| Ego Phase0 | 109925 | 34128 | 75797 | 1582442 | 29911 | 0 |
| Ego Phase1a | 109925 | 34128 | 75797 | 2689738 | 29911 | 1107296 |
| RSU Phase0 | 109953 | 34843 | 75110 | 1834361 | 31591 | 0 |
| RSU Phase1a | 109953 | 34843 | 75110 | 2973048 | 31591 | 1138687 |

## Downstream invariants

- `encode_grid_q8` / zlib input and payload bytes: bit-exact true
- fusion（Ego Known優先、Ego UnknownのみRSU Known）: bit-exact true
- EgoUnknown: identical true
- RSUKnown: identical true
- actual Risk/Roadを使ったPriority: bit-exact true
- production Common Phase0 vs deprecated Legacy: Ego/RSUともbit-exact true

## Phase 1b / 1b.5 preservation

CURRENT scenarioの保存measurement/replayで次を再現した。

| Sensor | Target Phase0 False Free | Target Phase1a False Free | new False Free |
|---|---:|---:|---:|
| Ego | 13 | 59 | **46** |
| RSU | 51 | 111 | **60** |

Phase 1b解析のEgo 46 / RSU 60と一致する。scenario z、GT、height slabは変更していない。

## Tests

`python -m pytest -q`: **129 passed, 0 failed**（production switch後）。

## 次段階

Phase 1b.6はREADYであるため、次は1要因ずつ進める原則に従い、**Phase 1b.7: Grounded Scenario Baseline確定**を提案する。本Phaseでは接地修正を実施していない。

