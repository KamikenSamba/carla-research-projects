# Common OGM Engine

Phase 1b.6 では、Priority 用 standard Cooperative OGM の Ego / RSU 更新を `ogm_project.ogm_engine` に統一した。これは refactor only であり、Phase 1c の対策、height band、scenario 接地、Free scale比較は含まない。

```text
                         LiDAR measurement
                    raw parse / WORLD transform
                                  |
                  +---------------+---------------+
                  |                               |
              Ego points                      RSU points
              Ego config                      RSU config
                  |                               |
                  +---------------+---------------+
                                  |
                         Common OGM Engine
                z-filter / ray trace / cell updates
                                  |
                    float32 log-odds / label masks
```

## 責務

`OGMUpdateConfig` は frozen dataclass で、既存定数から次を注入する。

- z のexclusive下限・上限
- LiDAR range
- Free / Occupied log-odds
- Free scale
- clipping上下限
- Known Free / Occupied threshold
- decay rate と初期log-odds

`OGMGridContract` は現行standard Cooperative実装の `world_to_grid`、`in_bounds`、`bresenham` をそのまま受け取る。丸め、境界、tie-breakを再実装していない。

主APIは以下である。

- `update_lidar_measurement`: 共通z-filter、valid-return更新、任意のPhase 1a rejected-return Free
- `update_valid_returns`: intermediate Free / endpoint Occupied
- `update_extra_free_from_rejected_returns`: height slab内のextra Freeのみ
- `apply_decay`: 現行のin-place線形decayとclipping
- `probability_from_logodds` / `known_masks` / `classify_logodds`: 共通ラベル判定

Engine APIには `sensor_name` がない。したがって同じpoints、origin、initial grid、Config、decayなら、Ego/RSUという名称による出力差は構造上発生しない。

## Ego / RSUで異なるもの

現行standard Cooperative Configの差は次の2点だけである。

| Config | Ego | RSU |
|---|---:|---:|
| `free_scale` | 1.0 | 0.15 |
| `decay_rate` | 0.4 /s | 0.3 /s |

sensor pose、origin、point cloudはmeasurement input差であり、Config内に固定しない。他のz-filter、range、increment、clipping、thresholdは同じである。実走時の差分は各runの `ogm_engine_shadow/config_diff.json` にも保存する。

## callback境界

callbackはraw `float32` parsing、既存 `transform_to_world` 呼出し、measurement時のorigin取得、可視化用point capture、diagnostic hookを担当する。z判定とlog-odds更新はCommon Engineを通る。local→WORLD変換はEgo/RSUで元から同じ関数であり、数値変更を避けるため移動・書換えをしていない。

Phase 1aのpaired診断もvalid-returnとrejected-return extra FreeをCommon Engineへ委譲する。Road Maskは更新アルゴリズムに入れず、診断とPriorityでのみ使う。

## Shadow migrationとLegacy

`--ogm-engine-shadow-compare` は、同一measurementとdecay dtから以下を並走させる。

- deprecated Legacy Phase0 / Phase1a reference
- Common Engine Phase0 / Phase1a
- production Common Phase0

`shadow_comparison.json` にgrid・label・intermediate counts・communication payload・fusion・EgoUnknown・RSUKnown・Priorityの一致を保存する。`legacy_update_from_points_reference` と `legacy_decay_logodds_reference` は回帰用として残してあり、production callbackからは呼ばない。

Ego-only `SparseWorldOGM` は別契約（疎WORLD grid、異なるz上限/range/clipping/decay timing）なのでPhase 1b.6のmigration対象外である。詳細は [変更前監査](ogm_engine_unification_audit.md) を参照する。
