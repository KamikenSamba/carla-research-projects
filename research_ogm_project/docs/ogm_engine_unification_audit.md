# Phase 1b.6: Common OGM Engine 変更前監査

作成日: 2026-09-29  
監査対象: 実コード（設計上の想定ではなく、現在の呼出し経路）

## 1. 対象経路

Priority 計算で使われる standard Cooperative OGM の実行経路は次の通りである。

```text
scripts/run_coop_comm.py
  -> cooperative_runner.run_coop_comm()
    -> coop_comm_compat.main()
      -> on_ego() / on_rsu()
        -> dense float32 log-odds grid
```

`scripts/run_ego_ogm.py -> ego_ogm_runner -> ego_ogm_compat` は Ego-only 可視化用の別経路である。こちらは `SparseWorldOGM`、WORLD 原点基準の無限疎グリッド、`z_max=1.00`、`range=100 m`、`[-1.5, 1.5]` clipping、固定 simulation tick decay を使う。Priority の dense Cooperative Ego OGM（`z_max=2.00`、`range=500 m`、`[-4, 4]`、callback wall-clock decay）とは契約が異なる。この差は今回勝手に揃えない。Common Engine の production migration 対象は、研究の EgoUnknown / RSUKnown を生成する standard Cooperative OGM の Ego/RSU 二経路とする。Ego-only 疎グリッドは監査対象だが未変更とする。

## 2. Cooperative Ego / RSU 差分表（変更前）

分類は `Same`、`Different by parameter`、`Different implementation` のいずれかである。

| # | 処理 | Ego 実装 | RSU 実装 | 分類 | 統一方法 |
|---:|---|---|---|---|---|
| 1 | raw LiDAR parsing | `np.frombuffer(..., float32).reshape(-1,4)[:, :3]` | 同左 | Same | callback に残す |
| 2 | sensor-local → WORLD | `transform_to_world(arr, lidar_tf)` | 同じ関数 | Same | 数式・dtype・順序を保持し callback に残す |
| 3 | sensor origin | measurement 時の Ego LiDAR transform | measurement 時の RSU LiDAR transform | Different by parameter | `(x,y,z)` input として Engine へ渡す |
| 4 | WORLD z-filter | `z > 0.10 & z < 2.00` | 同左 | Same | Config を用いる共通関数へ移す |
| 5 | world_to_grid | `coop_comm_compat.world_to_grid` | 同じ関数 | Same | callable を含む Grid contract として渡す |
| 6 | bounds | `coop_comm_compat.in_bounds` | 同じ関数 | Same | 同じ callable を使う |
| 7 | Bresenham | `coop_comm_compat.bresenham` | 同じ関数 | Same | 同じ callable を使う。tie 条件を変更しない |
| 8 | Free update | `cell += 1.0 * L_FREE` | `cell += 0.15 * L_FREE` | Different by parameter | `config.free_scale` のみを変える |
| 9 | Occupied update | endpoint に `L_OCC` | 同左 | Same | Common Engine |
| 10 | Free scale | `1.0` | `0.15` | Different by parameter | frozen Config に明示 |
| 11 | clipping | `np.clip(...,-4,4)` を各更新ごと | 同左 | Same | 演算順と代入をそのまま移植 |
| 12 | decay | callback wall-clock `dt`、rate `0.4` | callback wall-clock `dt`、rate `0.3` | Different by parameter | 共通 `apply_decay`、rate と同一 dt は input |
| 13 | Known Free | sigmoid `p <= 0.48` | 同左 | Same | 共通 classify/masks |
| 14 | Occupied | sigmoid `p >= 0.60` | 同左 | Same | 共通 classify/masks |
| 15 | Unknown | `~(free | occupied)` | 同左 | Same | 共通 classify/masks |
| 16 | Phase 1a extra Free | `Phase1aPairedComparison.update("ego",...)` | 同じ method を `"rsu"` と scale 0.15 で使用 | Different by parameter | rejected-return helper を Engine へ移し、名前分岐を排除 |
| 17 | diagnostic hooks | Phase0 observer + paired shadow | 同左 | Same | callback から観測結果と sensor label を logger のみに渡す |
| 18 | Road mask | update には不使用。診断・Priority のみ | 同左 | Same | Engine へ入れない |
| 19 | callback ordering | decay → parse → transform → z-filter → point capture → update → paired → diagnostics | 同じ順序 | Same | この順序を保持する |
| 20 | sensor-specific special case | direct `update_from_points`、scale 1.0 | `update_from_points_with_origin` wrapper（`static_mask_arr` は未使用）、scale 0.15 | Different implementation | RSU の無作用 wrapper を廃止せず deprecated reference とし、両方 Common API を呼ぶ |

## 3. 差の分類

### A. 意図した parameter 差

- `free_scale`: Ego `1.0`、RSU `0.15`
- decay rate: Ego `0.4 /s`、RSU `0.3 /s`
- sensor origin / pose は Config 定数ではなく measurement input
- point cloud は各 sensor の実観測 input

z-filter、log-odds increment、clipping、range、threshold は Cooperative Ego/RSUで同じであり、差として残さない。

### B. sensor pose / input 差

- Ego LiDAR は vehicle attach、RSU LiDAR は world fixed actor
- callback ごとに `get_transform()` した行列、origin、raw returns が異なる
- 変換関数そのものは既に同一

### C. 不要な implementation 差

- RSU だけ `update_from_points_with_origin` を経由するが、`static_mask_arr` は使用されず Ego と同じ関数へ転送される
- callback 内に同一 z-filter と update dispatch が重複する
- `phase1a_paired.py` に valid-return / rejected-return 更新組立てが別置きされている
- log-odds分類は `coop_comm_compat.py` と `logodds.py` に重複する
- Bresenham は `coop_comm_compat.py`、`grid_utils.py`、Ego-only 実装に等価コードがある。今回は standard Cooperative の generator 実装を正とし、丸め・境界を変更しない

## 4. Ego-only 疎グリッドとの差（統合対象外）

| 項目 | Ego-only `ego_ogm_compat` | Cooperative standard | 判断 |
|---|---|---|---|
| grid | `SparseWorldOGM`、WORLD floor cell | 固定 dense grid、origin-relative `int` | 意図が異なるため未変更 |
| z-filter | `(0.10, 1.00)` exclusive | `(0.10, 2.00)` exclusive | 既存値維持 |
| LiDAR range | 100 m | 500 m | 既存値維持 |
| clipping | `[-1.5, 1.5]` | `[-4, 4]` | 既存値維持 |
| decay | tick ごと `dt=0.05`, rate 0.6 | callback wall-clock、Ego 0.4 / RSU 0.3 | 既存時刻方式維持 |
| callback | 最新点群を保持し main tick で一度処理 | sensor callback 内で即更新 | 既存順序維持 |

## 5. migration gate

Production switch は、同じ measurement / initial float32 grid / dt に対する旧実装と Common Engine の次を全て `np.array_equal` で確認した後にのみ行う。

- Ego / RSU Phase0、Phase1a、decay、clipping
- z pass/reject、Free/Occupied update counts、touched masks
- label masks、communication payload bytes、fusion、Priority inputs/output
- current `priority_crossing_v1` CARLA shadow run
- 全 pytest

Legacy 関数は削除せず、deprecated regression reference として残す。Phase 1c、grounding、height band、occlusion、free-scale比較は行わない。
