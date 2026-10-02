# Communication Priority Map (Phase 2)

Risk Score、Ego Unknown、RSU Known、Road を同じ WORLD Grid 上で積算する、独立したオフライン処理です。
OGM の値を書き換えず、送信処理にも Priority を渡しません。

```text
Priority = Risk × EgoUnknown × RSUKnown × Road
baseline_candidate = EgoUnknown & RSUKnown & Road
```

## 実装ファイル

- `src/ogm_project/grid_contract.py`: 空間契約、Risk v1 metadata 変換、完全な Grid 比較。
- `src/ogm_project/risk_priority.py`: log-odds 検証、Known 判定、Road metadata 検査、Priority・集計・可視化。
- `src/ogm_project/priority_snapshot.py`: 任意 debug snapshot。Ego と送信前 RSU local を保存。
- `scripts/build_communication_priority_map.py`: オフライン CLI。CARLA の import / 接続は不要。
- `tests/test_risk_priority.py`: 単体・snapshot・CLI E2E・既存関数の変更防止テスト。

Python 3.10 以上、NumPy、Matplotlib が必要です。テストには pytest。
Coop の実行・snapshot 取得には、従来どおり CARLA 環境が必要です。

## オフライン生成

本プロジェクトのルートで実行します。

```powershell
python scripts/build_communication_priority_map.py `
  --ogm-dir "D:\path\to\coop_run\priority_debug" `
  --risk-dir "D:\path\to\risk_map" `
  --output-dir "D:\path\to\priority_map" `
  --require-experiment-match
```

`--require-experiment-match` を指定すると、後述の実験条件が不足する場合も停止します。
省略時は Grid だけの検証が可能ですが、実験条件が不足すれば `experiment_compatible=null` / `unverified` と記録します。
明示された実験条件に不一致があれば、どちらのモードでも停止します。
空間情報の不足・不一致は常に `GridMismatchError` で停止します。再投影・平行移動・転置・shape による補完は行いません。
すべての入力検証を終えてから出力先を作成します。同名出力は再実行で上書きします。

### 必須入力

```text
risk_map/
  risk_aggregate.npy        float32, (ny,nx), finite, [0,1]
  metadata.json             既存 Heatmap v1 の metadata

priority_debug/
  ego_logodds.npy            finite 2D floating array
  rsu_logodds.npy            RSU local（受信後配列ではない）
  ego_unknown.npy            bool
  rsu_known.npy              bool
  road_mask.npy              bool
  ogm_grid_metadata.json
  mask_metadata.json
```

保存済み Unknown / Known と log-odds から再計算した結果の完全一致も確認します。
閾値は標準 Coop と共有する既存 `ogm_project.logodds.OCC_TH=0.60` / `FREE_TH=0.48` を使用します。
`p >= .60 または p <= .48` が Known、Ego はその反転が Unknown です。
Risk は Risk Score、log-odds から変換する p は OGM の占有値であり、別レイヤです。

## Grid 契約

共通 `GridContract` は frozen dataclass。schema_version は文字列 `"1.0"`。

```text
schema_version, map_name, frame, resolution_m, nx, ny,
x_min_m, x_max_m, y_min_m, y_max_m, center_x, center_y,
array_axis_0, array_axis_1, cell_coordinate
```

- `frame=CARLA_WORLD`、`array_axis_0=WORLD_Y`、`array_axis_1=WORLD_X`、`cell_coordinate=center`。
- 境界は絶対 WORLD [m]、配列は `[iy,ix]`。幅・高さは解像度×セル数、中心は境界の中点。
- map 名は正確に比較します。runtime export は実際の `world.get_map().name` の短名を取得します。
- 浮動小数点の空間値比較は絶対許容差 1e-8、相対許容差 0。1セルずれ等は受理しません。
- OGM metadata は実行時 origin と相対境界も保存します。`origin + relative bound = WORLD bound` を検査します。
  origin は基準点、center は幾何学中心です。現在の対称±30 mでは一致しますが、一般には別です。
- Risk v1 は既存の `array_convention` / `cell_sampling` / `bounds_convention` の既知の文字列を明示的に変換します。
  欠落・未知の規約を推定で補いません。既存 Heatmap のコード・metadata の書換えは不要です。

標準 Coop の `world_to_grid` は引き続き int 切り捨てです。
そのため下限のわずか外側の点が index 0 に入る既存の制約は残ります。
本契約はセルが表す WORLD 空間を検証します。既存 OGM の観測値の正確さまで保証するものではありません。

## Road Mask と provenance

標準の mask 生成経路は `scripts/build_static_mask.py → legacy_runner.run_static_mask → legacy/build_static_mask_from_hdmap_V3.py`。
生成コードはセル中心で `get_waypoint(project_to_road=False)` が None のとき True。
ローカル CARLA の API 定義では省略された lane_type の既定値は Driving です。
したがって、その生成仕様が確認できた Static Mask なら `road = ~static_mask` とします。
既存ソース内の「任意 lane」のコメントだけを根拠にはしていません。

**現存する `D:\CARLA_DATA\masks\static_mask.npy` には metadata がありません。Phase 2 では受理できません。**
shape が 300×300 でも、生成時 map・origin・時刻が不明なファイルへ推測で sidecar を付けないでください。
元の生成記録を確認するか、目的の Grid 上で mask を生成し、生成時の実値を sidecar に記録してください。
既存 mask 生成ソース・legacy・従来の mask loader は今回変更していません。

sidecar は GridContract の全項目に加え、以下を必須とします。

```text
mask_semantics: "static_non_driving_true" または "road_true"
array_sha256: 対象 .npy ファイルの SHA-256（ファイルのバイト列）
provenance: 生成方法・根拠を示す空でない文字列
```

`static_non_driving_true` は反転、`road_true` はそのまま使用します。
Grid 不一致、sidecar 不在、未知の意味、hash 不一致、bool 以外はエラーです。
未確認の mask を全道路扱いにする fallback はありません。

## 任意の Coop snapshot

検証済み mask と sidecar を用意して、標準 entrypoint に flag を渡します。
`cooperative_runner.py` の既存 passthrough が本体へ転送します。

```powershell
python scripts/run_coop_comm.py --scenario scenario_A `
  --save-priority-debug `
  --priority-mask "D:\path\to\verified_static_mask.npy" `
  --priority-mask-metadata "D:\path\to\verified_mask_metadata.json" `
  --priority-target-id "target_01" `
  --priority-time-reference-id "shared_experiment_clock_id"
```

- flag を省略したときは従来動作。debug 用 lock・コピー・NPY保存を行いません。
- `--priority-mask` 省略時は標準 `STATIC_MASK_PATH`。metadata 引数は debug 時必須。
- sidecar と hash を接続前に検査し、実 WORLD/map/origin 確定後に Grid 一致を検査します。
- Ego / RSU callback 完了後の frame と timestamp が両方一致したときだけ、一度保存します。
  一致しなければ後続 snapshot 時点または終了時に再試行し、最後まで揃わなければ未保存を表示します。
- 配列更新とコピーだけを debug lock で保護し、保存はコピーに対して実行します。
- 保存先は `<Coop run>/priority_debug/`。上記必須7ファイルを生成します。
- RSU は local 配列を使用します。量子化・マスク処理・疑似通信後の受信配列と混同しないでください。
- USE_STATIC_MASK の値にかかわらず、Priority debug には検証済みの道路情報が必要です。
- debug の同期・I/Oには実行負荷があり、latency評価用機能ではありません。

## 実験条件の照合

Risk / OGM metadata の `experiment` オブジェクトで、空間契約とは別に以下を比較します。

```text
scenario_id
scenario_sha256       scenario定義ファイル全体のSHA-256
ego_initial_pose      x,y,z,yaw の finite 数値
target_id
time_reference_id
reference_time_s     同じ実験時計でのRisk予測起点/OGM観測時点
```

snapshot は選択シナリオ、scenarioファイルhash、Ego初期姿勢、センサーtimestampを記録します。
Targetは自動推定せず、未指定なら null。時計IDの既定値は run の絶対パスで、別runが自動的に一致することはありません。
既存Risk v1 metadataにはこれらの実験情報がないため、現在の実runをscenario一致と認定できません。
今後、同じ実験からRiskを生成した時点で根拠のある情報を記録してください。IDだけを後付けで揃えないでください。

## 出力

```text
priority_map.npy               float32, (ny,nx), finite, [0,1]
priority_map.png               WORLD XY / aspect=equal
priority_components.png        Risk, Unknown, Known, Road, Priority, Baselineを同じextentで表示
priority_metadata.json         Grid、式、閾値、入力パス、Risk hash、mask根拠、実験照合結果
priority_summary.json          各mask件数、候補件数、最大値、正値平均、Grid/実験照合結果
baseline_candidate_mask.npy    bool
```

`reduction_ratio = 1 - priority_positive_cells / baseline_candidate_cells`。
baseline が 0 なら null、Priority正値がない場合の平均は 0 とします。
これは **候補セル数削減率** です。payload を変更していないので、送信量削減率ではありません。

## 検証

```powershell
python -m pytest tests/test_risk_priority.py -q
```

合成300×300 E2Eは、Risk中央20×20、Unknown中央30×30、Known右半分、Road横10行。
4条件の交差は100セル、baselineは150セルです。画像は同一 WORLD extent の8パネルで生成します。
実データの制約・変更範囲・最終判定は `AUDIT_communication_priority.md` を参照してください。

**Risk-based selective transmission は未実装です。**
Top-K、payload、encode_grid_q8、zlib、SimChannel、遅延・損失評価、ns-3 は変更していません。

## 実 CARLA 統合検証

同一シナリオ・同一基準 frame・同一 WORLD Grid で Future Risk、Ego/RSU OGM、Road Mask を接続した検証結果は `REAL_CARLA_INTEGRATION_REPORT.md` に記録しています。再現用 entrypoint は `scripts/run_real_priority_integration.py`、設定は `configs/priority_crossing_v1.json` です。
