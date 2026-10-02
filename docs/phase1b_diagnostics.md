# Phase 1b: Target False Free の原因診断

Phase 1b は観測データの保存とオフライン解析です。Phase 0、Phase 1a、通信、Risk / Priority の更新処理を変更しません。

## 実行

専用 CARLA サーバー起動後、空の出力先を指定します。

```powershell
python scripts/run_real_priority_integration.py --output-dir D:\CARLA_DATA\outputs\phase1b_run --phase1b-ray-diagnostics
python scripts/analyze_phase1b.py D:\CARLA_DATA\outputs\phase1b_run
```

`--phase1b-ray-diagnostics` は `--phase1a-paired-diagnostics` も有効にします。
保存した measurement は CARLA を再起動せず再解析できます。
旧 `--phase1-free-ray` は valid ray を置換する deprecated な Phase 1 実装であり、この診断には使いません。

## 保存範囲

- `phase1b_capture`: 各 measurement の raw XYZ、実際に使用した WORLD XYZ、measurement pose による比較用 WORLD XYZ、frame / timestamp、使用 pose、measurement pose、world snapshot、actor と bounding box の姿勢、extent、WORLD vertices。
- `phase1b_analysis`: 新規 Target False Free セル、原因 ray × cell、3D交差 ray、sensor timing の CSV、集計 JSON、拡大図、代表 ray 断面図、LOW / HIGH 別の診断配列。

raw measurement は全点を保存します。詳細な ray × cell CSV は最終的な新規 Target False Free セルへ更新した ray だけに限定します。
ray ID は `sensor:measurement_frame:raw_return_index` です。

## 判定と集計の意味

新規 False Free は `target_gt & (phase0_label != Free) & (phase1a_label == Free)` です。
3D判定は有限線分を box local へ逆変換した OBB 交差です。actor と bounding box の位置・回転を合成します。
XY判定は WORLD box vertices の凸包と有限線分の交差です。

セルの主原因は更新 ray 数が最多の分類です。同数の場合は `3D_INTERSECTION`, `XY_ONLY_ABOVE`, `XY_ONLY_BELOW`, `RASTERIZATION_OR_EDGE`, `UNRESOLVED` の順を使います。
全原因の ray 数をセル CSV に残すため、混合原因も確認できます。
3D_INTERSECTION は sensor から endpoint までの線分全体の分類です。その ray が個々の更新セルでも box 内にいるとは限りません。セル付近の `z_at_cell` と併読してください。
ABOVE / BELOW は footprint と重なる線分全体の高さで判定します。傾いた box など、単純な上下関係で説明できない非交差は UNRESOLVED です。

LOW / HIGH の trade-off は診断用再生で測ります。同一 valid return、同一 decay、同一 clipping のまま、診断配列へそれぞれの extra Free のみを加えます。本番・Phase 1a 配列へ書き戻しません。
「当該種類の ray が触れたセル数」と「当該種類だけで閾値を越えたセル数」は別項目です。LOW / HIGH の値は一般には足し算できません。

`extra_logodds_total` は decay / clipping 適用前の入力 evidence 総和です。最終 log-odds 差分ではありません。
高さ分布は ray × cell を1標本とします。ray 数は ID で重複除去します。
代表断面図は ray の鉛直平面で OBB を実際に切断した多角形です。box 全体を平面へ投影した長方形とは異なります。

## 解釈の制約

CARLA actor bounding box は形状を囲む箱であり、LiDAR の collision mesh そのものではありません。
箱の貫通だけでは車体実形状の貫通を証明できません。raw LiDAR は hit actor / material を含まないため LOW を ground と断定しません。
既存の GT rasterization は維持し、cell center が連続座標の footprint 外かどうかを追加記録します。

解析は記録した測定列を元の Phase 1a 関数で再生し、実走保存された Phase 0 / Phase 1a と `np.array_equal` で検証します。
LOW / HIGH 別の更新カウント合計と、全 Target 詳細 provenance の件数も、元の Phase 1a カウントに照合します。
