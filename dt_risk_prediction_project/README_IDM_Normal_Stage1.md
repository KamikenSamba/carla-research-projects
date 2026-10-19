# 通常速度域 Stage 1：設計・監査・オフライン検証

Issue #16。PR #15 を基点とする追加シナリオ。**長距離道路は未選定・未監査であり、現在の配布設定は実走不可**。56m区間の座標を延長していない。実CARLA挙動、走行成立、T推定精度の改善は未検証。

## 設定と実行入口

`configs/idm_normal_stage1.json` は CARLA 0.9.16 / Town10HD_Opt、vehicle.tesla.model3、dt=0.05s、先行希望4.0m/s、追従希望4.5m/s、初期中心間隔22m。TM要求車間パラメータ12mは実測gapと区別する。実測初期間隔は車両bboxから別途監査する。想定bbox前後長の合計4.85022mならバンパーgapは17.14978m。

warmupは最大8sかつ先行移動25m以下、capture15s、snapshot10s。走行方向の道路長200m以上、終端の前端余裕20m以上を要求する。Stage 1の希望速度は一定であり、Stage 2を許可しない。

サーバへ接続しない検証：

```powershell
python dt_risk_prediction_project/Run_IDM_Following_Experiment.py --config dt_risk_prediction_project/configs/idm_normal_stage1.json
```

`executable_route:false` は設定の構文・設計検証が通っただけで、実走許可ではない。`--execute` / 内部worker / Harness直接呼出しのすべてが未監査経路を最初のCARLA RPCより前に拒否する。従来の56m設定・fingerprint・既存V1/V2/Capture/Context/traits形式は保持する。新しい安全gateは `normal_stage1` 設定で有効になる。歴史的な56m入口に新シナリオの代用品として戻らないこと。

## 初期化方針

第一候補は既存TMのみ。物理settle 20tickの間は未登録、次にTM登録・経路・希望速度設定、以降は同一workerだけがtickする。手動throttleや反復 `set_target_velocity` を併用しない。物理settleとwarmupはcaptureに混入しない。

| 方式 | 研究上の扱い | 状態 |
| --- | --- | --- |
| TMで静止から発進 | TMの発進を含む物理挙動をそのまま観測。比較条件が最も少ない | 実装。加速度gateで発進過渡を監査。通常速度でも出力飽和自体は起こり得る |
| 初回だけ速度設定 | 発進過渡を除ける可能性はあるが、運動量の導入方法が比較条件になる | 未実装・実走拒否。実現性・接地・物理整合を別途評価 |
| TM未登録の助走→引継ぎ | 適切な発進制御と解除・登録境界の設計が必要。引継ぎ時の再過渡もあり得る | 未実装・実走拒否。TMと手動制御の同時適用は禁止 |

`initialization` はconfigとfingerprintに残す。未知の方式はオフライン案として記述できるが実走不可。希望速度を上げるだけで前回の発進過加速が解決したとは扱わない。TM発進がgateに抵触すれば中止し、設定を緩めて再試験しない。

## 加速度・整定・既存安全gate

加速度は `get_acceleration()` をそのframeの車体前方3次元ベクトルへ射影した値。速度差分やIDM式で補完しない。物理settle、warmup、captureで両車を取得してから判定する。

- `|a_long| > 5m/s²` を1frame観測すれば中止。
- `|a_long| > 3m/s²` が同一actorで2frame連続すれば中止。間の正常frameでリセット。
- 取得不能・非有限の加速度は中止。RPC例外は既存HealthGateをpoisonし、追加RPCを抑止。
- capture開始には連続2sを要求（dt=0.05、41snapshotで2sの観測範囲）。各車が自身の希望速度±0.2m/s、`|a_long|<=0.5m/s²`、同じleader・lane、許容heading/lateral内。
- 整定窓のgap変動幅<=1.2m、相対速度絶対値<=0.6m/s、安全停止gap以上。希望速度差0.5m/sによる2s間の1m閉塞は許容する暫定定義。leader変更・lane変更・観測欠落・条件外で窓をリセットする。窓の境界値もテストする。
- 希望速度と実速度を区別し、上限は先行4.5 / 追従5.0m/s。TM追従のため追従車が4.5±0.2へ到達しなければ `warmup_not_ready` で中止する。希望値を達成したことにしない。
- 従来のgap>=2m、TTC>=1s、車線/経路逸脱、停止、信号、actor消失、frame/timestamp異常、Capture/Context不一致、Context有効率90%以上などを維持する。最低速度1.0m/s、T推定器の品質条件は不変。

異常frameは両車を `last_observation.json` と `acceleration_observations.jsonl` に保全。加速度異常は `safety_decision.json` に理由・actor・frame・phase・時刻・連続数を記録する。Capture/Context/Controlは成功した観測frameで従来どおり完全一致。異常が通常CSVの確定前に見つかった場合はrawの最終frameを参照し、600行の成功と混同しない。観測・安全判定は追加tickを行わない。safe_early_stopと走行成立成功は区別する。

## 距離予算と限界

停止モデル（仮定）は `d = v*tau + a*tau²/2 + (v+a*tau)²/(2*b)`。停止判断が伝わるまで加速aを続け、その後一定bで減速すると仮定する。設計値 a=5m/s²、tau=0.3s、b=2m/s²。これらと加速度gateは暫定設計値であり、CARLA車両や実車両の制動性能を保証しない。信頼区間や実測下限として自動推定していない。

- 追従速度上限5m/sでd=12.2875m。先行車が瞬時に静止すると仮定し、毎frameの実測バンパーgapに `d(v_F)+2m` より大きい余裕を要求する。先行車の停止距離を差し引かない。
- 各車の経路終端前端余裕にも `max(20m,d(v))` を要求。速度上限・加速度gateを併用し、予想外の加速を正常扱いしない。
- 先行：初期22 + warmup25 + capture上限4.5*15 = 114.5m。追従：warmup上限5*8 + capture上限5*15 = 115m。両方の最大値を使用する。
- 車体前端2.42511m、終端20mを加えた必要長は142.42511m。200m道路なら最悪設計時の前端余裕77.57489m。実bboxが予算を超えればspawn後・tick前に中止。実測経路長でも照合する。
- warmup8sは必ず使える時間ではない。先行25mを超えれば先に中止。4m/s定常なら約6.25sで到達するため、その前に2s整定を終える必要がある。

| 遅延tau | 制動b | 5m/sからの停止距離（遅延中a=5） | 22m初期中心間隔 / 終端20m |
| --- | --- | --- | --- |
| 0.2s | 2m/s² | 10.10m | 仮定上は適合 |
| 0.3s | 2m/s² | 12.2875m | 仮定上は適合 |
| 0.3s | 1m/s² | 22.85m | gap・終端とも不足 |
| 0.5s | 2m/s² | 17.1875m | gap不足 |
| 0.5s | 1m/s² | 31.25m | gap・終端とも不足 |

検証CLIと `distance_budget.json` はtau=0.2/0.3/0.5、b=1/2/3の9条件を出力する。例えばb=1,tau=0.3の再設計では中心間隔30m、終端30mなら仮定上の余裕を確保できるが、現在の22m policyでは実走不可。設定と安全条件の再レビューが必要。

制動悪化は速度・加速度だけで発生前に保証できない。**b>=2、応答<=0.3sの根拠が未確認のまま22m案を実走してはいけない**。有効な人手レビュー記録を実行前条件とし、空欄の物理根拠や弱い下限では拒否する。オンラインgateも同じ仮定に依存するため、記録があっても物理性能の保証にはならない。レビューの虚偽や条件外の路面/車両変化をソフトウェアだけで証明・防止することはできない。

## 長距離道路の監査（次回の別作業）

今回はCARLA接続も地図RPCも実施しない。保存済みの対象バージョンのOpenDRIVEがあれば、まずオフラインで候補を抽出する。なければ将来の明示承認を得た地図読み取り作業を別途行う。world reloadや車両生成は地図監査に不要。

1. Town10HD_OptのOpenDRIVE全体SHA-256を記録する。56m区間からの延長を仮定せず、同じroad/section/laneで200m以上の単一路を新規選定する。現在の実装は整数sを1mずつ減らす方向のみ対応。s増加方向や複数road接続は未対応として拒否する。
2. 1mごとにwaypointのx/y/z、road/section/lane、Driving lane、幅>=3.4m、非junction、単一next(1m)、nextの位置連続性、全体yaw変化<=0.5度を監査。点間距離0.95〜1.05m、標高差<=0.02m/点（約2%）も要求する。地形・上下動・視界は追加で人手確認する。
3. 全経路と先の20mに規制landmarkがなく、停止線・信号・標識・分岐・障害物の影響がないことを監査。静的路上車両bboxも調べる。周辺レーンにいる信号等もTMが停止対象にしないか確認する。
4. 車体bbox、初期22m間隔、衝突余裕、両車の距離予算とTM終端挙動を監査する。対象blueprint・物理設定・路面条件・dtについて、制動/応答仮定の根拠をレビューする。根拠不足なら実走を保留し、安全側の再設計を行う。
5. ローカルの新configにroad/section/lane/follower_s/leader_s/end_sを設定し、`policy_sha256` を算出する。`route_audit` はこのhashに結びつける。実験データや監査dumpはGitへ加えない。

必要な `route_audit` スキーマは `NormalStage1.require_executable()` を正とする：`policy_sha256`、`opendrive_sha256`、`evidence_sha256`（各64桁SHA-256）、`reviewed_by`、`reviewed_at`、`evidence_reference`、`carla_version:"0.9.16"`、`map_name`、`points:[[x,y,z],...]`（開始〜終端の全1m点）、`physical_bounds:{braking_lower_bound_mps2,response_upper_bound_s,evidence_reference}`。監査証跡のhashと参照先はレビュー担当が実物と照合する。自己申告フィールドは監査実施の暗号学的証明ではない。

起動後にもOpenDRIVE hash、全点geometry（0.1m以内）、経路連続性、非junction、landmark、静的障害物を読み取り専用で再確認し、合わなければTM生成・WorldSettings変更・spawn前に中止する。専用session、現在PID/開始時刻、version/map、既存actor、初期WorldSettings、8000未所有、他tick client不在というREADME_IDM_Harness.mdの前提も維持する。

## 次回Stage 1に進む条件

上記の道路/物理仮定の人手監査、対応configのオフライン再検証、新しい実走承認がすべて必要。正式入口は `Run_IDM_Following_Experiment.py --config <ローカル監査済み設定> --execute --stage stage1 --server-pid <その時点のPID> --output-root <新規証跡親ディレクトリ>`。この資料は実走承認を代替しない。

1回だけ実行し、停止時は再試験・上限緩和をせず保存する。終了分類、fatal stderr、所有actorのみ削除、外部actor保持、WorldSettings復元を監査する。TM登録は呼出し結果と所有PIDの証拠を報告し、取得できない登録状態を真と補完しない。速度/gap/制御入力、全frameキー、Context有効率を監査してStage 1合否を判断する。T推定・Stage 2・未来予測比較は本作業の対象外。
