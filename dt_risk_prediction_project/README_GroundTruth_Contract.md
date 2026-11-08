# Source future収集契約v1

この契約は、保存済みSourceとPredictionの構造・来歴・因果性をオフライン検証する。CARLAへ接続しない。既存IDM/PID/FleetController、入力制限、安全gate、予測評価器の用途は変更しない。

正常な入力でも `CONTRACT_VALID_PENDING_APPROVAL` である。初期差閾値、独立予測精度、collision/near-miss検出性能は未承認。`route_audit=null`、`physical_bounds=null`、実走 `BLOCKED` を維持する。

## ファイルと実行

- `dt_source_future_contract_v1.json`: version付き機械可読契約。汎用JSON Schemaではなく、validatorがrequired一覧を共有する宣言的仕様。
- `dt_gt_manifest.py`: 保存ファイルの検証と診断値の保存。
- `Validate_DT_GroundTruth.py`: CLI。新規出力だけを許可し、既存出力・入力を上書きしない。
- `tests/gt_manifest_fixture.py`: テスト時だけ一時ディレクトリに2台の合成bundleを作成。実測データではない。

```powershell
python dt_risk_prediction_project/Validate_DT_GroundTruth.py `
  --manifest '<保存済みbundle>/manifest.json' `
  --output '<Repository外の未使用結果JSON>'
```

入力違反は `INVALID` / exit 2、構造検証終了は承認待ち / exit 0。exit 0は独立評価や実走のGOではない。出力の `independent_forecast_accuracy_enabled`、`risk_detection_performance_enabled`、`runtime_execution_enabled` は常にfalse。

既存の `read_snapshot`、`csv_rows`、`physics_settings` と軌跡indexのframe/time検証を再利用する。既存 `evaluate` を別用途で呼び出さず、ADE/FDEや検出性能も計算しない。追加依存ライブラリは不要。

## 必須manifestとartifact

```text
manifest
  schema_version / evidence_kind（synthetic または recorded_carla）
  registry -> Source単位のsplit/protocol/commit/data manifest hash
  source
    run_id / protocol / code_commit / seed / dt_s
    origin {frame:F0, simulation_time_s:t0}
    prefix_start_frame / horizon_steps / actor_map
    control {logical:{longitudinal,steering,parameters,state_capture}}
    artifacts {snapshot,prefix,future,clock,environment,physics,bbox,initial_control,events}
  prediction
    run_id / code_commit / seed / method
    parent_source_id / parent_snapshot_sha256 / parent_prefix_sha256
    reference_origin（SourceのF0/t0）/ origin（予測worldの実G0/time）
    dt_s / horizon_steps / actor_map / control
    initialization / inputs / model_data
    artifacts {snapshot,future,clock,environment,physics,bbox,initial_control,events}
  evaluation {purpose,requested_metrics,initial_policy,information_policy}
  safety {experiment_status:BLOCKED,route_audit:null,physical_bounds:null}
```

全artifactはbundle内の相対pathとSHA-256で参照する。Source/Predictionの実Actor IDは異なってよく、logical IDで対応する。同一streamでのID再利用・切替は拒否する。

Snapshotは既存ActorState JSONを再利用する。futureは既存PredictionState CSVのframe、相対時間、位置・yaw・速度を使う。clock CSVにはF0/G0と全future frameの実world snapshot timestampを保存する。prefixはSourceの連続ActorState相当CSVとし、終端がSource Snapshotと一致する必要がある。任意の選択済み疎な行集合を連続prefixと偽らない。

environmentにはclient/server version、map、OpenDRIVE/route hash、天候の実測値、WorldSettingsの取得値、外部actor inventory、既知外生情報を保存する。比較contextの不一致は拒否する。v1は未検証の将来scheduleを入力に入れないよう、既知外生情報を空集合に限定する。TM seed/portや車両別制御設定はcontrol.parametersに記録する。

physicsには車両別raw値と取得不能項目を残す。static項目不足は拒否し、比較可能値は既存physics_settingsで取り出す。動的wheel位置/RPM等はstatic同一性の根拠にしない。bboxはlocal location/extent/rotation、初期実controlはthrottle/brake/steer/hand_brake/reverse/gear/manual_gear_shiftと取得frame/timeを保存する。PID/物理内部状態の取得不能はunknownとして明示し、復元済みとしない。

## 因果性・学習分割

Source prefixの全行でframe<=F0、timestamp<=t0を確認する。親Source ID、親Snapshot/prefix hash、Predictionの入力prefix参照、実ファイルの最大frame/timeを相互照合する。未来行、虚偽のmax frame、不完全な入力inventory、重複・欠測、終端Snapshot不一致は拒否する。traits推定も同じprefixだけを入力とし、選択mask/行キーと推定器・校正設定を原記録へ保存する。

Source futureは評価専用の保留artifactであり、予測時の推定入力へ渡さない。予測コードの全入力一覧や取得ledgerのレビューも必要で、hashだけで隠れた入力や測定の真正性を証明できるとはしない。

registryはSource run単位でtrain/tune/testを固定する。同じSourceの重複windowを別splitへ置かない。重複Source ID、同じdata manifest hashの別名Source、test Sourceをtrain/tuneへ指定する違反を拒否する。履歴registryの宣言hashは保存済みdataset manifestと別途照合する必要がある。

## frame・時刻・初期状態

Source参照F0/t0は親manifestの保存値と一致させる。Prediction worldの絶対G0/timeはSourceと異なってよい。初期化後・最初の予測指令/tick前の実G0状態を測定し、最初の指令G0→応答G0+1を記録する。future先頭は相対dtでありt=0ではない。

実clock、frame原点、相対時刻k*dt、WorldSettingsのdtを検証する。1e-6秒の照合値は数値表現誤差用であり、物理遅延や1frameずれの許容ではない。座標合わせ・補間・最良時間shift・初期差の差引きを行わない。

**初期状態残差は合否へ使わない。** 位置XYZ norm、速度XYZ norm、heading/roll/pitch差を `initial_residuals` に保存する。bbox・static physics・初期実controlの値の差も `initial_configuration_differences` に保存する。非ゼロでも構造・因果性が正しければ承認待ちであり、ゼロでも比較可能性の合格を意味しない。常に `initial_state_assessment=NOT_EVALUATED_PENDING_POLICY` とする。

旧Replayの0.02m/0.01m/s/0.2度も、完全一致ルールも、新規許容値も適用しない。契約JSONに記した位置1mm、速度0.001m/s、姿勢0.01度等は未承認の議論用候補である。5秒一定速度差と5m直進の単純例で初期差予算を約6.9mmにする意図だが、非線形physicsや追従応答の保証ではない。正式採用には評価分解能、初期化残差分布、反復再現性の検討と別途承認が必要。

manifestへAPPROVED等を書くだけでは有効化できない。`initial_policy` はstatus=PROPOSED_UNAPPROVED、thresholds=null、approval_reference=nullのみを受け付ける。

## A/B/Cとeventラベル

|目的|固定条件と解釈|
|---|---|
|A_reconstruction|同じ制御・パラメータのSource/Replay。再構成再現性だけを扱う|
|B_method_forecast|同じ保留Source futureに対するTM/IDM等の方式比較。Source制御バイアス・操舵/経路差を明示。スコア計算は無効|
|C_T_counterfactual|縦制御・操舵・v0/s0/a/b等を固定しTだけ変更。別Tの未来を実測したとは呼ばない|

Aの制御metadata不一致、CのT以外のparameter変更は目的契約違反として拒否する。これは初期状態残差の閾値判定とは別である。実初期状態やphysicsの診断差があれば、それらも含めた比較可能性を今後レビューする。

eventsは原検出定義、対象ID・範囲、event.frame/time/pair/provenance/raw payloadを保存する。actor XY距離near-miss、bumper gap/TTC、sensor collisionを同じ定義として扱わない。

独立labelには全pair・全future frameの非重複coverage、positive/negative、注釈protocol/provenanceが必要。sensor collisionとの矛盾も拒否する。partial/missing/proxy labelはtrajectory-only資料の制約として残せるが、risk_detection要求時は拒否する。complete_independentという自己申告だけで注釈の独立性を認定せず、完全な合成labelでも性能計算を有効化しない。

## 将来の2台Source収集

1. 低速のscenario_id/seed/replicateとSource制御・許可情報を固定する。known Tはoracle以外の予測入力へ渡さない。
2. 実走の別途承認、安全条件、server identity、排他session、単一tick ownerが成立してから既存収集器を使う。現在はBLOCKED。
3. 初期化区間を分離し、同一F0で全車の実Snapshot・実control・physics/bboxを保存する。観測処理で追加tickを発生させない。
4. F0+1から5秒以上のfutureと実clock、既存raw/ledger、collision記録を保留する。dt=.05なら主要区間は100frame。安全中断を正常windowへ補完しない。
5. prefix/Snapshot/入力inventoryをhash固定し、各Predictionの実G0状態を保存する。残差は記録し、正式な比較採否は未承認policyの決着まで保留する。
6. hash・ID・clock・因果性・環境・原event・完全window・終了/cleanup/WorldSettings復元を確認し、採用/不採用理由を両方保存する。
7. 2台で保存契約を確認後4〜8台へ拡張する。反復とデータ分割はSource run単位とし、車両/frame/重複windowを独立反復に数えない。

既存V2は従来の再構成再現性で利用できる。TM800行の対応予測不足やRD300件の初期差/一部prefix問題は保持し、新manifestを捏造して評価へ通さない。

v1はcomplete fixed cohort、最大16MiB/file・100000 actor-frame点のオフライン実装。処理量上限はCARLA安全閾値ではない。途中消失・衝突後censor、known schedule、完全な学習履歴認定は別レビュー。収集契約の導入と実CARLA実行許可は別である。
