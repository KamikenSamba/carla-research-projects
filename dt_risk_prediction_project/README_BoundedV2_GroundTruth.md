# bounded V2観測sidecarと契約v1 adapter

既存bounded V2のraw/CSV/Snapshot・制御ledger・診断sidecarを再利用する。新しいIDM、下位制御、走行ルート、予測方式、契約を作らない。**実CARLA試験はBLOCKED**、`route_audit=null`、`physical_bounds=null`。この実装の導入・mock成功は実走承認ではない。

## 観測の責任と既定動作

観測は既定で無効。既存configの最上位`ground_truth_observation` metadataでopt-inする。`scenario`、fingerprint、6秒/120frame、120秒worker上限、IDM/PID/FleetController、scale/cap/slew、安全gate、停止方式は変更しない。新しい引数転送やsupervisorを作らず、既存configがhostとworkerへ渡る経路を使う。

`dt_gt_observation.py`はCARLAをimportしない。mixinは既存`super.tick()`の戻りWorldSnapshotだけを受け取り、tick/get_snapshotを追加しない。初期control/bbox/stateは既存rawから取得し、physicsとXODRは既存prepareの返却値を補足する。callbackでworld/actor getterを追加しない。

通常時のVehicleControl tuple・発行順・PID/actuator状態・tick数は観測ON/OFFで一致する。観測失敗は記録不全として既存例外/cleanupへ進む。cleanup中は観測エラーの再送出で既存停止・削除・復元を妨げず、finalize失敗も元の異常を隠さない。単車の既存abort brake=0.70を維持する。2台の既存cleanup経路も変更しない。追加I/Oのwall負荷はあり、期限を自動延長しない。

現行プロトコルはRoad8/section0/lane−1、s170→230、約60mの探索区間である。旧lane1/195m監査と別の区間であり、正式な監査・停止性能保証は得られていない。

## 保存内容

|sidecar|由来・時点|
|---|---|
|gt_run_provenance.json|run ID、source/prediction role、protocol、実source commit/dirty、seed宣言と適用根拠、input role、設定bytesのhash。workerは読んだconfig bytesとsupervisor hashを照合|
|gt_clock.jsonl|既存WorldSnapshotのframe/elapsed_seconds。local `sim_time_s`とは別の実world clock。delta/platform timestampは不明ならnull|
|gt_origin.json|fleet生成/再構成比較後・最初のfleet.step前のF0/G0。全車ID、同frameのraw初期control/bbox、prefix境界、原Snapshot/hash|
|gt_input_view/snapshot.json・prefix.csv|F0で既存rawの連続prefixを固定。実world time使用。optional既知T/v0はnull。元initial_snapshotには変更を加えない|
|gt_opendrive.xodr・gt_environment_receipt.json|実取得xml bytes/hash、既存environment/local_routeのhash、prepare取得の外部actor inventory。originで全環境を再計測したという意味ではない|
|gt_physics_extras.json|既存vehicle/leader_physicsへの参照とuse_sweep_wheel_collision、各wheelのlat_stiff_max_load/lat_stiff_value/long_stiff_value。未知static項目を列挙|
|gt_internal_state.json|originのPID integral/previous、acceleration filter、actuator状態の観測subset。完全な制御器/物理内部状態ではない|
|collision_events.jsonlの追加欄|観測有効時のみevent.timestamp、owner、callback ID、受信monotonic時刻を補足。既存raw payloadとcollision flagを保持|
|gt_event_frames.jsonl|既存near-miss CSVの行番号と実frame/time。XY中心距離の定義を維持し、bumper gap/TTCと混同しない|
|gt_capture_coverage.json|sensor登録/停止、原イベント数、callback/I/Oエラー、最終frame、delivery fenceの可否、cleanup結果|
|gt_artifact_inventory.json|cleanup後に安定した既存ファイル/receiptのpath/hash。書込中stdoutや後で生成されるworker_resultは対象外|

物理の動的wheel RPM・suspension/solver等はunknown。adapterの`complete_static`は列挙した既知static属性の検査範囲に限定され、内部状態完全復元の認定ではない。元physicsの動的位置/RPM/unknown情報は原記録とsidecarへ保持し、static比較と分ける。

## metadataとseed

metadataには`enabled=true`、`role`、`evidence_kind`、`run_id`、`protocol`、`parameter_policy`、`inputs`、`inventory_complete=true`を明示する。実workerで`synthetic`は拒否する。Sourceは`split=test`、Predictionは親run/origin/固定Snapshot・prefix hashと`model_data`を持つ。実行のために正常値を仮設定してはならない。

`seed`は`{value, status, components, evidence}`で記録する。現行プロトコルへseed適用を追加していない。未適用/不明は`status=unknown`やnullで残し、adapterは`status=applied`と実来歴を申告できなければ拒否する。0等を補完しない。申告hashや適用宣言は真正性の証明ではない。

## 入力・oracle・futureの境界

SourceのF0とPredictionのG0は別のworld frame/絶対timeでよい。Source親F0/t0をPredictionのreferenceへ残し、G0は初期化後・最初の指令前に実記録する。command F→response F+1を既存raw・ledger・CSV・診断・clockで相互照合する。ループ時刻から実clockを生成しない。

SourceのprefixをF0でfreezeし、その後のfutureとは別viewにする。通常viewにSource既知T/v0を含めない。元Snapshot/割当parameterは参照archiveとして保持する。既存A再構成のSnapshotRestorerはknown T/v0を使うため`parameter_policy=reconstruction_known`と明示する。この接続でprefixからtraits推定した、あるいは通常予測を実行したと主張しない。

各input entryは`path/sha256/role`を持つ。`model_observation`には最大Source frame/timeを必須とし、固定viewのhashと実ファイルを照合する。Source既知traitsを含む`model_parameters`はoracle/reconstruction_knownだけに許可する。futureは`evaluation_only`または既存安全採否用`safety_admission`に限定する。未申告データへの全アクセスを遮断するsandboxではなく、宣言されなかった入力の検出保証はない。

既存replayの`verify_completed`はSourceのfuture/停止結果を安全採否に読む。この経路を削除せずhost_admission_readsへ役割を記録するが、全runtime read ledgerとは呼ばない。Source完走による選択バイアスも残る。Bのdefault/estimated/oracle比較やCの新しい実走プロトコルは追加していない。adapterは既存A再構成のペアだけを扱う。

## adapterのオフライン実行

```powershell
python dt_risk_prediction_project/Adapt_V2_GroundTruth.py `
  --source-run '<保存済みSource run>' `
  --prediction-run '<保存済みPrediction run>' `
  --output-root '<Repositoryと両入力runの外にある未使用ディレクトリ>'
```

CARLA接続はない。既存validatorの出力guardを再利用し、既存出力・Repository・入力run配下・リンク/junction迂回を拒否する。安定した親ディレクトリを前提とし、悪意ある並行ディレクトリ差替えへの完全なsandboxではない。

入力artifactを一度取得したbytesでhashと解析を一致させる。原raw/CSVとsidecarのframe/ID/clock/control/親子hashを検査して、契約用Snapshot/prefix/future/clock/environment/physics/bbox/control/eventsを新規生成する。元証跡は変更しない。source_data_manifestはSource artifactと来歴だけを先に固定してregistryから参照し、最終manifestの自己hash循環を作らない。

不完全な旧runは`adapter_report.json: INPUT_INCOMPLETE`と不足理由を保存し、正常manifestを作らない。生成した契約の構造不一致は既存validatorで`INVALID`となる。成功も`CONTRACT_VALID_PENDING_APPROVAL`でexit 0、その他はexit 2。初期残差は数値診断のみで閾値採用・完全一致による合否判定はしない。独立予測精度・リスク検出性能・実行許可は無効のまま。

## event coverageと現時点の限界

sensor stopの復帰・wall待機・イベント0件ではcallback配送完了や独立negativeを証明できない。現行実CARLAのcoverageはunknownであり、実測recorded_carlaの正常bundleは生成できない。mockだけが明示synthetic delivery fenceを提供でき、正常E2Eを検証する。mock fenceをrecorded_carlaへ流用すると拒否する。

独立ラベルは生成せず`label_status=missing`とする。XY near-missは元検出定義のraw情報であり独立正解ではない。外部actor collisionは元IDを保持して不適合とし、架空logical pairへ変換しない。callback timestamp・coverageやstatic physicsの欠測を0/defaultで埋めない。

実走再開には、現行60mルート・接地/障害物・停止余裕・制動性能/応答・server identity/排他session/資源と別途明示承認が必要。今回のテストは合成World/Actor/eventのみであり、実CARLAの物理挙動・実GT成立・予測精度を保証しない。
