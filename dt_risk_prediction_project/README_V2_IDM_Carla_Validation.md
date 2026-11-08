# 既存V2 IDMFleetControllerの実CARLA検証

## bounded abortの指令遮断と証跡

安全是正のオフライン実装は実走許可ではない。`route_audit=null`、
`physical_bounds=null`、実CARLA試験BLOCKEDを維持する。

collision callbackはevent属性の解釈・GT observer・ファイルI/Oより先に
不可逆なabort latchを設定する。fleetループ前、車両別actuator更新前、
HealthGate/IdentityGate確認後の最終制御RPC受付前で遮断する。
通常のphase名を`abort`へ変えても受付を迂回できない。

latch設定と指令受付は同じ短いLockで直列化する。RPC・I/O・GT処理中は
このLockもcollision記録用Lockも保持しない。各harnessは最初の制御threadに
束縛し、受付済みcontrol RPCを最大1件に制限する。この保証は当該harnessの
制御APIに限り、別clientや別harnessの不在を証明しない。
callback到着、latch設定、指令受付、API呼出し開始・完了をmonotonic時刻と
sequenceで別記録する。受付後〜呼出し開始前や実行中の割込みでは、既受付の
1件を取消できない。2台への送信は非atomicで、1台目のみ受付となり得る。
続く2台目の通常指令やdrive tickで帳尻を合わせない。

異常時は`result.abort`と`abort_result.json`に車両別の事実を保存する。
`ABORT_REQUESTED`、`DRIVE_COMMANDS_BLOCKED`、`STOP_COMMAND_ATTEMPTED`、
`STOP_COMMAND_ACKNOWLEDGED`、`STOP_OBSERVED`、`STOP_UNVERIFIED`、
`CLEANUP_ATTEMPTED`、`CLEANUP_COMPLETED`は独立した単調な事実である。
API返却のACKは`ack_kind=api_call_returned`であり、server内部受領・control echo・
物理停止とは異なる。actor削除や旧`safe_early_stop`も停止の証明にしない。

2台abortでは、通信/identity/frame/owned slotsが有効で、直近の同frame記録と
最終成功指令が整合し、速度<=0.10m/s、gear/姿勢/道路/入力が適格な場合だけ、
各車に最大1回の限定要求を行う。入力はthrottle=0、直近適格steerを保持し、
`brake=min(0.02, previous_brake + 0.10*elapsed)`。
`elapsed=min(0.05,max(0,(current_frame-last_successful_control_frame)*0.05))`
なので、同frameのbrake増分は0で、他車のslew予算を流用しない。
brakeが0のまま、速度が高い、状態/制御履歴が未知、直近指令が未echo、
brakeが既存cap外、slotが不完全等の場合は新規制御をskipする。
高速側のneutralizeや強制制動を追加していない。

単車abortの既存brake=0.70、入力順序、追加tick=0は維持するが、2台へは移植しない。
通信例外でpoisonした後は、他車停止・sensor停止・削除・設定復元を含む追加RPCを
禁止する。停止未確認やcleanup未完遂を成功扱いせず、自動再接続/再試行しない。

追加abort tick/観測loopは0。したがって物理停止を観測できなければ必ず
`STOP_UNVERIFIED`を残し、既存cleanup結果は独立して記録する。
CARLA非依存`StopWindow`は外から与えられたpost-request両車同frame記録だけを
評価する。既存`stopped()`条件、control echo、dt=0.05、11連続sample/0.5秒が
必要で、重複/欠測frameを補間しない。100sample以内に窓が成立しなければtimeout。
このreducerはtickを生成せず、実worldの停止確認を有効化しない。
永続disk障害・native終了・通信断では最終証跡や復元の完遂も保証できない。

`tests/test_bounded_abort.py`のT01〜T36は合成データとmockの検証であり、
実車両の停止性能・路面安全性・実session排他性の実証ではない。

指令と実測加速度の定義、frame対応、入力制限、サーバ不要の集計と追加診断sidecarは
[IDM忠実度評価](README_IDM_Fidelity.md)を参照。指令追従の検証と、走行経路が機能したことを区別する。

Issue #40。PR #39の安全なtick・観測・cleanup・監視機構を継承する。
研究対象は既知の車両別IDMパラメータによる走行・DT予測であり、手動PIの完成や
単車データからのT推定を目的としない。IDM式・leader検出・PID計算を作り直さない。

## 既存機能

- idm_model.py：IDMParameters、自由走行／相互作用加速度、希望gap、PID。
- idm_controller.py：車両ごとのパラメータdictを受け取るIDMFleetController、
  state読み取り→leader探索→IDM→フィルタ→速度PID＋feed-forward→操舵・入力。
- idm_snapshot.py：V2 snapshotの読み書き・論理IDとtraitsの解決。
  traits JSONで個別指定できるのはT/v0。s0/a/b等は標準CLIでは共通値だが、
  Fleetのparameters dictへは任意のIDMParametersを車両ごとに渡せる。
- Run_DT_Risk_V2_IDM.py：snapshot再構成によるCARLA予測、車両状態・IDM指令、
  V1のcollision callback・near-miss評価を再利用。near-missはXY中心距離・接近速度。

これらのオフライン機能と、CARLA物理での実応答・安全な制動・事故リスクの妥当性は別。
信号・標識への交通規則遵守、複雑な交差点、車線変更、road境界を越えるleader同定は
元のV2の対応範囲に含まれない。IDMの最大減速度パラメータは実制動下限ではない。

## 実験版と後方互換性

IDMFleetControllerにkeyword-onlyのcontrol_sinkとdisable_autopilotを追加した。
省略時は従来どおりのapply_controlとautopilot解除を行い、数式・PID・ログschemaは不変。
今回のみ新規所有actorに対してdisable_autopilot=Falseを使い、TM取得／登録操作を
行わない。fresh spawn、TM／Autopilot呼出しなしを根拠とし、登録状態の照会成功とはしない。
control_sinkは元のV2入力を保持したまま、明示的な実験用入力制限を適用する。

単車：Town01_Opt road8/section0/lane-1、s170→230の60m探索区間、Model 3、dt0.05。
v0=0.5m/s、T=1.0s、s0=2m、a=0.3m/s²、b=1.5m/s²、delta=4、max_decel=2m/s²。
settle2秒、IDM自由走行6秒／120frame、進行上限3m、速度上限0.7m/s。
V2の元PID・alpha0.30・lookahead6mを維持し、search30m、steer上限0.10を明示する。
正のV2 effortを0.10倍、throttle上限0.10・slew0.02/sとする。
負のV2 effortは速度0.10m/s超では未校正として中止し、そのままfull brakeを送らない。
低速brakeは上限0.02、増加0.10/s。fallbackは入力適用前に中止する。
この制限は物理応答を保証せず、元のV2が無制限入力で検証済みとも扱わない。
飽和・slew制限がある区間で、IDM計算加速度と実加速度の一致を主張しない。

IDM区間後の実験終了は既存StagedStopを使用する。速度≤0.10m/sでbrakeを
0.005ずつ0.02へ増加する。4frameゼロ入力区間を必須にしない。
停止profileの期限5秒・連続停止0.5秒を維持する。終了停止はIDM自由走行とは分けて記録。
peak5m/s²、sustained3m/s²二frame、衝突・逆走・車線・姿勢・終端20m、最大400frame、
worker120秒、RPC5秒、poison後追加RPC禁止、所有actorのみcleanupと設定復元を継承する。
停止性能の仮定を使わず、3m＋0.7m/s×5秒＋前端2.42511m＋終端20mでも28.92511m。
これはsample速度gateに基づく計画で、サブステップの物理限界の証明ではない。

## 記録・進行条件

v2_validation.csvは、計算元frameFと観測frameF+1を分けて保存する。
V2の計算元速度・leader/gap・相対速度・IDM／filter加速度・目標速度・元入力・fallback、
実入力・実位置速度加速度・T/v0/s0/a/bを保持する。同時刻の指令と応答を混同しない。
initial_snapshot.csv/.jsonは既存V2保存関数、resolved_idm_parameters.jsonは元の型を使用。
raw/CSV/commandのframe照合と、保存した指令の既存IDM式によるoffline replayを行う。

Run_V2_IDM_Carla_Validation.py --config <外部JSON>はオフライン。
--execute --server-pid <現在の実identity> --port 2100 --output-root <新規外部dir>のみ実行する。
session attestation、OS・TCP両端の外側監視、cooperative leaseを使う。
全外部client不在は完全証明できない。strict監査BLOCKED、route_audit/physical_bounds=null。
単車の正常IDM走行・制御停止・cleanupが成立した場合のみ、複数車両とsnapshot再構成へ
進む。不成立後に同じ設定を自動再試行しない。

## 2台とsnapshot再構成

trial=fleetは単車single_runの完了・raw/CSV・制御停止を接続前に再検証する。
初期中心間隔20m、leaderはT=1.0s/v0=0.5m/s、followerはT=2.0s/v0=0.6m/s。
他のIDM値は単車と同一。V2のleader検索と相互作用項が制御判断を担当し、
PR #39の手動gap PIは使用しない。Fleetが発行済みの2台の命令を受け取ってから
tickを1回だけ進める。gap8m・TTC5秒・全安全gate・個別collision sensorを維持。
IDM6秒、両車の進行各3mまで、両車同時の0.5秒停止を要求する。
20＋6×0.7＋5×0.7＋前端2.42511＋終端20=50.12511mは60m以内。
さらに既存の経験停止距離を2倍する保守的予算も要求する。保証値は作らない。

trial=replayは正常なfleetのsource_runも必須。同じ初期snapshotと解決済みパラメータを
既存read_snapshot/resolve_parametersで読み、既存V1/V2 spawn_from_snapshotを再利用する。
最初のspawn失敗で中止し、既存helperの高さ再試行は許可しない。
静止snapshotのみ対応し、初期velocityの設定は各車1回のみ。走行中の速度強制やteleportはしない。
元のhelperが初期zを最低0.2mとする点を記録し、その後は同じgear初期化・settleを行う。
元の初期状態に対し位置0.02m・速度0.01m/s・heading0.2度以内でなければ走行を開始しない。
map/version・weather・外部actor ID・適用/元WorldSettings・観測可能なphysics設定も照合する。
engine RPMやサスペンション内部状態の完全復元を主張しない。
wheel positionは現在のWheel Locationから取得されるため、静的physics設定の比較から
分離して両側の実測値を保存する（[CARLA 0.9.16実装](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp#L452)）。
mass・torque curve・gear比・friction等、取得した設定項目は引き続き比較する。

prediction_states.csvは既存V2のPredictionState形式で、IDM区間の実CARLA将来軌跡を
0.05秒刻み・6秒で保存する。near-missは既存detect_near_missesを使用し、
閾値4m/接近速度1m/s/cooldown1秒を変更しない。collisionは共通sensor記録に残す。
低速・gap安全gateのもとでeventがないことを、事故リスク検出の有効性の証明にしない。
sourceとreplayの差は再構成の探索的再現性であり、Tの因果効果や予測精度改善と区別する。
実測状態を固定したTの反実仮想加速度比較にも既存compute_idm_accelerationを使う。

実験結果はRepository外に保存する。既存V2の通常runnerやTM比較をこの実験用
入力制限の妥当性確認なしに安全と認定しない。T推定・事故リスク有効性評価は別段階。
