# 既存V2 IDMFleetControllerの実CARLA検証

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
進む。不成立後に同じ設定を自動再試行しない。現時点のfleet/replay実行入口は拒否する。

実験結果はRepository外に保存する。既存V2の通常runnerやTM比較をこの実験用
入力制限の妥当性確認なしに安全と認定しない。T推定・事故リスク有効性評価は別段階。
