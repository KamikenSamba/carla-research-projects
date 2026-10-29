# IDM指令からCARLA応答までの忠実度評価

Issue #42。PR #41をbaseとするオフライン計測・分析。実走の承認を含まない。
IDM式、PID計算、ゲイン、scale/cap/slew、安全gate、Snapshot再構成を変更しない。

## 処理と時刻

| 処理 | ファイル・関数 | 時刻・主な設定 |
|---|---|---|
| 状態・leader | idm_controller.IDMFleetController.step / idm_leader.sample_vehicle, trace_lane, find_leader | 全車状態を指令前に取得。速度は3Dノルム。bboxのbumper gap、同一road/section/lane、検証版探索30m |
| IDM | idm_model.compute_idm_acceleration | frame Fの速度・gap・先行速度。自由走行と相互作用、既存の最大加減速度clamp |
| 加速度フィルタ | idm_controller.VehicleController.command | af[F]=0.30*aIDM[F]+0.70*af[F-1]。初期値0 |
| 速度目標 | 同command | vtarget=max(0,v[F]+af[F]*0.05)。毎frame現在速度から作り直す |
| PID | idm_model.PID.step | error=vtarget-v[F]。Kp=3, Ki=0.35, Kd=0.05、積分上限±3。加速度偏差のPIDではない |
| feed-forwardと入力 | VehicleController.command | af/(aまたは最大減速度)をPID出力へ加算。effortの正負をthrottle/brakeへ分け0..1にclamp |
| 実験用制限 | idm_carla_validation.BoundedV2Actuator.step | 生VehicleControl→正入力0.10倍→throttle cap0.10→slew0.02/s（0.001/frame）。brakeは0.10倍/cap0.02/増加0.005/frame。brake要求時throttle=0 |
| 適用 | V2SingleHarness.initialize_fleetのsink→既存issue | Fでapply_control、ledgerへ指令と呼出し結果。TM/Autopilotへ判断を委ねない |
| 物理更新・観測 | V2SingleHarness.capture / V2FleetHarness.capture→tick→idm_brake_harness.BrakeHarness.observe | 単一workerが1tick。F+1のsnapshot・velocity・get_control・get_accelerationを取得。観測による追加tickなし |
| 記録 | V2SingleHarness.log_idm | Fの指令とF+1の応答を同一CSV行に対応付ける |
| 評価 | idm_fidelity.analyze_run | e=API縦加速度[F+1]-生IDM加速度[F]。フィルタ後との比較も別集計 |

target>0ならerror=af*dtであり、次frame実測加速度との誤差をPIDに戻していない。
従って、ゲインを「加速度追従PIDのゲイン」と解釈できない。feed-forwardも車両physicsの
逆モデルとして校正されたものではない。検証用の0.10倍・slewを含む系の忠実度を評価する。

## 加速度の定義と検証

- API縦加速度: API acceleration XYZと現在の車体forwardの内積。
- vector FD縦加速度: (v[F+1]-v[F])/dtとF+1のforwardの内積。
  既存CSVは設定dt=0.05を使用。分析はworld timestamp差でも再計算しdt一致を確認する。
- signed speed FD: v·forwardの差分。旋回時には基底の変化項も含むのでvector FDと異なる。
- magnitude FD: 3D速度の大きさの差分。後退符号を失い、ゼロ付近の符号反転で大きく異なる。
- signed position interval speed: 符号付き位置差分/dt。区間平均であり、APIの区間終端速度と同一ではない。

CARLA0.9.16の[WorldObserver.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/WorldObserver.cpp#L244)はAPI加速度を速度差分から計算する。
APIとFDの一致は同じ保存状態の整合性であり、独立した物理センサによる検証ではない。
[VehicleControl.h](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/rpc/VehicleControl.h#L22)の入力はfloat。
オフライン再生もfloat32境界を再現して、原始ledgerの指令との丸め差を区別する。

lag scanはcommand Fに対してresponse F+1+kを比較し、k=-10..10で同じ内側command集合を使う。
端点の最小RMSEを物理遅延と認定しない。立上がりを含む非定常・短い系列の探索である。
0.5秒ブロック平均も分解能の比較のみ。rawの5m/s² gateを平滑化・置換しない。

## 保存済みデータの解析

既存証跡のanalyze_v2_runs.pyで計算していたRMSEを拡張し、ライブ起動supervisorのimportを除いた。
同じリスク・snapshot図は再生成しない。標準ライブラリだけでJSON/CSVが作れ、図だけmatplotlibが必要。

```powershell
python dt_risk_prediction_project/Analyze_IDM_Fidelity.py `
  --evidence-root '<既存V2実験証跡>' `
  --output-root '<Repository外の新規分析ディレクトリ>' --plots
```

入力は既存single_idm_once/fleet_idm_once/snapshot_replay_onceのcontroller_result.jsonと、
各runのv2_validation.csv、calibration_observations.csv、command_ledger.jsonl、解決済み設定。
raw JSONLとsnapshot診断もsource hashへ記録する（このCLIの数値検証はCSV/ledgerに対するもの）。
原始JSONL/snapshotとの直接照合は別の調査証跡に記録し、hashだけを原始検証と主張しない。
成功receipt・全履歴・一意なactor/frame・連続F→F+1・時刻・IDM式・制御再生・control echoを検証する。
欠測・不整合は拒否し、0/前値で埋めない。元証跡内への出力と既存出力への上書きを拒否する。

- fidelity_summary.json: 車両別RMSE/MAE/bias、区間・定義別、lag、入力制限回数、source hash。
- fidelity_frames.csv: 600行などのframe対応・測定定義・入力系列。元CSVは変更しない。
- *_fidelity.png / lag_scan.png: 時系列・制限前後・PID再生・記述的な時間shift比較。

既存データにPID内部値は記録されていない。既知の初期PID/filter状態から既存Controllerを再生し、
ログ済みtarget/filter/throttle/brakeと一致した場合だけ`*_replayed`として保存する。
これは過去の実測値ではない。異なるplant応答や入力制限解除後の挙動は生成しない。
settleとstaged stopにはIDM指令がないため、そこでのIDM誤差はnull。負のIDM制動も未計測。
first_4s/after_4sは時間区分であり、定常・整定の成立を意味しない。

## 次回以降の追加ログ（実走は別途承認）

既存V1/V2 CSV列とcommandの戻り値を維持し、検証Harnessに
v2_control_diagnostics.jsonlを追加。state/command/response frameとtime、actor論理ID/実ID、
clamp前後加速度、filter、速度error、PIDのP/I/D/積分/出力、feed-forward、未clamp effort、
scale/cap/slew前後、実get_control、API/FD加速度を記録する。制御値を計算し直して適用しない。
provenance=recorded_during_control。既存CSVを読む他のツール・snapshot形式は不変。

## 負のIDM加速度の検証計画と現在の実行不可条件

自由走行でv>v0、または十分に小さいgap/接近速度で相互作用項が自由走行余裕を超えると負になる。
単にIDMを負にしてもfilterにより直ちにbrakeにならない場合がある。PID状態・feed-forward・
raw VehicleControl・最終brake・実応答を区別して評価する。

現行BoundedV2Actuatorは**brake要求かつspeed>0.10m/sを未校正として中止**する。
v0=0.5/0.6、cap0.7のまま通常の自由走行減速へ進める実行入口はない。このgateを解除しない。
正式route_audit・physical_boundsもnullであり、現時点で負のIDM実走を実行可能と認定しない。

1. 追加走行の承認前に、専用session/identity/外部tick owner・資源・経路と終端余裕を確認する。
   まず単車で現行の正加速度6秒・cap0.7・進行3m・既存停止を1回だけ、新sidecarで比較する案。
   負の制動・2台・自動反復をこの承認へ含めない。
2. 次に独立した小制動protocolをレビューする。安定したv<=0.10の初期条件を取得できることが前提。
   例としてv=0.08、a=0.3、自由走行でaIDM=-0.03を狙うなら固定v0=v/(1.1**0.25)≈0.078116m/s。
   これはオフラインの設計例で、配布policyへ追加しない。毎frame v0や速度を強制して履歴を作らない。
   現行の極低速振動でこの初期条件を確認できなければ中止し、近い先行車を置いて負加速度を強制しない。
3. 正の駆動との併用なし、brake上限0.02・既存slewを維持して短い負指令から始める案を別途承認する。
   command F→get_control→減速→連続停止0.5sを分け、反応不明・欠測・5m/s²超過・衝突・逆走・
   車線/終端余裕・TTC・RPC異常は既存どおり中止。worker/frame上限、所有actor cleanup、設定復元を維持。
4. 評価はAPI/FDのraw・0.5秒分解能を併記し、制御反映frame、減速開始frame、停止時間/距離を記録。
   throttle taper、throttle=0/brake=0、brake追加区間を分離する。応答が不明なら校正不成立。
   車輪RPM・接触力等は未取得であり、振動のタイヤ/駆動/solver原因を断定しない。

距離は単なるv²/(2b)で認定しない。遅延中加速を含め
d=vτ+0.5*Aτ²+(v+Aτ)²/(2b)に発進距離・車体前端・終端余裕を加えた感度を確認する。
v=0.7,A=5（既存peak gate値、性能保証ではない）、発進3m、前端2.42511m、終端20mの例:

| 仮定τ(s) | 仮定b(m/s²) | 停止区間(m) | 合計必要(m) | 60mとの差(m) |
|---|---|---|---|---|
|0.5|0.5|11.215|36.64011|23.35989|
|0.5|0.25|21.455|46.88011|13.11989|
|1.0|0.25|68.18|93.60511|-33.60511|

τ/bは実測上限/下限ではない。制御停止は瞬時ではなく、速度/加速度gateで距離保証はできない。
長遅延・弱制動では60mでも不足する。現時点の試験結果から正式制動性能、T識別性、事故リスク精度は主張しない。
