# 研究プロジェクト進捗整理

## 最新追記: 2026-07-14 経路継承・交通信号統制実験

経路継承の単独効果を評価するため、source route adherence precheckと交通信号timeline replayを追加した。source経路は正解futureから作らず、交通生成前にCARLA 0.9.16の`GlobalRoutePlanner`で生成する。従来は詳細waypoint列を保存しながらTraffic Managerへ交差点の`Left/Right/Straight`だけを渡していたため、`TrafficManager.set_path`で保存waypoint位置列を渡す方式へ修正した。route記録時はAutopilot登録直後、次の同期tickより前にpathを再適用する。

source precheckは、source future各点と保存route waypoint列の順序制約付き最近傍距離を算出する。適格条件はadherence 0.80以上、平均距離3.0 m以下、p95距離5.0 m以下、移動量5.0 m以上、future 10.0秒以上、一意なID対応、正しいroute provenance、junction通過時の出口一致である。Town10HD_Optでは重複するOpenDRIVE road IDへの投影揺れがあるため、road/lane sequence LCSは診断値とし、主判定には空間距離とroute index順序を用いる。

交通信号はCARLA Actor IDを主キーにせず、map、丸めた位置・yaw、junction ID、pole indexの複合値をSHA-1化した`stable_signal_id`で識別する。同期切替直後にjunction情報を取得できない場合は、同一map、pole index、位置1 m以内、yaw 2度以内の決定的fallbackを使う。`traffic_light_metadata.json`には位置、group、色時間、初期state、関連logical IDを保存し、`traffic_light_timeline.csv`には各source world frameのstate、elapsed time、各色時間、prediction開始からの相対simulation timeを保存する。

`--traffic-light-control-mode`として`native`、`snapshot_restore`、`timeline_replay`、`freeze_snapshot`を実装した。主比較は`timeline_replay`であり、各fixed tickに最も近いsource信号sampleを適用する。車両の正解future位置、速度、yaw、車間は使用していない。CARLA 0.9.16はtraffic-light elapsed timeを直接設定できないため、要求値と実測値をログに残し、stateとgreen/yellow/red durationを適用する。

固定source runは`route_signal_source_v7_none_r1_20260714`である。prediction start frameは1170445、seed/TM seedは42、fixed deltaは0.1 s、observation/futureは10 s/10 sである。全Actor 8、走行Actor 7、評価適格Actorは`2,3,4,6,7`の5台である。不適格はActor 1（保存routeから逸脱しjunction出口も不一致）、Actor 5（平均route距離3.601 mで閾値超過）、Actor 8（移動量5 m未満）である。全Actor平均route adherenceは0.868、junction exit match rateは0.667、maneuver match rateは0.875である。

同じsource CSV、route、signal timeline、blueprint、logical ID、TM seed、fixed deltaを使い、baselineとfull_routeを各3回実行した。全runで8台が10秒後まで生存し、lane escapeは0、信号15基は全てmappingできた。信号stateの即時読戻し一致率は0.976898、timeline alignment error最大値は`4.94e-10 s`である。state遷移tickの即時読戻しにはCARLA側反映タイミングによる不一致が残る。

| 評価群 | mode | ADE [m] | FDE 1 s | FDE 3 s | FDE 5 s | FDE 10 s | road | lane | route |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 全Actor | baseline | 1.863 | 1.169 | 1.830 | 2.113 | 3.216 | 0.916 | 0.906 | 0.958 |
| 全Actor | full_route | 2.046 | 1.147 | 1.803 | 2.159 | 2.619 | 0.886 | 0.860 | 1.000 |
| 適格走行Actor | baseline | 2.547 | 1.385 | 2.448 | 2.912 | 4.766 | 0.877 | 0.860 | 0.933 |
| 適格走行Actor | full_route | 2.838 | 1.353 | 2.407 | 2.984 | 3.803 | 0.829 | 0.788 | 1.000 |

full_routeは適格群の1秒FDEを0.031 m、3秒FDEを0.041 m、10秒FDEを0.963 m改善し、Actor 4でbaseline反復中に発生したwrong turnを解消した。一方、5秒FDEは0.072 m、ADEは0.291 m悪化し、road/lane一致率も低下した。Actor別ADEが改善したのは1と4であり、適格Actorでは4のみである。Actor 2、3、6、7は悪化した。3反復は完全一致せず、baseline ADE標準偏差0.139 m、full_route ADE標準偏差0.230 mである。

この結果だけでは経路既知条件を標準条件として確定できない。route一致は改善したが、短中期位置誤差とlane一致率の改善が一貫しないためである。次は信号state適用のpost-tick検証、junction進入・退出時刻および信号停止・発進時刻のログ列追加を優先する。その後、経路既知条件を採用するか判断してから速度・車間特性の継承実験へ進むべきである。速度・車間校正は今回実装していない。

成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\route_signal_control_evaluation_20260714`に保存した。主要ファイルは`route_signal_actor_comparison.csv`、`route_signal_group_summary.csv`、`route_signal_repeat_summary.csv`、`route_signal_overall_summary.json`、各信号復元ログ、PNG/PDF図、`route_signal_experiment_summary.md`である。

## 最新追記: 2026-07-14 経路条件付きAutopilot未来予測

交通生成前にCARLA同梱の`GlobalRoutePlanner`で各車両の目的地・予定経路・交差点分岐を決定し、`vehicle_route_plans.json`と`vehicle_route_waypoints.csv`へ保存する機能を追加した。経路のprovenanceは`source_planner`であり、正解future CSVの座標から逆算したOracle経路ではない。初期実装ではwaypoint列から交差点単位のRoadOption列を作り`TrafficManager.set_route`へ渡していたが、source route adherence監査により詳細経路が失われることを確認した。現在はCARLA 0.9.16の`TrafficManager.set_path`へ保存waypoint位置列を渡す方式へ置き換えている。GlobalRoutePlannerの依存としてvenvへ`networkx 3.6.1`を追加した。CARLA/Python APIは0.9.16のままである。

`vehicle_state_stream.py`の論理IDは`world.get_actors()`の列挙順で決まり、source actor ID順とは限らない。このため、記録終了後に`vehicle_states.csv`の`carla_actor_id -> id`を用いてroute planのlogical IDを再対応付けする処理を追加した。replay側は`none`、`destination`、`next_maneuver`、`full_route`を選択でき、経路割当結果を`route_assignment_log.csv`と`route_assignment_summary.json`へ保存する。今回実行したdestination/full_routeでは8台中8台の割当が成功し、Actor survivalは全条件100%、lane escapeは0であった。

固定sourceは`route_plan_source_v3_none_20260714_01`である。観測区間はframe 624775から624875までの10秒、正解futureはその後10秒、fixed deltaは0.1秒、車両8台、seed/TM seedは42である。同一source CSV・同一route planを用いた比較runは、baseline=`route_plan_source_v3_none_20260714_01`、destination=`route_conditioning_v3_destination_20260714_01`、full route=`route_conditioning_v3_full_route_20260714_01`である。

全8台集計では、baselineのADE 5.660 m、5秒FDE 4.152 m、10秒FDE 17.879 m、route一致62.5%に対し、destinationはADE 10.899 m、5秒FDE 11.768 m、10秒FDE 21.556 m、route一致87.5%、full_routeはADE 7.092 m、5秒FDE 6.061 m、10秒FDE 18.319 m、route一致62.5%であった。destinationはwrong-turn Actorを3台から1台へ減らしたが、位置誤差は悪化した。

source自身が保存経路へ80%以上遵守し、かつ走行中であったActor 1、2、5、7だけを分離すると、full_routeはbaselineに対してADEを2.574 mから2.407 m、1秒FDEを0.384 mから0.126 m、3秒FDEを0.374 mから0.139 m、5秒FDEを0.364 mから0.134 mへ改善した。一方、10秒FDEは12.216 mから12.547 mで改善しなかった。既知の経路意図の継承は短中期の進路再現に有効だが、長期誤差を解消するには速度、加速度、車間、信号通過タイミングの継承が必要である。今回、それらの個別校正は実装していない。

source route監査では8台中5台のみが評価区間で保存経路へ80%以上遵守した。残るActorはTMの経路キューを使い切った、またはsource生成中に経路から逸脱した可能性がある。これはroute assignment成功率とは別の再現性リスクであり、今後は目的地までの残距離を十分長くし、記録開始・評価区間前のsource route遵守precheckを必須化する必要がある。

追加CLIは`--record-route-plan`、`--use-existing-source-csv`、`--route-conditioning-mode`、`--route-plan-json`、`--route-waypoints-csv`、`--require-route-plan`、`--evaluate-route-conditioning`である。PowerShellラッパーにも対応する引数を追加した。比較成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\route_conditioning_v3_comparison_20260714_021020\route_conditioning_evaluation`に保存した。

作成日: 2026-07-03  
調査ルート: `C:\CARLA\user_projects`

この文書は、現在の研究リポジトリを次のChatGPTまたは新しい開発者へ引き継ぐための現状整理である。コード修正は行わず、既存コード、README、設定ファイル、実行ログの読み取りに基づいて記述する。コードから確認できた内容と、未確認または今後の課題は明確に分ける。

## 1. 研究の全体目的

本研究は、CARLA上で車両軌跡を取得し、その状態をデジタルツインとして再生・再構成したうえで、Autopilotによる数秒先の将来シミュレーション、危険接近・衝突イベントの抽出、さらにEgo/RSUのOGM共有による死角補完を検証することを目的としている。

現在のリポジトリには、大きく3系統が存在する。

1. `dt_risk_prediction_project`: 自作のデジタルツイン危険予測V1である。既存Worldの車両状態をcaptureし、snapshotから車両を再構成し、Autopilotで未来を進め、collision/near_miss/risk_regionをCSVへ出力する。
2. `carla_simulate_project`: 先輩研究プログラムを利用した車両状態CSV記録、UDP replay、Autopilot切替、評価・可視化の系統である。既存コードを残しつつ、一括実行用の補助スクリプトも追加されている。
3. `research_ogm_project`: Ego OGM、協調OGM、静的マスク、Spectator追従を整理したOGM実験系である。`legacy/`に元コードを保持し、`scripts/`と`src/ogm_project/`から実行する構成である。

現時点で実装済みの範囲は、車両状態のCSV記録、replay用CSV変換、UDP送信、CARLA上での再現、Autopilot切替、collisionログ、near miss後処理、軌跡図・道路重ね合わせ図、Ego/RSU OGM出力、静的マスク生成である。研究として今後扱う範囲は、切替前の運転特性を保持した将来予測、危険領域をOGM通信の送信セル選択へ接続するrisk mask、Ego単独とRSU共有の定量比較である。

## 2. 現在のシステム全体像

デジタルツイン危険予測系の流れは次のとおりである。

```text
CARLA Town10HD_Opt
  -> 交通車両生成
  -> vehicle_state_stream.py で状態CSV記録
  -> convert_vehicle_state_csv.py で replay 用CSVへ変換
  -> send_udp_frames_from_csv.py でフレーム単位UDP送信
  -> replay_from_udp_future_exp.py でCARLA上に再現
  -> 指定payload frameで future-mode=autopilot に切替
  -> actor.csv / collisions.csv / id_map.csv / meta.json を保存
  -> 評価・可視化ツールで軌跡、事故、品質を確認
```

自作DT risk V1の流れは次のとおりである。

```text
既存Worldのvehicle actor
  -> Run_DT_Risk_V1.py --mode capture
  -> capture_states.csv / snapshot_states.csv
  -> Run_DT_Risk_V1.py --mode predict
  -> snapshot poseからvehicleを再spawn
  -> set_target_velocity後にAutopilot化
  -> prediction_states.csv
  -> risk_events.csv / risk_regions.csv
```

OGM系の流れは次のとおりである。

```text
CARLA Town10HD_Opt
  -> build_static_mask.py
  -> D:\CARLA_DATA\masks\static_mask.npy

Ego OGM:
  scripts\run_ego_ogm.py
  -> src\ogm_project\ego_ogm_runner.py
  -> src\ogm_project\ego_ogm_compat.py
  -> D:\CARLA_DATA\outputs\ego_ogm\<RUN_TAG>\

Cooperative OGM:
  scripts\run_coop_comm.py
  -> src\ogm_project\cooperative_runner.py
  -> src\ogm_project\coop_comm_compat.py
  -> Ego / RSU / Fused OGM, metrics, PLY
  -> D:\CARLA_DATA\outputs\coop_comm\<RUN_TAG>\
```

## 3. ファイル構成と役割

| ファイル | 役割 | 入力 | 出力 | 依存先・実装本体 | 実行方法・状態 |
|---|---|---|---|---|---|
| `dt_risk_prediction_project\Run_DT_Risk_V1.py` | 自作DT risk V1の実行入口。`capture`、`predict`、`all`を持つ。 | CARLA World、または`snapshot_states.csv` | `capture_states.csv`、`snapshot_states.csv`、`reconstructed_states.csv`、`prediction_states.csv`、`risk_events.csv`、`risk_regions.csv`、`summary.json` | `dt_risk_common.py` | 実行入口として実装済み。実験結果の妥当性は別途検証が必要である。 |
| `dt_risk_prediction_project\dt_risk_common.py` | 共通定数、CSV writer、CARLA接続、ActorState/PredictionState/RiskEvent/RiskRegion、同期モード管理。 | なし | なし | CARLA Python API | `DEFAULT_OUTPUT_ROOT=D:\CARLA_DATA\DT_RiskPrediction\runs`、`DEFAULT_MAP_NAME=Town10HD_Opt`、`DEFAULT_FIXED_DELTA_SECONDS=0.05`を定義する。 |
| `dt_risk_prediction_project\check_dt_risk_environment.py` | CARLA接続、Python/CARLA API、map availabilityの確認用。 | CARLA server | 標準出力 | `dt_risk_common.py` | `python check_dt_risk_environment.py --host 127.0.0.1 --port 2000`。 |
| `carla_simulate_project\scripts\autopilot_simulation.py` | vehicle、walker、cyclistをspawnする初期検証用。 | CARLA World | CARLA actor | CARLA API | docstringにはCARLA 0.10.0とあるため、現在の0.9.16環境では互換性確認が必要である。 |
| `carla_simulate_project\scripts\vehicle_state_stream.py` | World内の既存actor状態をCSVへ記録する。actor生成は行わない。 | CARLA World | `vehicle_states.csv`、任意でtiming CSV | CARLA API | `--mode wait`または`--mode on-tick`。出力先ディレクトリは事前に存在する必要がある。 |
| `carla_simulate_project\scripts\convert_vehicle_state_csv.py` | `vehicle_state_stream.py`のCSVをreplay用の縮約CSVへ変換する。 | source CSV | `frame,id,type,x,y,z`列のCSV | Python標準CSV | `id`列をそのままreplay用IDに使う。`--min-spawn-z`既定値は`0.2`。 |
| `carla_simulate_project\send_data\send_udp_frames_from_csv.py` | 縮約CSVをframeごとにまとめてUDP送信する。 | replay用CSV | UDP JSON payload | `send_data\send_udp_from_csv.py` | `--start-frame`、`--end-frame`、`--frame-stride`、`--interval`を使う。 |
| `carla_simulate_project\scripts\udp_replay\replay_from_udp_future_exp.py` | UDP受信、CARLA上のactor再現、future-mode切替、collision sensor、actor log出力。 | UDP payload | `actor.csv`、`collisions.csv`、`id_map.csv`、`meta.json`、`control_state.json` | CARLA API、任意でtorch | `--future-mode autopilot`、`--switch-payload-frame`、`--future-duration-sec`等を持つ。 |
| `carla_simulate_project\scripts\run_trajectory_autopilot_pipeline.py` | 交通生成、記録、変換、replay、Autopilot、可視化を一括実行する補助オーケストレータ。 | CARLA、既存スクリプト | Dドライブのrunフォルダ、ログ、図 | 上記先輩コード群 | 自作補助。既存コード本体は変更せず呼び出す。 |
| `carla_simulate_project\run_trajectory_autopilot_pipeline.ps1/.bat` | 一括実行用PowerShell/バッチ入口。 | CLI引数 | 上記pipeline出力 | `scripts\run_trajectory_autopilot_pipeline.py` | Pythonは`C:\CARLA\venv312\Scripts\python.exe`を使用する。 |
| `carla_simulate_project\scripts\plot_csv_replay_autopilot_overlay.py` | 縮約CSVのtracking軌跡とreplay後のAutopilot軌跡をTown10HD_Opt道路中心線上に重ねる。 | `vehicle_states_reduced.csv`、`actor.csv`、`meta.json`、任意で`collisions.csv` | PNG、PDF、summary md | CARLA Map topology、matplotlib | 1秒マーカー、進行方向、切替点、collision/accidentマーカー、図中サマリーを描画する。 |
| `carla_simulate_project\scripts\plot_vehicle_trajectories.py` | 走行軌跡CSVの静止画プロット。`control_mode`で区間分割し、1秒ごとの位置マーカーを表示する。 | 1つ以上のCSV | PNGまたは画面表示 | matplotlib | `--fixed-delta-seconds`、`--time-marker-interval-s`実装済み。 |
| `carla_simulate_project\scripts\animate_vehicle_trajectories.py` | 車両軌跡CSVから動画またはGIFを生成する。 | `vehicle_state_stream.py`出力CSV | mp4/gif等 | matplotlib、任意でimageio_ffmpeg | 可視化補助。 |
| `carla_simulate_project\scripts\extract_future_accidents.py` | `results/.../logs/collisions.csv`からfuture accidentを抽出する。 | results root | `future_accidents_ge1000.csv`等 | `meta.json`、`collisions.csv` | `--threshold`既定値1000、`--require-is-accident`あり。 |
| `carla_simulate_project\scripts\actor_quality_metrics.py` | replay/future後のactor品質をCARLA map上で評価する。 | results root、CARLA map | per-run/aggregate CSV | CARLA waypoint API、`actor.csv`、`meta.json` | lane distance、加速度、yaw rate等を評価する。 |
| `research_ogm_project\scripts\run_ego_ogm.py` | Ego OGMラッパー。 | scenario JSON | Ego/World OGM出力 | `src\ogm_project\ego_ogm_runner.py` -> `ego_ogm_compat.py` | ラッパーである。 |
| `research_ogm_project\scripts\run_coop_comm.py` | 協調OGMラッパー。 | scenario JSON、static mask | Ego/RSU/Fused OGM、metrics | `src\ogm_project\cooperative_runner.py` -> `coop_comm_compat.py` | ラッパーである。 |
| `research_ogm_project\scripts\build_static_mask.py` | 静的マスク生成ラッパー。 | CARLA HD map | `D:\CARLA_DATA\masks\static_mask.npy` | `src\ogm_project\legacy_runner.py` -> `legacy\build_static_mask_from_hdmap_V3.py` | 元legacy本体をpatchして実行する。 |
| `research_ogm_project\scripts\show_spectator_pose.py` | Spectator pose確認ラッパー。 | CARLA World | 標準出力 | `src\ogm_project\legacy_runner.py` | 表示確認用。 |
| `research_ogm_project\configs\scenarios.json` | OGM実験のシナリオ設定。 | なし | 実行時設定 | `scenario_loader.py`、compat実装 | `scenario_A/B/C`を持つ。candidate_01のOGM本組み込みは未確認である。 |

未使用または旧版と思われるものとして、`carla_simulate_project\reference_sources\kamioka_senpai_program\...`配下には先輩由来の参照元が存在する。これは自作成果物として扱わない前提である。また、リポジトリ直下`README.md`の日本語部分は文字化けしており、内容確認には英語部分または各プロジェクトREADMEを参照する必要がある。

## 4. デジタルツイン／危険予測実験の現状

`Run_DT_Risk_V1.py`は、`--mode capture`、`--mode predict`、`--mode all`を持つ。既定値は、host `127.0.0.1`、port `2000`、map `Town10HD_Opt`、固定タイムステップ`0.05`秒、capture duration `10.0`秒、prediction seconds `8.0`秒、Traffic Manager port `8000`、Traffic Manager seed `42`である。

`capture`では、現在World内に存在する`vehicle.*` actorを列挙し、同期モードでtickしながら`capture_states.csv`へ書き出す。`--snapshot-at`を指定しない場合、capture終了時刻付近をsnapshotとして、コード上は`snapshot_states.csv`へ保存する。列は`ActorState` dataclassに基づき、phase、carla_frame、sim_time_s、logical_actor_id、source_actor_id、role_name、blueprint_id、位置、姿勢、速度、speed_mps、autopilot_enabledを含む。

`predict`では、`snapshot_states.csv`を読み込み、必要に応じて`--reload-world`でmapを読み直し、snapshot poseからvehicleをspawnする。spawn直後にsnapshotの速度を`set_target_velocity`で与え、その後`actor.set_autopilot(True, args.tm_port)`でTraffic Managerに制御を渡す。各再構成actorには、コード上は`collision sensor`を付与し、衝突を`risk_events.csv`と`risk_regions.csv`へ保存する。

near miss判定は、全vehicleペアのXY距離が`--near-miss-distance-m`以下、かつ互いに近づく相対速度が`--min-closing-speed-mps`以上の場合に記録する。既定値は距離`4.0 m`、接近速度`1.0 m/s`、cooldown`1.0 s`である。risk regionの半径はcollisionが`3.0 m`、near_missが`4.0 m`である。

再現性に関係する設定として、同期モード、固定タイムステップ、Traffic Manager seed、snapshot時刻、map名、reload有無がある。ただし、Autopilotの挙動はTraffic Manager内部の経路選択・spawn状態・周辺環境に依存するため、同じsnapshotから完全に同じ未来軌跡になるとは限らない。

## 5. 車両軌跡・Autopilot切替に関する現状と課題

先輩コード系では、`vehicle_state_stream.py`で記録したCSVを`convert_vehicle_state_csv.py`で`frame,id,type,x,y,z`へ変換し、`send_udp_frames_from_csv.py`でUDP送信する。`replay_from_udp_future_exp.py`はUDP payloadを受信し、tracking中はCSVに基づいてactorのposeを更新する。指定した`--switch-payload-frame`またはlead/end frame条件でfuture modeへ入り、`--future-mode autopilot`の場合は既存のvehicle/bicycle actorに`set_autopilot(True)`を適用する。

一括実行補助`run_trajectory_autopilot_pipeline.py`では、交通生成、CSV記録、交通actor cleanup、CSV変換、replay、Autopilot切替、道路重ね合わせ図生成を順に行う。既定出力は次のとおりである。

```text
D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\<RUN_TAG>\
D:\CARLA_DATA\DT_RiskPrediction\05_visualization_outputs\trajectory_map_overlay\<RUN_TAG>\
```

確認済みの既存ログとして、`cleanup_before_replay_test_20260702`ではCARLA 0.9.16、Town10HD_Opt、vehicle 8台、walker 0台、sensor 8台で記録され、replay後のcleanupはvehicle/walker/sensor=0/0/0であることが`pipeline_summary.json`から確認できる。ただし、これはパイプライン通過の確認であり、危険予測性能を示す正式な実験結果ではない。

重要な課題は、Autopilot切替時点で、CSV tracking中の個別運転特性が十分に引き継がれないことである。引き継いでいるものは主に現在位置、姿勢、速度である。一方、ドライバごとの加減速傾向、車間保持傾向、車線変更意図、交差点での進行方向、目的地、ルート意図は標準Autopilotへ渡していない。したがって、切替後の軌跡は「元軌跡の自然な延長」ではなく、「同じ位置・速度からCARLA Traffic Managerが選んだ未来挙動」になる。この課題は、事故再現、危険予測、risk mask生成、OGM通信評価のすべてに影響する。

また、UDP replayでは受信poll間隔とUDP送信intervalの関係が重要である。一括実行補助では、複数CSV frameが1 tick内で消費される問題を避けるため、`--poll-interval 0.02`、`--udp-interval 0.2`を既定としている。tracking表示用にはpre-roll frameを送ってから評価・描画開始する構成である。

## 6. OGM・協調通信実験の現状

`research_ogm_project\src\ogm_project\paths.py`では、データルートを`CARLA_DATA_ROOT`環境変数から取得し、未設定時は`D:\CARLA_DATA`を使う。主要パスは次のとおりである。

```text
DATA_ROOT = D:\CARLA_DATA
OUTPUT_ROOT = D:\CARLA_DATA\outputs
COOP_OUTPUT_ROOT = D:\CARLA_DATA\outputs\coop_comm
EGO_OUTPUT_ROOT = D:\CARLA_DATA\outputs\ego_ogm
MASK_ROOT = D:\CARLA_DATA\masks
STATIC_MASK_PATH = D:\CARLA_DATA\masks\static_mask.npy
LOG_ROOT = D:\CARLA_DATA\logs
```

Ego OGMは`run_ego_ogm.py`から`ego_ogm_runner.py`を経由し、`ego_ogm_compat.py`を実行する。出力は`EGO-OGM`と`WORLD-OGM`であり、PNG、heatmap、`metrics_grid.csv`、`run_meta.json`、PLY、`ego.mp4`、`world.mp4`を保存する。Ego単体版では`rsu.mp4`と`fused.mp4`は生成しない構成である。

協調OGMは`run_coop_comm.py`から`cooperative_runner.py`を経由し、`coop_comm_compat.py`を実行する。出力は`ego`、`rsu`、`fused`のPNG、`metrics_comm.csv`、`metrics_grid.csv`、PLY等である。`fuse_logodds_prefer_ego()`はEgo既知セルを優先し、Ego unknownかつRSU knownのセルをRSUで補完する。通信量は`SimulatedChannel`または互換実装内のchannelで`tx_bytes`、`rx_bytes`、queue長として記録する。

静的マスクについては、`build_static_mask.py`が`legacy\build_static_mask_from_hdmap_V3.py`を`legacy_runner.py`経由で実行し、`D:\CARLA_DATA\masks\static_mask.npy`へ保存する。協調OGMでは`USE_STATIC_MASK`が有効な場合、RSU送信前に`grid_to_send[static_mask] = L0`として静的マスク領域を送信グリッドから抑制する処理が確認できる。ただし、これは現状の静的道路・固定物マスクによる符号化対象抑制であり、将来の危険領域に基づくdynamic risk maskとは別物である。

混同してはいけない点として、「マスクを使ってPNG可視化の色やlogodds符号化対象を抑える処理」と、「実通信で送信対象セルを危険度に応じて選択する処理」は同じではない。現状コードでは、risk regionをOGM座標へ変換して送信セル選択へ接続する処理は未実装である。

Spectator追従は`src\ogm_project\spectator_utils.py`に共通化され、`topdown`と`chase`を持つ。既定は`topdown`、高さ`35.0 m`、`ENABLE_REALTIME_PREVIEW=False`である。これは観察用であり、OGM生成・通信・評価値には使わない。

## 7. 既存の評価・可視化ツール

`plot_vehicle_trajectories.py`は、CSVの`control_mode`列を見て軌跡を分割し、`autopilot`は赤、`direct`等は青系として描画する。コード上、`sim_time_s`列があればそれを優先し、なければ`prediction_time_s`を使い、それもなければ`frame * --fixed-delta-seconds`で時刻を作る。`--time-marker-interval-s`は既定`1.0`秒で、0を指定すると時間マーカーを無効化する。`--no-endpoints`は開始点・終了点の強調表示だけを無効化し、時間マーカーには影響しない実装である。

`animate_vehicle_trajectories.py`は、軌跡CSVから時系列アニメーションを作成する。`--fps`既定値は20、`--dpi`既定値は150である。出力形式は拡張子によりGIFまたはMP4等を想定している。

`plot_csv_replay_autopilot_overlay.py`は、CARLAに接続してTown10HD_Optの`world_map.get_topology()`からDriving laneの中心線を抽出し、その上にtracking区間とAutopilot区間を重ねる。点線がCSV tracking input、実線がAutopilot replay output、丸が指定秒間隔の位置、矢印が進行方向、星印が切替frame、X印がcollision/accidentログである。PNGに加えてPDFも保存する。背景は道路中心線であり、車線幅、進行方向矢印付きのHD lane boundary、実写俯瞰画像ではない。そのため、右側通行・左側通行の厳密な視覚判断には注意が必要である。

2026-07-03時点で、`plot_csv_replay_autopilot_overlay.py`には`--time-marker-interval-s`、`--label-time-markers`、`--label-events`、`--no-pdf`、`--no-summary-box`が追加されている。既定では1秒ごとの位置マーカー、進行方向矢印、開始点、切替点、終了点、collision/accidentマーカー、図中サマリーボックスを描画し、PNG/PDFの両方を保存する。`run_trajectory_autopilot_pipeline.py`の`pipeline_summary.json`もPNG/PDFの両方を`plot_outputs`へ記録する。

2026-07-03追記として、同スクリプトにCARLA lane方向診断を追加した。`world_to_plot()`は恒等変換であり、道路背景と車両軌跡の両方に同じCARLA world座標`x,y`を使う。`--show-lane-arrows`、`--show-lane-boundaries`、`--show-lane-ids`、`--write-lane-diagnostics`、`--show-lane-violations`により、lane進行方向、推定lane boundary、lane ID、診断CSV/JSON、逆走候補・lane外候補を表示または保存できる。さらに`--carla-topdown-view`を指定すると、すべての描画後にMatplotlibのY軸だけを反転し、CARLA +Yを画面下方向に表示する。これにより、CSVやwaypoint診断のworld座標値は変えずに、CARLAウィンドウの俯瞰に近い向きで道路重ね合わせ図を読める。

`extract_future_accidents.py`は、`.../<method>/lead_<N>/rep_<N>/logs/collisions.csv`と`meta.json`を走査し、衝撃量threshold以上のfuture accident候補をCSV化する。`--require-is-accident`を指定すると`is_accident==1`を要求する。

`actor_quality_metrics.py`は、同じresults構造を対象に、switch後のactor軌跡についてCARLA waypointとの距離、off-road、lane中心からの距離、加速度、yaw rate等を評価する。CARLA接続を必要とする。

評価スクリプト`evaluation_accident\summarize_hotspots_with_nearmiss.py`も既存後処理として利用されているが、本ドキュメント作成時の必須対象外であり、詳細は過去のPhase Dレポートを参照する必要がある。

## 8. 現時点で確認できる成果

コードから実装済みと判断できるものは次のとおりである。

- CARLA 0.9.16へ接続し、Town10HD_Optを前提にした実行系が存在する。
- 交通生成、CSV記録、replay用CSV変換、UDP replay、Autopilot切替、collision sensorログ出力の処理連鎖が存在する。
- `cleanup_before_replay_test_20260702`の`pipeline_summary.json`では、8台記録、replay後cleanup成功、CARLA client/server 0.9.16、Town10HD_Optが確認できる。
- Ego OGM、協調OGM、静的マスク出力先は`D:\CARLA_DATA`配下へ統一されている。
- `plot_vehicle_trajectories.py`の1秒ごと位置マーカーはコード上実装済みである。

動作確認済みとまでは言えるが、正式な実験結果として確定していないものは次のとおりである。

- Autopilot切替後の軌跡が実交通軌跡の将来区間をどの程度再現するか。
- risk regionをOGM通信制御へ接続した場合の通信量削減効果。
- candidate_01の死角シナリオを既存OGM実験へ正式に組み込んだ結果。
- near missやcollisionが安定して発生する危険シナリオでの評価結果。

数値結果がコードやログから確認できない項目については、結果ログ未確認である。

## 9. 未解決課題・技術的リスク

| 優先度 | 課題 | 内容 |
|---|---|---|
| High | Autopilot切替時の個別運転特性消失 | 位置・姿勢・速度は引き継ぐが、車両ごとの加減速傾向、車間、車線変更意図、交差点での進路意図は標準Autopilotへ渡していない。事故再現性に直結する。 |
| High | 交差点での進路意図の欠落 | CSV上の車両がどの方向へ進む予定だったかをTraffic Managerに渡していない。右左折・直進が変わると危険予測結果が変わる。 |
| High | risk regionとOGM通信制御の未接続 | `risk_events.csv`や`risk_regions.csv`は存在するが、Ego/RSU OGMの実送信セル選択へ変換する処理は未実装である。 |
| High | 静的マスクと通信セル選択の意味の混同 | 静的マスクによる可視化・符号化抑制と、危険度に基づく送信優先制御は別である。論文・発表で混同しない設計整理が必要である。 |
| Medium | snapshot再構成時のspawn重なり | `Run_DT_Risk_V1.py`はz offsetを試すが、密なsnapshotではspawn失敗や重なりが起こり得る。 |
| Medium | 初速度の扱い | `set_target_velocity`は行うが、その後のAutopilot挙動が元軌跡の加速度・制御履歴を保持するわけではない。 |
| Medium | Autopilotのランダム性 | TM seedは設定可能だが、周辺actor状態、map、同期設定に依存する。再現実験ではrun条件固定が必要である。 |
| Medium | 実軌跡との比較評価不足 | replay後future軌跡と実際の将来区間の位置、速度、加速度、TTC比較が体系化されていない。 |
| Medium | 可視化summaryの文字化け | `plot_csv_replay_autopilot_overlay.py`内の文字化けした日本語summary行は2026-07-03に英語のUTF-8文へ置換済みである。ただし、root READMEなど他ファイルの文字化けは残る。 |
| Medium | collision/near miss表示の意味 | `plot_csv_replay_autopilot_overlay.py`は`collisions.csv`の`is_accident=1`をcollision/accidentとして表示する。near miss評価CSVを同じ図へ重ねる処理はまだ未実装である。 |
| Medium | 図上で左側通行に見える問題 | CARLA world座標の`+Y`は車両が`+X`を向くと右側に対応するが、Matplotlib標準表示では`+Y`が画面上方向になるため、通常の俯瞰図として見ると左右が直感と逆に見える。2026-07-03に`--carla-topdown-view`を追加し、表示時だけY軸を反転できるようにした。CSV座標、waypoint診断、`lane_alignment_diagnostics.csv`、`lane_alignment_summary.json`は従来どおりCARLA world座標である。 |
| Low | root README日本語部分の文字化け | リポジトリ直下`README.md`の日本語部が文字化けしている。研究実行には直接影響しないが、引き継ぎ性を下げる。 |
| Low | 先輩由来コードと自作補助コードの境界 | `carla_simulate_project`内に参照元と補助スクリプトが共存する。GitHub公開対象の管理に注意が必要である。 |

## 10. 次に行うべきこと

### 最優先

- `cleanup_before_replay_test_20260702`のような一括実行条件を、run tag、seed、車両台数、tracking frame、future duration、固定delta、UDP interval込みで再現可能な標準ベースラインとして固定する。
- `actor.csv`、`meta.json`、`vehicle_states_reduced.csv`を用いて、tracking区間とAutopilot区間の境界、actor数、control_mode、cleanup結果を自動検査する。
- 強化済みの`plot_csv_replay_autopilot_overlay.py`を標準可視化として使い、PNG/PDF、1秒マーカー、進行方向、切替点、collision位置を毎runで保存する。
- 可視化時は`lane_alignment_diagnostics.csv`と`lane_alignment_summary.json`も保存し、図上の左右ではなくCARLA waypoint基準でlane整合、逆向き候補、lane外候補を確認する。
- 道路重ね合わせ図は標準で`--carla-topdown-view`を使い、CARLA +Yが画面下方向になる俯瞰表示として保存する。診断CSV/JSONを読む場合は、表示反転ではなくCARLA world座標の値として解釈する。
- 標準Autopilotをベースラインとして保存し、「現在の実装は標準Autopilotへ切り替えた場合の結果である」と明示する。
- candidate_01をOGMシナリオへ組み込む前に、Ego/Target/RSUの座標、速度、時間、可視性条件を`scenarios.json`または専用設定へ再現可能に固定する。

### 次点

- 切替前履歴から速度、加速度、車間、車線変更傾向、yaw rateを抽出する。
- 車両ごとにTraffic Manager設定を変える校正済みAutopilotを検討する。
- 実軌跡の将来区間がある場合、Autopilot futureとの位置誤差、速度誤差、加速度、TTCを比較する。
- `risk_regions.csv`のworld座標をEgo中心OGM grid座標へ変換する補助処理を設計する。
- Phase D等で生成されるnear missイベントCSVを、道路重ね合わせ図へ重ねる入力形式として整理する。

### 将来

- IDM等による個別追従モデルを導入する。
- 交差点での進路意図推定を行う。
- 学習型の運転スタイル継承を検討する。
- risk maskを用いたOGM通信優先制御を実装する。
- ns-3等を用いた実通信シミュレーション連携を検討する。

## 11. 実行方法

コードから確認できた代表的な実行方法のみを記載する。

CARLA起動条件は、CARLA 0.9.16、Town10HD_Opt、RPC port 2000である。手動起動例は次のとおりである。

```powershell
C:\CARLA\CarlaUE4.exe -d3d11 -windowed -carla-rpc-port=2000
```

自作DT risk V1の環境確認:

```powershell
cd C:\CARLA\user_projects\dt_risk_prediction_project
C:\CARLA\venv312\Scripts\python.exe check_dt_risk_environment.py --host 127.0.0.1 --port 2000
```

自作DT risk V1のcapture:

```powershell
cd C:\CARLA\user_projects\dt_risk_prediction_project
C:\CARLA\venv312\Scripts\python.exe Run_DT_Risk_V1.py --mode capture --host 127.0.0.1 --port 2000 --duration 10
```

自作DT risk V1のpredict:

```powershell
cd C:\CARLA\user_projects\dt_risk_prediction_project
C:\CARLA\venv312\Scripts\python.exe Run_DT_Risk_V1.py --mode predict --snapshot "D:\CARLA_DATA\DT_RiskPrediction\runs\<run_id>\snapshot_states.csv" --prediction-seconds 8
```

車両軌跡記録からAutopilot切替、図生成までの一括実行:

```powershell
cd C:\CARLA\user_projects\carla_simulate_project
.\run_trajectory_autopilot_pipeline.bat
```

パラメータ変更例:

```powershell
.\run_trajectory_autopilot_pipeline.ps1 `
  -VehicleCount 8 `
  -Seed 42 `
  -RecordSeconds 30 `
  -TrackingFrames 100 `
  -FutureDurationSec 30 `
  -RunTag "my_run"
```

手動での状態CSV記録:

```powershell
cd C:\CARLA\user_projects\carla_simulate_project
New-Item -ItemType Directory -Force "D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\manual_run" | Out-Null
C:\CARLA\venv312\Scripts\python.exe scripts\vehicle_state_stream.py `
  --host 127.0.0.1 `
  --port 2000 `
  --mode wait `
  --include-velocity `
  --frame-elapsed `
  --wall-clock `
  --include-object-id `
  --output "D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\manual_run\vehicle_states.csv" `
  --timing-output "D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\manual_run\vehicle_state_timing.csv"
```

OGM実験の代表入口:

```powershell
cd C:\CARLA\user_projects\research_ogm_project
C:\CARLA\venv312\Scripts\python.exe scripts\build_static_mask.py
C:\CARLA\venv312\Scripts\python.exe scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
C:\CARLA\venv312\Scripts\python.exe scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

OGM用PowerShell自動実行:

```powershell
cd C:\CARLA\user_projects\research_ogm_project
.\run_experiment.ps1 -Mode mask
.\run_experiment.ps1 -Mode ego -Scenario scenario_A
.\run_experiment.ps1 -Mode coop -Scenario scenario_A
.\run_experiment.ps1 -Mode spectator
```

未確認の点として、現在のCARLA起動状態で上記すべてのコマンドを本ドキュメント作成時に再実行したわけではない。ここではコードと既存ログから確認できる実行方法を記載している。

## 12. ChatGPT／次の開発者向けの引き継ぎ要約

このリポジトリは、CARLA上の車両軌跡を記録・再生し、Autopilotで数秒先を進めて危険接近や衝突を検出するデジタルツイン系と、Ego/RSUのOGM共有により死角を補完する協調認識系を並行して扱っている。`dt_risk_prediction_project`は自作のDT risk V1で、capture、snapshot、predict、risk event出力までを1本の入口で実装している。`carla_simulate_project`は先輩研究コードを利用したCSV記録、UDP replay、Autopilot切替、後処理・可視化の系統であり、一括実行補助も追加されている。`research_ogm_project`はOGM系の整理済みプロジェクトで、出力先は`D:\CARLA_DATA`配下に統一されている。現状の最大課題は、Autopilot切替後に車両ごとの運転特性・進路意図が失われること、およびrisk regionがOGM通信セル選択へまだ接続されていないことである。次は、標準Autopilotベースラインを再現可能に固定し、candidate_01をOGMシナリオへ安全に組み込み、Ego単独とRSU共有の差を定量化する段階である。

## 付録: Git状態

初回文書作成前の`git status --short`は空であり、未コミット変更は確認されなかった。2026-07-03の可視化改善後は、`CURRENT_PROGRESS.md`、`carla_simulate_project\scripts\plot_csv_replay_autopilot_overlay.py`、`carla_simulate_project\scripts\run_trajectory_autopilot_pipeline.py`が変更対象である。

## 付録: 2026-07-09 パイプライン再実行

`seed=42`、車両8台、記録10秒、tracking 100 frame、future 30秒で一括パイプラインを再実行した。最終runは
`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\rerun_fixed_20260709_102916`
であり、可視化は
`D:\CARLA_DATA\DT_RiskPrediction\05_visualization_outputs\trajectory_map_overlay\rerun_fixed_20260709_102916`
へ保存した。

8台すべてについてtracking区間とAutopilot区間が`actor.csv`に記録され、切替payload frameは33242、future modeはautopilot、future durationは30秒であった。切替前後の位置差は最大0.083 mであり、Actor消失と大きな位置ジャンプは確認されなかった。logical ID 7の移動量が小さい点はreplay処理ではなく、元の交通生成CSVですでにほぼ停止していたことに由来する。

交通生成時の`denger_traffic.py`が出力する`collisions.csv`は従来プロジェクト直下へ保存され、runの来歴から外れていた。自作オーケストレータを修正し、交通生成プロセスのworking directoryをrunフォルダに変更したうえで、ログを`traffic_generation_collisions.csv`として保存し、行数と絶対パスを`pipeline_summary.json`へ記録するようにした。最終runでは交通生成時collisionが10行、UDP replay時collisionが0行であった。この差は、元交通の衝突がreplayで再現されなかったことを示すため、今後の再現性評価で区別して扱う必要がある。

lane診断では3208点すべてがdriving lane上にあり、lane escape candidateは0、reverse candidateは59点であった。道路重ね合わせ図は`--carla-topdown-view`を用い、PNG/PDFの両方を生成した。CARLA +Yを画面下方向に表示する処理は表示だけに適用され、診断値はCARLA world座標のままである。

## 付録: 2026-07-09 collision再現性監査

基準run `rerun_fixed_20260709_102916` に対し、元交通のcollisionとUDP replay + Autopilot側の軌跡をlogical IDで対応付けて監査した。元collisionはsource Actor 56/60、logical ID 1/4の1ペアであり、5つの物理接触イベントが両側collision sensorから報告されたため10行となっていた。`is_accident=1`はframe 33107と33112の2イベント、両側報告を合わせて4行である。

元collisionの最初のframe 33107はsender開始33112より前であり、frame 33112を含む残りの接触もreplay側の最初の完全なpayload frame 33123より前であった。したがって、元collisionへ至る接近区間がreplayへ入力されておらず、replay collision 0件の主因は標準Autopilotの未来挙動ではなくtracking replay開始の遅さである。元交通の最小中心間距離は3.305 m、replay側は5.900 mであった。車体外形を含むsensor接触であるため、中心間距離が0 mでないこと自体は矛盾ではない。

監査用として`scripts\audit_collision_reproduction.py`と`scripts\audit_reverse_candidates.py`を追加した。一括オーケストレータは道路重ね合わせ図の生成後に両監査を自動実行し、run内の`audit`へCSV、JSON、PNG、PDF、標準出力・標準エラーを保存する。自動監査付きrun `collision_audit_20260709_113159`でもパイプラインと監査は正常終了した。このrunでも元collision 10行、replay collision 0行であり、最初のcollision frame 731089に対して最初の完全なreplay payloadは731100であった。

基準runのreverse candidate 59点は、低速・停止近傍20点、junction内21点、非junctionかつ非低速で要確認18点に分類された。要確認18点はすべてlogical ID 8のAutopilot開始後1.3～3.4秒に集中し、推定速度0.55～1.80 m/s、heading error絶対値約135～175度であった。前者2分類を直ちに逆走と解釈してはならない。logical ID 8の18点は、Autopilot切替直後の向き回復または実際の進行方向不整合として、軌跡とCARLA画面を追加確認する必要がある。

## 付録: 2026-07-09 デジタルツイン再現・未来予測評価

研究上の主目的を事故再現から、CSV交通状態のCARLA再構築と標準Autopilotによる未来予測へ整理した。`scripts\evaluate_digital_twin_pipeline.py`を追加し、tracking再現精度、Autopilot切替連続性、30秒future品質、道路重ね合わせ図をrunごとに出力する。一括オーケストレータはこの評価を標準後処理として実行し、主要summaryを`pipeline_summary.json`へ格納する。collisionは補助情報として別監査に残すが、future品質の合否には使用しない。

基準run `rerun_fixed_20260709_102916`の評価により、従来のreplay用CSVが`frame,id,type,x,y,z`だけで、元CSVのyawと速度を失っていたことが判明した。位置誤差は平均0.248 mであったがyaw誤差は平均33.2度、最大約180度であり、位置だけを再現する構成であった。このため`convert_vehicle_state_csv.py`へ後方互換な`--include-motion`を追加し、一括パイプラインではroll、pitch、yaw、velocity_x/y/zを保持するようにした。UDP senderは存在するmotion列だけをpayloadへ追加し、replay側は値がある場合にyawと速度を適用する。従来の6列CSVと既存CLIは引き続き利用できる。

改善確認runは`digital_twin_eval_20260709_120312`である。replay actor logはpayload適用後の1 tick後に記録されるため、replay frame Nをsource frame N+1と対応付けて評価した。8台、115 frame、920サンプルについて、位置誤差は平均0.0553 m・最大0.1609 m、速度誤差は平均0.0807 m/s・最大2.2965 m/s、yaw誤差は平均0.188度・最大2.606度であり、ID欠落は0であった。

Autopilot切替では8台すべてが残り、位置ジャンプは平均0.139 m・最大0.323 m、yawジャンプは平均1.55度・最大3.78度、lane変更とlane escapeは0であった。一方、速度ジャンプは平均4.09 m/s・最大7.34 m/sで、3 m/s超が5台、2 m/s超から0.5 m/s未満への低下が2台あった。したがって空間・姿勢連続性は合格だが、運動学的連続性は未達である。

30秒futureでは全8台が最後まで存在し、全2400診断点がDriving lane上、lane escape 0、5 m超の突然の位置ジャンプ0、完全停止Actor 0であった。future reverse candidate 36点は低速6点、junction内30点で、切替直後回復と追加確認必須の候補は0であった。改善前にlogical ID 8で見られた非junctionの向き不一致は、yaw保持後には再発していない。

現段階のパイプラインは、CSVから交通状態をCARLA上へ再構築し、標準Autopilotへ切り替えて道路上の未来挙動を生成するベースラインとして使用できる。ただし切替直後の速度連続性と、実軌跡futureに対する予測誤差の検証が残るため、校正済み・高精度な未来予測器としては未完成である。

## 付録: 2026-07-13 Autopilot切替速度の再監査と3条件比較

以前報告した切替速度ジャンプ平均4.09 m/s・最大7.34 m/sは、tracking末尾とAutopilot先頭の位置差から導出した速度を、payload適用後1 tickのログ位相差を補正せず比較した評価誤差であった。`actor.csv`へ実際の`actor.get_velocity()`、control throttle/brake/steer、信号状態、前走車距離、切替直前・`set_autopilot(True)`直後の速度ベクトル、Autopilot tick番号を追加し、連続CARLA tick間で再評価した。

新規runは`speed_switch_baseline_20260713_042856`、`speed_switch_preserve_20260713_043140`、`speed_switch_tm_calibrated_20260713_043348`である。baselineでは速度保持とTM速度校正を無効、preserveでは切替直前に最後のtracking目標速度を再適用、TM calibratedではこれに車両別TM速度差設定を追加した。

baselineの1 tick速度ジャンプは平均0.0545 m/s・最大0.2400 m/s、3 m/s超0台、ほぼ停止への低下0台であった。`set_autopilot(True)`直前と直後の速度は全Actorで一致し、最初のtickでもbrakeは0であった。赤信号車両は切替前から約0.98 m/sの低速であり、TM切替による急停止ではない。

preserve条件は平均0.9058 m/s・最大1.6654 m/s、TM calibrated条件は平均1.0698 m/s・最大2.2197 m/sとなり、いずれもbaselineより悪化した。最後のtracking payloadが持つ目標速度と切替瞬間の物理Actor実速度には差があり、`set_target_velocity(last_tracking_velocity)`による再上書きが最初のtickの加速ジャンプを生んだ。TM速度校正は1秒後の平均誤差を一部抑えたが、最初のtick連続性を改善せず採用条件を満たさなかった。

3条件ともlane escape 0、reverse要確認0、30秒後生存8台であり、道路上のfuture品質に破綻はなかった。標準設定には両改善フラグOFFのbaselineを採用する。`--preserve-switch-velocity`と`--tm-speed-calibration`は比較実験用の任意CLIとして残す。最終結論はD、すなわち大きな速度ジャンプは評価タイミングの問題であり、制御自体は正常である。
## 追補: AutoPilot切替速度連続性の5方式監査（2026-07-13）

同一の `vehicle_states.csv` / `vehicle_states_reduced.csv` を固定し、baseline、preserve_velocity、two_phase、tm_calibrated、two_phase_tm_calibratedの5方式を新規runで比較した。固定条件はTown10HD_Opt、START_FRAME=352、SWITCH_FRAME=482、fixed delta=0.1 s、future=30 s、TM seed=42、8台である。

旧runで報告された平均4.09 m/s、最大7.34 m/sの速度ジャンプは、位置差分由来速度を異なるログ位相で比較した評価ずれが主因であった。`actor.get_velocity()` を用いて最後のtracking tickと最初のpost-switch tickを比較すると、baselineは平均0.0545 m/s、最大0.2400 m/sで、3 m/s超およびほぼ停止は0台であった。`set_autopilot()` の同一tick前後速度は全Actorで一致した。

preserve_velocityとtm_calibratedはbaselineを改善しなかった。two_phaseは平均1.1029 m/s、最大2.6740 m/s、two_phase_tm_calibratedは平均0.7794 m/s、最大1.6524 m/sとなり、余分なsettle tickにより位置差・yaw差も増えた。全方式でActor消失0、lane escape 0、30秒後生存8台、Future品質passであった。したがって標準パイプラインは互換性のある `--switch-handoff-mode baseline` を採用し、他方式は実験オプションに留める。

Actor別監査では主因を全台A（評価frameまたは時刻の対応ずれ）と判定した。切替後2秒内のbrake、赤信号、junction、低速状態は副次的な通常TM挙動であり、切替同一tickの速度消失ではない。残る課題は、標準AutoPilotが車両固有の運転特性・進路意図を継承しない点である。

成果物は `D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\switch_speed_experiment_20260713_190211` に保存した。

## 追補: 元CSVを正解とした標準AutoPilot未来予測baseline（2026-07-14）

30秒間新規記録したsource CSVを固定し、frame 9415～9514の10秒を観測区間、frame 9515～9614の10秒を正解未来区間として分離した。UDP senderはframe 9514で停止し、正解未来区間は評価時にだけ使用した。評価runは`future_prediction_blueprint_baseline_20260714_004001`で、MapはTown10HD_Opt、車両8台、seed/TM seed=42、fixed delta=0.1秒、切替方式はbaselineである。

初回評価ではreplayが全車をLincoln MKZとしてspawnしていることが判明したため、`blueprint_id`を任意列として変換・UDP送信・replay spawnまで保持する後方互換対応を追加した。最終runではlogical ID 1～8、source actor ID、replay actor ID、元blueprintがすべて一意に対応し、blueprint一致8/8、除外0である。

予測時間別の平均位置誤差は1秒1.066 m、3秒3.559 m、5秒7.447 m、10秒15.638 mであった。全区間ADEは7.569 m、平均速度絶対誤差2.103 m/s、平均yaw誤差11.971度、road一致率70.4%、lane一致率69.1%、進路一致率62.5%である。1秒時点のroad/lane一致率は100%だが、10秒時点ではroad 50%、lane 37.5%へ低下した。Actor生存率100%、Driving lane率100%、lane escape 0である。

Actor 1～3は10秒FDE 0.89～2.92 mで同じ進路を概ね維持した。Actor 4は左折方向自体は一致したが交差点通過タイミングとlaneが分岐した。Actor 5と7は元軌跡が右折/左折であるのにAutoPilotは直進し、Actor 8は元軌跡が直進であるのにAutoPilotは左折した。Actor 6は両方でほぼ停止した。したがって長時間誤差の主因は速度差だけではなく、目的進路、車線選択、車間・速度傾向を標準AutoPilotが継承しない点である。

`scripts\evaluate_future_prediction.py`を追加し、一括パイプラインには`--evaluate-future-prediction`、`--observation-seconds`、`--prediction-seconds`、`--prediction-horizons`、`--future-ground-truth-csv`を任意CLIとして統合した。未指定時は従来どおり評価を行わない。次は標準baselineを維持したまま、目的進路と切替前の速度・加速度・車間履歴を車両別に継承する提案手法との比較を設計する。

成果物は `D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\future_prediction_blueprint_baseline_20260714_004001\future_prediction_evaluation` に保存した。

## �Ǖ�: �o�H������r�Ƒ��s�������o�i2026-07-14�j

### Phase A: none / next_maneuver / full_route��r

�Œ�source��`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\route_signal_source_v7_none_r1_20260714`�ł���B`vehicle_states.csv`�A`vehicle_route_plans.json`�A`vehicle_route_waypoints.csv`�A`traffic_light_timeline.csv`�A�ؑ�frame 1170445�Afixed delta 0.1 s�Afuture 10 s�ATM seed 42�����ʂƂ��A3 mode���e3����s�����B�S9 run��8 Actor���������Acollision 0�Alane escape 0�A�I����cleanup�����ł������B

`next_maneuver`�́Aprediction�J�n�ʒu����ɂ���ŏ���junction maneuver������`TrafficManager.set_route()`��1 command�œn���BActor 2/3/4/8��right�AActor 5/6��straight�AActor 7��left�ł������BActor 1�ɂ͎c��maneuver���Ȃ��A�W��Autopilot�֖����I�Ƀt�H�[���o�b�N�����B��p���O��`next_maneuver_assignment_log.csv`�ł���B

��]���Q�ł���source route adherence�K�i���sActor�ilogical ID 2,3,4,6,7�j�̌��ʂ͎��̂Ƃ���ł���B

| mode | ADE [m] | FDE 1 s | FDE 3 s | FDE 5 s | FDE 10 s | route | road | lane | wrong turn |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| none | 2.027 | 1.216 | 2.233 | 2.706 | 2.702 | 1.000 | 0.902 | 0.902 | 0 |
| next_maneuver | 2.290 | 1.428 | 2.463 | 2.925 | 3.057 | 1.000 | 0.897 | 0.862 | 0 |
| full_route | 2.927 | 1.533 | 2.668 | 3.129 | 3.715 | 1.000 | 0.824 | 0.819 | 0 |

�SActor��ADE��none 1.454 m�Anext_maneuver 1.678 m�Afull_route 2.251 m�ł���BADE��3�����W���΍��͏���0.264�A0.200�A0.115 m�ł���B�����9 run�ł͑Smode��wrong turn��0�ł���Anext_maneuver���h�~���ׂ�wrong turn���̂��Č����Ȃ������Bnext_maneuver��Actor 6��ADE�̂�0.061 m���P���A��7 Actor�͈��������Bfull_route��Actor��ADE�����P���Ȃ������B

�M��timeline�͗v��state�ݒ�A�����ǖ߂��A`world.tick()`�Atick��ǖ߂��̏��Ō��؂����Bpost-tick��v����0.999926�i13,500����1���s��v�j�ł���B�s��v��full_route����3��transition tick 1���ŁA�e���m�F���a40 m���ɊY���ԗ��͂Ȃ��A�ԗ��e���s��0���ł������B��~��geometry��source�ɕۑ�����Ă��Ȃ����߁A�M����~�����B�E�ʉߎ����͐��肹�����񍐂Ƃ����B

�K�i���sActor�̕���junction�i�������덷��none 0.500 s�Anext_maneuver 0.500 s�Afull_route 1.408 s�A�ޏo�����덷��0.733 s�A0.767 s�A0.800 s�ł���B��~�J�n�����덷��0.156 s�A0.133 s�A-0.211 s�A���i�����덷��0.183 s�A0.150 s�A-0.400 s�ł���B�Smode��route��v��1.0�̂��߁A����̎c���͌o�H��Ԃ�葖�s�^�C�~���O�E�ԗ����ݍ�p�ɗR������B

�w��D�揇�ʂɏ]���W���o�Hmode��`none`�Ƃ���B���R�́Awrong turn��junction exit��v�������̏����ŁA1�`5�bFDE�AADE�Aroad/lane��v�����ŗǂ��������߂ł���B`next_maneuver`��`full_route`�́Awrong turn���Č�����ʃV�i���I�p�̔C�Ӕ�r�����Ƃ��Ďc���B

### Phase B: �ϑ���������̑��s����

frame 1170346�`1170445��10.0 s�A�eActor 100�T���v�����瑬�x�A���������x�Ejerk�A����road/lane���������̑O���Ԍ��Ajunction�O��lane change�Alane���S����̕����t�����΍��𒊏o�����B8 Actor��7 Actor��`partially_valid`�AActor 6�͊ϑ������̕��ϑ��x0 m/s�̂���`stationary_actor`�ł���B

source CSV�ɑ��x�������Ȃ��A�M����~��geometry���ۑ�����Ă��Ȃ����߁A���x������E���x���ߗ��E�M����������/�����͖��Z�o�ł���B���̊ϑ����ł͓���lane�O���Ԃ����o���ꂸ�Aheadway�ETHW�ETTC���⊮���������Ƃ����Bjunction����lane ID�ω���OpenDRIVE road�J�ڂ�lane change�Ƃ��Đ������A�SActor�̌��olane change��0���ł���B

`driving_behavior_tm_parameter_candidates.csv`�ɂ͊ϑ����ϑ��x�Abounded desired speed�A�O���ԋ������Alane change���Alane offset����ۑ��������ATraffic Manager�ւ͓K�p���Ă��Ȃ��B���̔�r�����֐i�ޑO�ɁA���x�����̎擾���@�ƑO���Ԃ����݂���ϑ���Ԃ��m�ۂ���K�v������B�ŏ��ɔ��f��������������͕��ϑ��x�ł���A�ԊԂ͗L���T���v���������Ă��爵���B

���ʕ���`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\route_mode_final_evaluation_20260714`�ɕۑ������B��v�t�@�C����`route_mode_final_summary.md`�A`route_mode_group_summary.csv`�A`junction_signal_event_timing.csv`�A`traffic_light_post_tick_validation.csv`�A`driving_behavior_features.csv`�A`driving_behavior_tm_parameter_candidates.csv`����ъePNG/PDF�ł���B

## 追補: 車両別希望速度を継承する未来予測実験（2026-07-14）

Town10HD_Opt上で同一blueprint `vehicle.tesla.model3` の8台へ既知の希望速度を設定し、slow 3台、normal 2台、fast 3台からなる固定sourceを生成した。使用APIは`TrafficManager.set_desired_speed`で、速度比は道路制約のためslow 0.30、normal 0.45、fast 0.60、制限速度30 km/hに対する希望速度は9.0、13.5、18.0 km/hである。warmup 5秒、観測10秒、正解future 10秒、fixed delta 0.1秒、seed/TM seed 42とした。正解プロファイルは`vehicle_behavior_profiles.json`へ保存し、観測推定には使用していない。

source precheckでは、観測平均速度比のslow-normal差0.141、normal-fast差0.132、slow-fast速度差2.275 m/sを確認した。Actor 1は同一laneの先行車影響により主評価から除外したが、各profileに2台以上の有効Actorが残りprecheckは合格した。観測CSVだけからのprofile分類精度は8/8、速度比MAE 0.00829、希望速度MAE 0.249 km/hである。

同一source CSV、観測区間、正解future、logical ID、spawn順、seedを固定し、route conditioningは`none`として、baseline、全車共通速度、車両別推定速度、車両別oracle速度を各3回実行した。UDP replayはActor初期化中の先頭frame欠落を避けるため、送信側へ`--initial-frame-repeat`を追加し、最初の観測snapshotを20回再送してから通常frameを送る。これはfuture情報を使用しない初期化処理である。12 runすべてで8 ActorへのAutopilot切替、future 10秒、cleanup、ground-truth評価が成功した。

主評価群であるprecheck合格7 Actorの3反復平均は次のとおりである。

| mode | speed MAE [m/s] | ADE [m] | FDE 1 s [m] | FDE 3 s [m] | FDE 5 s [m] | FDE 10 s [m] | road | lane |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 3.995 | 18.862 | 1.418 | 10.355 | 19.153 | 37.786 | 0.791 | 0.672 |
| common | 0.872 | 4.111 | 0.656 | 2.440 | 4.127 | 8.148 | 0.947 | 0.837 |
| inferred | 0.294 | 1.364 | 0.297 | 0.872 | 1.459 | 2.403 | 0.988 | 0.891 |
| oracle | 0.180 | 0.645 | 0.210 | 0.477 | 0.724 | 0.943 | 0.993 | 0.902 |

車両別推定はcommonに対して速度MAEを66.2%、ADEを66.8%改善した。baselineに対しては速度MAEを92.6%、ADEを92.8%改善した。oracleは診断用上限であり提案方式ではない。inferredはoracleまでのcommon基準の速度改善幅の約83.5%を達成したが、fast群ではinferred speed MAE 0.606 m/sに対してoracle 0.182 m/sであり、推定値またはTM応答の残差が大きい。slow、normal、fastの全群でcommonからADEは改善した。一方Actor 7ではinferred speed MAEがcommonよりわずかに悪化しており、全Actor一様の改善ではない。

追加実装は`scripts/generate_speed_profile_source.py`、`scripts/evaluate_speed_behavior_source.py`、`scripts/evaluate_speed_behavior_experiment.py`、replayの速度conditioning、pipelineの速度CLIと初期frame再送である。速度割当modeは`none`、`common`、`individual_inferred`、`individual_oracle`で、実適用値とfallback理由を速度割当ログへ保存する。既存CLIの既定値は速度conditioning無効であり、従来動作を維持する。

成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\speed_behavior_experiment_20260714`に保存した。主集計は`evaluation\speed_behavior_overall_summary.json`、`speed_behavior_actor_comparison.csv`、`speed_behavior_profile_summary.csv`、`speed_behavior_repeat_summary.csv`、`speed_behavior_experiment_summary.md`とPNG/PDF図である。

残る課題は、Actor 1の先行車影響を分離した追加source、fast群での推定値とTM実速度応答の差、希望速度以外の加減速応答、車間追従、信号反応である。今回のsourceはjunctionを避けているため、指定地点到達時刻とjunction進入・退出時刻は評価対象外である。次段階で車間特性へ進む前に、速度conditioningの独立再現runとfast群のTM応答監査を行うべきである。
道路割当は、logical ID 1: road 4/lane 2、2: road 8/lane -1、3: road 4/lane 1、4: road 1/lane -2、5: road 1/lane 1、6: road 10/lane 1、7: road 10/lane -2、8: road 1/lane 2で、各120 m、速度制限30 km/h、サンプル経路上の信号から35 m超、予測区間内junctionなしである。profile割当はfast=1/4/8、slow=2/3/6、normal=5/7である。顕著なsource特性を意図的に与えた理由は、既存ランダム交通では車両間の速度差が設計されておらず、観測推定、TM変換、未来予測改善の因果を速度要因だけで検証できなかったためである。
## 追補: 速度残差監査と車間特性基礎実験（2026-07-15）

### Phase A: 速度残差監査

前回の固定sourceと12 runを変更せず再利用し、`scripts\audit_speed_response_residuals.py`で希望速度、実速度、throttle/brake、前走車、曲率、過渡応答を監査した。fast群のinferredとoracleの定常速度誤差差は0.239 m/sである。主因はTMの0.5秒周期付近の速度制御振動と初期過渡であり、特にActor 4のinferredは定常誤差0.941 m/s、振動幅3.193 m/s、settling未達である一方、oracleは定常誤差0.104 m/s、2.9秒でsettlingした。推定希望速度差だけでは説明できず、曲率とTM応答の組合せが支配的である。

Actor 1はsource観測でActor 4を前走車として検出し、中心距離は最小19.588 m、平均19.694 mであった。ただしinferredの定常誤差0.103 m/s、oracle 0.106 m/sであり、定常速度抑制は小さい。主評価除外は独立速度特性の厳格な条件による。Actor 7のfuture速度MAEはcommon 0.161 m/s、inferred 0.189 m/sで0.028 m/s悪化したが、3反復のpaired差標準偏差0.050 m/s、t相当値0.96であり、有意な悪化とは判断できない。制御方式は変更せず、次回はTM周期応答を反復付きで分離評価する。

成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\speed_behavior_experiment_20260714\speed_response_residual_audit`に保存した。

### Phase B: 車間特性source gate

同一blueprintのleader/followerを6組生成し、short 6 m、normal 12 m、long 20 mを`TrafficManager.distance_to_leading_vehicle`で設定するsource生成・観測推定処理を追加した。Town10HD_Optでは250～400 mの交差点・信号なし独立レーンを6本確保できず、最大130 mを使用した。全profile共通速度はleader 8 km/h、follower 12 km/h、warmup 15秒、observation 15秒、future 10秒、fixed delta 0.1秒、lane change禁止である。

最大3回のsource生成を行った。最終試行の実現中央値はshort 6.26 m、normal 7.65 m、long 18.55 mで、short-normal差1.39 mが合格基準3 mに届かなかった。normal-long差10.90 mとshort-long差12.29 mは合格した。0.1秒刻みのTM制御では急制動と再加速が周期的に生じ、有効なsteady pairは0であった。観測のみの診断推定はprofile分類66.7%、headway MAE 2.135 m、RMSE 2.835 m、time-headway MAE 0.644秒だが、fallbackを含むため成功結果として扱わない。

source gate不合格のため、baseline/common/inferred/oracleの各3反復、gap/TTC/ADE/FDE評価、動的追従試験は実行していない。replayとpipelineには車間conditioning CLI、3～30 m clamp、FollowerだけへのTM適用、割当CSV/JSONを後方互換で追加した。次の最小作業は、長い単一路線の1ペアでCARLA 0.9.16の距離設定と速度域を校正し、その結果から6ペアの初期条件を固定してsource gateを再実行することである。速度＋車間を組み合わせた事故リスク予測へはまだ進まない。

成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\headway_behavior_experiment_20260715`に保存した。
## 追補: 1ペアTraffic Manager車間応答校正（2026-07-15）

前回の6ペアsource gateではshort 6.26 m、normal 7.65 m、long 18.55 mとなり、short-normal差が1.39 mしか得られなかった。原因を切り分けるため、`scripts\run_headway_calibration_experiment.py`を追加し、Town10HD_Opt上でLeader/Follower各1台だけを使う校正を実施した。全車`vehicle.tesla.model3`、fixed delta 0.1秒、seed/TM seed 42、lane change禁止、初期bumper gap 27.5 mである。

road/section/lane IDを変えず、junctionと信号を避けられる最長候補はroad 10 / section 0 / lane -2の128.324 mであった。基準速度12/18 km/hの技術試験はsteady到達前に道路長を使い切ったため、明示された補助速度8/14 km/hを使用した。TM設定4、6、8、10、12、16、20、24、28 mを各3回、合計27 run実行した。

steady判定は3秒窓で相対速度絶対値0.30 m/s以下、gap標準偏差0.80 m以下、Leader検出、同一road/lane、Follower速度1.0 m/s以上を確認し、この状態が5秒継続した後の10秒を正式評価区間とした。全27 runでsteady followingは成立しなかった。collision、lane change、leader変更は全設定・全反復で0であり、終了後のActor cleanupとworld設定復帰は成功した。

steady未達runの終端10秒を診断目的で集計したbumper gap中央値は、設定4 mで4.513 m、6 mで6.302 m、8 mで7.997 m、10～28 mではすべて8.710 mであった。反復差は決定論的条件下でほぼ0 mである。一方、終端gap標準偏差は4 mで0.423 m、6 mで0.240 m、8 mで0.852 m、10 mで1.520 m、12 mで1.973 m、16～28 mで2.019 mとなった。4～8 mでは診断値が増加したが、10 m以上は約8.71 mに飽和し、同時に制御振動が増加した。これらは正式なsteady実現車間ではない。

3種類の安定帯、各profile成功率2/3、short-normal 3 m、normal-long 5 m、short-long 8 mというPhase A条件を満たさないため不合格とした。`selected_headway_profiles.json`は生成していない。ゲート規則に従い、校正済み6ペアsource、観測推定、baseline/common/inferred/oracle各3回の未来予測、TTC・ADE・FDE比較へは進んでいない。

追加CLIは校正設定、反復数、Leader/Follower速度、steady窓・継続時間・gap標準偏差・相対速度、最大待機時間、profile選定差である。`run_headway_calibration_experiment.ps1`から同じ条件を再実行できる。成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\headway_calibration_experiment_20260715`に保存した。

今回の条件では`TrafficManager.distance_to_leading_vehicle`だけで研究用の安定したshort/normal/longを構成できない。次の最小作業は、より長い標準マップlaneまたは小さいfixed deltaで1ペア応答を再確認することである。それでも分離しなければIDM等の明示的な追従制御を比較対象として導入する必要がある。速度＋車間を統合した事故リスク予測へ進める状態ではない。
## 追補: steady following未達のtick単位監査（2026-07-16）

既存の1ペア校正27 runを再実行せず、保存済み`headway_calibration_samples.csv`だけを用いてsteady following未達をtick単位で監査した。入力CSVは変更していない。主条件はfixed delta 0.1秒、相対速度絶対値0.30 m/s以下、3秒窓gap標準偏差0.80 m以下、窓条件5秒連続である。

Leader検出は全runで11.1秒に開始し、その後40.0秒まで連続して成立した。Followerはroad 10 / lane -2を維持し、junction進入とlane changeは0、残存lane距離は最小6 mで停止閾値5 mを下回らなかった。したがってLeader対応、lane変化、junction、道路長による直接停止はsteady未達の主因ではない。ただしLeader側road/lane/yawと信号状態は元CSVに個別保存されておらず、same_road、same_lane、follower_is_behindは`leader_detected`の合成条件としてのみ監査可能である。

TM設定4 mでは、Leader検出後の相対速度条件成立率が10.0%、3秒窓gap標準偏差条件が87.7%、瞬時全条件の最長連続が0.2秒であった。6 mでは相対速度条件12.1%、gap標準偏差条件93.1%、最長0.2秒である。8 m以上でも相対速度条件は15.9～19.7%、瞬時最長0.4秒であり、正式な3秒窓全条件は全設定0.0秒であった。4 mと6 mの直接原因はgap分散ではなく、FollowerとLeaderの相対速度が周期的に閾値を超えることである。

全27 runで周期的加減速を検出し、Followerのthrottle/brake切替は92～118回、主要周期は約0.82～2.11秒であった。相対速度は概ね-2.5～+2.0 m/sを往復した。相対速度閾値0.20～0.50 m/s、gap標準偏差0.50～1.50 m、連続時間3/5/8秒の感度分析でも成立runは0であり、主閾値だけが過度に厳しいとは判断できない。元steady判定と同じrolling窓・連続カウンタを再現した結果は一致し、結果を覆す実装不具合は確認されなかった。

成果物は`D:\CARLA_DATA\DT_RiskPrediction\02_reproduction_runs\trajectory\headway_calibration_experiment_20260715`内の`headway_steady_failure_*.csv/json`およびPNG/PDFである。次の比較候補は長道路とfixed delta 0.05秒であり、特に同期刻みによるTM振動を切り分けるためfixed delta比較を優先する。ただし本監査では長道路実験、6ペア実験、未来予測、IDM実装は行っていない。