# Town01_Opt 読み取り専用scene監査

Issue #22。PR #21の未監査設定を変更せず、独立CLIで実scene取得値とオフライン参照を比較する。今回の実装作業ではCARLA接続を行っていない。CLIの完成は接続・生成・走行の承認ではない。

## 入口と既定動作

```powershell
python dt_risk_prediction_project/Run_Audit_Town01_Scene.py
```

既定はオフライン。CARLA SDKもimportせず、OSのサーバ調査も行わない。Town01の固定policy、設定fingerprint、allowlist、未確認事項を出力する。参照ファイル未指定なら参照照合はunknown。`--reference-opendrive <外部xodr>` と `--reference-geometry <外部json>` の両方を渡すと参照hash・196点を検証する。片方だけなら拒否する。

geometry JSONは`points`（196個のXYZ）、`waypoints`（road/section/lane/s/yaw/pitch/roll/width）、`geometry_sha256`を持つ独立オブジェクト。または既存の`geometry_only`オブジェクトを読み込む。既存監査の承認属性は消費しない。地図・座標をこのRepositoryへコピーしない。

- Policy: `single_brake_town01_v2`、Town01_Opt、road 8 / section 0 / lane 1、s=250→55。
- OpenDRIVE SHA-256: `97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709`。
- Geometry SHA-256: `65f6c60358d155ec6f9678061a8821483705250f157b0958600ef1ac7342ee47`。
- 座標実長: 195.00316911583323 m。整数s差195 mに置き換えない。
- 幅員4 m、非junction、Driving、s減少方向。既存の純粋なgeometry検証も適用する。

参照hashは参照の同一性を示すだけで実scene監査承認にはならない。`offline_reference.json`、`route_geometry.json`の`observed_*`、`route_comparison.json`を分離し、欠損した観測を参照座標で埋めない。配布config・route_audit・物理性能値は書き換えない。

## 既存コードの副作用と再利用範囲

| 呼出元 | 計算/getter/OS/ファイル | 状態変更への到達 | 新CLI |
|---|---|---|---|
| BrakeHarness.run | 制御・観測・保存・cleanup | tick、apply_control、destroy、設定復元 | 使用しない |
| BrakeHarness.prepare | 道路getter・監査・保存 | apply_settings、spawnへ進む | 使用しない |
| Harness.inspect_world_route | 道路getter・幾何計算・保存 | このメソッド自体はgetter、所属Harnessは変更系を持つ | 使用しない |
| inspect_town01 | getter・幾何計算・保存 | Harnessへの依存、route_audit必須 | 使用しない |
| inspect_spawn_bbox | getter・幾何計算 | 生成済みactorを前提 | 使用しない |
| Run_Calibrate_Brakes | offlineまたは実験監視親 | execute分岐がHarness.runへ到達 | importしない |
| HealthGate | PID/markerのOS読取り、getterの例外遮断 | 自身に変更/tick/cleanupなし | call/checkのみ再利用 |
| listener/PID処理 | OS情報読取り | 既存PortOwnershipはbindも行う | PortOwnershipは使わず専用PID/start/TCP読取り |
| idm_brake_route内の幾何関数 | 純粋計算 | なし | angle、box_vertices/rotate、hull、overlaps、ribbons、segment_distance、geometry_sha256、validate_review_geometryのみ |

`idm_scene_reference`は純粋な設定・参照検証、`idm_scene_os`はWindows OS読取り、`idm_scene_audit`はgetter限定の観測と比較、CLIはオフライン計画と子プロセス監視を担当する。実験Harnessのrun/prepare/cleanupを呼ぶ経路はない。

## 将来の接続モード（今回実行禁止）

別途、**特定の専用Town01 sessionへの1回の読み取り専用接続**について承認を得た後にだけ使用する。既存Town10HD_Optを切り替える機能はない。サーバ起動やmap変更はこのCLIの権限外。

```text
python dt_risk_prediction_project/Run_Audit_Town01_Scene.py
  --connect-readonly
  --host <明示したloopback IPまたはlocalhost> --port <RPC待受port>
  --server-pid <OS確認PID> --server-started-utc <offset付き起動時刻>
  --tm-port <監査するTM待受port> --session-reference <専用session申告JSON>
  --reference-opendrive <外部参照xodr> --reference-geometry <外部参照json>
  --output-root <Repository外の既存ディレクトリ>
```

host/port/PID/開始時刻/TM portに既定値はない。ローカルOSのPID所有関係を確認できるloopbackのみ対応する。session申告JSONはこれら5項目を同じkey（`server_started_utc`等）で持ち、`operator`、offset付き`reviewed_at`、`dedicated_session_declared: true`を必要とする。申告は外部client不存在の証明ではなく、完全な不在はunknownのまま残る。

接続前にpolicy/参照、PID・起動時刻（許容差1 ms）、CARLAプロセス名、RPC listenerの単独所有、既存TM listenerと既存TCP clientがないことを確認する。Windows以外の実接続は現在拒否する。OS観測失敗時も接続しない。明示flagそのものはユーザー承認の代用ではない。

## API allowlistと禁止到達性

| 対象 | 許可メソッド |
|---|---|
| client | get_server_version、get_client_version、get_world |
| world | get_map、get_settings、get_weather、get_actors、get_snapshot、get_level_bbs、get_environment_objects |
| map | to_opendrive、get_waypoint_xodr、get_all_landmarks |
| waypoint | next |
| traffic actor | get_transform、get_state、get_stop_waypoints、get_affected_lane_waypoints |

唯一の接続生成は子プロセス内の`carla.Client(host, port)`。これもledgerに記録する。`set_timeout`を含むsetterは使わず、監視親の120秒期限で監査子のみ終了させる。Client/world/getterの後始末としてCARLA RPCを送らない。workerは結果をflush後`os._exit`し、SDK destructorの復旧・cleanup呼出しを避ける。

各CARLAメソッドは`Reader.call`でkind/method allowlistを検査し、呼出前後のUTC・連番・成否を保存する。許可外methodはオブジェクトの属性を取得する前に拒否する。禁止APIのAST検査、禁止属性アクセスで即失敗するsentinel、RPC例外後にledgerが増えないmock試験をCIで実行する。既存制御Harnessの試験もそのまま回す。

根拠となるCARLA 0.9.16一次資料: [World API](https://github.com/carla-simulator/carla/blob/0.9.16/PythonAPI/docs/world.yml)、[Map / Waypoint API](https://github.com/carla-simulator/carla/blob/0.9.16/PythonAPI/docs/map.yml)、[Actor / TrafficLight API](https://github.com/carla-simulator/carla/blob/0.9.16/PythonAPI/docs/actor.yml)、[Client定義](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/Client.h)。get_snapshotは現在のsnapshotを取得し、tick/waitではない。waypoint/map照会は既存地図の検索。交通actor getterとtrigger_volume属性は観測だけに用いる。Level BB/EnvironmentObject bboxはworld座標、traffic triggerはactor相対なのでtransformを合成する。

## 監査内容と停止

サーバ/client versionがともに0.9.16、mapとOpenDRIVE hashが一致し、非同期worldであることを必須とする。既存vehicle/walker/sensor/controllerがあれば直ちに中断する。spectatorと監査対象traffic以外の未レビューactor（実行時に置かれたstatic.prop等）も、静的物体APIの網羅性を仮定せずunknownで拒否する。

196点はXYZ、姿勢、s、road/section/lane、Driving、非junction、幅員を全点取得する。位置差≤1 cm、姿勢差≤0.01度、s差≤0.0001 m、幅員差≤0.000001 m。隣接点はnext(1.0)が単一で同じ車線を返し、位置/s差≤2 cmであることを確認。座標実長も参照との差≤1 cmで比較する。新しい許容差は読み取り照合の条件であり、制御/安全gateを変更しない。

road8のXML signals/objectsはs=35→270の拡張範囲を調べる。map全landmark、traffic trigger、light stop/affected waypointを調べ、routeから20 m以内のlandmark/stop/affected、corridorに重なるtriggerがあれば拒否する。XMLのsが欠ける等の必要情報欠損はunknownで中止する。

全semantic labelのLevel BBとAnyのEnvironmentObjectsを取得する。経路ごとのsegmentに沿った幅員全幅＋保守的end cap（初期後端を含む）とのXY重なりとZ範囲を保存する。車線標高−0.1〜＋2.1 mとの重なりもある非ground descriptorは障害物として拒否。ground descriptorは保存するが走行可能meshの証明とはしない。triggerは高さにかかわらずXY重なりを拒否する保守的判定。全collision形状の網羅性はunknown。

開始/終了UTC、snapshot frame/elapsed/platform timestamp、world episode/map/hash/settings/actor IDs・属性/weatherを取得前後で比較する。通常の非同期frame前進は許容し、巻戻り・frame/timeの不整合、状態変更は結果を失効させる。各getterの完全な同一frameを主張しない。両端の間だけで発生・復帰した変更の不存在は証明できない。

getter失敗・RPC/native異常後は追加RPC・再接続・cleanupを一切行わない。部分JSON/ledger/stdout/stderrと監視親のexit code/native判定を保存し、実験可否は常にfalse。通常の監査完了、意味的不適合、観測不能、native/timeoutを分けて残す。

## 証跡・未確認・次の承認

Repository外に毎回unique directoryを作り、server_status / environment / world_settings / actor_inventory / route_geometry / route_comparison / static_obstacles / regulatory_elements / audit_results / api_call_ledger のJSON、report_ja.md、hash一覧を保存する。全JSONにUTC、source commit、schema、synthetic識別を持たせる。config/参照ファイル/hashと取得結果を追跡できる。新しい監査証跡と旧証跡を上書きしない。テストの座標・地図・制御結果は全てsynthetic。

実道路mesh完全性、全collision、材質/摩擦、新規車両のbbox/接地/physics、制動性能/遅延は未確認。route_audit=null、physical_bounds=nullを維持する。次回は専用Town01 sessionのPID・port・開始時刻・専用申告と独立参照を用意し、1回の読み取り専用監査への明示承認が必要。その承認にサーバ起動/map変更、静止車両生成、制御、tick、制動走行は含まれない。それらは各段階で別途承認を得る。
