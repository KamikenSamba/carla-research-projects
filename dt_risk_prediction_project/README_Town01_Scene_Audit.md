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

### 速度標識triggerの意味分類（Issue #28）

CARLA 0.9.16の[SpeedLimitComponent](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Traffic/SpeedLimitComponent.cpp)はoverlapでControllerへ速度制限を渡す。[TrafficSign](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/TrafficSign.h)のtrigger_volumeはactor記述のbboxで、[BoundingBoxCalculator](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Util/BoundingBoxCalculator.cpp)は標識の複数triggerを結合する場合がある。これは標識支柱/板のcollision形状ではない。標識の`actor.bounding_box`を別の物理形状と仮定して代用しない。

旧gateは全traffic triggerのXY重なりを拒否した。新版は登録済み`traffic.speed_limit.30`〜`.120`（10刻み）だけを最大速度規制として分類し、有限のactor/trigger位置・両回転・正のextent・非退化world頂点を保存する。名前から不正な値を推測補完しない。corridor外なら非関係として記録。重なる場合は以下がすべて成立する時だけ規制領域として受理し、後続の物理障害物監査へ進む。

- 既存`map.get_all_landmarks()`の速度標識type274のうちroute road上のものを候補とし、`landmark.get_lane_validities()`で対象laneを検証する。`world.get_traffic_sign(landmark)`によるSDKのsign-ID対応で対象actorに一意に結びつくことを要求し、位置の近さで対応を作らない。
- landmarkの有限値・既知単位・XML road長内の位置・XML ID/種別/値/単位/向き・宣言されたlane validityを照合する。s減少経路の開始位置より上流側に標識があることも要求。欠落/曖昧/不明はUNKNOWN、不整合はFAILで停止する。orientationの名前だけから適用車線を推測せず、実際のvalidity範囲と検証済み経路方向を使う。
- 通常signalはkm/h、mph、m/sを明示変換し、actorの名目値km/hと一致することを要求する（数値表現差の絶対許容1e-6km/h、道路の幾何許容差は不変）。actor suffixは実Controller値の測定ではない。以下の限定object経路だけ比較値を分離する。校正の既存速度上限1.2m/sは宣言単位の換算値と内部解釈値の小さい方以内であることを要求する。標識値を車両目標/安全上限へ採用しない。

**Town01の単位と限定解釈（Issue #30）**：[ObjectParser.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp)はRoadRunnerのSpeed_90等の数値を保持し、元XMLにないmph metadataを付加する。MapBuilder/Signal/Landmarkは換算しない。[TrafficLightManager.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Traffic/TrafficLightManager.cpp)の生成経路はsubtypeからBlueprintを選び、raw valueをSpeedLimitComponentへ渡す。同componentのoverlapはその値をそのままkm/h契約のControllerへ渡す。既存Actorを再利用する別分岐やBlueprint実行時状態を、このソース追跡だけで測定済みとはしない。

この0.9.16のobject経路についてのみ、既存の一意SDK Actor対応・車線・上流位置・XML ID/向き照合に加え、XML objectが正確な`Speed_<登録値>`/`speed_<登録値>`、type=-1、明示unit/value/validityなし、Landmark name/sub_type/country/is_dynamicがparser生成値と一致し、valueが名目値、unitがmphであることを要求する。版は既存のclient/server両方0.9.16検査で固定する。未対応のSTATIC名、明示単位付きobject、生成経路不明は拒否する。

保存形式は元の`value=90, unit=mph, limit_kmh=144.84096`を維持する。XML属性を`xml_source`、parser照合属性を`parser_metadata`、別の比較値90 km/hを`internal_interpretation`に根拠・source versionと共に保存する。内部解釈はsource由来の期待契約であり、`measured_controller_value=null`、実component/spawn履歴はunknown。通常signalの90mphを90km/hへ読み替えない。XMLに正当な90mphがあれば144.84096km/hとして照合し、名目90との不一致を拒否する。実ID52とLandmarkの対応は依然UNKNOWNであり、Phase B-2のBLOCKED判定とroute_audit/physical_bounds=nullは維持する。

追加のLandmark name/sub_type/country/is_dynamicは取得済みMapデータの公開属性で、追加RPCはない。getter allowlist、呼出し量、timeout、worker期限は不変。ローカルSDK/実XMLとsynthetic Actor対応を組み合わせた試験は、実Actor52の対応確認ではない。

速度標識以外のstop/yield/traffic_lightのtrigger重なり拒否、信号のstop/affected waypoint、XMLの既存s近接条件、landmark>20m条件は維持する。他の未知traffic型はUNKNOWN。Level BB/Environment Objects/static.prop.meshの判定も維持し、規制として受理した標識の物理監査を省略しない。速度標識のtransform/trigger/valueを前後状態へ追加して厳密比較する。WorldSettings/天候/actor集合の既存比較は変更しない。

追加allowlistは`landmark.get_lane_validities`（SDK内の地図データ読取り）と`world.get_traffic_sign`（SDK内部でGetActors/sign-ID照合）だけ。[World.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/World.cpp)で読み取り経路を確認。両方Reader/HealthGate/ledgerを通し、例外後の追加getterを禁止する。同じroad/landmark IDの照会結果は当該regulatory取得内だけcacheする。監査前後のmap hash/actor集合比較も継続する。

正常完了時のSDKアクセス予算は従来の`437 + 2T + 4L + 4P`に`V + Q`を追加する。Vは実際に評価した同road速度landmarkのvalidity getter数、Qは対象lane候補のsign lookup数。少なくとも1つの速度triggerが重なる時に必要分を照会する。SDK内部通信回数とは別。候補がなければ推測許可せずUNKNOWN。RPC既定5秒/worker120秒を変更せず、追加lookupを次回の明示承認範囲に含める。

### static.prop.meshの幾何gate（Issue #26）

`static.prop.mesh`だけを追加の幾何監査対象に分類する。他の未知actor、vehicle/walker/sensor/controllerの拒否は維持する。mesh名や椅子・テーブルという属性は安全根拠にしない。属性は文字列辞書と非空の`mesh_path`を要求し、名称による許可リストは設けない。

各meshについて開始側で`actor.get_transform()`と`actor.bounding_box`を各1回取得する。後者はCARLA 0.9.16のproperty内部で`get_actor_bounding_box` RPCを呼ぶため、property専用allowlistとReader/HealthGateを通し、ledgerへ記録する。例外時はpoisonして後続getterを禁止する。id/type_id/attributesはSDKのactor記述値である。

正の有限extent、有限の位置とyaw/pitch/rollを要求する。欠損・非数値・NaN/Infinity・ゼロ/負extent・取得例外・非有限変換結果はUNKNOWNで中断し、補完しない。local頂点をbbox固有の回転→bbox中心移動→actor回転→actor移動の順でworld座標へ変換する。既存`box_vertices`を再利用し、変更しない。異常に巨大な有限boxも縮小せずそのまま幾何評価する（演算結果が非有限ならUNKNOWN）。

196点のlive routeを既存条件で検証後、既存`ribbons`の車線全幅と保守的端部cap、Z範囲`[min(route.z)-0.1, max(route.z)+2.1]`をそのまま使う。world頂点のXY凸包とcorridorが重なり、Zも重なる場合はINTERSECTとして拒否する。接触も重なりに含む。妥当な幾何情報で非干渉の場合だけCLEARとなる。これは幾何上の非干渉であり、完全なcollision形状・摩擦・制動安全性を保証しない。

終了側のactor集合/属性比較後、同じmeshのtransformとbboxを再取得する。集合・属性・transform・local bbox・world頂点を厳密比較し、変化を拒否する。`static_prop_geometry.json`へ時刻付きの部分結果と前後比較を保存する。前後で元に戻る変化や全外部client/tick ownerの不在は確認できない。既存Level BB/environment、交通状態、天候、WorldSettings比較も継続する。幾何gateがCLEARでも監査全体の完了まではPASSではなく、`route_audit`/`physical_bounds`は常にnullのままである。

正常完了時の追加呼出し量はmesh数Pに対し`get_transform`が2P、`bounding_box`propertyが2P（内部bbox RPCも2P）。追加SDKアクセスは4P。既存の明示getter数は`437 + 2T + 4L`（Tはtraffic actor、Lはtraffic light）なので、新版は`437 + 2T + 4L + 4P`、別にClient構築1回。P=110/T=57/L=36なら合計1135アクセス＋Client構築1回、うち追加bbox RPCは220回。これはSDKアクセス数で、全ネットワーク通信回数の保証ではない。途中失敗時はそこで中断するため減る。

RPC timeout既定5秒、worker上限120秒は変更しない。次回の別途承認には追加property RPCを含める。110個の実bboxを取得できるか、120秒以内に終了できるかは未検証。外側からidentity/port/resource/worker heartbeatを監視し、異常時は監査workerのみ中止する。サーバ停止・閾値緩和・自動再試行は行わない。過去のPIDは現在の生存保証にならず、接続前にnative PID/開始FILETIME/実行パス・3ポート所有・資源と新規session申告を再確認する。

根拠: CARLA 0.9.16の[Actor.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/Actor.cpp)、[Simulator.h](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/detail/Simulator.h)、[BoundingBox.h](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/geom/BoundingBox.h)。Actorのbbox propertyとTransform/BoundingBoxのローカル型試験は、実サーバのmesh bbox・接地・collisionの検証とは別である。

| 対象 | 許可メソッド |
|---|---|
| client | get_server_version、get_client_version、get_world |
| world | get_map、get_settings、get_weather、get_actors、get_snapshot、get_level_bbs、get_environment_objects、get_traffic_sign |
| map | to_opendrive、get_waypoint_xodr、get_all_landmarks |
| waypoint | next |
| traffic actor | get_transform、get_state、get_stop_waypoints、get_affected_lane_waypoints |
| landmark | get_lane_validities |
| prop | get_transform、bounding_box（property専用allowlist） |

唯一の接続生成は子プロセス内の`carla.Client(host, port)`。これもledgerに記録する。`set_timeout`を含むsetterは使わず、監視親の120秒期限で監査子のみ終了させる。Client/world/getterの後始末としてCARLA RPCを送らない。workerは結果をflush後`os._exit`し、SDK destructorの復旧・cleanup呼出しを避ける。

各CARLAメソッドは`Reader.call`でkind/method allowlistを検査し、呼出前後のUTC・連番・成否を保存する。許可外methodはオブジェクトの属性を取得する前に拒否する。禁止APIのAST検査、禁止属性アクセスで即失敗するsentinel、RPC例外後にledgerが増えないmock試験をCIで実行する。既存制御Harnessの試験もそのまま回す。

根拠となるCARLA 0.9.16一次資料: [World API](https://github.com/carla-simulator/carla/blob/0.9.16/PythonAPI/docs/world.yml)、[Map / Waypoint API](https://github.com/carla-simulator/carla/blob/0.9.16/PythonAPI/docs/map.yml)、[Actor / TrafficLight API](https://github.com/carla-simulator/carla/blob/0.9.16/PythonAPI/docs/actor.yml)、[Client定義](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/Client.h)。get_snapshotは現在のsnapshotを取得し、tick/waitではない。waypoint/map照会は既存地図の検索。交通actor getterとtrigger_volume属性は観測だけに用いる。Level BB/EnvironmentObject bboxはworld座標、traffic triggerはactor相対なのでtransformを合成する。

## 監査内容と停止

サーバ/client versionがともに0.9.16、mapとOpenDRIVE hashが一致し、非同期worldであることを必須とする。既存vehicle/walker/sensor/controllerがあれば直ちに中断する。spectatorと監査対象traffic以外の未レビューactor（実行時に置かれたstatic.prop等）も、静的物体APIの網羅性を仮定せずunknownで拒否する。

196点はXYZ、姿勢、s、road/section/lane、Driving、非junction、幅員を全点取得する。位置差≤1 cm、姿勢差≤0.01度、s差≤0.0001 m、幅員差≤0.000001 m。隣接点はnext(1.0)が単一で同じ車線を返し、位置/s差≤2 cmであることを確認。座標実長も参照との差≤1 cmで比較する。新しい許容差は読み取り照合の条件であり、制御/安全gateを変更しない。

road8のXML signals/objectsはs=35→270の拡張範囲を調べる。map全landmark、traffic trigger、light stop/affected waypointを調べ、routeから20 m以内のlandmark/stop/affected、corridorに重なる非速度標識triggerがあれば拒否する。速度標識の重なりは上記Issue #28の厳格な意味照合を必要とする。XMLのsが欠ける等の必要情報欠損はunknownで中止する。

全semantic labelのLevel BBとAnyのEnvironmentObjectsを取得する。経路ごとのsegmentに沿った幅員全幅＋保守的end cap（初期後端を含む）とのXY重なりとZ範囲を保存する。車線標高−0.1〜＋2.1 mとの重なりもある非ground descriptorは障害物として拒否。ground descriptorは保存するが走行可能meshの証明とはしない。非速度標識triggerは高さにかかわらずXY重なりを拒否する保守的判定。全collision形状の網羅性はunknown。

開始/終了UTC、snapshot frame/elapsed/platform timestamp、world episode/map/hash/settings/actor IDs・属性/weatherを取得前後で比較する。通常の非同期frame前進は許容し、巻戻り・frame/timeの不整合、状態変更は結果を失効させる。各getterの完全な同一frameを主張しない。両端の間だけで発生・復帰した変更の不存在は証明できない。

getter失敗・RPC/native異常後は追加RPC・再接続・cleanupを一切行わない。部分JSON/ledger/stdout/stderrと監視親のexit code/native判定を保存し、実験可否は常にfalse。通常の監査完了、意味的不適合、観測不能、native/timeoutを分けて残す。

## 証跡・未確認・次の承認

Repository外に毎回unique directoryを作り、server_status / environment / world_settings / actor_inventory / route_geometry / route_comparison / static_obstacles / regulatory_elements / audit_results / api_call_ledger のJSON、report_ja.md、hash一覧を保存する。全JSONにUTC、source commit、schema、synthetic識別を持たせる。config/参照ファイル/hashと取得結果を追跡できる。新しい監査証跡と旧証跡を上書きしない。テストの座標・地図・制御結果は全てsynthetic。

実道路mesh完全性、全collision、材質/摩擦、新規車両のbbox/接地/physics、制動性能/遅延は未確認。route_audit=null、physical_bounds=nullを維持する。次回は専用Town01 sessionのPID・port・開始時刻・専用申告と独立参照を用意し、1回の読み取り専用監査への明示承認が必要。その承認にサーバ起動/map変更、静止車両生成、制御、tick、制動走行は含まれない。それらは各段階で別途承認を得る。
