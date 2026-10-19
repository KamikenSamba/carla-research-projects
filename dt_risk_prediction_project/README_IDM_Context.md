# 実CARLA用・同tickの追従履歴収集器（Issue #10）

PR #9の[sidecar契約](README_IDM_History.md)に従い、既存captureと同じtick・同じactor対応表から
`following_context.csv`を生成する。**実CARLA収集・実データ推定精度は未検証**。
今回はoffline実装・mock検証と実験計画のみで、サーバ起動・接続は行っていない。

単一TM/tick ownerで2台を走行させる専用入口は[実験Harness](README_IDM_Harness.md)を参照。
既存captureのCLIは維持し、新Harnessの実CARLA挙動は未検証。

## 接続方式の比較と採用

| 方式 | 利点 | 問題・判断 |
|---|---|---|
| V1 captureへ任意callback | tick、logical ID、snapshot時刻を共有できる | V1への小変更が必要。未指定時を維持できるため採用 |
| V2でcaptureループを複製 | V1のsourceを変更しない | tick・採番・例外処理が二重管理になり、将来乖離するため不採用 |
| 実行時のWorld/関数差替え | 見かけの差分が小さい | 隠れた依存と並行実行時の危険があり不採用 |

V1の`capture_current_state(..., *, capture_observer=None)`がcapture行を作成した直後に
observerを一度呼ぶ。V2の`--collect-following-context`指定時だけ収集器を接続する。
callbackにはworld、同tickのsnapshot、同じvehicle一覧、書き出したcapture行を渡す。
collectorは書き出したcontext行のlistも返す。従来の呼び出しは戻り値を無視するため互換。
未指定時は以前の3引数呼出しを維持し、V1 CLI・既存CSV/JSON列・既定の処理順序は変えない。
V1のprediction/cleanup・IDM数式・traits優先順位は変更しない。

収集器自身はtick、spawn、destroy、apply_control、set_autopilot、TM取得を呼ばない。
capture全体が従来から行うWorldSettings/TM同期設定とその復元は既存処理に委ねる。
TM全体のshutdownやIssue #7の修正は行わない。

## 出力と同tick保証

| 出力 | 内容 |
|---|---|
| capture_states.csv | 既存19列を維持 |
| snapshot_states.csv/json | 既存V2のsnapshot形式を維持 |
| following_context.csv | canonical context列、同一logical ID/frame、補助の時刻・source ID、空欄の任意加速度 |
| following_context.meta.json | running/completed/failed、行数・tick数・最終frame、観測設定、理由集計、observerのwall時間 |

- 一つのcollectorで一capture/episodeのみ。sidecarは排他的に新規作成し、既存成果物を上書きしない。
- captureのtick戻り値とsnapshot frameを照合し、観測前後にもworld frameが変化していないことを確認する。
  観測中に他clientがtickを進めた場合は失敗として中断する。
- captureのlogical ID/frame/source IDとvehicle集合の整合を確認する。重複・欠落・時刻不一致・
  非単調frameは中断する。callback中に独自のlogical IDを作らない。
- getterで得たTransform/velocityを凍結して既存`sample_vehicle`へ渡し、capture行の位置・速度・
  姿勢と再照合する。不一致や読取り不能はunknown行にする。
- contextは各frameでflushする。正常終了時のみmetaをcompletedにする。例外時はfailed、
  native終了等で終了処理へ到達しない場合はrunningが残る。部分CSVを消去しない。
- 推定器は対応する`.meta.json`があればcompletedと行数/key集合の完全一致を要求する。
  running/failed・途中切断・captureとの差異は拒否する。metaなしの従来の手動sidecarは引き続き読める。

同期worldを単一tick ownerで使うことが前提。frame照合は、同じframe内に別clientがactorを操作する
すべての競合を検知する保証ではない。観測RPCとflushによるwall時間増加はあり、metaの
`observation_seconds`と`maximum_observation_seconds`に成功したcallbackの実測時間を保存する。
追加のsimulation tickやsleepは挿入しない。native crashに対するflushの永続性保証はない。

## 観測・除外の実装

### Leader、lane、gap

`idm_leader.sample_vehicle/trace_lane/find_leader/path_projection`を再利用する。
同laneの前方候補を経路に射影し、逆向き・隣接lane・後方・junction候補を除外する。
source actor IDをleader_idとして保存する。対象外の車両も観測候補に含め、操作しない。

bboxのlocal中心offsetをTransformでworldへ変換する。長さはbbox yawと車両姿勢を考慮して
local lane方向へ投影した半extentを使い、中心間の経路距離から前後車の半extentを引く。
roll/pitchが1度超の傾斜や欠損・空bboxはunknown。既存IDMのgeometry/controller自体は変更しない。
直線路が初期対象で、急曲線・3D路面・特殊bboxでのgapは近似である。
観測できない車両が一台でもあれば、隠れたleaderの可能性を考慮し、そのframe全体をfallbackにする。

### Lane change

意図やTM内部状態を読まず、履歴と幾何から保守的に判定する。
lane key変化、lane中心から幅の15%超のずれ、heading dot<0.98を検出した場合はtrueとし、
1秒間保持する。初回・actor ID切替・frame欠損・時間不連続では履歴をリセットし、
0.3秒以上の安定観測が得られるまでunknownにする。
道路/section切替もlane key変化として除外する。
これはlane changeの観測proxyであり、計画中の車線変更や操舵意図まで検出する保証はない。
将来の実走で除外範囲と閾値を監査する必要がある。

### Signal stopとroute fallback

CARLAの`is_at_traffic_light()`は局所的な検出であり、falseだけで信号の影響なしとしない。
waypointから前方landmarkを探索し、不明な規制の影響を除外する。
APIの適用範囲は[CARLA 0.9.16 Python API](https://carla.readthedocs.io/en/0.9.16/python_api/)を参照。

- at-light=trueはsignal_stop=true。
- 前方にlandmarkがある場合はsignal_stop=unknown。国別のsign typeを仮定せず、stop/yield/
  signal以外のlandmarkも保守的に除外するため、過剰除外の可能性がある。
- API欠損・例外・不正値はunknown。観測開始時、規制候補通過後、未知状態の回復後も2秒の履歴guardを置く。
- signal_stop=falseにするのは、有効なat-light=falseと空の前方landmark結果、履歴guard終了が
  揃った場合だけ。探索距離はmax(30 m, v²/4 + 2v + 2 m)とし、停止距離と反応区間を保守的に含める。
- 停止標識等を全世界の動的actorから完全に復元する機能ではない。map landmarkの欠落、
  他車の信号待ちによるqueue、規制の過去影響がguardより長く続く場合は識別できない可能性がある。
  real-CARLAでの正解監査なしに「全信号影響を除外した」と主張しない。
- 経路のfork/end/junction/lane transition、heading不整合、重なりはfallback_reasonへ保存する。
  route境界は既存controllerと同様、lookahead/停止距離の範囲内で除外する。

unknownはCSV空欄と理由で保存し、`Estimate_IDM_T.py`の既存flag/context filterで除外する。
縦加速度は追加API/センサで取得せず任意列を空欄にし、推定器の連続速度窓から算出する。
加速度vectorのnormは使わない。metadataのcontext_validはT推定採用を意味しない。

## 入出力例（実CARLAは承認後）

```powershell
# dt_risk_prediction_project/ から。今回は実行していない。
python Run_DT_Risk_V2_IDM.py --mode capture --collect-following-context --duration 15 --snapshot-at 10 --fixed-delta-seconds 0.05 --idm-desired-speed-mps 8 --output-root D:/CARLA_DATA/DT_RiskPrediction/context_runs

# CAPTURE_DIRは上のrun directory。meta completedと正解監査を確認してから推定。
python Estimate_IDM_T.py --history CAPTURE_DIR/capture_states.csv --context CAPTURE_DIR/following_context.csv --snapshot CAPTURE_DIR/snapshot_states.csv --idm-desired-speed-mps 8 --replace-snapshot-headway --output-dir FIT_DIR

# 同一snapshotと固定係数で一律Tと推定Tを比較する例。
# V2 snapshotにはTが保存されるため、一律条件はT=1.5を設定したsnapshot/明示traitsを用意する。
python Run_DT_Risk_V2_IDM.py --mode predict --prediction-controller idm --snapshot CAPTURE_DIR/snapshot_states.csv --prediction-seconds 5
python Run_DT_Risk_V2_IDM.py --mode predict --prediction-controller idm --snapshot CAPTURE_DIR/snapshot_states.csv --actor-traits FIT_DIR/actor_traits.json --prediction-seconds 5
```

`--replace-snapshot-headway`はcaptureで保存されたTを推定値で置換する明示的な許可。
別の明示traitsがあれば推定CLIへ`--actor-traits`で渡して保護する。
固定v0やその他のIDM係数は両予測で同一にする。`--mode all`はpaired比較に使わない。

有効なcontextの行例（概念。実ログではない）：

```json
{"logical_actor_id":"role:follower","carla_frame":201,"sim_time_s":10.0,
 "source_actor_id":101,"leader_id":102,"leader_speed_mps":3.0,"gap_m":12.0,
 "road_id":10,"section_id":0,"lane_id":1,"signal_stop":false,"lane_change":false,
 "is_junction":false,"fallback_reason":"","observed_acceleration_mps2":""}
```

## Offline検証結果

- DT：既存84件＋追加33件＝117件成功。OGM143件成功。
- 両projectのcompileall、OGM config JSON、V2 CLI help、git diff --check成功。
- observer有無でcapture CSVのbyte一致、同じtick数・WorldSettings復元・actor保持をmockで確認。
- bbox中心offset/回転、nearest leader、逆向き/隣接/後方除外、route、信号unknown、
  lane change/hold、欠損・重複・actor切替、観測中の別tick、途中失敗、meta/key整合を検証。
- mock E2E：12秒・2車両、capture/context各480行、241tick（初期1+観測240）。
  snapshot=10秒、真値T=1.8秒。追従車は有効154件、T=1.8秒、加速度RMSE約0.0040 m/s²。
  snapshot後の40件/actorは推定から除外。先行車不在の車両は有効0件・既存値へfallback。
  traits JSONを既存V2 predictionへのparameter受渡しまで確認した。

合成IDM由来のmockによる機能確認であり、実走T推定精度・信号判定精度・physics精度の根拠ではない。
CSV/JSONの実験結果はローカルevidenceへ保存しGitへ追加しない。

## 承認後の実CARLA計画と安全条件

1. CARLA 0.9.16/Python 3.12、Town10HD_Opt、fixed delta=0.05秒、直線・非交差点でTesla Model 3
   2台を使う。role/logical ID、初期Transform/速度、bbox、road/section/lane、server起動条件を固定・記録する。
   信号・標識・分岐から十分離れた、15秒走行と探索距離を確保できる区間を事前確認する。
2. 交通生成側のseed=42を固定する。既存captureは`--tm-seed`をTMへ適用していないため、
   capture CLIへ指定するだけでseed固定を保証しない。TMを管理する交通生成側がseedを設定し、
   その起動記録を保全する。collectorから他clientのseedや制御を変えない。
3. 観測15秒、snapshot=10秒とし、最初の10秒だけでfitする。後半5秒はholdout truthとして分離。
   開始・中央・終端の複数frameでleader IDとbboxに基づくgapを独立確認し、信号/lane-changeの
   true/unknown区間とflag解除後を監査する。初回は1captureに限定し、無断で反復しない。
4. 同期key、meta completed、出力有限性、有効件数/時間、RMSE、profile、境界、T採用/fallback理由を
   確認する。単に出力ができたことやsample数だけで推定成功としない。品質不足なら既存値を維持する。
5. 同一snapshotから一律T=1.5秒と推定TをIDMで予測。v0・他係数・seed・physics・horizonを統一し、
   0.5/1/2/3/5秒の位置・速度・gap誤差、trajectory平均/終端誤差、fallback率をholdoutと比較する。
   T以外を同時に校正しない。不採用actorと悪化例も報告する。
6. 専用環境でsingle tick owner、既知の元WorldSettings、既存actor一覧を記録する。
   収集器はactorを生成・破棄しない。試験準備で作成するactorも、別途承認された所有IDに限定する。
   world reload、TM全体shutdown、既存actorの操作・破棄、自動server再起動はしない。
7. Issue #7のnative abort/server描画障害は未解決。親process側でclient/server PID・起動終了時刻・
   exit code・stdout/stderr・fatalを別々に保存し、CSV完了やmeta completedだけで正常終了と判断しない。
   server異常時は直ちに中断してdump/logを保全し、追加RPCを無理に送らない。
   server生存時だけactor/settingsの事後監査を行い、監査不能は不明として報告する。

実走はこの計画の確認と明示的な承認後に行う。今回の実装をIssue #7の対策として採用せず、
PR #6/#9のReady変更、main merge、既存調査の終了は行わない。
