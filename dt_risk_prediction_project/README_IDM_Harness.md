# IDM追従実験Harness（Issue #12）

PR #11をbaseとする新規の実験入口。**実CARLAには未接続・未実走**。
デフォルトCLIは設定検証だけを行い、CARLAをimportしない。実走成立、T識別性、予測精度の改善を主張しない。

発進時のPID飽和、制御frameの解釈、中止frameのraw観測拡張と次回の条件付き設計は
[低速発進の監査と対策](README_IDM_Startup.md)を参照。制御・安全閾値の変更を有効化するものではない。

## 既存構成の問題と確認範囲

旧`capture_trial/trial_worker.py`と起動元`trial_supervisor.py`はGit管理外のローカル証跡で、
本変更では移動・上書きしていない。workerはCARLA/Python標準ライブラリに依存し、
設定に記録したPR #11 worktreeの`Run_DT_Risk_V2_IDM.py`を子プロセスで起動していた。
V2 captureはV1 capture、SynchronousMode、FollowingContextCollectorへ処理を委譲する。

旧親workerがTM8000を作成・sync化・seed設定・2車両登録・path/速度設定を行い、
capture中は子の終了をpollする。子は既存TMへRemote接続しworld.tickを進める。
[CARLA 0.9.16 TrafficManagerRemote.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/TrafficManagerRemote.cpp)
の`SynchronousTick()`はfalseを返すだけでlocal TMを進めない。
[Simulator.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/detail/Simulator.cpp)
からのTM tickは、そのプロセス内のTM mapを走査する。
この構成上の不整合は確認できるが、当時の具体的なthrottle/brake、登録実体、停止hazardは未記録。
「残留ブレーキ値が原因」と確定したわけではない。

旧CSVのautopilot_enabledはrole_nameをautopilot/heroと比較する推定値。
独自roleでFalseでも登録失敗の根拠にはならない。この既存列の意味・形式は変更しない。

## 実行構成・所有確認

`Run_IDM_Following_Experiment.py`が監視用親プロセスを作り、制御用workerを一つだけ起動する。
**TM、登録、速度/path、world tick、capture/context、監査、cleanupはすべて同じworker**。
監視親はCARLAをimportせず、OSのPID/port確認、worker exit/stdout/stderr、結果保存だけを担当する。
旧構成のようにTMとtickを別プロセスへ分けない。

1. loopbackの既存server PIDとlisten portが一致することをOSで確認。起動/reload/restart/停止はしない。
2. 同じserver portに対して協調的な排他leaseを取得。残存leaseは自動削除しない。
3. workerはfresh interpreter。worldが既に同期中、または外部vehicle/walker/sensor/controllerがあれば開始しない。
4. TM portが無所有でbind可能なことを確認してからget_trafficmanagerを呼ぶ。
5. **取得後のlisten PIDがworker自身だけ**であることを再確認。Remoteへの競合接続なら、
   seed/sync/actor操作の前に中断。get_portの値や単一プロセスだけでは所有を判断しない。
6. world sync/dt/substepを設定して読み戻し、local TM syncを同じworkerで設定。
   TM getterによるsync状態照合はAPIにないため、命令正常returnと所有証明を別々に保存する。
7. tickは一つのメソッドを経由し、前後frameの連続性、戻り値、elapsed time差0.05秒を照合。
   観測前後にもframe不変を確認する。collectorはtickを呼ばない。

OS所有検査はWindows Get-NetTCPConnection / Linux /procに対応し、不明ならfail closed。
外部TM portをshutdown/reseed/sync変更しない。TM全体shutdownも呼ばない。
協調leaseやframe照合は任意の別clientをOSレベルで排除するものではない。
**専用sessionで他のtick/controller/TM clientがいないことが実走前提**。
別portの外部TMや同frame内の外部操作を完全検知するAPIはなく、専用sessionの確認は実行者が行う。

### Seedと外部信号の保護

[0.9.16 TrafficManagerLocal.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/TrafficManagerLocal.cpp)
のSetRandomDeviceSeedは全交通信号をresetする。
Town10HDの既存信号に影響させないため、traffic_light actorがある場合はsetterを呼ばず、
`seed_requested=42, seed_applied=false`と理由を記録する。
既存信号がない専用環境でだけ一度適用する。未適用時の内部seedはunknownであり、
「seed42固定の再現実験」と扱わない。信号をfreeze/resetして帳尻を合わせない。

## 2段階のシナリオ

初期案を機械検証可能な`configs/idm_following_harness.json`へ移した。
私有evidenceのJSON/座標監査結果をそのまま公開せず、必要な実験条件のみを設定として管理する。

- CARLA0.9.16、Town10HD_Opt、dt0.05秒、Tesla Model3×2。
- road10/section0/lane+1、F s120・L s108、中心12m、nominal bumper gap約7.208m。
  spawn zは路面+0.4m、初期速度0、20tick接地待ち後にAutopilotを明示登録。
- sを減らす向きにs64まで56m、1m刻みのpath、auto_lane_change=False。
- 全点のlane/junction/heading/分岐・幅、前方95mのlandmark、level bboxの静的道路利用者を検査。
  static sceneryの衝突形状や地図の欠落を完全監査するものではなく、実地確認も必要。
- TM希望速度はkm/hへ変換。Fの自由速度2.2m/s=7.92km/h、TM車間parameter5m。
  set_target_velocityの反復、teleport、IDM式による観測値生成は使わない。
- warmupはL1.4m/s、両車1m/s以上が連続1秒、gap4〜12m、同lane・leader一致。
  最大6秒かつL移動6m以内。発進失敗を15秒収集で引き延ばさない。
- warmup後15秒capture、10秒snapshot。collectorはcapture開始で新規に作るので、
  初期2秒のhistory guardは従来通り残る。

**Stage 1**はL1.4m/s一定で走行成立と観測有効率を確認する。T学習の成功条件ではない。
**Stage 2**は正常終了・cleanup完了の、同じ設定fingerprintを持つStage 1のsupervised resultを要求する。
各stageは別の明示的な実走指示であり、自動連続実験にしない。

| Stage 2 capture時刻 | Lの希望速度 |
|---|---|
| 0〜2秒 | 1.4m/s |
| 2〜4秒 | 1.4→1.8m/s（線形） |
| 4〜5秒 | 1.8m/s |
| 5〜7秒 | 1.8→1.3m/s |
| 7〜9秒 | 1.3m/s |
| 9〜11秒 | 1.3→1.7m/s |
| 11〜15秒 | 1.7m/s |

Stage2の名目距離23.3m。L実速度上限1.9m/sなら28.5m、warmup6m、初期offset12m、
前端2.425mで合計48.925m。56m終端まで7.075mの余裕があり、実際の前端余裕6mで中断する。
F上限2.5m/s、0.5秒反応・2m/s²減速を仮定した停止距離は約2.813mだが、
これは設計上の概算でありCARLA制動性能の実証ではない。early stopでは同期worldを進めず、
自分の2台を登録解除・破棄して元設定へ戻す。危険なframeを進めて制動確認しない。

最低速度1m/sと整合する希望速度を使うが、実速度の下振れ・warmup失敗・TMの低速追従は未検証。
Stage2でF速度range<0.3m/sかつgap range<1mなら刺激不足で不成立。
これを満たしてもT識別成功とはせず、既存fitのresponse/profile/boundary/RMSE判定に従う。
先行車は刺激用であり、先行車自身のno_leader/T不採用は正常。

低速で識別性不足なら、56mで速度や時間だけ増やさない。代替案は、別途監査した
約200m以上の直線でL3〜5m/s、15秒収集、warmup<=20m、初期間隔約18m、終端余裕>=15m。
必要長の例は18+20+5.5×15+車体前端2.5+15=138mで、200mには追加余裕を持つ。
20〜30秒へ延長する場合は同じ式で道路長を再計算し、新しいconfig/schema/監査をレビューする。
この代替map/座標は未選定であり、現Harnessは固定監査区間以外を拒否する。

## 出力契約と観測

- capture_states.csv：既存19列。FollowingContextCollectorへ同じsnapshot/rows/凍結getter値を渡す。
- following_context.csv/meta.json：既存schema。captureとlogical ID/frame順が完全一致。
- snapshot_states.csv/json：既存V2形式、phase=snapshot、T初期1.5s、固定v0=2.2m/s。
- control_audit.csv：captureと同じ600key。実control/状態/加速度、TM命令参照、route/leader/context。
- warmup_control_audit.csv：warmupの全観測。settleの20tickはcommand_ledgerのsettle_observationへ記録。
- command_ledger.jsonl：TM ownership、seed、登録/解除、path全文、速度命令、実行前後のreturn/例外。
- last_observation.json：中断を起こしたframeの取得済みraw状態/制御。検査でCSVへ進めない場合も残す。
- config_used/environment/launch/worker_started/worker_result/result.jsonとstdout/stderr。

命令値と実測値を分離。`command_based_on_frame`は指示前frame、`control_read_at_frame`は取得frame。
TM計算・physics反映の遅れを無視して、同frameの速度変化がその命令によるものと断定しない。
API加速度vectorと車体前方向への射影を記録し、normや想定IDM加速度で補完しない。
contextの任意加速度列は従来通り空欄とし、推定器の速度微分を変更しない。

set_autopilotは通常Noneを返すため、`call_returned`は登録一覧による成功証明ではない。
`autopilot_registered_verified`は空欄。port所有証明、登録呼び出し、TM action、実control/応答を
別々の根拠として監査する。実登録一覧getterがあると仮定しない。
取得不能なacceleration/gear/action等は空欄、必要なcontrolが欠ければ中断。
collision sensorは作らないため、`collision_event`はunknownの空欄。bbox gap/TTCで接触リスクを検知する。

既存V1/V2のcapture関数・CSV/JSON、IDM式、推定器は変更しない。
collectorの唯一のAPI拡張は、書いたcontext行のlistをreturnすること。引数・ファイル形式は同じで、
戻り値を無視する既存callbackと互換。新しい実験ループはTM所有・warmup・速度profile・監査を
まとめる専用入口とし、V1の既定挙動へ制御hookを追加しない。既存writer/state/snapshot/collectorを再利用する。

## 中止・cleanup・終了の分類

中止理由、対象actor、simulation時刻、最終確認frame、wall時刻をworker_resultへ保存。
Capture中の中断はcollector metaをfailedにし、部分履歴をcompletedとして推定器へ渡さない。

| 条件 | 処理 |
|---|---|
| warmup3秒以後、Fが1秒継続0.1m/s未満／6秒・6mで未成立 | 中止 |
| capture中Fが0.5秒継続1m/s未満 | 中止 |
| gap<=2m、closing時TTC<1秒、bboxがlane外 | 中止 |
| lane/junction変化、Driving waypoint不明、終端前端余裕<6m | 中止 |
| 横偏差>0.3mまたはheading差>5°が0.25秒継続 | 中止 |
| L>1.9m/s、F>2.5m/s、hand_brake/reverse、制御欠損・非有限 | 中止 |
| leader不明/不一致0.25秒、初期guard以外のcontext無効0.5秒 | 中止 |
| actor消失/外部actor出現、signal true/unknown、frame/time不整合 | 中止 |
| TM sync失敗、capture/contextキー不一致 | 中止 |
| RPC runtime/connection/timeout、serverまたは監視親の消失 | gateをpoisonし、以後全CARLA呼出し禁止 |

正常終了の観測目標はF moving>=12秒、guard後の全13秒およびsnapshot前2〜10秒の
context有効率>=90%。fitでは微分window端点等をさらに除外するので、実際のsample/timeは別集計する。

健全なserver・frame連続性・自分のport所有を確認できた時だけ、**自分のactor IDのみ**を
登録解除→破棄。追加world tickなしでTM syncを解除し、WorldSettings全fieldを復元・照合、
所有actor残存0・既存ID保持を確認。既存actorへ制御・解除・destroyは送らない。
外部tick介入などで所有状態を失った場合は、他clientの状態を上書きせずcleanup_incompleteとする。
RPC異常後はcleanupも送らず、記録をflushしてos._exitでC++ destructorの暗黙RPCを回避する。
不明な復元/残存を成功とみなさない。

| status | exit code / 意味 |
|---|---|
| completed | 0、収集/品質ゲート/cleanup完了 |
| safe_early_stop | 2、事前条件または早期中止、cleanup不要/完了 |
| exception | 1、Python例外、cleanup完了/不要 |
| server_unavailable | 4、RPC禁止、復元/残存は未確認 |
| cleanup_incomplete | 5、元の終了理由とcleanup失敗を併記 |
| native_abnormal_exit | 3、signal/Windows native code/fatal stderr、監視親のみ記録 |
| worker_timeout / abnormal_exit | 6 / 1、監視timeout／結果とexitの不整合 |

native中断時の復元は保証できない。監視親はCARLAの復旧RPCや再起動を行わない。
Issue #7の修正・解決判定ではなく、障害時に被害を拡げず証跡を残す実験境界である。
CARLAのprocess終了時に起こるnative障害も実走では未検証。

## オフライン検証と実走前手順

```powershell
# 接続なし。CARLA wheelも不要。
python dt_risk_prediction_project/Run_IDM_Following_Experiment.py
python -m pytest -q dt_risk_prediction_project/tests
python -m pytest -q research_ogm_project/tests
python -m compileall -q research_ogm_project dt_risk_prediction_project
git diff --check
```

mock E2Eは300frame、各600行、settle20+warmup20+capture300=340tickを確認。
制御入力はfixtureのkinematicsでありCARLA物理/真のTの評価ではない。
同じ履歴を既存Estimate_IDM_Tへ渡し、traits出力をV2のresolve_parametersで読む。
snapshot後100行/actorの除外、先行車fallback、異常時meta拒否も確認する。

次回の明示的な実走指示後に、以下を一回ずつ行う。**この作業では下記--executeを実行していない**。

1. 現在のserver PID・開始時刻・CARLA0.9.16・map・listen PIDを読み取り確認。
   PID29436を固定で再利用しない。既存serverを起動/停止/restart/reloadしない。
2. 他client/TM/tick ownerがない専用sessionを確認。静的障害物・路面・spawnと距離予算をレビュー。
   seed未適用になる条件と再現性の限界も確認する。
3. config/fingerprint・source commit・出力先を記録し、Stage1を一回だけ明示実行する。

```powershell
# SERVER_PIDは読み取りで確認した値。実行には別途実走指示が必要。
python dt_risk_prediction_project/Run_IDM_Following_Experiment.py --execute --stage stage1 --server-pid SERVER_PID --output-root OUT_ROOT
```

4. result=completed・worker exit0・fatalなし・server生存・復元/残存監査、速度/gap/route/control/actionと
   観測有効率を人が確認。Stage1でT採用を要求しない。不成立時は理由を調査し、自動反復しない。
5. Stage2を別途明示実行。同じ設定のStage1 resultを渡す。

```powershell
python dt_risk_prediction_project/Run_IDM_Following_Experiment.py --execute --stage stage2 --server-pid SERVER_PID --stage1-result STAGE1_DIR/result.json --output-root OUT_ROOT
```

6. Stage2の正常終了・cleanup・独立leader/gapを確認後、オフライン推定。

```powershell
python dt_risk_prediction_project/Estimate_IDM_T.py --history STAGE2_DIR/capture_states.csv --context STAGE2_DIR/following_context.csv --snapshot STAGE2_DIR/snapshot_states.csv --idm-desired-speed-mps 2.2 --replace-snapshot-headway --output-dir FIT_DIR
```

T採用はsample/time、response/profile、境界、RMSE<=0.5m/s²等の既存条件で判断する。
低速のTM/PID走行にIDMが適合せず不採用になる可能性は残る。閾値を緩めて成功扱いしない。
今回の完了はオフライン準備完了まで。実走成立を確認してから将来予測への反映・比較を別作業で行う。

## 単車制動校正

単車・低速で制動入力と物理応答を観測する独立Harnessは
[README_Brake_Calibration.md](README_Brake_Calibration.md) を参照。
配布設定は未監査で実走不可。実走には新たな明示的承認が必要。
