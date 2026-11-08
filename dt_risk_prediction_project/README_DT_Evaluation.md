# DT軌跡・事故リスク評価の契約

この評価器は保存済みCSVだけを読み、CARLAへ接続しない。予測モデル・制御・安全gateは変更しない。実CARLA追加試験は **BLOCKED**、`route_audit=null`、`physical_bounds=null` のままである。

## データの役割を先に固定する

|役割|定義|主張できる範囲|
|---|---|---|
|Source実軌跡|CARLA physicsから観測した位置・速度。制御器も記録する|その制御条件で起きた運動|
|再構成Replay|Source Snapshotから再生成した車両のCARLA実軌跡|再構成再現性。Sourceと同一IDMなら独立した予測精度ではない|
|予測軌跡|予測開始時刻までの情報だけで生成した将来状態|対応する保留済みSource futureがある場合に限り条件付き予測誤差|
|理想1D・counterfactual|既存IDM式の数値積分、または実測状態への別Tの再計算|数式・理想モデル上の感度。CARLA実測ではない|
|合成fixture|正解誤差を既知にしたテストデータ|評価処理の正しさ|

`prediction_states.csv`、`ground_truth`という名前だけで役割を決めない。TM制御Sourceも実測だが、TMを基準とする比較はTMに有利になり得る。自然交通・人間の運転の正解とは扱わない。Source側もIDMなら、Tのoracle比較はモデル内の再現性であり外的妥当性ではない。

## 現時点で実行する最小評価

`Evaluate_DT_Trajectories.py` は既存V2の完成済み `fleet` と、その `replay` の組だけを受け付ける。入力は各runの `result.json`、`environment.json`、`config_used.json`、`resolved_idm_parameters.json`、`launch.json`、`initial_snapshot.json`、`prediction_states.csv`、車両physics、およびReplayの `reconstruction_comparison.json`。既存Snapshot読込・CSVユーティリティ・physics比較関数を再利用する。

```powershell
python dt_risk_prediction_project/Evaluate_DT_Trajectories.py `
  --source-run '<完成済みfleet run>' `
  --replay-run '<そのSnapshotからの完成済みreplay run>' `
  --output-root '<Repository外の未使用ディレクトリ>' `
  --horizons '1,2,3,5'
```

入力内への出力・既存出力の上書きは拒否する。全読込ファイルのSHA-256を保存する。map/version/weather/dt/外部actor ID集合/設定/制御commit/シナリオ/車両別パラメータ/比較可能physicsが一致すること、完走・衝突なし・cleanup完了を要求する。logical IDと各runの実Actor IDはSnapshotおよび再構成リンクで照合する。

既存の再構成許容値（位置0.02m、速度0.01m/s、yaw 0.2度）を再確認し、初期残差を保存する。残差を差し引いて軌跡を合わせない。これらの値は道路・停止性能の認定値ではない。エンジン・サスペンション等の内部物理状態まで復元されたことは証明しない。

出力は `evaluation.json`、`actor_horizon_metrics.csv`、`time_errors.csv`、`paired_errors.csv`。purposeは `reconstruction_repeatability`、`independent_ground_truth_used=false`。このCLIは独立した予測精度や検出性能を出力しない。数値コアのもう一つの用途は `synthetic_validation` のみ。

## 指標・時間・欠測の契約

予測開始を各Snapshotのframe F0とする。比較時刻は `t=k*dt, k=1..H/dt`、初期t=0をADEに含めない。世界の絶対frameはSourceとReplayで異なってよいが、各streamで `frame-k=F0` を要求する。別runの同じframe番号を同じ時刻として扱わない。時刻シフト最適化・座標合わせ・補間・外挿はしない。

車両i、時刻kのXY誤差を `e(i,k)=norm(p_pred_xy-p_ref_xy)` とする。

- **ADE_XY(H)**: k=1..H/dtのeの平均。
- **FDE_XY(H)**: Hちょうどのe。XYZ版は別カラムにする。
- **速度誤差**: 3D速度ベクトルの大きさの差。MAE・RMSE・biasを出す。縦方向の符号付き速度誤差とは別定義。
- **時間別誤差**: 各tの車両平均とpaired/total車両数。H=1,2,3,5秒を同一cohortで比較する。

logical IDは両streamの和集合。欠測・途中消失を黙ってintersectionで捨てない。車両・Hの全時刻が揃わなければそのADE/FDEはnull、coverageを残す。NaN・無限大・重複キー・off-grid・実Actor ID切替・frame原点不整合はエラー。速度とvx/vy/vzの3D定義も照合する。

数値コアは明示されたcollision/disappearance時刻以降をcensorし、その時刻を跨ぐHのスコアをnullにする。イベントを欠測から推定しない。現CLIは衝突ゼロの完走組に限定する。衝突例の正式評価は将来、独立ラベルと固定したcensor方針を備えてから行う。

現在の集計は「単一組・単一シナリオ内の車両等重み」。有効車両のみの平均には必ず分母とcoverageを添える。方式間の優劣を比較する際は共通cohort・共通Hを事前固定し、方式固有の欠測率も報告する。将来の多シナリオ集計は車両→run→シナリオの順にmacro平均し、長い軌跡や多車両runだけが重くならないようにする。重複window・車両・frameを独立反復として信頼区間を計算しない。

## リスク指標は混ぜない

|指標|既存定義・計算可否|評価上の制約|
|---|---|---|
|CARLA near-miss|V1/V2のactor位置XY距離と接近速度の閾値、pair cooldown。既定距離4m、接近1m/s、cooldown 1s|bbox bumper gapやTTCではない。actor原点を近似中心として使用|
|Collision|衝突sensorのcallback、impulse、logical pair|V1共通RiskEventはevent.frameを直接保持せず、callback時のclient経過時間を使う。精密なラベル照合にはevent.frame/timeの保存が必要|
|最小bumper gap|既存following context/leader検出、または理想1Dの車長控除|道路に沿う順序・bbox・投影方法・適用laneを固定。XY距離から推測変換しない|
|縦TTC|同一進行方向・追いつく場合 `gap/(v_f-v_l)`|非接近はnull、gap<=0は接触状態。交差点・横断衝突の一般的TTCではない|
|検出性能|将来の独立したevent/negative windowラベルが必要|現評価器はprecision/recall/FPRをnullのまま保持|

将来、同じ車両pair・同じ時間window・同じ閾値・同じ重複抑制でTP/FP/FN/TNを確定した場合のみ、precision=TP/(TP+FP)、recall=TP/(TP+FN)、FPR=FP/(FP+TN)を計算する。FPRとfalse discovery rate=FP/(TP+FP)は別。分母0はnull。誤警報/時は時間当たりの別指標として定義する。同じTTC閾値から生成したラベルを評価して「衝突検出精度」と呼ばない。

## 次の多車両比較の設計（実行はBLOCKED）

1. 安全条件が成立し別途承認された後、2台の低速Sourceを先に収集。free flow・一定追従・接近・穏やかな先行減速・安全negative controlを分離する。交差/車線変更は別段階とする。理想分析の15〜30m/sを現行低速プロトコルへ移植しない。
2. t0の車両全状態・logical mapping・bbox・physics・制御器状態・map/hash・道路条件・信号・天候・外部交通・seed・dtを保存し、その後の実測futureを5秒以上保留する。t0後をtraits推定や初期化へ混入させない。t0実状態と再構成直後実状態の差を別途検証する。
3. 同じSource/windowからTM、既定IDM、車両別推定T-IDMを比較。速度trait等はT-only実験内で固定する。既知Tのoracleは上限参考で別枠。TM/IDMは縦制御・操舵・経路・制約が異なるため方式比較と記録し、Tだけの因果差と呼ばない。
4. まずシナリオ層ごと5独立seedのpilotを計画し、paired run差の分散から必要数を決める。次段階の目安は10以上/層だが検出力の保証ではない。2台で成立後に4〜8台へ拡張し、混雑/車両消失/外部車両を別層にする。手法は同じseed/windowで対にし、学習・調整・testをSource run単位で分割する。
5. H=1/2/3/5秒、runごとのpaired誤差差・coverage・初期残差・collision/near-missを併記。信頼区間はSource run単位のcluster bootstrap等とする。正例・負例の独立ラベルがない段階は幾何リスク値の比較に限定する。

実行の残条件は正式route_audit、実測physical_bounds、停止距離/通信遅延、サーバidentity・排他session・単一tick owner、実行条件とcleanup/restoration確認、および操作者の明示承認。既存低速成功やオフラインCIをこれらの代替にしない。
