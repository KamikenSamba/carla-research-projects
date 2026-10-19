# Stage 1の低速発進：制御計算と次回設計

本書は固定バージョンのソース調査と実験設計。新たなCARLA接続・走行は行っていない。
ログの数値・frame表・実験データはローカル証跡に保全し、ここには一般化した式と設計条件のみを置く。
安全ゲートを通すだけでなく、定めた速度・gap・制御応答を安定維持することを成功条件にする。

## CARLA 0.9.16の確認結果

- [Constants.h](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/Constants.h)：
  都市部縦ゲイン=(12,0.05,0.02)、高速側=(20,0.05,0.01)、切替速度60km/h、
  PID内部DT=0.05秒、throttle上限0.85、通常brake上限0.70。
- [Parameters.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/Parameters.cpp)：
  exact desired speedを保存。速度割合設定は除去され、GetVehicleTargetVelocityでexact値を返す。
- [MotionPlanStage.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/MotionPlanStage.cpp)：
  希望速度を3.6で割り、landmark/turn/collisionによる修正後のdynamic targetをPIDに使う。
  車速はvelocityの3D長。emergency_stop時は通常PIDを上書きしbrake=1。
- [PIDController.h](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/PIDController.h)：
  次の誤差と演算で制御を作り、正ならthrottle、負ならbrakeを飽和する。

```text
e[n] = (v_dynamic_target[n] - |velocity[n]|) / v_dynamic_target[n]
u[n] = 12 e[n] + 0.05 (e[n] + e[n-1]) 0.05
                    + 0.02 (e[n] - e[n-1]) / 0.05
throttle = min(max(u,0),0.85)
brake    = min(max(-u,0),0.70)
```

第2項は直近2点の和で、誤差を長期間蓄積する一般的な積分器ではない。
停止時e≈1なので、希望速度を下げるだけでは出力は小さくならない。
P項だけでみた非飽和域は誤差およそ-5.83%〜+7.08%。低速ほど絶対速度での幅が狭い。
実際にはI/D項も加わる。nominal targetでの再計算はdynamic targetやhazardを取得したことにはならない。

[Simulator.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/detail/Simulator.cpp)
ではworld frame受信後にプロセス内TM Tick、
[TrafficManagerLocal.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/trafficmanager/TrafficManagerLocal.cpp)
では同期トリガ後に状態取得・計算・ApplyBatchSync(false)。追加world tickは送らない。
[Vehicle.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/Vehicle.cpp)
のGetControlはactor snapshotの値であり、直前TM計算結果を直接取得するAPIではない。
同じframeで観測したcontrolと速度から即時閉ループ応答を断定しない。

制動値が出たframeで試験を終えた場合、制動後の減速・停止距離・整定時間は未観測。
gear、snapshot配信、UE車両入力処理、physicsのどこにどれだけの遅れがあるかは別途観測が必要。
また[WorldObserver.cpp](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/WorldObserver.cpp)
のAPI加速度はframe間のvelocity差分から作られる。独立した加速度センサ値ではない。

## 今回実装する最小変更

制御・道路・希望速度・安全閾値・IDM式・T推定器は変更しない。
新たな速度controllerや初期化を導入する前に、中止frameの観測不足を解消する。

- last_observation.jsonをschema_version=2へ拡張。既存phase/rows/各rowのpose・velocity・controlは保持。
- 両車の状態を先に取得・保存し、それから既存のsemantic safety gateを適用する。
  一台目が逸脱しても二台目の同frame raw状態を失わない。
- acceleration XYZ、車体forward XYZへの射影、roll/pitch/yaw、bboxのlocal中心/extent/rotation、
  logical ID、world time、命令sequence/発行基準frame/希望速度とcontrol取得frameを記録。
- unknownはnullまたは理由付きの非有限値文字列。加速度・gapを予測値や既定bboxで埋めない。
  raw bboxは後の独立幾何監査用であり、Contextと同等のgap観測を追加したわけではない。
- 取得開始でpartial envelopeを書き、取得済みrowごとに更新する。途中RPC失敗で過去frameを
  現在frameと誤認しない。両車取得後のframe再照合が通った時だけframe_validated=true。
  completeは必須getter取得完了の意味で、optional accelerationがknownとは限らない。
- RPC失敗後は従来通り追加RPC禁止。観測のgetterは増えるが制御命令とworld.tickは増やさない。

既存capture/context/control CSV、snapshot/traits JSON、既存consumerの契約は維持する。
raw JSONを厳密な固定key集合で読む独自consumerはschema_versionの更新に対応が必要。

## 対策の比較

| 案 | 研究・比較公平性 | 再現性/物理 | 実装・安全性 |
|---|---|---|---|
| 1: TM維持、長い道路と通常速度域 | 対象運転器を変えずIDMを近似モデルとして評価できる | 同車種・初期条件・map・設定を固定。TM seed未適用の制限は残る。物理走行を維持 | 新route監査/config対応が必要。高速度だけでは発進飽和を消せない。停止距離と加速度gateを別に定義 |
| 2: 明示的な発進初期化 | 遷移区間を除外し、TM区間だけfit。初期化方法を全比較で揃える | 一回の初期velocity/gear設定等は人工的初期条件。以後の実運動を強制しない。受渡し後のphysical応答を確認 | 方式ごとのTM未登録→handoff状態機械が必要。rolling startだけで低速安定を保証しない |
| 3: 実験専用の独立速度制御 | TM運転特性ではなく新controllerの有効Tを評価する研究になる | PID/加速度/車種を固定し、全て実physics経由。IDM式で観測を生成しない | longitudinal/lateral両方の制御責任を明示。anti-windup、slew/出力制限、制動校正等が必要で実装量最大 |

案2の候補：一回だけ与えるrolling initial velocity、TM未登録下のbounded手動助走後handoff、
別の安全な助走区間からの収集開始。単なる目標速度rampは停止時の正規化誤差がほぼ1なので
throttle飽和を防ぐ保証がない。手動助走のcontrollerも物理校正が必要で、未検証のgainを安全値と呼ばない。

TMと手動の同時適用は禁止。例として `TM未登録の初期化 → 手動出力解除 → 同期boundaryでTM登録 →
十分な連続安定観測 → capture` の責任遷移を台帳に残す。初期化中も加速度/速度/gap/routeのgateを維持。
set_target_velocityの反復・teleport・IDM加速度での走行履歴生成は採用しない。

推奨はまず観測修正、次に案1を主軸に案2の明示初期化を必要に応じて組み合わせる設計レビュー。
案3はその後もTM条件で目的を満たせない場合に限る。今回のPRは観測修正のみで、以下の実走案を
実装済み・安全確認済み・直ちに実行可能とは扱わない。

## 次回Stage 1の条件案（どちらも未実装・未実走）

共通の仮定：制御/観測遅れを暫定0.2秒、利用可能な減速度を暫定2m/s²と置く。
停止距離見積りは `d=v*tau+v²/(2b)`。数値は設計仮定であり、今回未観測の制動性能を保証しない。
実測で仮定を満たすことを確認するまで長い評価へ進めない。

| 条件 | 現行56m区間の条件付き案 | 約200m以上の別直線路案（優先候補） |
|---|---|---|
| 道路 | 同監査区間。追加の助走距離が必要なら不成立 | 非交差点・信号/分岐/障害物なし、全車線を新規監査。座標未選定 |
| Stage1希望速度 | 両車1.4m/s、片方だけ2.2m/sへ加速させない診断条件 | L4.0 / F4.5m/s。TM継続、通常速度域での応答を評価 |
| 初期間隔 | 中心12m、bumper gap約7.2m | 中心22m、bumper gap約17.2m。制動仮定の悪化も考慮 |
| 初期化 | 両車の一回のrolling初期条件を検討。gear/制御handoffを監査 | bounded初期化/助走を検討。高速度にしただけで発進問題を解決した扱いにしない |
| warmup上限 | 6秒かつL移動6m以内 | 8秒かつL移動25m以内 |
| capture | 15秒、snapshot10秒 | 15秒、snapshot10秒 |
| 実速度上限 | 両車1.9m/s（現L上限維持）、希望からの上振れ予算0.5m/s | L4.5 / F5.0m/s、上振れ予算各0.5m/s。新たな速度/距離設計の一部 |
| 前端余裕 | >=6m。12+6+1.9×15+2.425+6=54.925mで56m内 | >=20m。22+25+4.5×15+2.425+20=136.925mで200m内 |
| 仮定停止距離 | v1.9で1.283m、gap2mを加えて3.283m | v5.0で7.25m、gap2mを加えて9.25m |

仮定の感度も確認する。tau=0.3秒・b=1m/s²まで悪化した場合、停止距離は短区間2.375m、
長区間のFで14m。gap2mを加えても、それぞれ初期間隔のbumper gap約7.2m/17.2m以内。
この範囲より制動が悪い可能性は除去できないため、実測なしに安全保証とは扱わない。

両案とも、開始前に実装/レビューが必要な追加条件：

- 初期化を含め|a_long|>5m/s²が1frame、または|a_long|>3m/s²が2frame連続で中止。
  現在の速度/経路/gap/TTC/hand_brake/RPC/actor保護gateは維持。加速度unknownなら評価開始しない。
- capture開始は速度が希望±0.2m/s、|a_long|<=0.5m/s²、gap/leader/lane安定が2秒連続。
  発進失敗をwarmup時間延長だけで吸収しない。
- 目標速度付近の減速・制動応答が実際に観測されること。今回のbrake出現だけで満たしたとしない。
- 押し出しの瞬間や初期化/受渡し/整定前をT fitへ混ぜない。T最小速度1m/s、sample/time、
  RMSE/profile/response/境界判定は変更しない。Stage1の定速診断はT識別成功を意味しない。

加速度gateだけで同tick内の衝突を防ぐ保証はない。初期化・距離予算・占有環境と併せて安全を設計する。
長道路でも別車両のgapを無視して走行を継続しない。上限を引き上げただけの再試験はしない。
今回の観測修正だけでは新速度域/初期化/加速度gateは有効にならない。現行設定の再試行とStage2は行わない。
