# 走行履歴からのIDM希望車間時間T推定（Issue #8）

PR #6のIDM式を使い、**過去の複数時刻の追従履歴からTだけをoffline推定**する。
`Estimate_IDM_T.py`はCARLAをimport・起動しない。V1、V2、IDM数式、既存CSV/JSONの
schema・traits優先順位は変更しない。追加runtime依存はない。

## 1．既存履歴の確認と不足情報

| 項目 | capture_states.csv | 今回の扱い |
|---|---|---|
| logical_actor_id、source_actor_id | 取得済み | snapshotと同じlogical IDで対応付ける |
| 時刻、CARLA frame | sim_time_s、carla_frame | snapshotと同じcapture・時計を使う |
| 速度 | speed_mps、vx/vy/vz | m/s。IDMの非負スカラー速度として使用 |
| 観測加速度 | なし | 連続した速度履歴の局所回帰で算出、またはsidecarで供給 |
| 先行車速度・車間距離 | なし | 同frameの先行車とバンパー間gapをsidecarで供給 |
| 道路・車線 | なし | road_id、section_id、lane_idをsidecarで供給 |
| 信号・車線変更・junction・fallback | なし | 明示的な状態をsidecarで供給。欠損をfalse扱いしない |

確認した既存captureは196行、19列であり、context不足のため有効0件・fallbackだった。
位置とblueprintだけから、曲線路のバンパー間gapや先行車を正確に復元できるとは仮定しない。
既存`idm_states.csv`にはleader/gap/lane/fallbackがあるが、制御前の状態であり、
予測後の`prediction_states.csv`と時刻がずれる。signal等も不足するのでそのまま学習入力にしない。

### 追加観測sidecarの最小構造（実CARLA収集器は未実装）

既存captureはそのまま残し、同じtickで次のCSVを別ファイルとして収集する設計。
join keyは **(logical_actor_id, carla_frame)** の完全一致。1ファイルは1capture/episodeに限定し、
別captureを連結しない。観測器は追加tickや制御変更をせず、既存のtick後に採取する。

| 列 | 単位・意味 |
|---|---|
| logical_actor_id、carla_frame | captureと同じ対応表・frame |
| leader_id | 同じlane上の最も近い有効先行車のID。先行車なしは空欄 |
| leader_speed_mps | 同frameの先行車速度、m/s |
| gap_m | lane経路に沿ったバンパー間距離、m。bounding box中心offsetと長さを考慮 |
| road_id、section_id、lane_id | 対象車のlane key |
| signal_stop | 信号・標識等による停止/減速の影響を受ける区間はtrue。不明は空欄 |
| lane_change | 車線変更中・移行周辺はtrue。不明は空欄 |
| is_junction | junction内はtrue |
| fallback_reason | route境界、経路曖昧、heading不整合等の理由。正常と確認した場合のみ空文字 |
| observed_acceleration_mps2（任意） | 非負速度の時間微分と同じ符号規約の縦加速度、m/s²。加速度vectorのnormは禁止 |

source_actor_idはcapture側から得る。先行車は外部車両を含めて観測するが操作・破棄しない。
先行車・gapの観測は既存`idm_leader.py`のsample/trace/findを再利用する案とし、
route境界に近い時などの除外理由も併記する。単に最近傍の車両を先行車としない。
信号の影響判定やlane-change区間の収集は追加設計・実CARLAでの確認が必要。
未観測の状態を都合よく正常で埋めて推定する運用は禁止する。

context列をすべて含むenriched history CSVも入力できる。sidecar指定時に上書きできるのは
上表のcontextと任意加速度のみで、速度・時刻・actor IDは変更しない。
sidecar keyの欠落・重複はエラー。joinできなかった行はcontext不足として除外する。

## 2．推定方法と品質ゲート

`idm_history.py`は`compute_idm_acceleration`を再利用する。s0、a_max、b、delta、v0、
最大減速度はactorごとに固定し、Tを0.3～4.0秒、0.01秒刻みで探索して加速度MSEを最小化する。
既存関数の加速度上限・下限もそのまま使用する。PIDや加速度filterの係数は校正しない。

- 各actorを時刻順に並べ、重複時刻は全該当行を除外する。時刻・速度が不正で位置付け不能な
  行があれば、そのactorは安全側にfallbackする。
- 速度1 m/s未満、信号停止、先行車なし、重なり、junction、車線変更、fallback、不正contextを除外する。
- 中心±3点の7点窓を使う。窓内の不正行、lane/leader/source ID変更、0.15秒超の間隔を跨がない。
  直接加速度がある場合も同じ窓で移行区間を除外する。
- 加速度がない場合は実時刻に対する速度の局所最小二乗直線の傾きを使う。単純な隣接差分より
  ノイズを抑えるが、速度の誤差や急変による推定biasを除去できる保証はない。
- 有効20件以上かつ連続区間の合計有効時間2秒以上を要求する。区間間の長い欠損は時間に加算しない。
- T探索端点の予測加速度差のRMSが0.1 m/s²以上、MSEが最小値+0.05²以内のT領域の幅が
  1秒以下であることを識別可能性の暫定ゲートとする。profile intervalは統計的信頼区間ではない。
- 境界到達、識別不能、RMSE>0.5 m/s²、数値計算不能は不採用。数値誤差だけで飽和区間から
  Tを決定しない。閾値は研究用の初期設定で、実データで妥当性確認が必要。

CLIはactorのsnapshot時刻より後の履歴を除外する（浮動小数の丸めに1e-9秒の許容）。
中心窓にも未来の点を混ぜない。APIの`estimate_traits(..., cutoff_times=...)`も同じ制約を指定できる。
`estimate_headway`は渡された履歴だけを扱うため、直接API利用時には呼出し側が時刻を制限する。

各actorの診断には推定T・採用T、有効件数/時間、直接/速度由来加速度の件数、除外理由別件数、
RMSE、境界、profile範囲、識別可能性、固定パラメータ、採用/fallback理由を記録する。
CLIは入力SHA-256も保存する。RMSEは**同じ履歴への当てはまり**であり、将来予測精度ではない。

## 3．既存V2との接続と優先順位

出力は既存`idm_snapshot.load_traits`で読めるactor別mappingであり、別schemaをV2へ導入しない。
V2の`--actor-traits`に`actor_traits.json`を指定する。診断情報は別の
`headway_diagnostics.json`に保存し、traits内へ混入させない。

推定時の優先順位：**明示traits T > snapshot T > 採用可能な推定T > 解決済み既存T**。
snapshotにTがある場合は標準では保護する。V2 captureで既定値として埋めたTを置き換えたい場合も、
明示的に`--replace-snapshot-headway`を指定する必要がある。この指定でも明示traitsは保護する。
protected actorでもfit品質は計算するが、`adopted=false`と保護理由を記録する。

v0は自動推定せず、明示traits > snapshot v0 > `--idm-desired-speed-mps`の順で固定する。
CLIでは最後の値を必須とし、snapshot速度を無意識にfree-flow speedと扱わない。
出力traitsに固定v0も保存し、推定時と予測時のv0が変わることを防ぐ。
ほかの固定IDM係数はtraits形式では表現できないため、予測CLIにも同じ値を指定すること。

target snapshotにいない履歴actorは診断に列挙してexportから除外する。
明示traitsにsnapshotにいないactorが含まれる場合はエラー。
推定不能actorも既存値をexportし、不採用理由を診断へ保存する。

```powershell
# dt_risk_prediction_project/ から実行。ここでは標準の固定IDM係数を使用。
# 出力先は未作成のディレクトリを指定する。既存結果は上書きしない。
python Estimate_IDM_T.py --history capture_states.csv --context following_context.csv --snapshot snapshot_states.csv --idm-desired-speed-mps 18 --output-dir fit_run

# 明示traitsがある場合は推定時に必ず併合して保護する。
python Estimate_IDM_T.py --history enriched_history.csv --snapshot snapshot_states.json --actor-traits explicit_traits.json --idm-desired-speed-mps 18 --output-dir fit_with_overrides

# 将来の実CARLA用コマンド例。今回実行していない。既存V2の読取り経路を使う。
python Run_DT_Risk_V2_IDM.py --mode predict --prediction-controller idm --snapshot snapshot_states.csv --actor-traits fit_run/actor_traits.json --prediction-seconds 5
```

V2が複数のtraitsファイルを自動併合するわけではない。既存の明示traitsを推定CLIへ渡した後、
生成した統合済みtraitsをV2に渡す。手作業で推定値を明示設定へ上書きしない。

## 4．オフライン結果（2026-10-19）

| 条件 | 有効件数 | 推定T | 加速度RMSE |
|---|---:|---:|---:|
| 合成追従、真値0.8秒 | 234/240 | 0.80秒 | 0 m/s² |
| 合成追従、真値1.5秒 | 234/240 | 1.50秒 | 0 m/s² |
| 合成追従、真値2.5秒 | 234/240 | 2.50秒 | 0 m/s² |
| 真値1.8秒、速度noise標準偏差0.025 m/s、速度由来加速度 | 234/240 | 1.80秒 | 約0.0942 m/s² |
| 既存capture 196行、contextなし | 0/196 | 不採用・既存値 | 評価不能 |

合成データは同じIDM式で生成した機能試験。無雑音での一致は実車traits推定の精度を示さない。
noise試験もseed=42の1条件であり、実データ全般への頑健性は未検証。
詳細な生成物はローカルevidenceに置き、CSV/JSON結果や実ログをGitへ含めない。

テストは既知T復元・車両差・noise・欠損・各除外条件・重複・時間/leader/lane/source切替・
加速度飽和・探索境界・モデル不適合・数値異常・snapshot時刻制限・優先順位・CLI export・
V2 predictionへのparameter受渡しを含む。CARLA importを禁止するoffline CLI試験も含む。
DTは既存44件＋追加40件＝84件、OGM既存143件が成功。
両projectのcompileall、OGM configsのJSON検証、CLI help、git diff --checkも成功。
実CARLAでの観測sidecar収集、実履歴Tの推定精度、物理controllerによる追従精度は未検証。

## 5．次の実CARLA比較実験計画（未実施）

1. Issue #7のserver描画障害とTM終了障害を別管理する。専用の検証環境と明示的な起動許可を
   得るまではサーバを起動しない。今回の合成試験を理由にTM安定性を合格にしない。
2. captureとcontextを同frameで保存し、signal/route/lane-changeの判定、バンパーgap、縦加速度の
   符号・時刻整合を少数の場面で監査する。actor対応表、map、blueprint、bbox、seed、frameを保全する。
3. 学習はsnapshot以前の独立した追従区間のみ。snapshot以後の実測をholdout truthとして分離する。
   時間窓・actor単位で分割し、同じ時刻を学習と評価で使わない。
4. 同一snapshot、車両、固定IDM係数/v0、physics/controller設定、horizonから、(a)一律T=1.5秒と
   (b)採用されたactor別Tを比較する。T以外は変更しない。採用不能actorはbaseline値を使用する。
   推定なしでも動く経路を保持し、採用率と除外理由も集計する。
5. 0.5/1/2/3/5秒などの各horizonで位置誤差（XY距離、可能ならlane方向/横方向）、速度絶対誤差、
   正しい先行車とのバンパーgap絶対誤差、軌跡全体の平均/最終誤差を計測する。
   leaderなし、leader変更、fallback、spawn失敗、出力欠損は別分類し、都合よく成功率から除かない。
6. まず少数のpaired runで時刻・出力・cleanupと既存actor/WorldSettingsを監査する。
   server異常時は中断し、再起動せず証跡を保全する。安定性と評価可能性を確認した場合だけ
   複数snapshot・車両・seedへ広げる。各条件のclient exit/fatalとserver状態も別に集計する。
7. 同じsnapshotごとの差分で評価し、改善車両率、平均/中央値/分位点を報告する。
   baselineより悪化した条件、fit RMSEとholdout誤差の関係、固定v0の誤指定に対する感度も確認する。
   T推定と物理校正の実験を同時に行わない。

推定したTは固定v0・モデル・観測条件に依存する有効値であり、運転者の普遍的な性格の確定値ではない。
実データでの改善が得られるまでは、研究の将来予測精度が向上したとは主張しない。
