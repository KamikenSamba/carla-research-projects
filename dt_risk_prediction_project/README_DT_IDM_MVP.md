# Snapshotからの2台IDM未来予測MVP（完全オフライン）

保存済みSource Snapshotのt0状態を初期値に、既存IDMから2台の将来軌跡を出力する最小接続です。新しいIDM式・FleetController・契約・安全監査は追加していません。CARLAのimport/接続/spawn/tick/制御はありません。

| 機能 | 既存実装と今回の接続 |
| --- | --- |
| Source Snapshot正規化・ID保持 | `idm_snapshot.read_snapshot_rows` |
| IDM加速度 | `idm_model.compute_idm_acceleration`（変更なし） |
| 同時2台更新、停止、接触打切り | `idm_t_sensitivity.rollout / advance / step_geometry`（変更なし） |
| DTのCARLA上再構成・制御 | `Run_DT_Risk_V2_IDM.py` / `IDMFleetController`に既存実装あり。今回は実行しない |
| Snapshot→オフラインDT状態・出力 | 新規`idm_dt_mvp.py`と`Predict_DT_IDM.py` |
| CSV・可視化 | 既存`idm_fidelity.write_csv`と既存分析CLIと同じmatplotlib/Agg方式 |
| 予測評価 | `dt_trajectory_evaluation.py`は再構成再現性/合成評価用。独立予測精度の評価には流用しない |

## モデルと入力

同一frame/timestampにある2台だけを対象とします。道路は平坦・直線、heading差5度以内、横方向差0.25m以内という1Dモデルの適用条件を検査します。これは走行安全gateではありません。Sourceの位置・姿勢・速度ベクトル・IDはsummaryへ保存します。予測上の初期XY/Z/yawはSourceと一致します。

Snapshotにはbboxが含まれないため、既存raw JSONLのF0までのprefixから両ActorのF0 bboxを取得した時点で読込みを終了します。さらに既存pair_rawのF0 stateだけを読み、logical/Actor ID・frame・run相対時刻・world時刻を照合します。両prefixはF0取得完了で停止し、消費したbytesだけのhashを記録します。Source future、将来制御、将来leader速度は予測へ渡しません。F0データが不足した場合、将来frameで補完しません。

明示source-run-idと3入力の同じ正規化Sourceディレクトリ名を照合し、別ディレクトリ・別ID・別frame・矛盾するclockは拒否します。これはrunのディレクトリ規約の確認であり真正性の証明ではありません。旧Snapshotに内包run ID/world時刻がない場合はNOT_RECORDEDを保存し、補って内包値照合済みとは主張しません。

F0位置XYZ・速度XYZ/ノルム・姿勢・進行方向の差と、bboxから導出した初期車間の差をf0_comparisonへ保存します。数値残差は診断値であり、未承認の許容閾値で合否を決めず、数値acceptanceは常にNOT_ASSESSED_NO_APPROVED_TOLERANCEです。非ゼロ残差でもSnapshotをrawへ合わせたり補正したりしません。必要なrawフィールド欠損は拒否します。PairHarnessの保守的footprint車間はV2 bbox車間と異なるため、別名で保存します。

車体原点とbbox中心のoffsetを分離し、車間距離は投影bboxのbumper gapです。位置更新は既存の区分一定加速度ballistic積分、負速度になる時刻で停止し、接触時は打ち切ります。車体Z・yaw・横方向位置は固定です。既存V2/感度分析と同じ3D速度ノルムを1D初速度として使います。これは速度ベクトル全成分の物理的再現ではなく、特に接地中の微小な鉛直速度を含む近似です。元の速度ベクトルは改変しません。

leaderはt0状態から既存IDM自由走行項で予測します。followerのみ既知の実験パラメータTを1秒/2秒に変更し、初期状態・leaderパラメータ・その他followerパラメータ・dt・horizonを固定します。Tは推定しません。Snapshot内の既知T/v0を暗黙のoracle入力にせず、CLIの明示値を使います。

既存Snapshotの`sim_time_s`はrun相対時刻の可能性があるため、F0 rawの`world_time_s`と別に保存します。出力はprediction_step/time_sであり、将来CARLA frameを観測したとは表現しません。

## 実行例（Repository root）

```powershell
python ./dt_risk_prediction_project/Predict_DT_IDM.py `
  --snapshot '<保存済みinitial_snapshot.json>' --raw-prefix '<対応するcalibration_raw.jsonl>' `
  --state-prefix '<同一runのpair_raw.jsonl>' --source-run-id '<Sourceディレクトリ名>' `
  --follower-id follower --leader-id leader --input-kind recorded_snapshot `
  --follower-v0 0.6 --leader-v0 0.5 --headways 1 2 `
  --a 0.3 --b 1.5 --s0 2 --max-decel 2 --dt 0.05 --horizon 6 `
  --output '<Repository外・入力ディレクトリ外の未作成出力先>' --plots
```

出力は`predicted_trajectories.csv`（2台×T別の位置・速度・IDM加速度・車間・TTC）、`T_comparison.csv`、`prediction_summary.json`、`prediction_comparison.png`です。最終時刻の加速度空欄は、次の指令を計算していないことを表し、0で補完しません。matplotlibは`--plots`時のみ必要です。既存ファイル・出力ディレクトリの上書きは禁止します。CLIの`input-kind`は申告分類であり真正性の認証ではありません。

実測Snapshot由来の低速条件と、T感度確認用の合成初期条件は別ディレクトリ・別input_kindとして保存します。合成の交通速度をCARLA低速試験の承認値と解釈しません。

## 検証と解釈

`python -m pytest -q dt_risk_prediction_project/tests/test_dt_idm_mvp.py`

初期状態/ID/clock、既存IDM式との一致、同時更新、連続時刻、有限値、T以外の固定、future未読、欠測拒否、原データ非改変、オフラインimportを検証します。

このMVPは数式上のDT予測であり、CARLA物理エンジン・PID・タイヤ・接地・操舵・事故挙動を再現していません。Sourceと予測の差を独立予測精度や事故検出性能として評価していません。`route_audit=null`、`physical_bounds=null`、実CARLA試験BLOCKEDを維持します。読取Stage A readerの未commit変更とは別worktreeで管理します。
