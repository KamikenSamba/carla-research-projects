# IDM希望車間時間Tのオフライン感度・軌跡比較

Issue #44。PR #43の計測機能をbaseとする独立した数理分析。
CARLA Client生成・接続・world取得・tick・spawn・制御を行わない。
IDM式、IDMFleetController、PID/入力制限、安全gate、既存snapshot schemaを変更しない。
route_audit=null、physical_bounds=null、実CARLA追加走行BLOCKEDを維持する。

## 再利用と追加部分

- idm_model.IDMParameters / compute_desired_gap / compute_idm_acceleration：全加速度計算に使用。
- idm_snapshot.read_snapshot：保存済みV2 snapshotを検証して読み込む。
- idm_fidelity.csv_rows / write_csv：保存済みCSVの読込と新規出力。
- idm_leader.find_leader / sample_vehicle：既存のgapはbbox中心間の進行距離から半車長を引く仕様。
  今回の1Dモデルも同じ定義を使うが、先行関係は指定した2台に固定する。
- Run_DT_Risk_V2_IDMの予測はCARLAのworld.tickで物理更新するため、オフラインでは実行しない。
  steering/PID/actuatorを模擬した別Fleetは作らず、理想化した運動学だけを追加する。

追加ファイルはidm_t_sensitivity.py（微分・2台運動学）、Analyze_IDM_T.py（固定条件の分析・CSV/図）、
対応unit testと本文書。既存V1/V2入出力を変更せず、研究データをcommitしない。

## T感度の定義

同一比較ではT以外のv、leader速度vl、bumper gap g、v0/s0/a/b/delta/最大減速度Bを固定する。

```
z = v*T + v*(v-vl)/(2*sqrt(a)*sqrt(b))
q = s0 + max(0,z)
g_eff = max(0.001,g)
A_raw = a * [1 - (v/v0)^delta - (q/g_eff)^2]
A = clip(A_raw, -B, a)
```

加速度Aそのものは必ず既存compute_idm_accelerationから取得する。
微分はg>0・clamp内部・z>0なら dA/dT=-2*a*q*v/g_eff²（単位m/s³）。
z<0なら動的gap項が不活性で0。無先行車、停止v=0、下限飽和内部も0。
g<=0は既存IDMが定数-Bを返すが、運動学はそのまま重なり状態を走行させず打ち切る。

z=0では通常、左微分0／右微分-2*a*s0*v/g_eff²。s0*v=0なら両側0で微分可能。
下限clamp境界は左が生の微分、右が0となり得る。異なる場合、両側微分はnull。
境界分類の数値許容差1e-10は**分析上の判定**で、既存IDM計算や安全gateを変更しない。
T=0では許容域T>=0の右側を使う。h=1e-5秒の左・右・中央数値差分も保存する。
折れ点で中央差分が左右の平均に近づいても、通常の微分として採用しない。

低速ではv*Tが小さく、約15mのgapに対する相互作用比の変化も小さい。
速度が高くgapが有限で、まだ減速度下限へ飽和していない条件で感度が増す。
ただしgapを極端に小さくすると全Tが-Bへ飽和し、出力差が再び0になる。
感度マップの0は「安全」や「Tが一般に無関係」を意味しない。max不活性・停止・clampを区別する。

## 軌跡モデルの用途と仮定

1Dの無限直線、車線変更なし、2台のbbox中心位置x、非負速度v、固定車長L。
gap=x_leader-x_follower-(L_follower+L_leader)/2。
各stepの同じ更新前状態から加速度を求め、同時にballistic更新する。

```
x_next = x + v*dt + 0.5*A*dt²
v_next = v + A*dt
```

減速でstep途中に速度0になる場合、tau=v/(-A)まで進んで以後停止を保持する。
位置を逆向きへ動かしてから速度だけ0にする処理は行わない。
加速度をstep中一定にする明示解法であり、位置式が2次でもIDM ODE全体の高次精度を主張しない。
CARLAのタイヤ/エンジン/接地/steering/PID・scale/cap/slewを含まない。
IDM指令が理想的に運動へ反映された場合の比較であり、実CARLAの予測軌跡ではない。

leaderは既存IDMの自由走行、一定速度、または宣言した-2m/s²（t=2〜6s）を使う。
外生的leader profileは全Tで共通。変えるのはfollowerのTだけで、初期状態・他のパラメータは固定。
一般速度例はv=15/20/30m/s等の合成条件。実Town01道路の長さや実行許可を前提としない。

保存snapshot例は元のflat姿勢、bbox offset/車長、local_route方向を使って中心位置を1D投影する。
既存V2と同じ3D速度の大きさを初期の前進速度として使い、微小な垂直/後退成分は再現しない。
記録済み最初のfollower gapと1e-5m以内で一致しなければ拒否する。
この投影はエンジン内部状態等を復元せず、物理snapshot再構成の再試行には当たらない。

## 接触・TTC・時間刻み

速度ゼロの切替点と相対位置の極値で各stepを分割し、最初のgap=0を二分探索する。
step端では正gapでも、途中で交差して再び離れるケースを見落とさない。
接触時点で軌跡を打切り、以降の位置をpadding/推測しない。初期gap<=0はinitial_overlap。
これは理想運動学内の幾何イベントであり、CARLA collisionイベントではない。

gap/正の接近速度を瞬時のTTCとする。非接近時はnull。これは速度一定近似の指標であり、
実際の将来接触時刻や事故確率ではない。最小TTCは観測stepと接触点でのsampled値。
最小gapは各step内の極値も含む。TTC=nullを安全保証へ変換しない。
既存CARLA near-missのXY中心距離/接近速度判定と同じ指標ではない。

dt=.2/.1/.05/.025/.0125秒を比較する。finest=.0125は参照解であって厳密解ではない。
非接触では同じhorizonの終端差、接触例では接触時刻差を記録する。
異なる接触時刻の終端位置を同一時刻の予測誤差として比較しない。

## 実行（完全オフライン）

```powershell
python dt_risk_prediction_project/Analyze_IDM_T.py `
  --output-root '<Repository外の新規ディレクトリ>' --plots
```

既存低速データとsnapshotを含める場合だけ、元証跡rootを追加する。

```powershell
python dt_risk_prediction_project/Analyze_IDM_T.py `
  --evidence-root '<既存V2実験証跡root>' `
  --output-root '<新規分析ディレクトリ>' --plots
```

旧証跡rootへの出力、既存出力の上書きは拒否。600状態の加速度を元のパラメータで再現後、
T=.5/1/1.5/2/3の反実仮想値を求める。新しい観測データやT推定結果として扱わない。
JSON/CSVは標準ライブラリ、--plotsだけmatplotlibが必要。server/GPU/CARLA importは不要。

- sensitivity_grid.csv：低速/通常域2640条件、パラメータ・解析/数値微分・regime。
- historical_T_counterfactual.csv：元600状態×5T、先行車なしも明記。入力がある場合のみ。
- trajectories.csv：各Tの状態時系列、IDM加速度、gap、sampled TTC。
- scenario_results.csv：初期条件・全IDM値・距離/速度/最小gap/TTC・接触打切り。
- time_step_comparison.csv：5刻み幅での比較。summary.json：仮定・source hash・未成立項目。
- T_acceleration.png / T_sensitivity_maps.png / heatmap_data.csv：静的感度とマップ全数値。
- *_trajectories.png：T別の速度・gap・位置・TTC。接触後の曲線は描かない。

今回判断できるのはIDM数式と理想化した軌跡上のT効果まで。
CARLAの物理応答で差を識別できるか、事故リスク予測精度が改善するかは、
新たな明示承認・未充足安全条件の解決・対応する実測/独立Ground Truthを必要とする。
