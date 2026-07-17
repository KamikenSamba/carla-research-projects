# CARLA Research Progress Update / CARLA研究進捗

Date / 日付: 2026-07-17

## 日本語

### 1. 目的

CARLAデジタルツイン未来予測において、観測履歴から車両固有の速度・車間特性を推定し、Traffic Managerへ反映する基礎実験を進めている。今回の更新では、速度特性実験の残差監査、1ペアの車間応答校正、steady following未達原因のtick単位監査までを記録する。

### 2. 速度特性実験

- slow / normal / fastの既知速度profileを持つ固定sourceを使用した。
- 観測履歴だけからのprofile分類精度は100%、希望速度MAEは0.249 km/hであった。
- 主評価7 Actorの未来予測では、baselineの速度MAE 3.995 m/s、ADE 18.862 mに対し、individual inferredは速度MAE 0.294 m/s、ADE 1.364 mまで改善した。
- fast群の残差は、主にTraffic Managerの周期的な速度制御と切替後の過渡応答による。
- Actor 7のcommonからinferredへの小さな悪化は、3反復のばらつきに対して有意とは判断できなかった。

### 3. 車間特性source gate

- Leader/Follower 6ペア、short 6 m、normal 12 m、long 20 mのsourceを試した。
- 最終試行の実現中央値はshort 6.26 m、normal 7.65 m、long 18.55 mであった。
- short-normal差が1.39 mしかなく、必須基準3 mを満たさなかった。
- source gate不合格のため、baseline / common / inferred / oracleの未来予測12 runは実行していない。

### 4. 1ペアTraffic Manager校正

- Town10HD_Optのroad 10 / section 0 / lane -2を使用した。
- Leader 8 km/h、Follower 14 km/h、初期bumper gap 27.5 mとした。
- `TrafficManager.distance_to_leading_vehicle`へ4, 6, 8, 10, 12, 16, 20, 24, 28 mを設定し、各3回、合計27 runを実行した。
- collision、lane change、leader変更は全runで0であった。
- steady following成功率は全設定0%であった。
- steady未達runの終端診断gapは4 m設定で4.51 m、6 mで6.30 m、8 mで8.00 mとなったが、10 m以上は約8.71 mに飽和した。
- 安定したshort / normal / longを選定できず、6ペア再実験へは進んでいない。

### 5. steady未達のtick監査

- 保存済み27 runだけを読み、CARLA実験は再実行していない。
- Leader検出は11.1秒以降40.0秒まで連続成立した。
- Followerはroad 10 / lane -2を維持し、junction進入なし、残存lane距離は最小6 mであった。
- 4 m設定では相対速度条件の成立率10.0%、gap標準偏差条件87.7%、瞬時全条件の最長連続0.2秒であった。
- 6 m設定では相対速度条件12.1%、gap標準偏差条件93.1%、最長0.2秒であった。
- 全設定で3秒窓全条件の連続成立は0秒であった。
- throttle/brake切替は92～118回で、約0.82～2.11秒周期の加減速を確認した。
- 相対速度・gap標準偏差・連続時間を緩和した感度分析でも成立runは0であった。
- steady判定ロジックに結果を覆す不具合は見つからず、主因はTraffic Managerの周期的な相対速度変動と判断した。

### 6. 次の作業

1. より長いlaneで道路長制約を除外する。
2. fixed delta 0.1秒と0.05秒を比較し、同期刻みによるTraffic Manager振動を切り分ける。
3. 安定した3車間帯を作れた場合だけ6ペアsource gateを再実行する。
4. source gate合格時だけ未来予測12 runへ進む。
5. Traffic Managerだけで安定分離できない場合に限り、IDM等の明示的追従制御を比較候補とする。

### 7. 公開範囲

先輩由来の`carla_simulate_project`、Dドライブ上のCSV・ログ・画像、CARLA公式コードはGitHubへ含めない。GitHubでは自作コードと進捗文書だけを管理する方針を維持する。

## English

### 1. Objective

This project studies whether vehicle-specific speed and headway characteristics can be estimated from observation history and transferred to CARLA Traffic Manager for digital-twin future prediction. This update records the speed-profile residual audit, one-pair headway calibration, and tick-level diagnosis of failed steady following.

### 2. Speed-profile experiment

- A fixed source with known slow, normal, and fast profiles was used.
- Observation-only profile classification reached 100%, with a desired-speed MAE of 0.249 km/h.
- For the seven primary actors, individual inferred conditioning improved speed MAE from 3.995 to 0.294 m/s and ADE from 18.862 to 1.364 m compared with baseline.
- The remaining fast-profile error was mainly caused by periodic Traffic Manager speed control and handoff transients.
- The small Actor 7 degradation from common to inferred was not significant relative to the three-repeat variation.

### 3. Headway source gate

- A six-pair leader/follower source was tested with requested short, normal, and long settings of 6, 12, and 20 m.
- Final realized medians were 6.26, 7.65, and 18.55 m.
- The short-normal difference was only 1.39 m, below the required 3 m.
- Because the source gate failed, the 12 baseline/common/inferred/oracle future-prediction runs were not executed.

### 4. One-pair Traffic Manager calibration

- Calibration used Town10HD_Opt road 10, section 0, lane -2.
- Leader and follower target speeds were 8 and 14 km/h, with an initial bumper gap of 27.5 m.
- Requested values of 4, 6, 8, 10, 12, 16, 20, 24, and 28 m were tested three times each, for 27 runs.
- No collisions, lane changes, or leader changes occurred.
- The steady-following success rate was 0% for every setting.
- Diagnostic terminal gaps increased from 4.51 m at the 4 m setting to 8.00 m at 8 m, but saturated near 8.71 m for settings of 10 m and above.
- Stable short, normal, and long bands could not be selected, so the six-pair rerun was not started.

### 5. Tick-level steady-failure audit

- The audit used only the saved 27-run CSV and did not rerun CARLA.
- Leader detection remained continuously valid from 11.1 to 40.0 s.
- The follower stayed on road 10 / lane -2, never entered a junction, and retained at least 6 m of lane distance.
- At the 4 m setting, the relative-speed condition passed 10.0% of detected ticks, while the rolling gap-standard-deviation condition passed 87.7%; the longest instantaneous all-condition interval was 0.2 s.
- At the 6 m setting, the corresponding rates were 12.1% and 93.1%, with the same 0.2 s maximum.
- No setting produced even one complete three-second all-condition window.
- Each run contained 92-118 throttle/brake transitions, with dominant acceleration periods of approximately 0.82-2.11 s.
- No run passed the sensitivity grid, even with a 0.50 m/s relative-speed threshold, a 1.50 m gap-standard-deviation threshold, and a three-second duration.
- No outcome-changing bug was found in the steady detector. Periodic Traffic Manager relative-speed oscillation is the direct cause.

### 6. Next steps

1. Remove the road-length constraint using a longer lane.
2. Compare fixed deltas of 0.1 and 0.05 s to isolate synchronization-step effects.
3. Rerun the six-pair source gate only if three stable headway bands are obtained.
4. Run the 12 future-prediction comparisons only after the source gate passes.
5. Consider an explicit following controller such as IDM only if Traffic Manager cannot produce stable separated bands.

### 7. Publication boundary

The senior research reference project, generated D-drive data, and official CARLA examples are excluded from GitHub. The repository continues to track only author-maintained code and progress documentation.
