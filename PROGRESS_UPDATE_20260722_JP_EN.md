# CARLA Research Progress Update / CARLA研究進捗

Date / 日付: 2026-07-22

## 日本語

### 1. 今回の目的

Traffic Managerでは安定したshort / normal / longの3種類の車間特性を生成できなかったため、IDM（Intelligent Driver Model）による1ペア追従制御を実装し、基礎校正を行った。本更新は、Traffic Manager長道路最終検証とIDM 1ペア実験の確定結果を記録する。

### 2. Traffic Manager長道路最終検証

- Town04_Optの`long_route_1045`を使用した。road / section / laneは45 / 0 / -4、経路長は503.772 mで、junction・信号・分岐はない。
- fixed delta 0.05秒、Leader/Followerは`vehicle.tesla.model3`、seed/TM seedは42とした。
- 12/18 km/h条件では4 mと8 m設定だけが各3/3でsteadyに達した。
- 16 mと24 mは各0/3で、実現gap約9.25 mへ飽和した。
- 16/24 mの相対速度標準偏差は約1.655 m/s、制御切替率は約2.467回/秒であった。
- 3種類の安定車間を生成できないため、最終判定は`reject_traffic_manager_headway`である。

### 3. IDM 1ペア追従制御

Followerの加速度は次の標準IDM式で計算する。

```text
a = a_max [1 - (v / v0)^delta - (s_star / s)^2]
s_star = s0 + max(0, v T + v delta_v / (2 sqrt(a_max b)))
delta_v = follower speed - leader speed
```

Leaderへ接近すると`delta_v`が正となり、希望動的車間と減速量が増える。FollowerはAutopilotを使わず、IDM加速度を次tickの目標速度へ積分し、独自速度PIDと加速度feed-forwardでthrottle/brakeへ変換する。操舵には保存経路を追従するlookahead PIDを使用した。

共通条件は次のとおりである。

- Leader: Traffic Manager Autopilot、希望速度12 km/h
- Follower: IDM、自由走行希望速度18 km/h
- 初期bumper gap: 30.0 m
- fixed delta: 0.05秒
- `s0=2.0 m`, `a_max=1.5 m/s2`, `b=2.0 m/s2`, `delta=4`
- 最大減速度: 4.0 m/s2、加速度filter alpha: 0.30
- 速度PID: Kp=3.0、Ki=0.35、Kd=0.05
- 操舵PID: Kp=1.25、Ki=0.02、Kd=0.12
- profile間で変更したパラメータは希望時間車間Tだけである。

### 4. IDM実験結果

normal T=1.5秒のsmoke test 1回と、short / normal / longを各3回、正式9 run実行した。

| profile | T [s] | steady | steady開始 [s] | 実現gap [m] | 実現THW [s] | 理論gap [m] | gap std [m] | 相対速度std [m/s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| short | 0.8 | 3/3 | 31.0 | 5.335 | 1.622 | 5.124 | 0.036 | 0.0068 |
| normal | 1.5 | 3/3 | 31.0 | 8.035 | 2.435 | 7.669 | 0.090 | 0.0151 |
| long | 2.5 | 3/3 | 30.0 | 11.991 | 3.608 | 11.305 | 0.198 | 0.0250 |

- 全runでsteady後15.05秒の評価を完了した。
- gap差はshort-normal 2.700 m、normal-long 3.956 m、short-long 6.656 mであった。
- THW差は0.813秒、1.173秒、1.987秒であった。
- Tの増加に対して実現gapと実現THWはともに単調増加した。
- collision、Leader変更、lane change、route deviation、emergency fallbackはすべて0であった。
- steady評価中のthrottle/brake切替率とIDM加速度clamp率は0であった。
- 各profileの反復差は決定論的条件下で実質0であった。

すべての事前合格条件を満たしたため、method decisionは`accept_idm_single_pair`、次段階は`observation_based_idm_parameter_estimation`とした。

### 5. Traffic Managerとの比較

Traffic Managerはsteady成功設定が2種類だけで、設定増加に対するgapの単調性も失われた。IDMは3 profileすべてでsteadyに達し、gapとTHWを単調に分離できた。IDMの相対速度標準偏差は最大0.025 m/sであり、TMの不安定な16/24 m条件の約1.655 m/sより小さい。ただし、TM設定距離[m]とIDMのT[s]は異なる物理量であり、設定値自体を直接比較していない。

### 6. 検証と次の作業

- IDM数式の単体テスト6件、Python `py_compile`、PowerShell構文、CLI `--help`を確認した。
- CARLA client/serverは0.9.16で一致した。
- 実験後はvehicle / walker / sensor = 0 / 0 / 0となった。
- Worldは非同期・可変deltaへ戻し、MapはTown10HD_Optへ復帰した。
- 次は観測履歴からT等のIDMパラメータを推定する。
- 6ペア、未来予測、速度と車間の統合、事故リスク評価はまだ実施していない。

### 7. 公開範囲

先輩由来コードを含む`carla_simulate_project`、Dドライブ上のCSV・JSON・ログ・図、CARLA公式コードはGitHubへ含めない。今回のGitHub更新では、リポジトリ管理方針に従い、この進捗文書のみを公開対象とする。

## English

### 1. Objective

Traffic Manager could not produce three stable short, normal, and long following profiles. An explicit one-pair IDM controller was therefore implemented and evaluated. This update records the final long-road Traffic Manager result and the completed IDM prototype experiment.

### 2. Final Traffic Manager validation

- The experiment used `long_route_1045` on Town04_Opt: road 45, section 0, lane -4, and 503.772 m long, with no junctions, traffic lights, or branches.
- The fixed delta was 0.05 s. Both vehicles were `vehicle.tesla.model3`, with seed and TM seed 42.
- Under the 12/18 km/h condition, only the 4 m and 8 m settings reached steady following in all three repeats.
- The 16 m and 24 m settings both failed 0/3 and saturated near the same realized gap of 9.25 m.
- Their relative-speed standard deviation was about 1.655 m/s, with about 2.467 control switches per second.
- Because three stable profiles could not be generated, the final decision was `reject_traffic_manager_headway`.

### 3. One-pair IDM controller

The follower acceleration uses the standard IDM formulation:

```text
a = a_max [1 - (v / v0)^delta - (s_star / s)^2]
s_star = s0 + max(0, v T + v delta_v / (2 sqrt(a_max b)))
delta_v = follower speed - leader speed
```

A positive `delta_v` means the follower is closing on the leader, increasing the desired dynamic gap and braking response. The follower does not use Autopilot. IDM acceleration is integrated into a next-tick target speed, then converted to throttle and brake by a custom speed PID with acceleration feed-forward. A route-lookahead PID controls steering.

Common settings were:

- Leader: Traffic Manager Autopilot at 12 km/h
- Follower: IDM with free-flow speed 18 km/h
- Initial bumper gap: 30.0 m
- Fixed delta: 0.05 s
- `s0=2.0 m`, `a_max=1.5 m/s2`, `b=2.0 m/s2`, `delta=4`
- Maximum deceleration: 4.0 m/s2; acceleration filter alpha: 0.30
- Speed PID: Kp=3.0, Ki=0.35, Kd=0.05
- Steering PID: Kp=1.25, Ki=0.02, Kd=0.12
- Only desired time headway T changed between profiles.

### 4. IDM results

One normal-profile smoke test and nine formal runs were completed, with three repeats per profile.

| profile | T [s] | steady | start [s] | realized gap [m] | realized THW [s] | theoretical gap [m] | gap std [m] | relative-speed std [m/s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| short | 0.8 | 3/3 | 31.0 | 5.335 | 1.622 | 5.124 | 0.036 | 0.0068 |
| normal | 1.5 | 3/3 | 31.0 | 8.035 | 2.435 | 7.669 | 0.090 | 0.0151 |
| long | 2.5 | 3/3 | 30.0 | 11.991 | 3.608 | 11.305 | 0.198 | 0.0250 |

- Every run completed 15.05 s of steady-state evaluation.
- Realized gap differences were 2.700, 3.956, and 6.656 m for short-normal, normal-long, and short-long.
- Realized THW differences were 0.813, 1.173, and 1.987 s.
- Both realized gap and THW increased monotonically with T.
- Collisions, leader changes, lane changes, route deviations, and emergency fallbacks were all zero.
- Steady-state throttle/brake switching and IDM acceleration clamping rates were zero.
- Repeats were effectively identical under deterministic conditions.

All predefined acceptance criteria passed. The method decision is `accept_idm_single_pair`, with `observation_based_idm_parameter_estimation` as the next step.

### 5. Traffic Manager comparison

Traffic Manager produced only two stable settings and lost monotonic gap response at larger requested distances. IDM produced three stable, monotonically separated profiles. The maximum IDM relative-speed standard deviation was 0.025 m/s, compared with approximately 1.655 m/s in the unstable TM 16/24 m settings. TM distance in metres and IDM time headway in seconds are different physical quantities and were not compared as equivalent parameter values.

### 6. Validation and next steps

- Six IDM unit tests, Python compilation, PowerShell syntax, and CLI help checks passed.
- CARLA client and server versions matched at 0.9.16.
- Final actor counts were vehicle / walker / sensor = 0 / 0 / 0.
- World settings returned to asynchronous mode with variable delta, and the map was restored to Town10HD_Opt.
- The next step is observation-based estimation of IDM parameters such as T.
- Six-pair testing, future prediction, combined speed/headway conditioning, and accident-risk evaluation have not started.

### 7. Publication boundary

The senior-code-containing `carla_simulate_project`, generated D-drive CSV/JSON/log/figure artifacts, and official CARLA code remain excluded from GitHub. Under the repository policy, only this progress document is included in the present GitHub update.
