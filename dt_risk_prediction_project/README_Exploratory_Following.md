# 探索的な単車安定走行・減速分離と追従試験

Issue #38。PR #37のv7探索試験を変更せず、独立した版として評価する。
正式Phase B-2はBLOCKED、route_audit/physical_boundsはnullを維持する。
サーバ起動・復旧は行わない。実行には操作者の研究専用session承認が必要。

## 単車の成立条件

Town01_Opt road8/section0/lane-1、s170→230の実測済み探索区間60m。
Model 3、dt0.05、目標1.0m/s、上限1.2m/s、throttle上限0.10。
発進は既存PIとslew0.02/s、速度0.70m/sからkp0.002/ki0.01へ
積分項を合わせて引き継ぐ。旧kp0.08/ki0.03の入力交互振動への仮説であり、
物理モデルや安定性の保証ではない。安全gateにはフィルタを使わない。

最大25秒・進行18m・全体700frame・worker120秒。
目標±0.10m/s、|縦加速度|≤0.5m/s²、|vz|<0.05m/sを2秒連続要求する。
この探索的走行窓は旧正式校正の発進5秒を変更しない。

成立後は実throttleを0.02/sで下げる。速度≤0.10m/sから4frameは
throttle=brake=0を明示的に観測し、その後brakeを0.005ずつ0.02へ増加する。
停止要求から5秒以内に既存停止条件を0.5秒連続確認する。
アクセル低下・自然減速・微小brakeの各区間を別記録する。brake前に
ほぼ停止した場合、追加制動の識別は不能とし、減速度保証値を作らない。
brake0.70を実行しない。

以前の1.0m/s目標試験の全停止profileは2.809m、4.45秒＋確認0.5秒。
今回の事前距離予算は18m＋経験値の2倍5.619m＋前端2.42511m＋終端20m
=46.045m、60mに対し約13.955mの計画余裕。ただし反復未確認の経験的予算で、
物理下限ではない。実行中は終端20m・速度・加速度gateを独立に維持する。

衝突sensor、peak5m/s²、sustained3m/s²二frame、逆走・車線・姿勢・入力・
frame整合・RPC異常時抑止・所有actor cleanup・WorldSettings復元は継承する。
失敗後の自動再試行やその場でのgate緩和は行わない。

## 2台への移行

単車の2秒整定・停止・cleanupの実測成立が必須。独立の2台プロトコルでは
同じ単一tick ownerで先行・追従を観測する。初期中心間隔20m、実bboxから
バンパーgapを算出し、gap8m未満・TTC5秒未満は中止する案を使用する。
実装はidm_pair_exploration.py。中心間隔20m、助走上限8m、整定後2秒の
探索的収集とする。距離は20＋8＋2×1.2＋2×単車停止距離＋0.3×1.2
＋実車体前端＋終端20mで計算し、60mを超える場合は拒否する。
バンパーgapから8mと停止距離予算を引いても余裕が必要。
前記18m・6秒案は距離予算が不足するため採用していない。
2台の条件・距離予算・停止余裕・mock E2Eが成立するまで実行を拒否する。
TM/Autopilotを使用せず、手動PIの追従結果と明示する。任意のIDM式から
軌跡を人工生成しない。単車データをT推定へ渡さない。

1frame内で2台へ命令してからworld.tickを1回だけ実行し、同frameの両車を
観測する。各車にcollision sensorと独立の逆走・加速度履歴を持つ。
pair_observations.csvとpair_raw.jsonlにlogical ID、actor ID、gap、相対速度、
実入力と命令参照frameを保存する。操作者宣言・cooperative lease・外側OS監視は
専用sessionの根拠であり、全外部client不在の完全な証明ではない。

実行configのtrialはfollowing、single_runに正常完了した今回版の単車runを指定する。
完了ラベルだけでなく、保存CSV/rawの2秒整定・0.5秒停止・cleanupを再検証する。
safe_early_stopや停止結果欠落の単車では、Client生成より前に拒否する。
既存v7の成功のみでは今回版の停止結果を代用できない。

2秒収集は最初の無衝突データ取得用で、T識別に十分とはしない。手動gap
controllerの見かけのIDM適合値を車両固有の不変特性と解釈しない。
T推定には既存履歴schemaへの対応、規制・fallback等の有効context、最低速度、
十分な追従時間・速度/gap変動が必要。未確認flagをFalseで埋めて有効化しない。

2026-10-29の今回版単車は2秒整定に成功したが、ゼロ入力4frameで速度が
0.0959→0.2747m/sへ再上昇し、coast_speed_reboundで安全早期終了した。
brake未実施、collision0、413frame、cleanup・設定復元完了。
この証跡はpairの実行条件を満たさない。2台コードはオフラインのみ検証し、
現在の結果を使った実走・正式校正・T推定は行わない。

CLIはRun_Exploratory_Following.py。--configだけではオフライン検証。
明示--executeで既存の単一worker監視・native identity・cooperative leaseを使う。
新規の外部証跡のみを使用し、実測データ・環境固有設定はcommitしない。
