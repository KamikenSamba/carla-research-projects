# fixed_objects.py
# -*- coding: utf-8 -*-
"""
評価用の固定オブジェクト定義
- すべて WORLD 座標で指定（Ego とは独立）
- Town10HD_Opt 前提の例。自分の環境に合わせて x,y,z,yaw を書き換えてください
"""

# オブジェクトのリスト
# kind      : "vehicle" or "walker"
# bp_filter : Blueprint のフィルタ文字列
# location  : WORLD 座標 (x,y,z)
# rotation  : 角度（省略可・必要な軸だけ指定）

FIXED_OBJECTS = [
    {
        "name": "center_car",
        "kind": "vehicle",
        "bp_filter": "vehicle.audi.tt",
        "location": {"x": 40.0, "y": 43.0, "z": 0.1},  # ★ここを自分で測った座標に変更
        "rotation": {"yaw": 90.0},                     # 進行方向
    },
    {
        "name": "side_car",
        "kind": "vehicle",
        "bp_filter": "vehicle.tesla.model3",
        "location": {"x": 50.0, "y": 66.0, "z": 0.1},
        "rotation": {"yaw": 180.0},
    },
]
