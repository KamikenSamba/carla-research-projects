# -*- coding: utf-8 -*-
"""
run_coop_comm_v1.py
Ego LiDAR + RSU(固定LiDAR) の協調認識（通信レイヤ実装付き・最小構成）
- RSU は自前の log-odds グリッドを生成 → 量子化 + zlib 圧縮して送信
- 疑似ネットワーク（遅延/損失/帯域計測）を通して Ego が受信し融合
- Free が潰れないように log-odds ベースで更新
- Ego優先 + UnknownセルのみRSUで補完
- 静的オブジェクト: static_mask.npy は現状「可視化専用レイヤ」としてのみ使用
"""

import os, time, math, zlib, csv, random
import numpy as np
from PIL import Image
import carla
import argparse
import json
import importlib.util



# ============================================================
# ここだけいじれば実験条件を変えられるパラメータゾーン
# ============================================================

# --- マスク関連 ---
# True  : static_mask.npy を読み込んで可視化に使う
# False : static_mask を全く使わない（全部通常グリッドとして扱う）
USE_STATIC_MASK = False
STATIC_MASK_PATH = "static_mask.npy"

# --- Ego 車両のスポーン設定 ---
USE_MANUAL_EGO_SPAWN = True  # 手動座標を使うかどうか

EGO_SPAWN_X   =  10.0   # 交差点手前の WORLD X
EGO_SPAWN_Y   = 70.0   # 交差点中心レーン上 WORLD Y
EGO_SPAWN_Z   =  0.2   # 路面から少し浮かせる（0.2〜0.5 くらい）
EGO_SPAWN_YAW =  0.0   # 進行方向（度）

# --- Ego の挙動 ---
EGO_AUTOPILOT = False
EGO_THROTTLE  = 0.5
EGO_STEER     = 0.0
EGO_BRAKE     = 0.0
EGO_HANDBRAKE = False
EGO_MAX_SPEED_KMH = 20.0  

# --- グリッド（WORLD座標系のローカルパッチ） ---
RES = 0.2
X_MIN, X_MAX = -30.0, 30.0   # [m] ORIGIN からの相対 X 範囲
Y_MIN, Y_MAX = -30.0, 30.0   # [m] ORIGIN からの相対 Y 範囲

# グリッド原点（WORLD 座標）を手動指定するかどうか
USE_MANUAL_GRID_ORIGIN = True   # False にすると Ego spawn を原点にする
GRID_ORIGIN_X = 42.0            # 交差点中心 WORLD X
GRID_ORIGIN_Y = 56.0            # 交差点中心 WORLD Y

# --- LiDAR 全般 ---
LIDAR_CHANNELS = 32
LIDAR_RANGE    = 500.0
LIDAR_ROT_HZ   = 10.0
LIDAR_PPS      = 200000

# Ego LiDAR マウント位置（車両座標系）
EGO_LIDAR_REL_X = 10.0
EGO_LIDAR_REL_Y = 0.0
EGO_LIDAR_REL_Z = 2.2

# RSU LiDAR の WORLD 座標＆向き
RSU_LIDAR_X = 42.00
RSU_LIDAR_Y = 40.00
RSU_LIDAR_Z = 2.0
RSU_PITCH   = 0.0
RSU_YAW     = 90.0

# --- 高さフィルタ（WORLD z） ---
# 参考文献ベースで [0.10, 2.50] m だけ使うイメージ
Z_MIN_WORLD = 0.10   # 地面付近ノイズ除去用の下限
Z_MAX_WORLD = 2.00   # 車両・歩行者などを主に見るための上限

# --- log-odds 関連 ---
L0 = 0.0
L_OCC  = float(np.log(0.8/0.2))     # ≈ +1.386
L_FREE = float(np.log(0.45/0.55))   # ≈ -0.080
L_MIN, L_MAX = -4.0, 4.0
DECAY_PER_SEC_EGO = 0.4
DECAY_PER_SEC_RSU = 0.3

# 可視化しきい値
OCC_TH  = 0.60
FREE_TH = 0.48

# RSU Free スケール（途中セル）
RSU_FREE_SCALE = 0.15

# --- 疑似通信レイヤ設定 ---
COMM_MODE      = "grid"  # "grid" 固定
LATENCY_MS     = 00      # 片道レイテンシ
DROP_RATE      = 0.00    # メッセージドロップ率
SEND_EVERY_N_TICKS = 1   # 送信周期（tick間引き）
QUANT_BITS     = 8       # 量子化ビット（int8）

# --- シミュレーション時間＆スナップショット ---
HOST, PORT = "127.0.0.1", 2000
USE_SYNC = True
FIXED_DELTA = 0.05         # 20Hz
SIM_DURATION_SEC = 6.0     # CARLA内時間でのシミュ長
MAX_TICKS = int(SIM_DURATION_SEC / FIXED_DELTA)

SNAPSHOT_INTERVAL_SEC = 0.1  # 0.1秒ごとにグリッドと統計を保存
SNAPSHOT_EVERY_N_TICKS = max(1, int(round(SNAPSHOT_INTERVAL_SEC / FIXED_DELTA)))

# --- 出力 ---
OUT_DIR = "out_grids"
RUN_TAG = time.strftime("%Y%m%d_%H%M%S")

# --- 固定オブジェクト（別ファイル fixed_objects.py） ---
USE_FIXED_OBJECTS = False   # ← ここで ON/OFF 切替

# ============================================================
# ここから下はあまり触らない領域（ロジック本体）
# ============================================================

# fixed_objects.py から定義を読む（無ければ空リスト）
try:
    from fixed_objects import FIXED_OBJECTS
except ImportError:
    FIXED_OBJECTS = []
    print("[INFO] fixed_objects.py not found. No fixed objects will be spawned.")

# ===== シナリオ（外部JSON） =====
USE_SCENARIO_ACTORS = False
SCENARIO_ACTORS_FILE = None

def load_scenarios_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def apply_scenario_config(cfg: dict):
    """scenarios.json の 1シナリオ分をグローバル設定へ上書き適用する（指定された項目のみ）。"""
    global USE_STATIC_MASK, USE_FIXED_OBJECTS
    global USE_MANUAL_EGO_SPAWN, EGO_SPAWN_X, EGO_SPAWN_Y, EGO_SPAWN_Z, EGO_SPAWN_YAW
    global USE_MANUAL_GRID_ORIGIN, GRID_ORIGIN_X, GRID_ORIGIN_Y
    global RSU_LIDAR_X, RSU_LIDAR_Y, RSU_LIDAR_Z, RSU_PITCH, RSU_YAW
    global USE_SCENARIO_ACTORS, SCENARIO_ACTORS_FILE

    if "use_static_mask" in cfg:
        USE_STATIC_MASK = bool(cfg["use_static_mask"])
    if "use_fixed_objects" in cfg:
        USE_FIXED_OBJECTS = bool(cfg["use_fixed_objects"])

    ego = cfg.get("ego_spawn")
    if ego:
        USE_MANUAL_EGO_SPAWN = True
        EGO_SPAWN_X = float(ego.get("x", EGO_SPAWN_X))
        EGO_SPAWN_Y = float(ego.get("y", EGO_SPAWN_Y))
        EGO_SPAWN_Z = float(ego.get("z", EGO_SPAWN_Z))
        EGO_SPAWN_YAW = float(ego.get("yaw", EGO_SPAWN_YAW))

    go = cfg.get("grid_origin")
    if go:
        USE_MANUAL_GRID_ORIGIN = bool(go.get("use_manual", USE_MANUAL_GRID_ORIGIN))
        GRID_ORIGIN_X = float(go.get("x", GRID_ORIGIN_X))
        GRID_ORIGIN_Y = float(go.get("y", GRID_ORIGIN_Y))

    rsu = cfg.get("rsu_lidar")
    if rsu:
        RSU_LIDAR_X = float(rsu.get("x", RSU_LIDAR_X))
        RSU_LIDAR_Y = float(rsu.get("y", RSU_LIDAR_Y))
        RSU_LIDAR_Z = float(rsu.get("z", RSU_LIDAR_Z))
        RSU_PITCH = float(rsu.get("pitch", RSU_PITCH))
        RSU_YAW = float(rsu.get("yaw", RSU_YAW))

    # 外部アクターファイル
    if "actors_file" in cfg:
        SCENARIO_ACTORS_FILE = str(cfg["actors_file"]) if cfg["actors_file"] else None
    if "use_scenario_actors" in cfg:
        USE_SCENARIO_ACTORS = bool(cfg["use_scenario_actors"])


def load_scenario_actors(actors_file: str):
    """scenario_B_actors.py のようなファイルから SCENARIO_ACTORS(list) を読み込む。"""
    if not actors_file:
        return []
    if not os.path.exists(actors_file):
        print(f"[SC_ACTORS][WARN] file not found: {actors_file}")
        return []
    spec = importlib.util.spec_from_file_location("scenario_actors_module", actors_file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore
    if not hasattr(mod, "SCENARIO_ACTORS"):
        print(f"[SC_ACTORS][WARN] SCENARIO_ACTORS not defined in {actors_file}")
        return []
    data = getattr(mod, "SCENARIO_ACTORS")
    if not isinstance(data, list):
        print(f"[SC_ACTORS][WARN] SCENARIO_ACTORS must be list in {actors_file}")
        return []
    return data


def spawn_actor_from_def(world, bp_lib, ego_vehicle, d: dict, actors_list):
    """
    定義dictからアクターをスポーンする（Ego相対/絶対）。
    追加要件:
      - control 内で Ego と同様の項目を設定できる:
        autopilot, throttle, steer, brake, handbrake, max_speed_kmh
      - 返り値は (actor, extra) で、extra は速度上限などの付帯情報
    """
    kind = d.get("kind", "vehicle")
    bp_filter = d.get("bp_filter", "vehicle.*")
    bps = bp_lib.filter(bp_filter)
    if not bps:
        print(f"[SC_ACTORS][WARN] no blueprint for '{bp_filter}', skip {d.get('name')}")
        return None, (None, None)
    bp = random.choice(bps)

    # 色指定（vehicle系のみ効くことが多い）
    color = d.get("color")
    if color and bp.has_attribute("color"):
        bp.set_attribute("color", str(color))

    ego_tf = ego_vehicle.get_transform()
    spawn_mode = d.get("spawn_mode", "relative_to_ego")

    if spawn_mode == "relative_to_ego":
        offset_m = float(d.get("offset_m", 10.0))
        lateral_m = float(d.get("lateral_m", 0.0))
        z = float(d.get("z", ego_tf.location.z))
        yaw_add = float(d.get("yaw_add_deg", 0.0))

        yaw_rad = math.radians(ego_tf.rotation.yaw)
        fx, fy = math.cos(yaw_rad), math.sin(yaw_rad)      # forward
        rx, ry = -math.sin(yaw_rad), math.cos(yaw_rad)     # right

        loc = carla.Location(
            x=ego_tf.location.x + fx * offset_m + rx * lateral_m,
            y=ego_tf.location.y + fy * offset_m + ry * lateral_m,
            z=z
        )
        rot = carla.Rotation(yaw=ego_tf.rotation.yaw + yaw_add)
        tf = carla.Transform(loc, rot)
    else:
        loc_cfg = d.get("location", {})
        rot_cfg = d.get("rotation", {})
        tf = carla.Transform(
            carla.Location(
                x=float(loc_cfg.get("x", ego_tf.location.x + 10.0)),
                y=float(loc_cfg.get("y", ego_tf.location.y)),
                z=float(loc_cfg.get("z", ego_tf.location.z)),
            ),
            carla.Rotation(
                pitch=float(rot_cfg.get("pitch", 0.0)),
                yaw=float(rot_cfg.get("yaw", ego_tf.rotation.yaw)),
                roll=float(rot_cfg.get("roll", 0.0)),
            ),
        )

    actor = world.try_spawn_actor(bp, tf)
    if actor is None:
        print(f"[SC_ACTORS][WARN] spawn failed: {d.get('name')} at {tf.location}")
        return None, (None, None)

    actors_list.append(actor)
    print(f"[SC_ACTORS] spawned {kind} '{d.get('name')}' at ({tf.location.x:.2f},{tf.location.y:.2f},{tf.location.z:.2f}) yaw={tf.rotation.yaw:.1f}")

    # 付帯情報（main 側で処理）
    extra_mode, extra_val = None, None

    # ---- control（新方式: Egoと同じ項目で指定） ----
    ctrl = d.get("control", {}) or {}

    if kind == "vehicle":
        # 新仕様: autopilot を優先
        if "autopilot" in ctrl:
            ap = bool(ctrl.get("autopilot", False))
            actor.set_autopilot(ap)

            if not ap:
                actor.apply_control(carla.VehicleControl(
                    throttle=float(ctrl.get("throttle", 0.0)),
                    steer=float(ctrl.get("steer", 0.0)),
                    brake=float(ctrl.get("brake", 0.0)),
                    hand_brake=bool(ctrl.get("handbrake", False)),
                ))

            # 速度上限（tickごとにクリップする）
            if "max_speed_kmh" in ctrl and ctrl.get("max_speed_kmh") is not None:
                try:
                    extra_mode, extra_val = "speed_cap", float(ctrl["max_speed_kmh"]) / 3.6
                except Exception:
                    pass

        else:
            # 互換: 旧仕様（mode: autopilot/stop/straight）
            mode = ctrl.get("mode", "autopilot")
            if mode == "autopilot":
                actor.set_autopilot(True)
            elif mode == "stop":
                actor.set_autopilot(False)
                actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            elif mode == "straight":
                actor.set_autopilot(False)
                actor.apply_control(carla.VehicleControl(
                    throttle=float(ctrl.get("throttle", 0.35)),
                    steer=float(ctrl.get("steer", 0.0)),
                    brake=float(ctrl.get("brake", 0.0)),
                    hand_brake=bool(ctrl.get("handbrake", False)),
                ))
                # straight でも上限指定があれば適用
                if "max_speed_kmh" in ctrl and ctrl.get("max_speed_kmh") is not None:
                    try:
                        extra_mode, extra_val = "speed_cap", float(ctrl["max_speed_kmh"]) / 3.6
                    except Exception:
                        pass
            else:
                actor.set_autopilot(False)

    return actor, (extra_mode, extra_val)

def parse_args():
    p = argparse.ArgumentParser(add_help=True)
    p.add_argument("--scenario-file", type=str, default="scenarios.json")
    p.add_argument("--scenario", type=str, default=None)
    p.add_argument("--list-scenarios", action="store_true")
    # 既存運用で余計な引数が混ざっても落ちないようにする
    args, _unknown = p.parse_known_args()
    return args

# WORLD グリッド原点（main 内で設定）
ORIGIN_X = 0.0
ORIGIN_Y = 0.0

# ===== グリッドサイズ =====
nx = int(np.ceil((X_MAX - X_MIN) / RES))
ny = int(np.ceil((Y_MAX - Y_MIN) / RES))


def world_to_grid(xw: float, yw: float):
    """
    WORLD 座標 (xw, yw) → グリッドインデックス (ix, iy)
    ORIGIN_X, ORIGIN_Y からの相対位置を X_MIN〜X_MAX, Y_MIN〜Y_MAX にマッピング
    """
    xr = xw - ORIGIN_X
    yr = yw - ORIGIN_Y
    ix = int((xr - X_MIN) / RES)
    iy = int((yr - Y_MIN) / RES)
    return ix, iy


def in_bounds(ix: int, iy: int) -> bool:
    return 0 <= ix < nx and 0 <= iy < ny


# ===== 静的マスクのロード =====
def load_static_mask():
    """static_mask.npy を読み込む。USE_STATIC_MASK=False の場合は全Falseを返す。"""
    if not USE_STATIC_MASK:
        print("[INFO] USE_STATIC_MASK = False -> static_mask is disabled.")
        return np.zeros((ny, nx), dtype=bool)

    if os.path.exists(STATIC_MASK_PATH):
        sm = np.load(STATIC_MASK_PATH)
        if sm.shape != (ny, nx):
            print("[WARN] static_mask shape mismatch. ignore & use empty mask.")
            return np.zeros((ny, nx), dtype=bool)
        print(f"[INFO] loaded static_mask from {STATIC_MASK_PATH}, shape={sm.shape}")
        return sm.astype(bool)
    else:
        print("[INFO] static_mask not found. use empty mask.")
        return np.zeros((ny, nx), dtype=bool)

# 初期ロード（シナリオ適用後に main() でも再ロードする）
static_mask = load_static_mask()


# ===== Bresenham =====
def bresenham(ix0, iy0, ix1, iy1):
    dx = abs(ix1 - ix0); sx = 1 if ix0 < ix1 else -1
    dy = -abs(iy1 - iy0); sy = 1 if iy0 < iy1 else -1
    err = dx + dy
    x, y = ix0, iy0
    while True:
        yield x, y
        if x == ix1 and y == iy1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


# ===== 減衰 =====
def decay_logodds(arr, dt, rate):
    """log-odds を L0 に向かって時間減衰させる"""
    if rate <= 0 or dt <= 0:
        return
    arr += (L0 - arr) * (rate * dt)
    np.clip(arr, L_MIN, L_MAX, out=arr)


# ===== 占有更新（WORLD座標ベース・任意原点） =====
def update_from_points(points_xyz_world: np.ndarray,
                       origin_xy_world,
                       target_logodds: np.ndarray,
                       free_scale: float = 1.0):
    """
    points_xyz_world : shape (N,3), WORLD座標 (xw,yw,zw)
    origin_xy_world  : LiDAR の WORLD 位置 (ox, oy)
    target_logodds   : 更新対象の log-odds グリッド
    free_scale       : Free 更新のスケール（RSU用に <1 を指定）
    """
    ox, oy = origin_xy_world
    ix0, iy0 = world_to_grid(ox, oy)
    if not in_bounds(ix0, iy0):
        return

    for xw, yw, zw in points_xyz_world:
        dx = xw - ox
        dy = yw - oy
        if dx * dx + dy * dy > LIDAR_RANGE * LIDAR_RANGE:
            continue

        ix1, iy1 = world_to_grid(xw, yw)
        if not in_bounds(ix1, iy1):
            continue

        for cx, cy in bresenham(ix0, iy0, ix1, iy1):
            if cx == ix1 and cy == iy1:
                target_logodds[cy, cx] = np.clip(
                    target_logodds[cy, cx] + L_OCC, L_MIN, L_MAX
                )
            else:
                target_logodds[cy, cx] = np.clip(
                    target_logodds[cy, cx] + free_scale * L_FREE, L_MIN, L_MAX
                )


def update_from_points_with_origin(points_xyz_world: np.ndarray,
                                   origin_xy_world,
                                   target_logodds: np.ndarray,
                                   free_scale: float = 1.0,
                                   static_mask_arr=None):
    """互換用ラッパ（将来 static_mask を更新に使う場合に備えて残している）"""
    update_from_points(points_xyz_world, origin_xy_world, target_logodds, free_scale=free_scale)


# ===== 配色 =====
COLOR_EGO_OCC  = (255, 80, 80)
COLOR_EGO_FREE = (0, 220, 255)
COLOR_RSU_OCC  = (255, 0, 255)
COLOR_RSU_FREE = (160, 255, 120)
COLOR_UNKNOWN  = (10, 10, 10)
COLOR_STATIC   = (180, 180, 255)


# ==== グリッド描画 ====
def render_grid(path_png,
                ego_logodds: np.ndarray | None = None,
                rsu_logodds: np.ndarray | None = None,
                mode: str = "ego"):
    """
    mode:
      - "ego"   : Ego単体（赤/シアン + Unknown黒 + 静的色）
      - "rsu"   : RSU単体（オレンジ/黄緑 + Unknown黒 + 静的色）
      - "fused" : Ego/RSU両方（4色） + 静的色 + Unknown黒
    """
    if ego_logodds is not None:
        p_ego = 1.0 / (1.0 + np.exp(-ego_logodds))
        ego_occ  = p_ego >= OCC_TH
        ego_free = p_ego <= FREE_TH
    else:
        ego_occ = ego_free = np.zeros((ny, nx), dtype=bool)

    if rsu_logodds is not None:
        p_rsu = 1.0 / (1.0 + np.exp(-rsu_logodds))
        rsu_occ  = p_rsu >= OCC_TH
        rsu_free = p_rsu <= FREE_TH
    else:
        rsu_occ = rsu_free = np.zeros((ny, nx), dtype=bool)

    if USE_STATIC_MASK:
        static_occ = static_mask
    else:
        static_occ = np.zeros_like(static_mask, dtype=bool)

    img = np.zeros((ny, nx, 3), dtype=np.uint8)

    if mode == "ego":
        dyn_occ  = ego_occ  & (~static_occ)
        dyn_free = ego_free & (~static_occ)
        unknown_mask = ~(dyn_occ | dyn_free | static_occ)

        img[unknown_mask] = COLOR_UNKNOWN
        img[dyn_occ]      = COLOR_EGO_OCC
        img[dyn_free]     = COLOR_EGO_FREE
        img[static_occ]   = COLOR_STATIC

    elif mode == "rsu":
        dyn_occ  = rsu_occ  & (~static_occ)
        dyn_free = rsu_free & (~static_occ)
        unknown_mask = ~(dyn_occ | dyn_free | static_occ)

        img[unknown_mask] = COLOR_UNKNOWN
        img[dyn_occ]      = COLOR_RSU_OCC
        img[dyn_free]     = COLOR_RSU_FREE
        img[static_occ]   = COLOR_STATIC

    elif mode == "fused":
        img[:] = COLOR_UNKNOWN
        img[static_occ] = COLOR_STATIC

        rsu_dyn_occ  = rsu_occ  & (~static_occ)
        rsu_dyn_free = rsu_free & (~static_occ)
        img[rsu_dyn_free] = COLOR_RSU_FREE
        img[rsu_dyn_occ]  = COLOR_RSU_OCC

        ego_dyn_occ  = ego_occ  & (~static_occ)
        ego_dyn_free = ego_free & (~static_occ)
        img[ego_dyn_free] = COLOR_EGO_FREE
        img[ego_dyn_occ]  = COLOR_EGO_OCC

    else:
        img[:] = COLOR_UNKNOWN

    Image.fromarray(img).save(path_png)
    print(f"[SAVE] {path_png} ({img.shape[1]}x{img.shape[0]})")


# ===== 座標変換 =====
def transform_to_world(points_xyz, transform: carla.Transform):
    mat = np.array(transform.get_matrix())
    ones = np.ones((points_xyz.shape[0], 1), dtype=np.float32)
    pts_h = np.hstack((points_xyz.astype(np.float32), ones))
    pts_w = pts_h @ mat.T
    return pts_w[:, :3]


def transform_world_to_ego(points_world, ego_transform: carla.Transform):
    mat_ego_w = np.array(ego_transform.get_matrix())
    mat_w_ego = np.linalg.inv(mat_ego_w)
    ones = np.ones((points_world.shape[0], 1), dtype=np.float32)
    pts_h = np.hstack((points_world.astype(np.float32), ones))
    pts_e = pts_h @ mat_w_ego.T
    return pts_e[:, :3]


# ===== Ego優先 + UnknownセルのみRSUで補完 =====
def fuse_logodds_prefer_ego(ego, rsu):
    p_ego = 1.0 / (1.0 + np.exp(-ego))
    p_rsu = 1.0 / (1.0 + np.exp(-rsu))

    ego_occ  = p_ego >= OCC_TH
    ego_free = p_ego <= FREE_TH
    ego_known = ego_occ | ego_free

    rsu_occ  = p_rsu >= OCC_TH
    rsu_free = p_rsu <= FREE_TH
    rsu_known = rsu_occ | rsu_free

    fused = np.full_like(ego, L0, dtype=np.float32)
    fused[ego_known] = ego[ego_known]

    use_rsu = (~ego_known) & rsu_known
    fused[use_rsu] = rsu[use_rsu]
    return fused


# ★ ラベル数カウント用ヘルパ
def compute_label_counts(ego_logodds: np.ndarray,
                         rsu_logodds: np.ndarray,
                         static_mask_arr: np.ndarray):
    def _count_with_known(arr: np.ndarray):
        p = 1.0 / (1.0 + np.exp(-arr))
        occ  = p >= OCC_TH
        free = p <= FREE_TH
        unk  = ~(occ | free)
        known = occ | free
        return occ, free, unk, known

    ego_occ, ego_free, ego_unk, ego_known = _count_with_known(ego_logodds)
    rsu_occ, rsu_free, rsu_unk, rsu_known = _count_with_known(rsu_logodds)

    fused = fuse_logodds_prefer_ego(ego_logodds, rsu_logodds)
    fused_occ, fused_free, fused_unk, _ = _count_with_known(fused)

    static_true = int(static_mask_arr.sum())
    total_cells = int(ego_logodds.size)

    rsu_known_cells = int(rsu_known.sum())
    dynamic_known = rsu_known & (~static_mask_arr)
    rsu_dynamic_known_cells = int(dynamic_known.sum())

    return {
        "ego_occ": ego_occ.sum(),
        "ego_free": ego_free.sum(),
        "ego_unknown": ego_unk.sum(),
        "rsu_occ": rsu_occ.sum(),
        "rsu_free": rsu_free.sum(),
        "rsu_unknown": rsu_unk.sum(),
        "fused_occ": fused_occ.sum(),
        "fused_free": fused_free.sum(),
        "fused_unknown": fused_unk.sum(),
        "rsu_known_cells": rsu_known_cells,
        "rsu_dynamic_known_cells": rsu_dynamic_known_cells,
        "static_true": static_true,
        "total_cells": total_cells,
    }


# ===== 疑似通信レイヤ =====
class SimChannel:
    def __init__(self, latency_ms=80, drop_rate=0.05):
        self.lat_ms = latency_ms
        self.drop = drop_rate
        self.queue = []  # (deliver_time, bytes)
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.sent = 0
        self.recv = 0

    def send(self, payload: bytes):
        if np.random.rand() < self.drop:
            self.sent += 1
            self.tx_bytes += len(payload)
            return
        t = time.time() + self.lat_ms / 1000.0
        self.queue.append((t, payload))
        self.sent += 1
        self.tx_bytes += len(payload)

    def poll(self):
        now = time.time()
        ready = [it for it in self.queue if it[0] <= now]
        self.queue = [it for it in self.queue if it[0] > now]
        out = [p for _, p in ready]
        self.recv += len(out)
        self.rx_bytes += sum(len(p) for p in out)
        return out


# エンコード/デコード（量子化した log-odds グリッドを送る）
def encode_grid_q8(logodds_arr: np.ndarray, qbits=8) -> bytes:
    arr = np.clip(logodds_arr, L_MIN, L_MAX)
    q = ((arr - L_MIN) / (L_MAX - L_MIN) * 255.0).astype(np.uint8)
    raw = q.tobytes()
    return zlib.compress(raw, level=6)


def decode_grid_q8(payload: bytes) -> np.ndarray:
    q = np.frombuffer(zlib.decompress(payload), dtype=np.uint8).reshape(ny, nx)
    arr = (q.astype(np.float32) / 255.0) * (L_MAX - L_MIN) + L_MIN
    return arr


# ===== lpy / PLY 書き出し =====
def write_lpy(path: str, points: np.ndarray):
    """
    CloudCompare で実環境の向きに近づけるため、
    出力時に Y 座標だけ符号反転して ASCII PLY 形式で出力する。
    """
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"points shape must be (N,3+) but got {points.shape}")
    pts = points[:, :3]
    n = pts.shape[0]

    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in pts:
            f.write(f"{x:.6f} {-y:.6f} {z:.6f}\n")

    print(f"[SAVE] {path} (PLY ascii, {n} points, Y flipped)")


# ===== メイン =====
def main():
    global ORIGIN_X, ORIGIN_Y

    args = parse_args()
    # --- シナリオ適用（任意） ---
    if args.scenario_file and os.path.exists(args.scenario_file):
        scenarios = load_scenarios_json(args.scenario_file)
        if args.list_scenarios:
            print("[SCENARIO] available:")
            for k in scenarios.keys():
                print(" -", k)
            return
        if args.scenario is not None:
            if args.scenario not in scenarios:
                raise ValueError(f"Scenario '{args.scenario}' not found in {args.scenario_file}. available={list(scenarios.keys())}")
            apply_scenario_config(scenarios[args.scenario])
            print(f"[SCENARIO] file={args.scenario_file} name={args.scenario}")
    else:
        if args.list_scenarios:
            print(f"[SCENARIO] scenario file not found: {args.scenario_file}")
            return

    # シナリオ適用後に static_mask を再ロード
    global static_mask
    static_mask = load_static_mask()


    os.makedirs(OUT_DIR, exist_ok=True)
    run_dir = os.path.join(OUT_DIR, RUN_TAG)
    os.makedirs(run_dir, exist_ok=True)

    ego_dir   = os.path.join(run_dir, "ego")
    rsu_dir   = os.path.join(run_dir, "rsu")
    fused_dir = os.path.join(run_dir, "fused")
    for d in (ego_dir, rsu_dir, fused_dir):
        os.makedirs(d, exist_ok=True)

    csv_path = os.path.join(run_dir, "metrics_comm.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time", "tx_bytes", "rx_bytes", "queue_len"])

    grid_csv_path = os.path.join(run_dir, "metrics_grid.csv")
    with open(grid_csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "sim_time", "tick", "total_cells",
            "ego_occ", "ego_free", "ego_unknown",
            "rsu_occ", "rsu_free", "rsu_unknown",
            "rsu_known_cells", "rsu_dynamic_known_cells",
            "fused_occ", "fused_free", "fused_unknown",
            "static_true",
        ])

    logodds_ego       = np.full((ny, nx), L0, dtype=np.float32)
    logodds_rsu_local = np.full((ny, nx), L0, dtype=np.float32)
    logodds_rsu_recv  = np.full((ny, nx), L0, dtype=np.float32)

    ego_points_all = []
    rsu_points_all = []

    ch = SimChannel(latency_ms=LATENCY_MS, drop_rate=DROP_RATE)

    client = carla.Client(HOST, PORT)
    client.set_timeout(90.0)
    world = client.get_world()
    bp = world.get_blueprint_library()

    orig_settings = world.get_settings()
    tm = None
    if USE_SYNC:
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(s)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

    actors, sensors = [], []
    speed_cap_actors = []  # (actor, vmax_mps)

    try:
        # === Ego スポーン ===
        veh_bp = bp.find('vehicle.tesla.model3')
        veh_bp.set_attribute('role_name', 'ego')

        if USE_MANUAL_EGO_SPAWN:
            spawn = carla.Transform(
                carla.Location(
                    x=EGO_SPAWN_X,
                    y=EGO_SPAWN_Y,
                    z=EGO_SPAWN_Z,
                ),
                carla.Rotation(
                    pitch=0.0,
                    yaw=EGO_SPAWN_YAW,
                    roll=0.0,
                ),
            )
        else:
            spawn_points = world.get_map().get_spawn_points()
            spawn = spawn_points[1]

        vehicle = world.try_spawn_actor(veh_bp, spawn)
        if vehicle is None:
            print(f"[ERROR] Failed to spawn ego at {spawn.location}. "
                  f"EGO_SPAWN_X/Y/Z を少し変えてください。")
            return
        

        actors.append(vehicle)

        # === シナリオ定義アクター（外部ファイル） ===
        if USE_SCENARIO_ACTORS and SCENARIO_ACTORS_FILE:
            scenario_defs = load_scenario_actors(SCENARIO_ACTORS_FILE)
            print(f"[SC_ACTORS] loaded {len(scenario_defs)} actors from {SCENARIO_ACTORS_FILE}")
            for d in scenario_defs:
                a, extra = spawn_actor_from_def(world, bp, vehicle, d, actors)
                if a is None:
                    continue
                mode, val = extra
                if mode == "speed_cap" and val is not None:
                    speed_cap_actors.append((a, float(val)))
        else:
            print(f"[SC_ACTORS] disabled. USE_SCENARIO_ACTORS={USE_SCENARIO_ACTORS}, file={SCENARIO_ACTORS_FILE}")

        vehicle.set_autopilot(EGO_AUTOPILOT)
        vehicle.apply_control(carla.VehicleControl(
            throttle=EGO_THROTTLE,
            steer=EGO_STEER,
            brake=EGO_BRAKE,
            hand_brake=EGO_HANDBRAKE,
        ))

        # === グリッド原点設定 ===
        if USE_MANUAL_GRID_ORIGIN:
            ORIGIN_X = GRID_ORIGIN_X
            ORIGIN_Y = GRID_ORIGIN_Y
            print(f"[INFO] WORLD grid origin set MANUALLY to ({ORIGIN_X:.2f}, {ORIGIN_Y:.2f})")
        else:
            ORIGIN_X = spawn.location.x
            ORIGIN_Y = spawn.location.y
            print(f"[INFO] WORLD grid origin set to Ego spawn ({ORIGIN_X:.2f}, {ORIGIN_Y:.2f})")

        # === 固定オブジェクト ===
        if USE_FIXED_OBJECTS and FIXED_OBJECTS:
            print(f"[FIXED] spawning {len(FIXED_OBJECTS)} fixed objects...")
            for obj in FIXED_OBJECTS:
                kind = obj.get("kind", "vehicle")
                bp_filter = obj.get("bp_filter", "vehicle.*")
                loc_cfg = obj.get("location", {})
                rot_cfg = obj.get("rotation", {})

                loc = carla.Location(
                    x=loc_cfg.get("x", ORIGIN_X),
                    y=loc_cfg.get("y", ORIGIN_Y),
                    z=loc_cfg.get("z", spawn.location.z),
                )
                rot = carla.Rotation(
                    pitch=rot_cfg.get("pitch", 0.0),
                    yaw=rot_cfg.get("yaw", 0.0),
                    roll=rot_cfg.get("roll", 0.0),
                )
                tf = carla.Transform(loc, rot)

                bps = bp.filter(bp_filter)
                if not bps:
                    print(f"[WARN] no blueprint for filter '{bp_filter}', skip {obj.get('name')}")
                    continue
                bp_obj = random.choice(bps)

                actor = world.try_spawn_actor(bp_obj, tf)
                if actor is None:
                    print(f"[WARN] failed to spawn fixed object '{obj.get('name')}' at {loc}")
                    continue

                actors.append(actor)

                if kind == "vehicle":
                    actor.set_autopilot(False)
                    actor.apply_control(carla.VehicleControl(
                        throttle=0.0,
                        steer=0.0,
                        brake=1.0,
                        hand_brake=True
                    ))
                # walker は棒立ち

                print(f"[FIXED] spawned {kind} '{obj.get('name')}' at {loc} (yaw={rot.yaw:.1f})")
        else:
            print("[FIXED] fixed objects disabled or empty.")

        # === Ego LiDAR ===
        lidar_bp = bp.find('sensor.lidar.ray_cast')
        for k, v in {
            'channels': LIDAR_CHANNELS,
            'range': LIDAR_RANGE,
            'rotation_frequency': LIDAR_ROT_HZ,
            'points_per_second': LIDAR_PPS,
        }.items():
            lidar_bp.set_attribute(str(k), str(v))
        lidar_ego = world.spawn_actor(
            lidar_bp,
            carla.Transform(
                carla.Location(
                    x=EGO_LIDAR_REL_X,
                    y=EGO_LIDAR_REL_Y,
                    z=EGO_LIDAR_REL_Z,
                )
            ),
            attach_to=vehicle
        )
        sensors.append(lidar_ego)

        # === RSU LiDAR（固定） ===
        lidar_bp_rsu = bp.find('sensor.lidar.ray_cast')
        for k, v in {
            'channels': LIDAR_CHANNELS,
            'range': LIDAR_RANGE,
            'rotation_frequency': LIDAR_ROT_HZ,
            'points_per_second': LIDAR_PPS,
        }.items():
            lidar_bp_rsu.set_attribute(str(k), str(v))
        rsu_tf = carla.Transform(
            carla.Location(x=RSU_LIDAR_X, y=RSU_LIDAR_Y, z=RSU_LIDAR_Z),
            carla.Rotation(pitch=RSU_PITCH, yaw=RSU_YAW)
        )
        lidar_rsu = world.spawn_actor(lidar_bp_rsu, rsu_tf)
        sensors.append(lidar_rsu)

        rsu_loc = rsu_tf.location
        ix0, iy0 = world_to_grid(rsu_loc.x, rsu_loc.y)
        print(f"[RSU] world=({rsu_loc.x:.2f}, {rsu_loc.y:.2f}) "
              f"grid=({ix0}, {iy0}) in_bounds={in_bounds(ix0, iy0)}")

        last_time_ego = time.time()
        last_time_rsu = time.time()
        tick_idx = 0
        snap_idx = 0

        # --- コールバック ---
        def on_ego(meas: carla.LidarMeasurement):
            nonlocal last_time_ego, ego_points_all
            now = time.time()
            decay_logodds(logodds_ego, now - last_time_ego, DECAY_PER_SEC_EGO)
            last_time_ego = now

            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            lidar_tf = lidar_ego.get_transform()
            ego_tf = vehicle.get_transform()

            pts_world = transform_to_world(arr, lidar_tf)
            mask_h = (pts_world[:, 2] > Z_MIN_WORLD) & (pts_world[:, 2] < Z_MAX_WORLD)
            pts_world = pts_world[mask_h]

            if pts_world.size > 0:
                pts_ego = transform_world_to_ego(pts_world, ego_tf)
                ego_points_all.append(pts_ego[:, :3].copy())

                origin_loc = lidar_tf.location
                origin_xy = (origin_loc.x, origin_loc.y)
                update_from_points(pts_world[:, :3], origin_xy, logodds_ego, free_scale=1.0)

        def on_rsu(meas: carla.LidarMeasurement):
            nonlocal rsu_points_all, last_time_rsu
            now = time.time()
            decay_logodds(logodds_rsu_local, now - last_time_rsu, DECAY_PER_SEC_RSU)
            last_time_rsu = now

            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            rsu_tf_now = lidar_rsu.get_transform()
            pts_world = transform_to_world(arr, rsu_tf_now)
            mask_h = (pts_world[:, 2] > Z_MIN_WORLD) & (pts_world[:, 2] < Z_MAX_WORLD)
            pts_world = pts_world[mask_h]

            if pts_world.size > 0:
                ego_tf = vehicle.get_transform()
                pts_ego = transform_world_to_ego(pts_world, ego_tf)
                rsu_points_all.append(pts_ego[:, :3].copy())

                rsu_loc_world = rsu_tf_now.location
                origin_xy = (rsu_loc_world.x, rsu_loc_world.y)
                update_from_points_with_origin(
                    pts_world[:, :3],
                    origin_xy,
                    logodds_rsu_local,
                    free_scale=RSU_FREE_SCALE,
                    static_mask_arr=static_mask
                )

        def safe_listen(cb):
            def _w(meas):
                try:
                    cb(meas)
                except Exception as e:
                    import traceback
                    print("[ERROR][callback]", e)
                    traceback.print_exc()
            return _w

        lidar_ego.listen(safe_listen(on_ego))
        lidar_rsu.listen(safe_listen(on_rsu))

        # --- ループ ---
        t0 = time.time()
        with open(csv_path, "a", newline="", encoding="utf-8") as f_comm:
            w_comm = csv.writer(f_comm)
            if USE_SYNC:
                while tick_idx < MAX_TICKS:
                    world.tick()
                    tick_idx += 1

                    sim_time = tick_idx * FIXED_DELTA



                    # --- シナリオ車両の速度上限（max_speed_kmh）を強制 ---
                    for _a, _vmax in speed_cap_actors:
                        try:
                            v = _a.get_velocity()
                            spd = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
                            if spd > _vmax and spd > 1e-3:
                                scale = _vmax / spd
                                _a.set_target_velocity(carla.Vector3D(v.x*scale, v.y*scale, v.z*scale))
                        except Exception:
                            pass

                    if tick_idx % SEND_EVERY_N_TICKS == 0:
                        if COMM_MODE == "grid":
                            # マスクありの場合は，静的領域の log-odds を送信前に潰して圧縮効率を上げる
                            grid_to_send = logodds_rsu_local
                            if USE_STATIC_MASK:
                                # コピーしてから静的セルだけ L0（中立値）にする
                                grid_to_send = logodds_rsu_local.copy()
                                grid_to_send[static_mask] = L0

                            payload = encode_grid_q8(grid_to_send)
                            ch.send(payload)


                    for pl in ch.poll():
                        rsu_grid = decode_grid_q8(pl)
                        logodds_rsu_recv[...] = rsu_grid

                    w_comm.writerow([f"{sim_time:.3f}", ch.tx_bytes, ch.rx_bytes, len(ch.queue)])

                    if tick_idx % SNAPSHOT_EVERY_N_TICKS == 0:
                        base = f"t{snap_idx:04d}"
                        ego_png   = os.path.join(ego_dir,   f"{base}.png")
                        rsu_png   = os.path.join(rsu_dir,   f"{base}.png")
                        fused_png = os.path.join(fused_dir, f"{base}.png")

                        render_grid(ego_png,   ego_logodds=logodds_ego,              mode="ego")
                        render_grid(rsu_png,   rsu_logodds=logodds_rsu_recv,         mode="rsu")
                        render_grid(fused_png, ego_logodds=logodds_ego,
                                                rsu_logodds=logodds_rsu_recv,
                                                mode="fused")

                        counts = compute_label_counts(logodds_ego, logodds_rsu_recv, static_mask)
                        with open(grid_csv_path, "a", newline="", encoding="utf-8") as f_grid:
                            w_grid = csv.writer(f_grid)
                            w_grid.writerow([
                                f"{sim_time:.3f}",
                                tick_idx,
                                counts["total_cells"],
                                counts["ego_occ"],
                                counts["ego_free"],
                                counts["ego_unknown"],
                                counts["rsu_occ"],
                                counts["rsu_free"],
                                counts["rsu_unknown"],
                                counts["rsu_known_cells"],
                                counts["rsu_dynamic_known_cells"],
                                counts["fused_occ"],
                                counts["fused_free"],
                                counts["fused_unknown"],
                                counts["static_true"],
                            ])
                        snap_idx += 1
            else:
                while time.time() - t0 < SIM_DURATION_SEC:
                    time.sleep(FIXED_DELTA)

        # --- センサ停止 ---
        for s in sensors:
            try:
                if getattr(s, "is_listening", False):
                    s.stop()
            except:
                pass

        # --- 最終スナップショット ---
        ego_path   = os.path.join(ego_dir,   f"final_{RUN_TAG}.png")
        rsu_path   = os.path.join(rsu_dir,   f"final_{RUN_TAG}.png")
        fused_path = os.path.join(fused_dir, f"final_{RUN_TAG}.png")

        render_grid(ego_path,   ego_logodds=logodds_ego,      mode="ego")
        render_grid(rsu_path,   rsu_logodds=logodds_rsu_recv, mode="rsu")
        render_grid(fused_path, ego_logodds=logodds_ego,
                                rsu_logodds=logodds_rsu_recv,
                                mode="fused")

        # LiDAR 全時間統合
        if len(ego_points_all) > 0:
            ego_all = np.vstack(ego_points_all)
            np.save(os.path.join(run_dir, "ego_points_agg.npy"), ego_all)
            write_lpy(os.path.join(run_dir, "ego_points_agg.ply"), ego_all)

        if len(rsu_points_all) > 0:
            rsu_all = np.vstack(rsu_points_all)
            np.save(os.path.join(run_dir, "rsu_points_agg.npy"), rsu_all)
            write_lpy(os.path.join(run_dir, "rsu_points_agg.ply"), rsu_all)

        print("[DONE] Saved:", ego_path, rsu_path, fused_path)
        print(f"[COMM] sent={ch.sent} recv={ch.recv} "
              f"tx={ch.tx_bytes/1024:.1f}KB rx={ch.rx_bytes/1024:.1f}KB "
              f"latency={LATENCY_MS}ms drop={DROP_RATE}")

        test_payload = encode_grid_q8(logodds_rsu_local)
        decoded_test = decode_grid_q8(test_payload)
        max_diff = float(np.max(np.abs(decoded_test - logodds_rsu_local)))
        mean_diff = float(np.mean(np.abs(decoded_test - logodds_rsu_local)))
        print(f"[CHECK] encode/decode max_diff={max_diff:.4f}, mean_diff={mean_diff:.4f}")

    finally:
        for s in sensors:
            try:
                if getattr(s, "is_listening", False):
                    s.stop()
            except:
                pass
            try:
                s.destroy()
            except:
                pass
        for a in actors:
            try:
                a.destroy()
            except:
                pass
        if USE_SYNC:
            try:
                tm = client.get_trafficmanager(8000)
                tm.set_synchronous_mode(False)
            except:
                pass
        world.apply_settings(orig_settings)


if __name__ == "__main__":
    main()
