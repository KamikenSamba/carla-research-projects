# -*- coding: utf-8 -*-
"""
run_coop_comm_v3.py
Ego LiDAR + RSU(固定LiDAR) の協調認識（通信レイヤ実装付き・評価用）
- RSU は自前の log-odds グリッドを生成 → 量子化 + zlib 圧縮して送信
- 疑似ネットワーク（遅延/損失/帯域計測）を通して Ego が受信し融合
- Free が潰れないように log-odds ベースで更新
- Ego優先 + UnknownセルのみRSUで補完（数値的には）
- 静的オブジェクト: static_mask.npy は現状「可視化専用レイヤ」としてのみ使用
- 評価フェーズ用:
    - CARLA内時間 SIM_DURATION_SEC 分だけ tick を回す
    - 0.1秒ごとに Ego/RSU/Fused のグリッドPNG & ラベル統計CSVを保存
    - Egoの真正面から近づいてくる車両を1台追加
"""

import os, time, math, zlib, csv, random
import numpy as np
from PIL import Image
import carla

# マスクの使用フラグ
USE_STATIC_MASK = False

# ====== 基本設定 ======
HOST, PORT = "127.0.0.1", 2000
USE_SYNC = True
FIXED_DELTA = 0.05  # 20Hz

# ★ CARLA 内のシミュレーション時間（秒）
SIM_DURATION_SEC = 15.0
# この秒数だけ進めるのに必要な tick 数
MAX_TICKS = int(SIM_DURATION_SEC / FIXED_DELTA)

# ★ 評価用スナップショット設定
SNAPSHOT_INTERVAL_SEC = 0.1  # 0.1秒ごとにグリッドと統計を保存
SNAPSHOT_EVERY_N_TICKS = max(1, int(round(SNAPSHOT_INTERVAL_SEC / FIXED_DELTA)))

# グリッド（WORLD 座標系のローカルパッチ）
RES = 0.2
X_MIN, X_MAX = -40.0, 40.0   # [m] ORIGIN からの相対 X 範囲
Y_MIN, Y_MAX = -40.0, 40.0   # [m] ORIGIN からの相対 Y 範囲

# WORLD グリッド原点（spawn 取得後に書き換える）
ORIGIN_X = 0.0
ORIGIN_Y = 0.0

# LiDAR
LIDAR_CHANNELS = 32
LIDAR_RANGE = 80.0
LIDAR_ROT_HZ = 10.0
LIDAR_PPS = 200000

# log-odds
L0 = 0.0
L_OCC  = float(np.log(0.8/0.2))     # ≈ +1.386
L_FREE = float(np.log(0.48/0.52))   # ≈ -0.080
L_MIN, L_MAX = -4.0, 4.0
DECAY_PER_SEC = 0.4

# 可視化しきい値
OCC_TH  = 0.60
FREE_TH = 0.48

# 融合方式（数値上の話。実際は Ego優先補完を使用）
FUSE_MODE = "sum"

# RSU Free スケール（途中セル）
RSU_FREE_SCALE = 0.15

# 疑似通信レイヤ設定
COMM_MODE = "grid"         # "grid"（推奨）
LATENCY_MS = 80            # 片道レイテンシ
DROP_RATE = 0.05           # メッセージドロップ率（0.0〜1.0）
SEND_EVERY_N_TICKS = 2     # 送信周期（tick間引き）
QUANT_BITS = 8             # 量子化ビット（int8）

# 出力
OUT_DIR = "out_grids"
RUN_TAG = time.strftime("%Y%m%d_%H%M%S")

# ===== グリッドサイズ =====
nx = int(np.ceil((X_MAX - X_MIN) / RES))
ny = int(np.ceil((Y_MAX - Y_MIN) / RES))


def world_to_grid(xw: float, yw: float):
    xr = xw - ORIGIN_X
    yr = yw - ORIGIN_Y
    ix = int((xr - X_MIN) / RES)
    iy = int((yr - Y_MIN) / RES)
    return ix, iy


def in_bounds(ix: int, iy: int) -> bool:
    return 0 <= ix < nx and 0 <= iy < ny


# ===== 静的マスクのロード =====
STATIC_MASK_PATH = "static_mask.npy"

if USE_STATIC_MASK and os.path.exists(STATIC_MASK_PATH):
    static_mask = np.load(STATIC_MASK_PATH)
    if static_mask.shape != (ny, nx):
        print("[WARN] static_mask shape mismatch. ignore & use empty mask.")
        static_mask = np.zeros((ny, nx), dtype=bool)
    else:
        print(f"[INFO] loaded static_mask from {STATIC_MASK_PATH}, shape={static_mask.shape}")
elif USE_STATIC_MASK:
    print("[INFO] static_mask not found. use empty mask.")
    static_mask = np.zeros((ny, nx), dtype=bool)
else:
    print("[INFO] USE_STATIC_MASK = False -> static_mask is disabled.")
    static_mask = np.zeros((ny, nx), dtype=bool)

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
def decay_logodds(arr, dt):
    if DECAY_PER_SEC <= 0 or dt <= 0:
        return
    arr += (L0 - arr) * (DECAY_PER_SEC * dt)
    np.clip(arr, L_MIN, L_MAX, out=arr)

# ===== 占有更新 =====
def update_from_points(points_xyz_world: np.ndarray,
                       origin_xy_world,
                       target_logodds: np.ndarray,
                       free_scale: float = 1.0):
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
    update_from_points(points_xyz_world, origin_xy_world, target_logodds, free_scale=free_scale)


# ===== 配色 =====
COLOR_EGO_OCC  = (255, 80, 80)
COLOR_EGO_FREE = (0, 220, 255)
COLOR_RSU_OCC  = (255, 190, 80)
COLOR_RSU_FREE = (160, 255, 120)
COLOR_UNKNOWN  = (10, 10, 10)
COLOR_STATIC   = (180, 180, 255)

# ==== グリッド描画 ====
def render_grid(path_png,
                ego_logodds: np.ndarray | None = None,
                rsu_logodds: np.ndarray | None = None,
                mode: str = "ego"):
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

# ===== Ego優先融合 =====
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

# ★ ラベル数カウント
def compute_label_counts(ego_logodds: np.ndarray,
                         rsu_logodds: np.ndarray,
                         static_mask_arr: np.ndarray):
    def _count(arr: np.ndarray):
        p = 1.0 / (1.0 + np.exp(-arr))
        occ  = p >= OCC_TH
        free = p <= FREE_TH
        unk  = ~(occ | free)
        return int(occ.sum()), int(free.sum()), int(unk.sum())

    ego_occ,  ego_free,  ego_unk  = _count(ego_logodds)
    rsu_occ,  rsu_free,  rsu_unk  = _count(rsu_logodds)
    fused = fuse_logodds_prefer_ego(ego_logodds, rsu_logodds)
    fused_occ, fused_free, fused_unk = _count(fused)

    static_true = int(static_mask_arr.sum())
    total_cells = int(ego_logodds.size)

    return {
        "ego_occ": ego_occ,
        "ego_free": ego_free,
        "ego_unknown": ego_unk,
        "rsu_occ": rsu_occ,
        "rsu_free": rsu_free,
        "rsu_unknown": rsu_unk,
        "fused_occ": fused_occ,
        "fused_free": fused_free,
        "fused_unknown": fused_unk,
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

    os.makedirs(OUT_DIR, exist_ok=True)
    run_dir = os.path.join(OUT_DIR, RUN_TAG)
    os.makedirs(run_dir, exist_ok=True)

    # サブフォルダ（スナップショット用）
    ego_dir   = os.path.join(run_dir, "ego")
    rsu_dir   = os.path.join(run_dir, "rsu")
    fused_dir = os.path.join(run_dir, "fused")
    for d in (ego_dir, rsu_dir, fused_dir):
        os.makedirs(d, exist_ok=True)

    # 通信ログ
    csv_path = os.path.join(run_dir, "metrics_comm.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["sim_time", "tx_bytes", "rx_bytes", "queue_len"])

    # グリッドラベル数ログ
    grid_csv_path = os.path.join(run_dir, "metrics_grid.csv")
    with open(grid_csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "sim_time",
            "tick",
            "total_cells",
            "ego_occ", "ego_free", "ego_unknown",
            "rsu_occ", "rsu_free", "rsu_unknown",
            "fused_occ", "fused_free", "fused_unknown",
            "static_true",
        ])

    logodds_ego = np.full((ny, nx), L0, dtype=np.float32)
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

    try:
        # ==== Ego ====
        veh_bp = bp.find('vehicle.tesla.model3')
        veh_bp.set_attribute('role_name', 'ego')
        spawn_points = world.get_map().get_spawn_points()
        spawn = spawn_points[1]
        vehicle = world.try_spawn_actor(veh_bp, spawn) or world.spawn_actor(veh_bp, spawn)
        actors.append(vehicle)
        vehicle.set_autopilot(False)
        vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            steer=0.0,
            brake=1.0,
            hand_brake=True
        ))

        ORIGIN_X = spawn.location.x
        ORIGIN_Y = spawn.location.y
        print(f"[INFO] WORLD grid origin set to ({ORIGIN_X:.2f}, {ORIGIN_Y:.2f})")

        # ==== 対向車（Ego正面の少し離れた位置に固定配置） ====
        # Egoのyawから前方ベクトルを計算
        yaw_deg = spawn.rotation.yaw
        yaw_rad = math.radians(yaw_deg)
        forward = carla.Vector3D(
            x=math.cos(yaw_rad),
            y=math.sin(yaw_rad),
            z=0.0
        )

        # Egoから見て「前方の反対側」に dist だけ離れた位置にスポーン（真正面から来るイメージ）
        dist = 35.0  # [m] 必ずLiDAR範囲内
        npc_loc = carla.Location(
            x=spawn.location.x - forward.x * dist,
            y=spawn.location.y - forward.y * dist,
            z=spawn.location.z
        )
        npc_rot = carla.Rotation(
            pitch=spawn.rotation.pitch,
            yaw=(yaw_deg + 180.0) % 360.0,
            roll=spawn.rotation.roll
        )
        npc_tf = carla.Transform(npc_loc, npc_rot)
        print(f"[NPC] spawn at {npc_loc} yaw={npc_rot.yaw:.1f} (dist={dist}m in front, opposite direction)")

        npc_bps = bp.filter('vehicle.*')
        npc_bp = random.choice(npc_bps)
        npc_bp.set_attribute('role_name', 'npc_oncoming')
        npc = world.try_spawn_actor(npc_bp, npc_tf) or world.spawn_actor(npc_bp, npc_tf)
        actors.append(npc)

        if tm is not None:
            npc.set_autopilot(True, tm.get_port())
            tm.ignore_lights_percentage(npc, 0.0)
            tm.auto_lane_change(npc, True)
        else:
            npc.set_autopilot(True)

        # ==== Ego LiDAR ====
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
            carla.Transform(carla.Location(x=0, z=2.2)),
            attach_to=vehicle
        )
        sensors.append(lidar_ego)

        # ==== RSU LiDAR ====
        lidar_bp_rsu = bp.find('sensor.lidar.ray_cast')
        for k, v in {
            'channels': LIDAR_CHANNELS,
            'range': LIDAR_RANGE,
            'rotation_frequency': LIDAR_ROT_HZ,
            'points_per_second': LIDAR_PPS,
        }.items():
            lidar_bp_rsu.set_attribute(str(k), str(v))
        rsu_tf = carla.Transform(
            carla.Location(x=-35.00, y=35.00, z=3.0),
            carla.Rotation(pitch=-8.0, yaw=-129.9)
        )
        lidar_rsu = world.spawn_actor(lidar_bp_rsu, rsu_tf)
        sensors.append(lidar_rsu)

        rsu_loc = rsu_tf.location
        ix0, iy0 = world_to_grid(rsu_loc.x, rsu_loc.y)
        print(f"[RSU] world=({rsu_loc.x:.2f}, {rsu_loc.y:.2f}) grid=({ix0}, {iy0}) in_bounds={in_bounds(ix0, iy0)}")

        last_time_ego = time.time()
        tick_idx = 0
        snap_idx = 0

        # --- コールバック ---
        def on_ego(meas: carla.LidarMeasurement):
            nonlocal last_time_ego, ego_points_all
            now = time.time()
            decay_logodds(logodds_ego, now - last_time_ego)
            last_time_ego = now

            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            lidar_tf_local = lidar_ego.get_transform()
            ego_tf = vehicle.get_transform()

            pts_world = transform_to_world(arr, lidar_tf_local)
            pts_world = pts_world[pts_world[:, 2] > 0.10]

            if pts_world.size > 0:
                pts_ego = transform_world_to_ego(pts_world, ego_tf)
                ego_points_all.append(pts_ego[:, :3].copy())

                origin_loc = lidar_tf_local.location
                origin_xy = (origin_loc.x, origin_loc.y)
                update_from_points(pts_world[:, :3], origin_xy, logodds_ego, free_scale=1.0)

        def on_rsu(meas: carla.LidarMeasurement):
            nonlocal rsu_points_all
            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            rsu_tf_now = lidar_rsu.get_transform()
            pts_world = transform_to_world(arr, rsu_tf_now)
            pts_world = pts_world[pts_world[:, 2] > 0.10]

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
        with open(csv_path, "a", newline="", encoding="utf-8") as f_comm:
            w_comm = csv.writer(f_comm)
            if USE_SYNC:
                while tick_idx < MAX_TICKS:
                    world.tick()
                    tick_idx += 1
                    sim_time = tick_idx * FIXED_DELTA

                    if tick_idx % SEND_EVERY_N_TICKS == 0:
                        if COMM_MODE == "grid":
                            payload = encode_grid_q8(logodds_rsu_local)
                            ch.send(payload)

                    for pl in ch.poll():
                        rsu_grid = decode_grid_q8(pl)
                        logodds_rsu_recv[...] = rsu_grid

                    w_comm.writerow([
                        f"{sim_time:.3f}",
                        ch.tx_bytes,
                        ch.rx_bytes,
                        len(ch.queue)
                    ])

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
                                counts["fused_occ"],
                                counts["fused_free"],
                                counts["fused_unknown"],
                                counts["static_true"],
                            ])

                        snap_idx += 1
            else:
                t0 = time.time()
                while time.time() - t0 < SIM_DURATION_SEC:
                    time.sleep(FIXED_DELTA)

        # --- 停止 ---
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

        fused_dyn = fuse_logodds_prefer_ego(logodds_ego, logodds_rsu_recv)

        render_grid(ego_path,  ego_logodds=logodds_ego, mode="ego")
        render_grid(rsu_path,  rsu_logodds=logodds_rsu_recv, mode="rsu")
        render_grid(fused_path,
                    ego_logodds=logodds_ego,
                    rsu_logodds=logodds_rsu_recv,
                    mode="fused")

        # ==== LiDAR統合点群 ====
        if len(ego_points_all) > 0:
            ego_all = np.vstack(ego_points_all)
            np.save(os.path.join(run_dir, "ego_points_agg.npy"), ego_all)
            write_lpy(os.path.join(run_dir, "ego_points_agg.ply"), ego_all)

        if len(rsu_points_all) > 0:
            rsu_all = np.vstack(rsu_points_all)
            np.save(os.path.join(run_dir, "rsu_points_agg.npy"), rsu_all)
            write_lpy(os.path.join(run_dir, "rsu_points_agg.ply"), rsu_all)

        print("[DONE] Saved final PNGs and point clouds.")
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
