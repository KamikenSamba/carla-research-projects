# -*- coding: utf-8 -*-
"""
run_coop_comm_v1.py
Ego LiDAR + RSU(固定LiDAR) の協調認識（通信レイヤ実装付き・最小構成）
- RSU は自前の log-odds グリッドを生成 → 量子化 + zlib 圧縮して送信
- 疑似ネットワーク（遅延/損失/帯域計測）を通して Ego が受信し融合
- Free が潰れないように log-odds ベースで更新
- Ego優先 + UnknownセルのみRSUで補完（数値的には）
- 静的オブジェクト: static_mask.npy を読み込み、
  事前既知の走行不可セルとして扱う（動的DOGMでは更新しない）
  ※ static_mask.npy は HDマップから作成する前提
"""

import os, time, math, zlib, csv
import numpy as np
from PIL import Image
import carla

# ====== 基本設定 ======
HOST, PORT = "127.0.0.1", 2000
USE_SYNC = True
FIXED_DELTA = 0.05  # 20Hz
DURATION_SEC = 15.0

# グリッド（Ego基準：x=右+, y=前+）
RES = 0.2
X_MIN, X_MAX = -30.0, 30.0
Y_MIN, Y_MAX = -10.0, 50.0

# LiDAR
LIDAR_CHANNELS = 32
LIDAR_RANGE = 80.0
LIDAR_ROT_HZ = 10.0
LIDAR_PPS = 300000

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

def world_to_grid(x, y):
    return int((x - X_MIN) / RES), int((y - Y_MIN) / RES)

def in_bounds(ix, iy):
    return 0 <= ix < nx and 0 <= iy < ny

# ===== 静的マスクのロード =====
# static_mask[y, x] == True なら
# 「事前に走行不可と分かっている静的Occupiedセル」（建物・縁石など）
STATIC_MASK_PATH = "static_mask.npy"

if os.path.exists(STATIC_MASK_PATH):
    static_mask = np.load(STATIC_MASK_PATH)
    if static_mask.shape != (ny, nx):
        print("[WARN] static_mask shape mismatch. ignore & use empty mask.")
        static_mask = np.zeros((ny, nx), dtype=bool)
    else:
        print(f"[INFO] loaded static_mask from {STATIC_MASK_PATH}, shape={static_mask.shape}")
else:
    print("[INFO] static_mask not found. use empty mask.")
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

# ===== 占有更新（Ego 原点, staticセルは更新せずビームを停止） =====
def update_from_points(points_xyz, target_logodds, static_mask_arr=None):
    ix0, iy0 = world_to_grid(0.0, 0.0)
    if not in_bounds(ix0, iy0):
        return

    for x, y, z in points_xyz:
        if x * x + y * y > LIDAR_RANGE * LIDAR_RANGE:
            continue
        if (x < X_MIN) or (x >= X_MAX) or (y < Y_MIN) or (y >= Y_MAX):
            continue

        ix1, iy1 = world_to_grid(x, y)
        if not in_bounds(ix1, iy1):
            continue

        for cx, cy in bresenham(ix0, iy0, ix1, iy1):
            # 静的セルは「事前既知の障害物」:
            #   - dynamic log-odds は更新しない
            #   - その先のセルも観測できないのでビームを止める
            """ if static_mask_arr is not None and static_mask_arr[cy, cx]:
                continue """

            if cx == ix1 and cy == iy1:
                target_logodds[cy, cx] = np.clip(
                    target_logodds[cy, cx] + L_OCC, L_MIN, L_MAX
                )
            else:
                target_logodds[cy, cx] = np.clip(
                    target_logodds[cy, cx] + L_FREE, L_MIN, L_MAX
                )

# ===== 占有更新（任意原点：RSU, staticセルは更新せずビームを停止） =====
def update_from_points_with_origin(points_xyz, origin_xy,
                                   target_logodds, free_scale=1.0,
                                   static_mask_arr=None):
    ox, oy = origin_xy
    ix0, iy0 = world_to_grid(ox, oy)
    if not in_bounds(ix0, iy0):
        return

    for x, y, z in points_xyz:
        if (x < X_MIN) or (x >= X_MAX) or (y < Y_MIN) or (y >= Y_MAX):
            continue

        ix1, iy1 = world_to_grid(x, y)
        if not in_bounds(ix1, iy1):
            continue

        for cx, cy in bresenham(ix0, iy0, ix1, iy1):
            """ if static_mask_arr is not None and static_mask_arr[cy, cx]:
                continue """

            if cx == ix1 and cy == iy1:
                target_logodds[cy, cx] = np.clip(
                    target_logodds[cy, cx] + L_OCC, L_MIN, L_MAX
                )
            else:
                target_logodds[cy, cx] = np.clip(
                    target_logodds[cy, cx] + free_scale * L_FREE, L_MIN, L_MAX
                )

# ===== 配色（統一定義） =====
COLOR_EGO_OCC  = (255, 80, 80)    # 明るい赤
COLOR_EGO_FREE = (0, 220, 255)    # 明るいシアン
COLOR_RSU_OCC  = (255, 190, 80)   # オレンジ
COLOR_RSU_FREE = (160, 255, 120)  # 黄緑
COLOR_UNKNOWN  = (10, 10, 10)     # ほぼ黒
COLOR_STATIC   = (180, 180, 255)  # HDマップ由来の静的領域

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

    # --- 動的DOGMの占有/Free マスク ---
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

    # --- 静的マスク（事前既知の走行不可セル） ---
    static_occ = static_mask  # True のところを静的色で塗る

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

        # まず静的マップを塗る（何よりも優先）
        img[static_occ] = COLOR_STATIC

        # RSU動的（静的以外）
        rsu_dyn_occ  = rsu_occ  & (~static_occ)
        rsu_dyn_free = rsu_free & (~static_occ)
        img[rsu_dyn_free] = COLOR_RSU_FREE
        img[rsu_dyn_occ]  = COLOR_RSU_OCC

        # Ego動的（静的以外）を上から重ねる（Ego優先したいので最後に描画）
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

# ===== Ego優先 + UnknownセルのみRSUで補完（動的レイヤの数値融合） =====
def fuse_logodds_prefer_ego(ego, rsu):
    """
    数値的には:
      - EgoがKnownのセル: Egoを採用
      - EgoがUnknownでRSUがKnown: RSUを採用
    静的レイヤは別に持っている前提（ここでは触らない）。
    """
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

# ===== lpy 書き出し =====
def write_lpy(path: str, points: np.ndarray):

    points = np.asarray(points, dtype=np.float32)
    
    with open(path, "w", encoding="utf-8") as f:
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    print(f"[SAVE] {path} ({points.shape[0]} points)")

# ===== メイン =====
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    run_dir = os.path.join(OUT_DIR, RUN_TAG)
    os.makedirs(run_dir, exist_ok=True)

    # ログ
    csv_path = os.path.join(run_dir, "metrics_comm.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time", "tx_bytes", "rx_bytes", "queue_len"])

    # グリッド（動的レイヤ）
    logodds_ego = np.full((ny, nx), L0, dtype=np.float32)
    logodds_rsu_local = np.full((ny, nx), L0, dtype=np.float32)
    logodds_rsu_recv  = np.full((ny, nx), L0, dtype=np.float32)

    # LiDAR生データ蓄積用（全時間統合：Ego座標系）
    ego_points_all = []  # list of (Ni,3)
    rsu_points_all = []  # list of (Ni,3)

    # 疑似チャネル
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
        # Ego
        veh_bp = bp.find('vehicle.tesla.model3')
        veh_bp.set_attribute('role_name', 'ego')
        spawn = world.get_map().get_spawn_points()[1]
        vehicle = world.try_spawn_actor(veh_bp, spawn) or world.spawn_actor(veh_bp, spawn)
        actors.append(vehicle)
        vehicle.set_autopilot(True)

        # Ego LiDAR
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

        # RSU LiDAR（固定）
        lidar_bp_rsu = bp.find('sensor.lidar.ray_cast')
        for k, v in {
            'channels': LIDAR_CHANNELS,
            'range': LIDAR_RANGE,
            'rotation_frequency': LIDAR_ROT_HZ,
            'points_per_second': LIDAR_PPS,
        }.items():
            lidar_bp_rsu.set_attribute(str(k), str(v))
        rsu_tf = carla.Transform(
            carla.Location(x=spawn.location.x + 25.0, y=spawn.location.y, z=6.0),
            carla.Rotation(pitch=-8.0, yaw=spawn.rotation.yaw + 180.0)
        )
        lidar_rsu = world.spawn_actor(lidar_bp_rsu, rsu_tf)
        sensors.append(lidar_rsu)

        last_time_ego = time.time()
        tick_idx = 0

        # --- コールバック ---
        def on_ego(meas: carla.LidarMeasurement):
            nonlocal last_time_ego, ego_points_all
            now = time.time()
            decay_logodds(logodds_ego, now - last_time_ego)
            last_time_ego = now

            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            ego_tf = vehicle.get_transform()
            pts_world = transform_to_world(arr, lidar_ego.get_transform())
            pts_ego = transform_world_to_ego(pts_world, ego_tf)

            # z > 0.10 の点のみ使用（グリッド更新と同じ条件）
            pts_ego = pts_ego[pts_ego[:, 2] > 0.10]

            # 全時間統合用に蓄積（Ego座標系）
            if pts_ego.size > 0:
                ego_points_all.append(pts_ego[:, :3].copy())

            update_from_points(pts_ego, logodds_ego, static_mask_arr=static_mask)

        def on_rsu(meas: carla.LidarMeasurement):
            nonlocal rsu_points_all

            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            rsu_tf_now = lidar_rsu.get_transform()
            pts_world = transform_to_world(arr, rsu_tf_now)
            ego_tf = vehicle.get_transform()
            pts_ego = transform_world_to_ego(pts_world, ego_tf)

            # z > 0.10 の点のみ使用
            pts_ego = pts_ego[pts_ego[:, 2] > 0.10]

            # 全時間統合用に蓄積（Ego座標系）
            if pts_ego.size > 0:
                rsu_points_all.append(pts_ego[:, :3].copy())

            rsu_loc_world = rsu_tf_now.location
            rsu_origin_world = np.array(
                [[rsu_loc_world.x, rsu_loc_world.y, rsu_loc_world.z]],
                dtype=np.float32
            )
            rsu_origin_ego = transform_world_to_ego(rsu_origin_world, ego_tf)[0, :2]

            update_from_points_with_origin(
                pts_ego, rsu_origin_ego,
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
        with open(csv_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if USE_SYNC:
                while time.time() - t0 < DURATION_SEC:
                    world.tick()
                    tick_idx += 1

                    # RSU → 送信（間引き）
                    if tick_idx % SEND_EVERY_N_TICKS == 0:
                        if COMM_MODE == "grid":
                            payload = encode_grid_q8(logodds_rsu_local)
                            ch.send(payload)

                    # 受信 → 反映（動的レイヤのみ受信）
                    for pl in ch.poll():
                        rsu_grid = decode_grid_q8(pl)
                        logodds_rsu_recv[...] = rsu_grid

                    w.writerow([time.time() - t0, ch.tx_bytes, ch.rx_bytes, len(ch.queue)])
            else:
                while time.time() - t0 < DURATION_SEC:
                    time.sleep(FIXED_DELTA)

        # --- 停止 ---
        for s in sensors:
            try:
                if getattr(s, "is_listening", False):
                    s.stop()
            except:
                pass

        # --- 出力 ---
        ego_path   = os.path.join(run_dir, f"ego_{RUN_TAG}.png")
        rsu_path   = os.path.join(run_dir, f"rsu_{RUN_TAG}.png")
        fused_path = os.path.join(run_dir, f"fused_{FUSE_MODE}_{RUN_TAG}.png")

        fused_dyn = fuse_logodds_prefer_ego(logodds_ego, logodds_rsu_recv)

        render_grid(ego_path, ego_logodds=logodds_ego, mode="ego")
        render_grid(rsu_path, rsu_logodds=logodds_rsu_recv, mode="rsu")
        render_grid(fused_path,
                    ego_logodds=logodds_ego,
                    rsu_logodds=logodds_rsu_recv,
                    mode="fused")

        # ==== LiDAR全時間統合データを保存（Ego座標系 XYZ） ====
        if len(ego_points_all) > 0:
            ego_all = np.vstack(ego_points_all)
            np.save(os.path.join(run_dir, "ego_points_agg.npy"), ego_all)
            write_lpy(os.path.join(run_dir, "ego_points_agg.lpy"), ego_all)

        if len(rsu_points_all) > 0:
            rsu_all = np.vstack(rsu_points_all)
            np.save(os.path.join(run_dir, "rsu_points_agg.npy"), rsu_all)
            write_lpy(os.path.join(run_dir, "rsu_points_agg.lpy"), rsu_all)

        print("[DONE] Saved:", ego_path, rsu_path, fused_path)
        print(f"[COMM] sent={ch.sent} recv={ch.recv} "
              f"tx={ch.tx_bytes/1024:.1f}KB rx={ch.rx_bytes/1024:.1f}KB "
              f"latency={LATENCY_MS}ms drop={DROP_RATE}")

        # 通信エンコード／デコードの自己テスト
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
"""  """