# -*- coding: utf-8 -*-
"""
Run_Ego_OGM_V9.py

Ego LiDAR のみを用いて、
- 内部では WORLD 基準の疎な OGM を保持し続ける
- 出力では Ego 中心のローカル OGM も切り出す

という 2 層構成の実験用スクリプト。

狙い:
- 「固定された交差点パッチ」ではなく、車両が新しい場所へ移動しても OGM を継続更新する
- ただし自動運転で実際に使いたいのは自車周辺なので、Ego 中心のローカル OGM も同時に保存する
- Run_Coop_Comm_V8.py のパス設定・シナリオ設定・保存構成に合わせる

主な仕様:
- RSU / 通信 / 融合は削除（Ego LiDAR のみ）
- static_mask は固定原点前提のため、この版では未使用
- OGM 本体は chunked sparse world map
- スナップショット時に Ego 中心ローカルパッチと World 全体可視化を保存
"""

import os
import time
import math
import csv
import random
import json
import argparse
import importlib.util

import numpy as np
from PIL import Image

import subprocess
import os

from ogm_project.spectator_utils import (
    SpectatorConfig,
    apply_spectator_config_from_dict,
    sleep_for_realtime_preview,
    update_spectator,
)

def make_mp4_from_png_sequence(ffmpeg_exe: str, image_dir: str, out_mp4: str, fps: int = 10):
    """
    image_dir 内の t0000.png, t0001.png, ... を mp4 にする
    """
    input_pattern = os.path.join(image_dir, "t%04d.png")

    cmd = [
        ffmpeg_exe,
        "-y",                      # 既存ファイルを上書き
        "-framerate", str(fps),
        "-i", input_pattern,
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",
        "-movflags", "+faststart",
        out_mp4,
    ]

    print("[FFMPEG] running:", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("[FFMPEG][ERROR]")
        print(result.stderr)
    else:
        print(f"[FFMPEG][OK] saved: {out_mp4}")
# --- optional: log-odds heatmap output (requires matplotlib) ---
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import colormaps as _mpl_colormaps
    _MATPLOTLIB_OK = True
except Exception as _e:
    _MATPLOTLIB_OK = False
    print("[WARN] matplotlib not available -> skip logodds heatmap:", _e)

import carla


# ===== パス設定（V8に合わせる） =====
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_ROOT = os.environ.get("CARLA_DATA_ROOT", r"D:\\CARLA_DATA")
OUTPUT_ROOT = os.path.join(DATA_ROOT, "outputs")
MASK_ROOT = os.path.join(DATA_ROOT, "masks")
LOG_ROOT = os.path.join(DATA_ROOT, "logs")
SAVE_LOG_MODE = "summary"   # "all" / "summary" / "none"


def log_save(msg: str):
    if SAVE_LOG_MODE == "all":
        print(msg)


def resolve_from_script(path_str: str | None):
    if not path_str:
        return path_str
    if os.path.isabs(path_str):
        return path_str
    return os.path.join(SCRIPT_DIR, path_str)


# ============================================================
# ここだけいじれば実験条件を変えられるパラメータゾーン
# ============================================================
USE_STATIC_MASK = False  # この版では未使用（固定原点マスクと整合しないため）
STATIC_MASK_PATH = os.path.join(MASK_ROOT, "static_mask.npy")

# --- Ego 車両のスポーン設定 ---
USE_MANUAL_EGO_SPAWN = True
EGO_SPAWN_X = 10.0
EGO_SPAWN_Y = 70.0
EGO_SPAWN_Z = 0.2
EGO_SPAWN_YAW = 0.0

# --- Ego の挙動 ---
EGO_AUTOPILOT = False
EGO_THROTTLE = 0.7
EGO_STEER = 0.0
EGO_BRAKE = 0.0
EGO_HANDBRAKE = False
EGO_MAX_SPEED_KMH = 40.0

# --- ローカルOGM（Ego中心で切り出す範囲） ---
# x: 右+, y: 前+
RES = 0.2
X_MIN, X_MAX = -15.0, 45.0
Y_MIN, Y_MAX = -30.0, 30.0

# --- World OGM chunk 設定 ---
CHUNK_SIZE = 128   # 1 chunk = 128x128 cells
WORLD_RENDER_MARGIN_CELLS = 20
WORLD_SNAPSHOT_EVERY_N_TICKS = 10   # 0.5 sec ごとに world 可視化

# --- LiDAR ---
LIDAR_CHANNELS = 32
LIDAR_RANGE = 100.0
LIDAR_ROT_HZ = 10.0
LIDAR_PPS = 200000
EGO_LIDAR_REL_X = 10.0
EGO_LIDAR_REL_Y = 0.0
EGO_LIDAR_REL_Z = 1.0

# --- 高さフィルタ（WORLD z） ---
Z_MIN_WORLD = 0.10
Z_MAX_WORLD = 1.00

# --- log-odds ---
L0 = 0.0
L_OCC = float(np.log(0.8 / 0.2))
L_FREE = float(np.log(0.45 / 0.55))
L_MIN, L_MAX = -1.5, 1.5
DECAY_PER_SEC_EGO = 0.6

# --- 可視化しきい値 ---
OCC_TH = 0.60
FREE_TH = 0.48

# --- シミュレーション ---
HOST, PORT = "127.0.0.1", 2000
USE_SYNC = True
FIXED_DELTA = 0.05
SIM_DURATION_SEC = 6.0
MAX_TICKS = int(SIM_DURATION_SEC / FIXED_DELTA)

SNAPSHOT_INTERVAL_SEC = 0.1
SNAPSHOT_EVERY_N_TICKS = max(1, int(round(SNAPSHOT_INTERVAL_SEC / FIXED_DELTA)))

OUT_DIR = OUTPUT_ROOT
RUN_TAG = time.strftime("%Y%m%d_%H%M%S")

# --- 固定オブジェクト ---
USE_FIXED_OBJECTS = False

# --- 環境オブジェクト（木など） ---
DISABLE_VEGETATION = False


# ============================================================
# ここから下はロジック本体
# ============================================================
try:
    from fixed_objects import FIXED_OBJECTS
except ImportError:
    FIXED_OBJECTS = []
    print("[INFO] fixed_objects.py not found. No fixed objects will be spawned.")


# ===== シナリオ（外部JSON） =====
USE_SCENARIO_ACTORS = False
SCENARIO_ACTORS_FILE = None
ENABLE_SPECTATOR_FOLLOW = True
SPECTATOR_VIEW_MODE = "topdown"
SPECTATOR_HEIGHT_M = 35.0
SPECTATOR_CHASE_DISTANCE_M = 20.0
SPECTATOR_CHASE_HEIGHT_M = 18.0
ENABLE_REALTIME_PREVIEW = False
SPECTATOR_CONFIG = SpectatorConfig()


def load_scenarios_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def apply_scenario_config(cfg: dict):
    """scenarios.json の 1シナリオ分をグローバル設定へ上書き適用する。"""
    global USE_FIXED_OBJECTS
    global USE_MANUAL_EGO_SPAWN, EGO_SPAWN_X, EGO_SPAWN_Y, EGO_SPAWN_Z, EGO_SPAWN_YAW
    global USE_SCENARIO_ACTORS, SCENARIO_ACTORS_FILE
    global ENABLE_SPECTATOR_FOLLOW, SPECTATOR_VIEW_MODE, SPECTATOR_HEIGHT_M
    global SPECTATOR_CHASE_DISTANCE_M, SPECTATOR_CHASE_HEIGHT_M, ENABLE_REALTIME_PREVIEW
    global SPECTATOR_CONFIG

    if "use_fixed_objects" in cfg:
        USE_FIXED_OBJECTS = bool(cfg["use_fixed_objects"])

    ego = cfg.get("ego_spawn")
    if ego:
        USE_MANUAL_EGO_SPAWN = True
        EGO_SPAWN_X = float(ego.get("x", EGO_SPAWN_X))
        EGO_SPAWN_Y = float(ego.get("y", EGO_SPAWN_Y))
        EGO_SPAWN_Z = float(ego.get("z", EGO_SPAWN_Z))
        EGO_SPAWN_YAW = float(ego.get("yaw", EGO_SPAWN_YAW))

    if "actors_file" in cfg:
        SCENARIO_ACTORS_FILE = str(cfg["actors_file"]) if cfg["actors_file"] else None
    if "use_scenario_actors" in cfg:
        USE_SCENARIO_ACTORS = bool(cfg["use_scenario_actors"])

    SPECTATOR_CONFIG = apply_spectator_config_from_dict(
        SpectatorConfig(
            enabled=ENABLE_SPECTATOR_FOLLOW,
            mode=SPECTATOR_VIEW_MODE,
            height_m=SPECTATOR_HEIGHT_M,
            chase_distance_m=SPECTATOR_CHASE_DISTANCE_M,
            chase_height_m=SPECTATOR_CHASE_HEIGHT_M,
            realtime_preview=ENABLE_REALTIME_PREVIEW,
        ),
        cfg.get("spectator"),
    )
    ENABLE_SPECTATOR_FOLLOW = SPECTATOR_CONFIG.enabled
    SPECTATOR_VIEW_MODE = SPECTATOR_CONFIG.mode
    SPECTATOR_HEIGHT_M = SPECTATOR_CONFIG.height_m
    SPECTATOR_CHASE_DISTANCE_M = SPECTATOR_CONFIG.chase_distance_m
    SPECTATOR_CHASE_HEIGHT_M = SPECTATOR_CONFIG.chase_height_m
    ENABLE_REALTIME_PREVIEW = SPECTATOR_CONFIG.realtime_preview



def load_scenario_actors(actors_file: str):
    if not actors_file:
        return []
    actors_file = resolve_from_script(actors_file)
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
    kind = d.get("kind", "vehicle")
    bp_filter = d.get("bp_filter", "vehicle.*")
    bps = bp_lib.filter(bp_filter)
    if not bps:
        print(f"[SC_ACTORS][WARN] no blueprint for '{bp_filter}', skip {d.get('name')}")
        return None, (None, None)
    bp = random.choice(bps)

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
        fx, fy = math.cos(yaw_rad), math.sin(yaw_rad)
        rx, ry = -math.sin(yaw_rad), math.cos(yaw_rad)

        loc = carla.Location(
            x=ego_tf.location.x + fx * offset_m + rx * lateral_m,
            y=ego_tf.location.y + fy * offset_m + ry * lateral_m,
            z=z,
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
    print(
        f"[SC_ACTORS] spawned {kind} '{d.get('name')}' at "
        f"({tf.location.x:.2f},{tf.location.y:.2f},{tf.location.z:.2f}) yaw={tf.rotation.yaw:.1f}"
    )

    extra_mode, extra_val = None, None
    ctrl = d.get("control", {}) or {}

    if kind == "vehicle":
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
            if "max_speed_kmh" in ctrl and ctrl.get("max_speed_kmh") is not None:
                try:
                    extra_mode, extra_val = "speed_cap", float(ctrl["max_speed_kmh"]) / 3.6
                except Exception:
                    pass
        else:
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
    p.add_argument("--scenario-file", type=str, default=os.path.join(SCRIPT_DIR, "scenarios.json"))
    p.add_argument("--scenario", type=str, default=None)
    p.add_argument("--list-scenarios", action="store_true")
    args, _unknown = p.parse_known_args()
    return args


# ===== Vegetation（木/草）を無効化 =====
def disable_vegetation_objects(world):
    try:
        objs = world.get_environment_objects(carla.CityObjectLabel.Vegetation)
        ids = [o.id for o in objs]
        if not ids:
            print('[VEG][INFO] no vegetation environment objects found (maybe foliage-baked map).')
            return [], False
        world.enable_environment_objects(ids, False)
        print(f'[VEG][OK] disabled vegetation objects: {len(ids)}')
        return ids, True
    except Exception as e:
        print('[VEG][WARN] failed to disable vegetation:', e)
        return [], False


# ===== 幾何変換 =====
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



def rotate_xy(x: float, y: float, yaw_deg: float):
    th = math.radians(yaw_deg)
    c = math.cos(th)
    s = math.sin(th)
    return c * x - s * y, s * x + c * y



def ego_to_world_xy(xe: float, ye: float, ego_tf: carla.Transform):
    xr, yr = rotate_xy(xe, ye, ego_tf.rotation.yaw)
    return ego_tf.location.x + xr, ego_tf.location.y + yr


# ===== Bresenham =====
def bresenham(ix0, iy0, ix1, iy1):
    dx = abs(ix1 - ix0)
    sx = 1 if ix0 < ix1 else -1
    dy = -abs(iy1 - iy0)
    sy = 1 if iy0 < iy1 else -1
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


# ===== 疎な WORLD OGM =====
class SparseWorldOGM:
    def __init__(self, res: float, chunk_size: int, l0: float, lmin: float, lmax: float):
        self.res = float(res)
        self.chunk_size = int(chunk_size)
        self.l0 = float(l0)
        self.lmin = float(lmin)
        self.lmax = float(lmax)
        self.chunks: dict[tuple[int, int], np.ndarray] = {}
        self.min_ix = None
        self.max_ix = None
        self.min_iy = None
        self.max_iy = None

    def world_to_cell(self, xw: float, yw: float):
        ix = int(math.floor(xw / self.res))
        iy = int(math.floor(yw / self.res))
        return ix, iy

    def cell_center_world(self, ix: int, iy: int):
        return (ix + 0.5) * self.res, (iy + 0.5) * self.res

    def _chunk_key(self, ix: int, iy: int):
        cx = ix // self.chunk_size
        cy = iy // self.chunk_size
        lx = ix - cx * self.chunk_size
        ly = iy - cy * self.chunk_size
        return cx, cy, lx, ly

    def _get_chunk(self, cx: int, cy: int, create: bool = False):
        key = (cx, cy)
        if key not in self.chunks:
            if not create:
                return None
            self.chunks[key] = np.full((self.chunk_size, self.chunk_size), self.l0, dtype=np.float32)
        return self.chunks[key]

    def get_cell(self, ix: int, iy: int) -> float:
        cx, cy, lx, ly = self._chunk_key(ix, iy)
        ch = self._get_chunk(cx, cy, create=False)
        if ch is None:
            return self.l0
        return float(ch[ly, lx])

    def add_cell(self, ix: int, iy: int, delta: float):
        cx, cy, lx, ly = self._chunk_key(ix, iy)
        ch = self._get_chunk(cx, cy, create=True)
        ch[ly, lx] = np.clip(ch[ly, lx] + delta, self.lmin, self.lmax)

        if self.min_ix is None:
            self.min_ix = self.max_ix = ix
            self.min_iy = self.max_iy = iy
        else:
            self.min_ix = min(self.min_ix, ix)
            self.max_ix = max(self.max_ix, ix)
            self.min_iy = min(self.min_iy, iy)
            self.max_iy = max(self.max_iy, iy)

    def decay_all(self, dt: float, rate: float):
        if dt <= 0 or rate <= 0 or not self.chunks:
            return
        for ch in self.chunks.values():
            ch += (self.l0 - ch) * (rate * dt)
            np.clip(ch, self.lmin, self.lmax, out=ch)

    def update_from_points_world(self, points_xyz_world: np.ndarray, origin_xy_world, free_scale: float = 1.0):
        ox, oy = origin_xy_world
        ix0, iy0 = self.world_to_cell(ox, oy)

        for xw, yw, _zw in points_xyz_world:
            dx = xw - ox
            dy = yw - oy
            if dx * dx + dy * dy > LIDAR_RANGE * LIDAR_RANGE:
                continue

            ix1, iy1 = self.world_to_cell(xw, yw)
            for cx, cy in bresenham(ix0, iy0, ix1, iy1):
                if cx == ix1 and cy == iy1:
                    self.add_cell(cx, cy, L_OCC)
                else:
                    self.add_cell(cx, cy, free_scale * L_FREE)

    def compute_counts(self):
        occ = free = unk = known = 0
        if not self.chunks:
            return {
                "occ": 0,
                "free": 0,
                "unknown": 0,
                "known": 0,
                "chunk_count": 0,
                "observed_total_cells": 0,
            }
        for ch in self.chunks.values():
            p = 1.0 / (1.0 + np.exp(-ch))
            occ_m = p >= OCC_TH
            free_m = p <= FREE_TH
            known_m = occ_m | free_m
            unk_m = ~known_m
            occ += int(occ_m.sum())
            free += int(free_m.sum())
            unk += int(unk_m.sum())
            known += int(known_m.sum())
        return {
            "occ": occ,
            "free": free,
            "unknown": unk,
            "known": known,
            "chunk_count": len(self.chunks),
            "observed_total_cells": len(self.chunks) * self.chunk_size * self.chunk_size,
        }

    def get_bounds(self):
        if self.min_ix is None:
            return None
        return self.min_ix, self.max_ix, self.min_iy, self.max_iy


# ===== 描画 =====
COLOR_OCC = (255, 80, 80)
COLOR_FREE = (0, 220, 255)
COLOR_UNKNOWN = (10, 10, 10)

nx_local = int(np.ceil((X_MAX - X_MIN) / RES))
ny_local = int(np.ceil((Y_MAX - Y_MIN) / RES))


def render_logodds_heatmap(path_png: str, logodds: np.ndarray, cmap_name: str = "coolwarm"):
    if not _MATPLOTLIB_OK:
        return
    try:
        arr = np.clip(logodds.astype(np.float32), L_MIN, L_MAX)
        dpi = 100
        fig = plt.figure(figsize=(arr.shape[1] / dpi, arr.shape[0] / dpi), dpi=dpi)
        ax = plt.Axes(fig, [0.0, 0.0, 1.0, 1.0])
        ax.set_axis_off()
        fig.add_axes(ax)
        cmap = _mpl_colormaps.get_cmap(cmap_name)
        ax.imshow(arr, vmin=L_MIN, vmax=L_MAX, cmap=cmap, origin="upper", interpolation="nearest")
        fig.savefig(path_png, dpi=dpi)
        plt.close(fig)
        log_save(f"[SAVE] {path_png} (logodds heatmap)")
    except Exception as e:
        print("[WARN] failed to save logodds heatmap:", e)



def render_local_ogm_arrays(world_ogm: SparseWorldOGM, ego_tf: carla.Transform):
    logodds = np.full((ny_local, nx_local), L0, dtype=np.float32)
    for iy in range(ny_local):
        ye = Y_MIN + (iy + 0.5) * RES
        for ix in range(nx_local):
            xe = X_MIN + (ix + 0.5) * RES
            xw, yw = ego_to_world_xy(xe, ye, ego_tf)
            wix, wiy = world_ogm.world_to_cell(xw, yw)
            logodds[iy, ix] = world_ogm.get_cell(wix, wiy)

    p = 1.0 / (1.0 + np.exp(-logodds))
    occ = p >= OCC_TH
    free = p <= FREE_TH
    unk = ~(occ | free)

    img = np.zeros((ny_local, nx_local, 3), dtype=np.uint8)
    img[unk] = COLOR_UNKNOWN
    img[occ] = COLOR_OCC
    img[free] = COLOR_FREE
    return img, logodds



def render_world_ogm_arrays(world_ogm: SparseWorldOGM):
    bounds = world_ogm.get_bounds()
    if bounds is None:
        img = np.zeros((1, 1, 3), dtype=np.uint8)
        logodds = np.zeros((1, 1), dtype=np.float32)
        return img, logodds, (0, 0, 0, 0)

    min_ix, max_ix, min_iy, max_iy = bounds
    min_ix -= WORLD_RENDER_MARGIN_CELLS
    max_ix += WORLD_RENDER_MARGIN_CELLS
    min_iy -= WORLD_RENDER_MARGIN_CELLS
    max_iy += WORLD_RENDER_MARGIN_CELLS

    width = max_ix - min_ix + 1
    height = max_iy - min_iy + 1
    logodds = np.full((height, width), L0, dtype=np.float32)

    for iy in range(height):
        gy = min_iy + iy
        for ix in range(width):
            gx = min_ix + ix
            logodds[iy, ix] = world_ogm.get_cell(gx, gy)

    p = 1.0 / (1.0 + np.exp(-logodds))
    occ = p >= OCC_TH
    free = p <= FREE_TH
    unk = ~(occ | free)

    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[unk] = COLOR_UNKNOWN
    img[occ] = COLOR_OCC
    img[free] = COLOR_FREE
    return img, logodds, (min_ix, max_ix, min_iy, max_iy)



def save_rgb(path_png: str, img: np.ndarray):
    Image.fromarray(img).save(path_png)
    log_save(f"[SAVE] {path_png} ({img.shape[1]}x{img.shape[0]})")



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

    log_save(f"[SAVE] {path} (PLY ascii, {n} points, Y flipped)")



def spawn_fixed_objects(world, bp, base_tf):
    actors = []
    if not USE_FIXED_OBJECTS or not FIXED_OBJECTS:
        print("[FIXED] fixed objects disabled or empty.")
        return actors

    print(f"[FIXED] spawning {len(FIXED_OBJECTS)} fixed objects...")
    for obj in FIXED_OBJECTS:
        kind = obj.get("kind", "vehicle")
        bp_filter = obj.get("bp_filter", "vehicle.*")
        loc_cfg = obj.get("location", {})
        rot_cfg = obj.get("rotation", {})

        loc = carla.Location(
            x=loc_cfg.get("x", base_tf.location.x),
            y=loc_cfg.get("y", base_tf.location.y),
            z=loc_cfg.get("z", base_tf.location.z),
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
            actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True))
        print(f"[FIXED] spawned {kind} '{obj.get('name')}' at {loc} (yaw={rot.yaw:.1f})")
    return actors


# ===== メイン =====
def main():
    args = parse_args()

    if args.scenario_file and os.path.exists(args.scenario_file):
        scenarios = load_scenarios_json(args.scenario_file)
        if args.list_scenarios:
            print("[SCENARIO] available:")
            for k in scenarios.keys():
                print(" -", k)
            return
        if args.scenario is not None:
            if args.scenario not in scenarios:
                raise ValueError(
                    f"Scenario '{args.scenario}' not found in {args.scenario_file}. "
                    f"available={list(scenarios.keys())}"
                )
            apply_scenario_config(scenarios[args.scenario])
            print(f"[SCENARIO] file={args.scenario_file} name={args.scenario}")
    else:
        if args.list_scenarios:
            print(f"[SCENARIO] scenario file not found: {args.scenario_file}")
            return

    os.makedirs(DATA_ROOT, exist_ok=True)
    os.makedirs(OUTPUT_ROOT, exist_ok=True)
    os.makedirs(MASK_ROOT, exist_ok=True)
    os.makedirs(LOG_ROOT, exist_ok=True)
    os.makedirs(OUT_DIR, exist_ok=True)

    run_dir = os.path.join(OUT_DIR, RUN_TAG)
    os.makedirs(run_dir, exist_ok=True)

    ego_dir = os.path.join(run_dir, "EGO-OGM")
    ego_hm_dir = os.path.join(ego_dir, "heatmap")
    world_dir = os.path.join(run_dir, "WORLD-OGM")
    world_hm_dir = os.path.join(world_dir, "heatmap")
    for d in (ego_dir, ego_hm_dir, world_dir, world_hm_dir):
        os.makedirs(d, exist_ok=True)

    metrics_csv = os.path.join(run_dir, "metrics_grid.csv")
    with open(metrics_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "sim_time", "tick",
            "ego_occ", "ego_free", "ego_unknown", "ego_total_cells",
            "world_occ", "world_free", "world_unknown", "world_known",
            "world_chunk_count", "world_observed_total_cells",
            "ego_x", "ego_y", "ego_yaw_deg",
        ])

    meta_json = os.path.join(run_dir, "run_meta.json")
    with open(meta_json, "w", encoding="utf-8") as f:
        json.dump({
            "script": os.path.basename(__file__),
            "data_root": DATA_ROOT,
            "output_root": OUTPUT_ROOT,
            "run_dir": run_dir,
            "res": RES,
            "local_x_min": X_MIN,
            "local_x_max": X_MAX,
            "local_y_min": Y_MIN,
            "local_y_max": Y_MAX,
            "chunk_size": CHUNK_SIZE,
            "sim_duration_sec": SIM_DURATION_SEC,
            "fixed_delta": FIXED_DELTA,
            "use_static_mask": USE_STATIC_MASK,
        }, f, ensure_ascii=False, indent=2)

    world_ogm = SparseWorldOGM(RES, CHUNK_SIZE, L0, L_MIN, L_MAX)
    ego_points_all = []
    latest_pts_world = None
    latest_meas_frame = -1
    last_processed_frame = -1

    client = carla.Client(HOST, PORT)
    client.set_timeout(90.0)
    world = client.get_world()
    bp = world.get_blueprint_library()

    veg_disabled_ids = []
    if DISABLE_VEGETATION:
        veg_disabled_ids, _ = disable_vegetation_objects(world)

    orig_settings = world.get_settings()
    tm = None
    if USE_SYNC:
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(s)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

    actors = []
    sensors = []
    speed_cap_actors = []

    try:
        veh_bp = bp.find('vehicle.tesla.model3')
        veh_bp.set_attribute('role_name', 'ego')

        if USE_MANUAL_EGO_SPAWN:
            spawn = carla.Transform(
                carla.Location(x=EGO_SPAWN_X, y=EGO_SPAWN_Y, z=EGO_SPAWN_Z),
                carla.Rotation(pitch=0.0, yaw=EGO_SPAWN_YAW, roll=0.0),
            )
        else:
            spawn = world.get_map().get_spawn_points()[1]

        vehicle = world.try_spawn_actor(veh_bp, spawn)
        if vehicle is None:
            print(f"[ERROR] Failed to spawn ego at {spawn.location}. EGO_SPAWN_X/Y/Z を少し変えてください。")
            return
        actors.append(vehicle)
        update_spectator(world, vehicle, SPECTATOR_CONFIG)

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

        if EGO_MAX_SPEED_KMH is not None:
            speed_cap_actors.append((vehicle, float(EGO_MAX_SPEED_KMH) / 3.6))

        actors.extend(spawn_fixed_objects(world, bp, spawn))

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
            carla.Transform(carla.Location(x=EGO_LIDAR_REL_X, y=EGO_LIDAR_REL_Y, z=EGO_LIDAR_REL_Z)),
            attach_to=vehicle,
        )
        sensors.append(lidar_ego)

        def on_ego(meas: carla.LidarMeasurement):
            nonlocal latest_pts_world, latest_meas_frame
            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            lidar_tf = lidar_ego.get_transform()
            pts_world = transform_to_world(arr, lidar_tf)
            mask_h = (pts_world[:, 2] > Z_MIN_WORLD) & (pts_world[:, 2] < Z_MAX_WORLD)
            latest_pts_world = pts_world[mask_h]
            latest_meas_frame = int(meas.frame)

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

        tick_idx = 0
        snap_idx = 0
        world_snap_idx = 0

        while tick_idx < MAX_TICKS:
            world.tick()
            update_spectator(world, vehicle, SPECTATOR_CONFIG)
            sleep_for_realtime_preview(ENABLE_REALTIME_PREVIEW, FIXED_DELTA)
            tick_idx += 1
            sim_time = tick_idx * FIXED_DELTA

            for _a, _vmax in speed_cap_actors:
                try:
                    v = _a.get_velocity()
                    spd = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                    if spd > _vmax and spd > 1e-3:
                        scale = _vmax / spd
                        _a.set_target_velocity(carla.Vector3D(v.x * scale, v.y * scale, v.z * scale))
                except Exception:
                    pass

            world_ogm.decay_all(FIXED_DELTA, DECAY_PER_SEC_EGO)

            ego_tf = vehicle.get_transform()

            if latest_pts_world is not None and latest_pts_world.size > 0 and latest_meas_frame != last_processed_frame:
                ego_points_all.append(transform_world_to_ego(latest_pts_world, ego_tf)[:, :3].copy())
                lidar_tf = lidar_ego.get_transform()
                origin_xy = (lidar_tf.location.x, lidar_tf.location.y)
                world_ogm.update_from_points_world(latest_pts_world[:, :3], origin_xy, free_scale=1.0)
                last_processed_frame = latest_meas_frame

            if tick_idx % SNAPSHOT_EVERY_N_TICKS == 0:
                base = f"t{snap_idx:04d}"
                ego_img, ego_logodds = render_local_ogm_arrays(world_ogm, ego_tf)
                ego_png = os.path.join(ego_dir, f"{base}.png")
                ego_hm = os.path.join(ego_hm_dir, f"{base}_logodds.png")
                save_rgb(ego_png, ego_img)
                render_logodds_heatmap(ego_hm, ego_logodds)

                p = 1.0 / (1.0 + np.exp(-ego_logodds))
                ego_occ = int((p >= OCC_TH).sum())
                ego_free = int((p <= FREE_TH).sum())
                ego_known = int(((p >= OCC_TH) | (p <= FREE_TH)).sum())
                ego_unknown = int(ego_logodds.size - ego_known)

                world_counts = world_ogm.compute_counts()
                with open(metrics_csv, "a", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow([
                        f"{sim_time:.3f}",
                        tick_idx,
                        ego_occ,
                        ego_free,
                        ego_unknown,
                        int(ego_logodds.size),
                        world_counts["occ"],
                        world_counts["free"],
                        world_counts["unknown"],
                        world_counts["known"],
                        world_counts["chunk_count"],
                        world_counts["observed_total_cells"],
                        f"{ego_tf.location.x:.3f}",
                        f"{ego_tf.location.y:.3f}",
                        f"{ego_tf.rotation.yaw:.3f}",
                    ])
                snap_idx += 1

            if tick_idx % WORLD_SNAPSHOT_EVERY_N_TICKS == 0:
                base = f"t{world_snap_idx:04d}"
                world_img, world_logodds, _bounds = render_world_ogm_arrays(world_ogm)
                world_png = os.path.join(world_dir, f"{base}.png")
                world_hm = os.path.join(world_hm_dir, f"{base}_logodds.png")
                save_rgb(world_png, world_img)
                render_logodds_heatmap(world_hm, world_logodds)
                world_snap_idx += 1

        final_ego_img, final_ego_logodds = render_local_ogm_arrays(world_ogm, vehicle.get_transform())
        final_ego_png = os.path.join(ego_dir, f"final_{RUN_TAG}.png")
        final_ego_hm = os.path.join(ego_hm_dir, f"final_{RUN_TAG}_logodds.png")
        save_rgb(final_ego_png, final_ego_img)
        render_logodds_heatmap(final_ego_hm, final_ego_logodds)

        final_world_img, final_world_logodds, _bounds = render_world_ogm_arrays(world_ogm)
        final_world_png = os.path.join(world_dir, f"final_{RUN_TAG}.png")
        final_world_hm = os.path.join(world_hm_dir, f"final_{RUN_TAG}_logodds.png")
        save_rgb(final_world_png, final_world_img)
        render_logodds_heatmap(final_world_hm, final_world_logodds)

        if ego_points_all:
            ego_all = np.vstack(ego_points_all)
            np.save(os.path.join(run_dir, "ego_points_agg.npy"), ego_all)
            write_lpy(os.path.join(run_dir, "ego_points_agg.ply"), ego_all)

        print("[DONE] Saved:", final_ego_png, final_world_png)
        print(f"[WORLD] chunks={len(world_ogm.chunks)} bounds={world_ogm.get_bounds()}")

        ffmpeg_exe = r"D:\FFmpeg\ffmpeg-8.0.1-essentials_build\bin\ffmpeg.exe"

        make_mp4_from_png_sequence(
            ffmpeg_exe,
            ego_dir,
            os.path.join(run_dir, "ego.mp4"),
            fps=10
        )

        make_mp4_from_png_sequence(
            ffmpeg_exe,
            world_dir,
            os.path.join(run_dir, "world.mp4"),
            fps=10
        )
        
    finally:
        for s in sensors:
            try:
                if getattr(s, "is_listening", False):
                    s.stop()
            except Exception:
                pass
            try:
                s.destroy()
            except Exception:
                pass

        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass

        if DISABLE_VEGETATION and veg_disabled_ids:
            try:
                world.enable_environment_objects(veg_disabled_ids, True)
            except Exception:
                pass

        if USE_SYNC:
            try:
                tm = client.get_trafficmanager(8000)
                tm.set_synchronous_mode(False)
            except Exception:
                pass
        world.apply_settings(orig_settings)


if __name__ == "__main__":
    main()
