import carla
import numpy as np

# ==== グリッド設定（Run_Coop_Comm_V1.py と合わせる）====
RES   = 0.2
X_MIN, X_MAX = -30.0, 30.0   # ORIGIN からの相対 X 範囲 [m]
Y_MIN, Y_MAX = -30.0, 30.0   # ORIGIN からの相対 Y 範囲 [m]

nx = int(np.ceil((X_MAX - X_MIN) / RES))
ny = int(np.ceil((Y_MAX - Y_MIN) / RES))

HOST, PORT = "127.0.0.1", 2000



# ==== グリッド原点設定 ====
# ORIGIN_MODE:
#   - "spawn" : スポーンポイントを原点にする（従来互換）
#   - "fixed" : 指定したWORLD座標を原点にする（交差点中心など）
ORIGIN_MODE = "fixed"   # "spawn" or "fixed"
SPAWN_INDEX = 1
ORIGIN_FIXED_X = 42.0
ORIGIN_FIXED_Y = 56.0
ORIGIN_FIXED_Z = 0.5

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    world = client.get_world()
    m = world.get_map()

    # ==== グリッド原点の決定 ====
    if ORIGIN_MODE == "fixed":
        origin_x = float(ORIGIN_FIXED_X)
        origin_y = float(ORIGIN_FIXED_Y)
        origin_z = float(ORIGIN_FIXED_Z)
        origin_tag = "FIXED"
    else:
        spawns = m.get_spawn_points()
        if not spawns:
            raise RuntimeError("No spawn points found in this map.")
        if SPAWN_INDEX < 0 or SPAWN_INDEX >= len(spawns):
            raise RuntimeError(f"SPAWN_INDEX out of range: {SPAWN_INDEX} (0..{len(spawns)-1})")
        spawn = spawns[SPAWN_INDEX]
        origin_x = spawn.location.x
        origin_y = spawn.location.y
        origin_z = spawn.location.z
        origin_tag = f"SPAWN[{SPAWN_INDEX}]"
    print(f"GRID ORIGIN (WORLD) = ({origin_x:.2f}, {origin_y:.2f}, {origin_z:.2f})  mode={origin_tag}")
    print(f"GRID RANGE X: [{X_MIN:+.1f}, {X_MAX:+.1f}]  Y: [{Y_MIN:+.1f}, {Y_MAX:+.1f}]  RES={RES}")

    static_mask = np.zeros((ny, nx), dtype=bool)

    print("Building static_mask from HD map (WORLD-fixed grid)...")

    for iy in range(ny):
        # グリッドYインデックス → WORLD座標Y（セル中心）
        yw = origin_y + Y_MIN + (iy + 0.5) * RES

        for ix in range(nx):
            # グリッドXインデックス → WORLD座標X（セル中心）
            xw = origin_x + X_MIN + (ix + 0.5) * RES
            zw = origin_z  # 高さはおおざっぱに車両高さ付近でOK

            loc = carla.Location(x=xw, y=yw, z=zw)

            # 道路にスナップせず、その地点が「どのレーンにも属していないか」を確認
            wp = m.get_waypoint(loc, project_to_road=False)

            if wp is None:
                # HDマップ上にレーンが無い領域
                # → 走行・歩行などの動的オブジェクトが基本いないとみなして静的マスク
                static_mask[iy, ix] = True
            else:
                # 何かしらのレーン（Driving, Sidewalk, Median, Parking, ...）上
                # → 動的オブジェクトが存在し得るので観測で判断
                static_mask[iy, ix] = False

        if iy % 20 == 0:
            print(f"row {iy}/{ny} processed...")

    print("static true cells:", static_mask.sum(), "/", static_mask.size)

    np.save("static_mask.npy", static_mask)
    print("saved static_mask.npy (shape =", static_mask.shape, ")")


if __name__ == "__main__":
    main()
