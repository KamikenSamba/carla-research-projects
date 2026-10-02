import time
import carla

HOST, PORT = "127.0.0.1", 2000

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(5.0)
    world = client.get_world()

    print("Spectatorの位置を監視します。UE4ウィンドウ側でWASD＋マウスで動かしてね。Ctrl+Cで終了。")

    try:
        while True:
            spec = world.get_spectator()
            tf = spec.get_transform()
            loc = tf.location
            rot = tf.rotation
            print(
                f"x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}, yaw={rot.yaw:.1f}"
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("終了")

if __name__ == "__main__":
    main()
