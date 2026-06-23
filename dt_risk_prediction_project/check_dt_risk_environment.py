from __future__ import annotations

import argparse
import sys

from dt_risk_common import (
    DEFAULT_MAP_NAME,
    carla_module_description,
    connect_client,
    current_python_description,
    get_available_map_names,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check DT risk prediction CARLA environment.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--map", default=DEFAULT_MAP_NAME, help="Map name expected to be available.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("[INFO] Python version:")
    print(current_python_description())

    print("[INFO] CARLA Python module:")
    try:
        module_info = carla_module_description()
        print(f"  path    = {module_info['module_file']}")
        print(f"  version = {module_info['module_version']}")
    except Exception as exc:
        print("[ERROR] carla module could not be imported.")
        print(f"        {exc}")
        return 1

    print(f"[INFO] Connecting to CARLA server: {args.host}:{args.port}")
    try:
        client = connect_client(args.host, args.port, args.timeout)
        world = client.get_world()
    except Exception as exc:
        print("[ERROR] CARLA server is not reachable.")
        print(f"        {exc}")
        print("        CARLAサーバを起動してから再実行してください。")
        return 2

    try:
        server_version = client.get_server_version()
    except Exception as exc:
        server_version = f"unknown ({exc})"

    try:
        current_map = world.get_map().name
    except Exception as exc:
        current_map = f"unknown ({exc})"

    print("[INFO] CARLA server:")
    print(f"  server version = {server_version}")
    print(f"  current map    = {current_map}")

    try:
        available_maps = get_available_map_names(client)
        can_use_requested_map = args.map in available_maps
    except Exception as exc:
        available_maps = []
        can_use_requested_map = False
        print(f"[WARNING] Failed to list available maps: {exc}")

    print(f"[INFO] Requested map: {args.map}")
    print(f"  available = {can_use_requested_map}")
    if available_maps:
        print("  installed maps:")
        for map_name in available_maps:
            print(f"    - {map_name}")

    if not can_use_requested_map:
        print(f"[WARNING] {args.map} was not found in client.get_available_maps().")
        return 3

    print("[INFO] Environment check completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
