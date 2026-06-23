from __future__ import annotations

import math
from dataclasses import dataclass

import carla


@dataclass
class SpectatorConfig:
    enabled: bool = True
    mode: str = "topdown"
    height_m: float = 35.0
    chase_distance_m: float = 20.0
    chase_height_m: float = 18.0
    realtime_preview: bool = False


def update_spectator_topdown(world, ego_vehicle, height_m: float = 35.0) -> None:
    spectator = world.get_spectator()
    ego_tf = ego_vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            ego_tf.location + carla.Location(z=height_m),
            carla.Rotation(pitch=-90.0, yaw=ego_tf.rotation.yaw, roll=0.0),
        )
    )


def update_spectator_chase(
    world,
    ego_vehicle,
    distance_m: float = 20.0,
    height_m: float = 18.0,
) -> None:
    spectator = world.get_spectator()
    ego_tf = ego_vehicle.get_transform()
    forward = ego_tf.get_forward_vector()
    camera_location = carla.Location(
        x=ego_tf.location.x - forward.x * distance_m,
        y=ego_tf.location.y - forward.y * distance_m,
        z=ego_tf.location.z + height_m,
    )
    spectator.set_transform(
        carla.Transform(
            camera_location,
            carla.Rotation(pitch=-40.0, yaw=ego_tf.rotation.yaw, roll=0.0),
        )
    )


def update_spectator(world, ego_vehicle, config: SpectatorConfig) -> None:
    if not config.enabled:
        return
    mode = (config.mode or "topdown").lower()
    try:
        if mode == "chase":
            update_spectator_chase(
                world,
                ego_vehicle,
                distance_m=config.chase_distance_m,
                height_m=config.chase_height_m,
            )
        else:
            update_spectator_topdown(world, ego_vehicle, height_m=config.height_m)
    except Exception as exc:
        print(f"[SPECTATOR][WARN] failed to update spectator: {exc}")


def apply_spectator_config_from_dict(config: SpectatorConfig, data: dict | None) -> SpectatorConfig:
    if not data:
        return config
    return SpectatorConfig(
        enabled=bool(data.get("enabled", config.enabled)),
        mode=str(data.get("mode", config.mode)),
        height_m=float(data.get("height_m", config.height_m)),
        chase_distance_m=float(data.get("chase_distance_m", config.chase_distance_m)),
        chase_height_m=float(data.get("chase_height_m", config.chase_height_m)),
        realtime_preview=bool(data.get("realtime_preview", config.realtime_preview)),
    )


def sleep_for_realtime_preview(enabled: bool, fixed_delta: float) -> None:
    if enabled:
        import time

        time.sleep(max(0.0, float(fixed_delta)))
