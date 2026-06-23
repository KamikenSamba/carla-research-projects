from __future__ import annotations


def set_actor_autopilot(actor, enabled: bool, traffic_manager_port: int | None = None) -> None:
    if traffic_manager_port is None:
        actor.set_autopilot(enabled)
    else:
        actor.set_autopilot(enabled, traffic_manager_port)
