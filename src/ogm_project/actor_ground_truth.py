"""Small vehicle-footprint ground truth helper for Phase-1 false-Free checks."""
from __future__ import annotations

import numpy as np
from PIL import Image, ImageDraw


def _convex_hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def vehicle_footprint_masks(actors, shape, world_to_grid):
    """Rasterise live CARLA vehicle boxes as combined and per-actor masks."""
    ny, nx = shape
    combined = Image.new("1", (nx, ny), 0)
    combined_draw = ImageDraw.Draw(combined)
    actor_masks = {}
    vehicle_count = 0
    for actor in actors:
        if not getattr(actor, "is_alive", False):
            continue
        if not str(getattr(actor, "type_id", "")).startswith("vehicle."):
            continue
        vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
        polygon = _convex_hull([world_to_grid(float(v.x), float(v.y)) for v in vertices])
        if len(polygon) >= 3:
            combined_draw.polygon(polygon, fill=1)
            actor_image = Image.new("1", (nx, ny), 0)
            ImageDraw.Draw(actor_image).polygon(polygon, fill=1)
            attributes = getattr(actor, "attributes", {})
            role = str(attributes.get("role_name", "")).strip()
            key = role if role else f"actor_{getattr(actor, 'id', vehicle_count)}"
            if key in actor_masks:
                key = f"{key}_{getattr(actor, 'id', vehicle_count)}"
            actor_masks[key] = np.asarray(actor_image, dtype=bool)
            vehicle_count += 1
    return np.asarray(combined, dtype=bool), actor_masks


def vehicle_footprint_mask(actors, shape, world_to_grid):
    """Backward-compatible combined vehicle mask and count."""
    combined, actor_masks = vehicle_footprint_masks(actors, shape, world_to_grid)
    return combined, len(actor_masks)
