"""Test-only support for running the offline suite without a CARLA install."""

from __future__ import annotations

import importlib.util
import sys
from types import ModuleType

import pytest


# The replay tests reuse numerical helpers from coop_comm_compat.  That module
# imports CARLA for its live-simulation paths, but the offline tests never call
# those paths.  Keep CI independent of a platform-specific CARLA wheel without
# changing production imports or pretending that live integration was tested.
CARLA_AVAILABLE = importlib.util.find_spec("carla") is not None

if not CARLA_AVAILABLE:
    carla_stub = ModuleType("carla")

    class _CarlaType:
        """Annotation placeholder; live CARLA APIs remain unavailable in CI."""

    carla_stub.Transform = _CarlaType
    carla_stub.LidarMeasurement = _CarlaType
    sys.modules["carla"] = carla_stub


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    """Skip the single test that exercises real CARLA geometry objects."""
    if CARLA_AVAILABLE:
        return
    marker = pytest.mark.skip(reason="CARLA wheel is not available in offline CI")
    for item in items:
        if item.name == "test_carla_box_adapter_matches_world_vertices_with_box_rotation":
            item.add_marker(marker)
