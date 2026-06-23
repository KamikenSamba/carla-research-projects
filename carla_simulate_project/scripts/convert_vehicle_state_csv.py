"""Convert CARLA vehicle state CSVs to a reduced format."""

from __future__ import annotations

import argparse
import csv
import sys
from typing import Iterable, TextIO

MIN_SPAWN_Z = 0.2


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "source",
        help="Source CSV file produced by vehicle_state_stream.py (use '-' for stdin)",
    )
    parser.add_argument(
        "destination",
        help="Destination CSV file (use '-' for stdout)",
    )
    parser.add_argument(
        "--min-spawn-z",
        type=float,
        default=MIN_SPAWN_Z,
        help=(
            "Minimum z coordinate for vehicle/bicycle replay spawn points "
            f"(default: {MIN_SPAWN_Z})."
        ),
    )
    return parser.parse_args(list(argv))


def map_actor_type(type_id: str) -> str:
    lowered = type_id.lower()
    if (
        lowered in {"vehicle.bh.crossbike", "vehicle.diamondback.century"}
        or "bike" in lowered
        or "bicycle" in lowered
    ):
        return "bicycle"
    if type_id.startswith("vehicle."):
        return "vehicle"
    if type_id.startswith("walker."):
        return "walker"
    return type_id


def normalise_spawn_z(actor_type: str, z_value: str, min_spawn_z: float) -> str:
    if actor_type not in {"vehicle", "bicycle"}:
        return z_value

    try:
        z = float(z_value)
    except (TypeError, ValueError):
        return z_value

    if z >= min_spawn_z:
        return z_value
    return str(min_spawn_z)


def convert_rows(
    source: TextIO, destination: TextIO, *, min_spawn_z: float = MIN_SPAWN_Z
) -> None:
    reader = csv.DictReader(source)
    fieldnames = ["frame", "id", "type", "x", "y", "z"]
    writer = csv.DictWriter(destination, fieldnames=fieldnames)
    writer.writeheader()

    for row in reader:
        actor_type = map_actor_type(row.get("type", ""))
        z_value = normalise_spawn_z(
            actor_type,
            row.get("location_z", ""),
            min_spawn_z,
        )
        writer.writerow(
            {
                "frame": row.get("frame", ""),
                "id": row.get("id", ""),
                "type": actor_type,
                "x": row.get("location_x", ""),
                "y": row.get("location_y", ""),
                "z": z_value,
            }
        )


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_arguments(argv or sys.argv[1:])

    if args.source == "-":
        source_stream = sys.stdin
        close_source = False
    else:
        source_stream = open(args.source, "r", newline="")
        close_source = True

    if args.destination == "-":
        destination_stream = sys.stdout
        close_destination = False
    else:
        destination_stream = open(args.destination, "w", newline="")
        close_destination = True

    try:
        convert_rows(
            source_stream,
            destination_stream,
            min_spawn_z=args.min_spawn_z,
        )
    finally:
        if close_source:
            source_stream.close()
        if close_destination:
            destination_stream.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
