from io import StringIO

from scripts.convert_vehicle_state_csv import (
    convert_rows,
    map_actor_type,
    normalise_spawn_z,
)


def test_map_actor_type() -> None:
    assert map_actor_type("vehicle.tesla.model3") == "vehicle"
    assert map_actor_type("vehicle.bh.crossbike") == "bicycle"
    assert map_actor_type("vehicle.diamondback.century") == "bicycle"
    assert map_actor_type("walker.pedestrian.001") == "walker"
    assert map_actor_type("static.prop") == "static.prop"


def test_normalise_spawn_z_for_vehicle_like_actors() -> None:
    assert normalise_spawn_z("vehicle", "0.001", 0.2) == "0.2"
    assert normalise_spawn_z("bicycle", "0.158", 0.2) == "0.2"
    assert normalise_spawn_z("vehicle", "1.0", 0.2) == "1.0"
    assert normalise_spawn_z("walker", "0.001", 0.2) == "0.001"


def test_convert_rows_reduces_columns() -> None:
    source = StringIO(
        "frame,id,type,location_x,location_y,location_z,ignored\n"
        "100,42,vehicle.tesla.model3,1.0,2.0,3.0,x\n"
    )
    destination = StringIO()

    convert_rows(source, destination)
    converted = destination.getvalue().strip().splitlines()

    assert converted[0] == "frame,id,type,x,y,z"
    assert converted[1] == "100,42,vehicle,1.0,2.0,3.0"


def test_convert_rows_preserves_bicycle_and_raises_low_spawn_z() -> None:
    source = StringIO(
        "frame,id,type,location_x,location_y,location_z,ignored\n"
        "100,42,vehicle.bh.crossbike,1.0,2.0,0.01,x\n"
    )
    destination = StringIO()

    convert_rows(source, destination)
    converted = destination.getvalue().strip().splitlines()

    assert converted[1] == "100,42,bicycle,1.0,2.0,0.2"
