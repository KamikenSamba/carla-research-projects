"""Synthetic acquisition failures and trigger-frame retention, no CARLA imports."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from idm_experiment_harness import raw_actor_observation
from test_idm_context import Transform, vector
from test_idm_harness import Actor, World, run


@pytest.mark.parametrize('which', ['follower', 'leader'])
def test_speed_gate_retains_both_vehicles_before_cleanup(tmp_path, which):
    def setup(world):
        original = world.tick
        def tick():
            result = original()
            if world.frame == 75:
                actor = world.actors[1 if which == 'follower' else 2]
                actor.speed = 3.0
            return result
        world.tick = tick
    result, world, gate, _ = run(tmp_path, setup=setup)
    assert result['status'] == 'safe_early_stop' and result['reason'] == 'speed_envelope'
    raw = json.loads((tmp_path/'last_observation.json').read_text())
    assert raw['schema_version'] == 2 and raw['acquisition_status'] == 'complete'
    assert raw['frame_validated'] and len(raw['rows']) == 2
    assert raw['carla_frame'] == 75 and raw['command_based_on_frame'] == 74
    assert raw['leader_desired_speed_mps'] == 1.4 and raw['follower_desired_speed_mps'] == 2.2
    for row in raw['rows']:
        assert row['frame'] == row['control_read_at_frame'] == result['final_frame']
        assert row['acceleration_source'] == 'get_acceleration'
        assert row['acceleration'] == dict(x=0, y=0, z=0) and row['a_long_mps2'] == 0
        assert row['forward'] == dict(x=1, y=0, z=0)
        assert row['bounding_box']['extent']['x'] == 2
        assert set(row['transform']) == {'x','y','z','yaw','pitch','roll'}
    assert result['cleanup']['complete'] and world.actors == [world.external]
    assert len([e for e in world.events if e[0] == 'tick']) == 75


def test_partial_rpc_failure_preserves_current_frame_and_prohibits_cleanup_rpc(tmp_path):
    def setup(world):
        original = world.try_spawn_actor
        def spawn(*args):
            actor = original(*args)
            getter = actor.get_acceleration
            def acceleration():
                if actor.id == 12 and world.frame == 75:
                    raise RuntimeError('test RPC failure')
                return getter()
            actor.get_acceleration = acceleration
            return actor
        world.try_spawn_actor = spawn
    result, world, gate, _ = run(tmp_path, setup=setup)
    raw = json.loads((tmp_path/'last_observation.json').read_text())
    assert result['status'] == 'server_unavailable' and result['actor_id'] == 12
    assert raw['carla_frame'] == 75 and raw['acquisition_status'] == 'partial'
    assert not raw['frame_validated'] and raw['expected_actor_ids'] == [11,12]
    assert [r['actor_id'] for r in raw['rows']] == [11]
    assert not result['cleanup']['attempted'] and gate.calls_after_poison == 0


def make_raw(acceleration):
    actor = Actor(World(), 42, 1, 'synthetic')
    transform = Transform(1, 2, 90)
    transform.get_forward_vector = lambda: vector(0.6, 0, 0.8)
    return raw_actor_observation(actor, 'follower', transform, vector(2,0,0), actor.control,
                                 acceleration, 99, 0.5)


def test_raw_acceleration_uses_full_forward_projection_not_norm():
    row = make_raw(vector(3,4,5))
    assert row['a_long_mps2'] == pytest.approx(5.8)
    assert row['a_long_mps2'] != pytest.approx(math.sqrt(50))
    assert row['forward'] == dict(x=0.6,y=0,z=0.8)
    assert row['observation_errors'] == []


@pytest.mark.parametrize('acceleration', [None, vector(float('nan'),0,0)])
def test_unknown_acceleration_is_never_replaced_by_a_prediction(acceleration):
    row = make_raw(acceleration)
    assert row['a_long_mps2'] is None and row['observation_errors'] == ['acceleration_unknown']
    assert json.dumps(row, allow_nan=False)
    if acceleration is None:
        assert row['acceleration'] == dict(x=None,y=None,z=None)
        assert row['acceleration_source'] is None
    else:
        assert row['acceleration']['x'] == 'nan'


def test_first_getter_failure_cannot_leave_previous_frame_as_latest(tmp_path):
    def setup(world):
        original = world.try_spawn_actor
        def spawn(*args):
            actor = original(*args)
            getter = actor.get_transform
            def transform():
                if actor.id == 11 and world.frame == 75:
                    raise RuntimeError('first actor unavailable')
                return getter()
            actor.get_transform = transform
            return actor
        world.try_spawn_actor = spawn
    result, _, gate, _ = run(tmp_path, setup=setup)
    raw = json.loads((tmp_path/'last_observation.json').read_text())
    assert result['status'] == 'server_unavailable'
    assert raw['carla_frame'] == 75 and raw['rows'] == []
    assert raw['acquisition_status'] == 'partial' and not raw['frame_validated']
    assert gate.calls_after_poison == 0


def test_frame_change_during_acquisition_is_not_validated(tmp_path):
    def setup(world):
        original = world.try_spawn_actor
        def spawn(*args):
            actor = original(*args)
            getter = actor.get_acceleration
            def acceleration():
                if actor.id == 12 and world.frame == 75:
                    world.frame += 1
                return getter()
            actor.get_acceleration = acceleration
            return actor
        world.try_spawn_actor = spawn
    result, world, _, _ = run(tmp_path, setup=setup)
    raw = json.loads((tmp_path/'last_observation.json').read_text())
    assert result['status'] == 'cleanup_incomplete'
    assert result['reason'] == 'unexpected_frame_progression'
    assert raw['carla_frame'] == 75 and not raw['frame_validated']
    assert len(raw['rows']) == 2
    assert len([e for e in world.events if e[0] == 'tick']) == 75
