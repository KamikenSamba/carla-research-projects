"""Weather schema contracts; optional real SDK tests never create a Client."""
import importlib.metadata
import json
from types import SimpleNamespace as NS

import pytest

from idm_scene_audit import (
    AuditAbort, Evidence, WEATHER_FIELDS, WEATHER_PRESETS, scalars, weather_scalars,
)

# Independent fixture schema from CARLA 0.9.16 rpc/WeatherParameters.h.
VALUES = dict(cloudiness=12., precipitation=3., precipitation_deposits=4.,
    wind_intensity=5., sun_azimuth_angle=6., sun_altitude_angle=-7.,
    fog_density=8., fog_distance=9., fog_falloff=.1, wetness=10.,
    scattering_intensity=11., mie_scattering_scale=.2,
    rayleigh_scattering_scale=.0331, dust_storm=13.)


class Weather:
    def __init__(self):
        self.__dict__.update(VALUES)


Weather.ClearNight = Weather()
Weather.DustStorm = Weather()


def test_schema_order_complete_and_json_deterministic(tmp_path):
    first = weather_scalars(Weather())
    second = weather_scalars(Weather())
    assert tuple(first) == tuple(sorted(VALUES)) == WEATHER_FIELDS
    assert first == second == VALUES
    assert all(type(v) is float for v in first.values())
    assert not (set(first) & WEATHER_PRESETS)
    encoded = json.dumps(first, allow_nan=False)
    assert encoded == json.dumps(second, allow_nan=False)
    evidence = Evidence(tmp_path, 'SYNTHETIC', synthetic=True)
    evidence.write('weather_test', first)
    saved = json.loads((tmp_path/'weather_test.json').read_text(encoding='utf-8'))
    assert saved['data'] == VALUES


@pytest.mark.parametrize('field', sorted(VALUES))
def test_every_weather_field_change_is_detected(field):
    old, new = Weather(), Weather()
    setattr(new, field, getattr(new, field)+1.)
    before, after = weather_scalars(old), weather_scalars(new)
    assert before != after
    assert {k for k in before if before[k] != after[k]} == {field}


@pytest.mark.parametrize('field', sorted(VALUES))
def test_missing_required_field_refused(field):
    weather = Weather()
    delattr(weather, field)
    with pytest.raises(AuditAbort, match='weather field unavailable: '+field) as error:
        weather_scalars(weather)
    assert error.value.status == 'unknown'


@pytest.mark.parametrize('value', [None, True, '1.0', [], {}, object(), lambda: 1.])
def test_nonnumeric_or_json_invalid_field_refused(value):
    weather = Weather()
    weather.cloudiness = value
    with pytest.raises(AuditAbort, match='nonnumeric weather field: cloudiness'):
        weather_scalars(weather)


@pytest.mark.parametrize('field', sorted(VALUES))
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_every_nonfinite_field_refused(field, value):
    weather = Weather()
    setattr(weather, field, value)
    with pytest.raises(AuditAbort, match='nonfinite weather field: '+field):
        weather_scalars(weather)


def test_integer_representation_is_canonical_without_default_fill():
    weather = Weather()
    weather.cloudiness = 12
    assert json.dumps(weather_scalars(weather)) == json.dumps(weather_scalars(Weather()))
    weather.cloudiness = 10**1000
    with pytest.raises(AuditAbort, match='invalid weather number: cloudiness'):
        weather_scalars(weather)


@pytest.mark.parametrize('value', [1., {}, object(), lambda: None])
def test_unexpected_public_instance_attribute_refused(value):
    weather = Weather()
    weather.future_field = value
    with pytest.raises(AuditAbort, match='unexpected weather attributes: future_field'):
        weather_scalars(weather)


def test_preset_shadow_and_wrong_structure_refused():
    weather = Weather()
    weather.ClearNight = Weather()
    with pytest.raises(AuditAbort, match='invalid weather preset attribute: ClearNight'):
        weather_scalars(weather)
    weather = NS(**VALUES, ClearNight={'unserializable': object()})
    with pytest.raises(AuditAbort, match='invalid weather preset attribute: ClearNight'):
        weather_scalars(weather)


@pytest.mark.parametrize('mode', ['field', 'preset', 'dir', 'instance_dict'])
def test_attribute_access_exceptions_are_explicit(mode):
    class Broken:
        def __dir__(self):
            if mode == 'dir':
                raise RuntimeError('synthetic dir failure')
            return list(VALUES) + (['ClearNight'] if mode == 'preset' else [])

        def __getattribute__(self, name):
            if name == '__dict__' and mode == 'instance_dict':
                raise RuntimeError('synthetic instance dictionary failure')
            if name == 'cloudiness' and mode == 'field':
                raise RuntimeError('synthetic field failure')
            if name == 'ClearNight' and mode == 'preset':
                raise RuntimeError('synthetic preset failure')
            if name in VALUES:
                return VALUES[name]
            return object.__getattribute__(self, name)

    with pytest.raises(AuditAbort, match='weather .*unavailable') as error:
        weather_scalars(Broken())
    assert error.value.status == 'unknown'


def test_common_serializer_contract_is_unchanged():
    settings = NS(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False)
    assert scalars(settings) == vars(settings)
    rotation = NS(pitch=0., roll=1., yaw=2.)
    assert scalars(rotation) == vars(rotation)
    assert scalars(NS(id=1, type_id='spectator')) == dict(id=1, type_id='spectator')
    with pytest.raises(AuditAbort, match='unserializable required struct field: ClearNight'):
        scalars(Weather())  # Weather exemption is not added to the shared path.
    with pytest.raises(AuditAbort, match='nonfinite'):
        scalars(NS(pitch=float('nan')))


def test_real_carla_0916_weather_and_json_without_client(monkeypatch, tmp_path):
    carla = pytest.importorskip('carla', reason='optional local SDK struct-only check')
    if importlib.metadata.version('carla') != '0.9.16':
        pytest.skip('requires CARLA 0.9.16 struct schema')
    monkeypatch.setattr(carla, 'Client', lambda *a, **kw: pytest.fail('Client forbidden'))
    weather = carla.WeatherParameters()
    actual_fields = {n for n in dir(weather) if not n.startswith('_')
                     and type(getattr(weather, n)) in (int, float)}
    actual_presets = {n for n in dir(weather) if not n.startswith('_')
                      and type(getattr(weather, n)) is carla.WeatherParameters}
    assert actual_fields == set(VALUES)
    assert actual_presets == WEATHER_PRESETS
    assert type(weather.ClearNight) is carla.WeatherParameters
    baseline = weather_scalars(weather)
    assert baseline == weather_scalars(carla.WeatherParameters())
    assert baseline == {name: float(getattr(weather, name)) for name in sorted(VALUES)}
    for name in sorted(WEATHER_PRESETS):
        payload = weather_scalars(getattr(weather, name))
        assert set(payload) == set(VALUES)
        json.dumps(payload, allow_nan=False)
    with pytest.raises(AuditAbort, match='ClearNight'):
        scalars(weather)
    weather.wetness = 27.5
    changed = weather_scalars(weather)
    assert changed != baseline and changed['wetness'] == 27.5
    evidence = Evidence(tmp_path, 'LOCAL_SDK_NO_SERVER', synthetic=True)
    evidence.write('weather_struct', changed)
    record = json.loads((tmp_path/'weather_struct.json').read_text(encoding='utf-8'))
    assert record['data'] == changed and record['synthetic'] is True
