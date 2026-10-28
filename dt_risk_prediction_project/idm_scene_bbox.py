"""World-descriptor diagnostics and signed-scale corner interpretation only.

CARLA 0.9.16 BoundingBoxCalculator::ApplyTransformToBB multiplies Extent by
component scale. Negative components permute the +/- corner set; they do not
remove an obstacle. The shared vehicle/prop box validator remains unchanged.
"""
import math
from types import SimpleNamespace

from idm_brake_config import finite
from idm_brake_route import box_vertices


def raw_bbox(box):
    result = dict(python_type=type(box).__module__+'.'+type(box).__name__,fields={})
    for group,axes in (('location',('x','y','z')),('extent',('x','y','z')),
                       ('rotation',('yaw','pitch','roll'))):
        try:
            obj = getattr(box,group)
        except Exception as exc:
            result['fields'][group] = dict(present=False,error=type(exc).__name__+': '+str(exc))
            continue
        fields = {}
        result['fields'][group] = dict(present=True,python_type=type(obj).__name__,fields=fields)
        for axis in axes:
            try:
                value = getattr(obj,axis)
                valid = finite(value)
                encoded = value if valid or type(value) in (str,bool,type(None)) else dict(
                    repr=repr(value),special='NaN' if type(value) is float and math.isnan(value) else
                    'Infinity' if type(value) is float and math.isinf(value) else 'non_numeric')
                fields[axis] = dict(present=True,python_type=type(value).__name__,value=encoded,finite=valid)
                if group=='extent':
                    fields[axis]['sign'] = ('negative' if value<0 else 'zero' if value==0 else 'positive') if valid else 'unknown'
            except Exception as exc:
                fields[axis] = dict(present=False,error=type(exc).__name__+': '+str(exc))
    return result


def obstacle_vertices(descriptor,source):
    """Use a saved raw descriptor, not another getter. Never infer missing data."""
    if source not in ('level_bbs','environment_objects'):
        raise ValueError('signed extent interpretation requires world obstacle source')
    values = {}
    for group,axes in (('location',('x','y','z')),('extent',('x','y','z')),
                       ('rotation',('yaw','pitch','roll'))):
        fields = descriptor['fields'].get(group,{})
        if fields.get('present') is not True:
            raise ValueError('unknown obstacle bbox '+group)
        values[group] = {}
        for axis in axes:
            item = fields.get('fields',{}).get(axis,{})
            if item.get('present') is not True or item.get('finite') is not True or not finite(item.get('value')):
                raise ValueError('unknown obstacle bbox '+group+'.'+axis)
            values[group][axis] = item['value']
    # abs is a vertex permutation, not a repair of coordinates or a skipped box.
    canonical = {k:abs(v) for k,v in values['extent'].items()}
    if not any(canonical.values()):
        raise ValueError('unknown obstacle bbox: all-zero extent')
    box = SimpleNamespace(location=SimpleNamespace(**values['location']),
                          extent=SimpleNamespace(**canonical),rotation=SimpleNamespace(**values['rotation']))
    vertices = box_vertices(box)
    if not all(finite(v) for point in vertices for v in point):
        raise ValueError('unknown obstacle bbox: nonfinite vertices')
    return vertices,dict(kind='carla_0.9.16_world_descriptor_signed_scale',
        original_extent=values['extent'],corner_extent=canonical,
        signed=any(v<0 for v in values['extent'].values()),
        justification='absolute extent permutes the eight SDK corner vertices; obstacle retained')
