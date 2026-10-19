"""Offline-only reference validation. A reference is never a route_audit approval."""
import hashlib
import json
import math
from pathlib import Path

import idm_brake_config as policy
from idm_brake_config import load_config, TOWN01_MODE
from idm_brake_route import geometry_sha256, validate_review_geometry

GEOMETRY_SHA256 = '65f6c60358d155ec6f9678061a8821483705250f157b0958600ef1ac7342ee47'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_policy(path):
    c = load_config(path)
    if c.mode != TOWN01_MODE or c.route_audit is not None:
        raise ValueError('scene audit requires the unapproved Town01 policy (route_audit=null)')
    return c


def read_reference(c, xml_path, geometry_path):
    if not xml_path or not geometry_path:
        raise ValueError('both independent reference OpenDRIVE and geometry files are required')
    xml_bytes = Path(xml_path).read_bytes()
    if sha(xml_bytes) != policy.TOWN01_MAP_SHA256:
        raise ValueError('reference OpenDRIVE SHA-256 mismatch')
    raw = Path(geometry_path).read_bytes()
    source = json.loads(raw)
    # Accept the prior geometry-only artifact; never consume its review/approval fields.
    data = source.get('geometry_only', source)
    points, poses = data.get('points'), data.get('waypoints')
    if (not isinstance(points, list) or len(points) != c.route_point_count
            or any(not isinstance(p, list) or len(p) != 3
                   or not all(policy.finite(v) for v in p) for p in points)):
        raise ValueError('reference requires 196 finite XYZ coordinates')
    validate_review_geometry(c, data)
    if geometry_sha256(points, poses) != GEOMETRY_SHA256:
        raise ValueError('reference is not the pinned offline Town01 geometry')
    return dict(kind='offline_reference_not_approval', map_name=c.map_name,
                policy_sha256=c.policy_sha256, opendrive_sha256=sha(xml_bytes),
                geometry_sha256=GEOMETRY_SHA256, source_file_sha256=sha(raw),
                points=points, waypoints=poses,
                measured_length_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])))
