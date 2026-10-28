"""Getter-only Town01 audit. No simulation Harness, ownership, cleanup or tick paths."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from idm_brake_config import finite
from idm_scene_bbox import raw_bbox, obstacle_vertices
from idm_brake_route import (ALL_LABELS, GROUND_LABELS, angle, box_vertices,
                             hull, overlaps, ribbons, segment_distance)

ALLOWLIST = {
    'client': ('get_world','get_server_version','get_client_version'),
    'world': ('get_map','get_settings','get_weather','get_actors','get_snapshot',
              'get_level_bbs','get_environment_objects','get_traffic_sign'),
    'map': ('to_opendrive','get_waypoint_xodr','get_all_landmarks'),
    'waypoint': ('next',),
    'traffic': ('get_transform','get_state','get_stop_waypoints','get_affected_lane_waypoints'),
    'prop': ('get_transform',),
    'landmark': ('get_lane_validities',),
}
READONLY_PROPERTIES = {'prop': ('bounding_box',)}
UNKNOWN = ['complete road mesh','all collision shapes','physical material/friction',
           'new vehicle contact/bbox/physics','braking performance/response latency',
           'absence of all external clients/tick owners']
FILES = ('server_status','environment','world_settings','actor_inventory','route_geometry',
         'route_comparison','static_obstacles','static_prop_geometry','regulatory_elements','audit_results','api_call_ledger')

# CARLA 0.9.16 rpc/WeatherParameters.h: numeric state and static presets.
# Keep alphabetical field order (as in scalars), without importing the SDK.
WEATHER_FIELDS = (
    'cloudiness', 'dust_storm', 'fog_density', 'fog_distance', 'fog_falloff',
    'mie_scattering_scale', 'precipitation', 'precipitation_deposits',
    'rayleigh_scattering_scale', 'scattering_intensity', 'sun_altitude_angle',
    'sun_azimuth_angle', 'wetness', 'wind_intensity',
)
WEATHER_PRESETS = frozenset((
    'Default', 'ClearNoon', 'CloudyNoon', 'WetNoon', 'WetCloudyNoon',
    'MidRainyNoon', 'HardRainNoon', 'SoftRainNoon', 'ClearSunset',
    'CloudySunset', 'WetSunset', 'WetCloudySunset', 'MidRainSunset',
    'HardRainSunset', 'SoftRainSunset', 'ClearNight', 'CloudyNight',
    'WetNight', 'WetCloudyNight', 'SoftRainNight', 'MidRainyNight',
    'HardRainNight', 'DustStorm',
))

# Registered 0.9.16 TrafficLightManager speed models. Values are controller km/h.
SPEED_TYPES = {'traffic.speed_limit.'+str(n): n for n in range(30,121,10)}


def speed_state(actor, tf):
    """Local trigger descriptor, never a physical sign-mesh bounding box."""
    require(actor.type_id in SPEED_TYPES,'unknown speed-limit actor type','unknown')
    box = actor.trigger_volume
    state = dict(speed_kmh=SPEED_TYPES[actor.type_id],unit='km/h',
                 value_source='registered actor type suffix; controller runtime value not measured',
                 transform=dict(location=vector(tf.location),rotation=rotation(tf.rotation)),
                 local_trigger=dict(location=vector(box.location),extent=vector(box.extent),rotation=rotation(box.rotation)))
    require(all(v>0 for v in state['local_trigger']['extent']),'invalid speed trigger extent','unknown')
    vertices = [list(p) for p in box_vertices(box,tf)]
    require(all(finite(v) for p in vertices for v in p)
            and len(hull([(p[0],p[1]) for p in vertices]))>=3
            and max(p[2] for p in vertices)>min(p[2] for p in vertices),
            'invalid speed trigger world geometry','unknown')
    state['trigger_vertices'] = vertices
    return state


def utc():
    return datetime.now(timezone.utc).isoformat()


def hash_manifest(directory):
    directory = Path(directory)
    hashes = {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
              for p in directory.iterdir() if p.is_file() and p.name!='artifact_hashes.json'
              and p.suffix in ('.json','.md','.log','.xodr')}
    (directory/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8')


class Evidence:
    def __init__(self, directory, commit, *, synthetic=False):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.meta = dict(source_commit=commit, synthetic=synthetic, created_utc=utc(),
                         schema='town01_readonly_scene_v1')
        self.data = {}
        for name in FILES:
            self.write(name, dict(status='unknown', reason='not acquired'))

    def write(self, name, data):
        self.data[name] = data
        record = dict(self.meta, written_utc=utc(), source=name, data=data)
        payload = json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False)
        temporary = self.directory / (name+'.tmp')
        temporary.write_text(payload+'\n', encoding='utf-8')
        temporary.replace(self.directory / (name+'.json'))

    def report(self):
        result = self.data['audit_results']
        text = ('# Town01 読み取り専用scene監査\n\n'
                f"- モード/状態: {result.get('mode','unknown')} / {result.get('status','unknown')}\n"
                f"- 理由: {result.get('reason','unknown')}\n"
                f"- Source commit: {self.meta['source_commit']}\n"
                f"- Synthetic: {self.meta['synthetic']}\n"
                '- CARLA getterの観測は取得時間窓であり、同一frameの値とは主張しない。\n'
                '- route_audit=null、physical_bounds=null。生成・走行は承認されない。\n\n'
                '未確認: '+', '.join(UNKNOWN)+'\n')
        (self.directory/'report_ja.md').write_text(text, encoding='utf-8')
        hash_manifest(self.directory)


class AuditAbort(Exception):
    def __init__(self, reason, status='fail'):
        super().__init__(reason)
        self.status = status


def require(ok, reason, status='fail'):
    if not ok:
        raise AuditAbort(reason, status)


def vector(p):
    values = [p.x,p.y,p.z]
    require(all(finite(v) for v in values), 'nonfinite position', 'unknown')
    return values


def pose(w):
    r = w.transform.rotation
    data = dict(road=w.road_id, section=w.section_id, lane=w.lane_id, s=w.s,
                yaw=r.yaw, pitch=r.pitch, roll=r.roll, width=w.lane_width)
    require(all(finite(data[k]) for k in ('s','yaw','pitch','roll','width')),
            'nonfinite waypoint pose','unknown')
    return data


def rotation(r):
    data = {k:getattr(r,k) for k in ('yaw','pitch','roll')}
    require(all(finite(v) for v in data.values()),'nonfinite rotation','unknown')
    return data


def scalars(obj):
    """Serialize all public scalar struct fields, not callable methods."""
    out = {}
    for name in dir(obj):
        if name.startswith('_'):
            continue
        value = getattr(obj,name)
        if callable(value):
            continue
        require(value is None or type(value) in (bool,int,float,str),
                'unserializable required struct field: '+name,'unknown')
        require(type(value) is not float or math.isfinite(value), 'nonfinite '+name,'unknown')
        out[name] = value
    require(bool(out), 'empty required struct', 'unknown')
    return out


def weather_scalars(obj):
    """Serialize the complete 0.9.16 weather state, not static preset objects."""
    try:
        public = {name for name in dir(obj) if not name.startswith('_')}
    except Exception as exc:
        raise AuditAbort('weather attributes unavailable', 'unknown') from exc
    unexpected = public - set(WEATHER_FIELDS) - WEATHER_PRESETS
    require(not unexpected, 'unexpected weather attributes: '+', '.join(sorted(unexpected)), 'unknown')
    out = {}
    for name in WEATHER_FIELDS:
        try:
            value = getattr(obj, name)
        except Exception as exc:
            raise AuditAbort('weather field unavailable: '+name, 'unknown') from exc
        require(type(value) in (int, float), 'nonnumeric weather field: '+name, 'unknown')
        try:
            value = float(value)
        except (ValueError, OverflowError) as exc:
            raise AuditAbort('invalid weather number: '+name, 'unknown') from exc
        require(math.isfinite(value), 'nonfinite weather field: '+name, 'unknown')
        out[name] = value
    try:
        instance_fields = vars(obj)
    except TypeError:  # Boost.Python WeatherParameters has no instance __dict__.
        instance_fields = {}
    except Exception as exc:
        raise AuditAbort('weather instance attributes unavailable', 'unknown') from exc
    for name in sorted(public & WEATHER_PRESETS):
        try:
            preset = getattr(obj, name)
            declared = any(name in vars(base) for base in type(obj).__mro__)
        except Exception as exc:
            raise AuditAbort('weather preset unavailable: '+name, 'unknown') from exc
        require(declared and name not in instance_fields and type(preset) is type(obj),
                'invalid weather preset attribute: '+name, 'unknown')
    return out


def snapshot(value):
    t = value.timestamp
    data = dict(frame=value.frame, elapsed_seconds=t.elapsed_seconds,
                delta_seconds=t.delta_seconds, platform_timestamp=t.platform_timestamp)
    require(type(data['frame']) is int and all(finite(data[k]) for k in data if k!='frame'),
            'snapshot unavailable','unknown')
    require(data['frame']>=0 and data['elapsed_seconds']>=0 and data['delta_seconds']>0
            and data['platform_timestamp']>=0,'invalid snapshot timestamp','unknown')
    return data


class Reader:
    """Single gate for every remote/local CARLA getter. Poison is permanent."""
    def __init__(self, evidence, health=None):
        self.evidence, self.health = evidence, health
        self.poisoned = False
        prior = evidence.data.get('api_call_ledger')
        self.ledger = list(prior) if isinstance(prior,list) else []

    def call(self, kind, obj, method, *args):
        require(not self.poisoned, 'additional API call prohibited after failure')
        require(method in ALLOWLIST.get(kind,()), 'API outside read-only allowlist: '+kind+'.'+method)
        return self._observe(kind+'.'+method, lambda: getattr(obj,method)(*args), args)

    def attribute(self, kind, obj, name):
        # Actor.bounding_box is a property backed by an SDK getter, not plain metadata.
        require(not self.poisoned, 'additional API call prohibited after failure')
        require(name in READONLY_PROPERTIES.get(kind,()), 'property outside read-only allowlist')
        return self._observe(kind+'.'+name, lambda: getattr(obj,name), (), 'property')

    def _observe(self, api, fn, args, access='method'):
        item = dict(sequence=len(self.ledger), api=api, access=access,
                    args=[str(v) for v in args], started_utc=utc(), status='requested')
        self.ledger.append(item)
        self.evidence.write('api_call_ledger', self.ledger)
        try:
            result = self.health.call(fn) if self.health else fn()
        except BaseException as exc:
            self.poisoned = True
            item.update(status='failed', ended_utc=utc(), reason=type(exc).__name__+': '+str(exc))
            self.evidence.write('api_call_ledger', self.ledger)
            raise AuditAbort('required getter failed; no further RPC: '+item['api'], 'unknown') from exc
        item.update(status='returned', ended_utc=utc())
        self.evidence.write('api_call_ledger', self.ledger)
        return result


class SceneAudit:
    def __init__(self, client, api, config, reference, evidence, health=None):
        self.client,self.api,self.c,self.ref,self.e = client,api,config,reference,evidence
        self.reader = Reader(evidence,health)
        self.call = self.reader.call
        self.started = utc()

    def run(self):
        try:
            self.inspect()
            result = dict(status='pass', reason='required getter observations coherent in acquisition window')
        except Exception as exc:
            # No final getter/cleanup/reconnect even for a semantic failure.
            result = dict(status=getattr(exc,'status','unknown'), reason=str(exc), exception=type(exc).__name__)
        result.update(mode='connect-readonly', acquisition_started_utc=self.started,
                      acquisition_ended_utc=utc(), route_audit=None, physical_bounds=None,
                      execute_allowed=False, unknown=UNKNOWN)
        self.e.write('audit_results',result)
        self.e.report()
        return result

    def actors(self, world):
        actors = list(self.call('world',world,'get_actors'))
        inventory = sorted((dict(id=a.id,type_id=a.type_id,attributes=dict(a.attributes)) for a in actors),key=lambda a:a['id'])
        self.e.write('actor_inventory',dict(status='pass', observed=inventory))
        require(not any(a.type_id.startswith(('vehicle.','walker.','sensor.','controller.')) for a in actors),
                'unexpected vehicle/walker/sensor/controller')
        require(all(a.type_id in ('spectator','static.prop.mesh') or a.type_id.startswith('traffic.') for a in actors),
                'unreviewed actor type; obstacle inventory cannot certify it','unknown')
        return actors, inventory

    def prop_geometry(self, actors, phase):
        """Acquire only mesh props, with partial evidence and no getter after failure."""
        previous = self.e.data.get('static_prop_geometry', {})
        records = []
        def save(status='unknown'):
            self.e.write('static_prop_geometry',dict(previous, status=status, **{phase:records}))
        for a in sorted((a for a in actors if a.type_id=='static.prop.mesh'),key=lambda a:a.id):
            record = dict(id=a.id,type_id=a.type_id,status='UNKNOWN',started_utc=utc())
            records.append(record);save()
            try:
                attributes = dict(a.attributes)
                require(type(a.id) is int and a.id>0 and all(type(k) is str and type(v) is str
                        for k,v in attributes.items()),'invalid mesh actor metadata','unknown')
                path = attributes.get('mesh_path')
                require(type(path) is str and bool(path.strip()) and '\x00' not in path,
                        'missing/invalid mesh_path metadata','unknown')
                record['attributes'] = attributes  # Names never certify geometry or physics.
                tf = self.call('prop',a,'get_transform')
                record['transform'] = dict(location=vector(tf.location),rotation=rotation(tf.rotation))
                save()
                box = self.reader.attribute('prop',a,'bounding_box')
                record['local_bbox'] = dict(location=vector(box.location),extent=vector(box.extent),
                                           rotation=rotation(box.rotation))
                require(all(v>0 for v in record['local_bbox']['extent']),
                        'nonpositive mesh bbox extent','unknown')
                vertices = [list(p) for p in box_vertices(box,tf)]
                require(all(finite(v) for p in vertices for v in p),
                        'nonfinite mesh world vertices','unknown')
                require(len(hull([(p[0],p[1]) for p in vertices]))>=3
                        and max(p[2] for p in vertices)>min(p[2] for p in vertices),
                        'degenerate mesh world vertices','unknown')
                record.update(world_vertices=vertices,geometry_valid=True,ended_utc=utc())
                save()
            except Exception as exc:
                record.update(status='UNKNOWN',reason=type(exc).__name__+': '+str(exc),ended_utc=utc())
                save()
                raise AuditAbort('mesh geometry unavailable: '+str(a.id),'unknown') from exc
        save()
        return records

    def prop_gate(self, records, phase, points, corridor):
        """Reuse the full existing obstacle corridor and inclusive Z screen."""
        data = self.e.data['static_prop_geometry']
        for record in records:
            vertices = record['world_vertices']
            poly = hull([(p[0],p[1]) for p in vertices])
            xy = any(overlaps(poly,r) for r in corridor)
            zmin,zmax = min(p[2] for p in vertices),max(p[2] for p in vertices)
            z = zmin<=max(p[2] for p in points)+2.1 and zmax>=min(p[2] for p in points)-.1
            record.update(xy_overlap=xy,z_overlap=z,z_min=zmin,z_max=zmax,
                          status='INTERSECT' if xy and z else 'CLEAR')
            self.e.write('static_prop_geometry',dict(data,**{phase:records},status='fail' if xy and z else 'unknown'))
            require(not (xy and z),'mesh bbox intersects route: '+str(record['id']))

    @staticmethod
    def prop_state(records):
        # Exact endpoint comparison: no movement/extent/rotation tolerance is introduced.
        return [{k:r[k] for k in ('id','type_id','attributes','transform','local_bbox','world_vertices')}
                for r in records]

    def inspect(self):
        server = self.call('client',self.client,'get_server_version')
        client = self.call('client',self.client,'get_client_version')
        self.e.write('environment',dict(server_version=server,client_version=client))
        require(server=='0.9.16' and client=='0.9.16','CARLA version mismatch')
        world = self.call('client',self.client,'get_world')
        episode = world.id
        m = self.call('world',world,'get_map')
        require(m.name.rsplit('/',1)[-1]==self.c.map_name,'map mismatch')
        xml = self.call('map',m,'to_opendrive')
        (self.e.directory/'observed_opendrive.xodr').write_text(xml,encoding='utf-8',newline='')
        digest = hashlib.sha256(xml.encode()).hexdigest()
        self.e.write('route_comparison',dict(status='unknown',observed_opendrive_sha256=digest,
                                           expected_opendrive_sha256=self.ref['opendrive_sha256']))
        require(digest==self.ref['opendrive_sha256'],'live OpenDRIVE hash mismatch')
        settings = scalars(self.call('world',world,'get_settings'))
        self.e.write('world_settings',dict(status='pass',before=settings))
        require(settings.get('synchronous_mode') is False,'existing synchronous world or missing mode')
        start = snapshot(self.call('world',world,'get_snapshot'))
        weather = weather_scalars(self.call('world',world,'get_weather'))
        self.e.write('environment',dict(status='pass',server_version=server,client_version=client,
                                       map_name=m.name,episode=episode,weather=weather,snapshot_start=start))
        actors, inventory = self.actors(world)
        props_before = self.prop_geometry(actors,'before')
        route, points, poses = [], [], []
        for i,s in enumerate(range(250,54,-1)):
            w = self.call('map',m,'get_waypoint_xodr',8,1,float(s))
            require(w is not None,'missing waypoint','unknown')
            p, q = vector(w.transform.location), pose(w)
            route.append(w);points.append(p);poses.append(q)
            self.e.write('route_geometry',dict(status='unknown',reason='acquisition in progress',
                                               observed_points=points,observed_waypoints=poses))
            require((w.road_id,w.section_id,w.lane_id)==(8,0,1) and not w.is_junction
                    and w.lane_type==self.api.LaneType.Driving,'lane/key/junction mismatch')
            ref = self.ref['waypoints'][i]
            require(math.dist(p,self.ref['points'][i])<=.01 and abs(q['s']-s)<=1e-4
                    and abs(q['width']-4)<=1e-6
                    and all(abs(angle(q[k],ref[k]))<=.01 for k in ('yaw','pitch','roll')),
                    'live waypoint/reference mismatch')
            if i:
                choices = list(self.call('waypoint',route[-2],'next',1.0))
                require(len(choices)==1 and (choices[0].road_id,choices[0].section_id,choices[0].lane_id)==(8,0,1)
                        and abs(choices[0].s-s)<=.02
                        and math.dist(vector(choices[0].transform.location),p)<=.02,
                        'waypoint discontinuity/branch')
        length = sum(math.dist(a,b) for a,b in zip(points,points[1:]))
        require(abs(length-self.ref['measured_length_m'])<=.01,'measured route length mismatch')
        self.e.write('route_geometry',dict(status='pass',observed_points=points,observed_waypoints=poses,
                                          measured_length_m=length,point_count=len(points)))
        self.e.write('route_comparison',dict(status='pass',opendrive_sha256=digest,
                    reference_geometry_sha256=self.ref['geometry_sha256'],point_count=196,
                    position_tolerance_m=.01,angle_tolerance_deg=.01,length_tolerance_m=.01))
        corridor = ribbons(route,self.c.front_extent_limit)
        self.prop_gate(props_before,'before',points,corridor)
        traffic_before = self.regulatory(world,m,xml,actors,points,corridor)
        self.obstacles(world,points,corridor)
        # Bracket the acquisition. Async forward progress is expected, not an extra tick.
        end_world = self.call('client',self.client,'get_world')
        require(end_world.id==episode,'world/episode changed during audit')
        end_map = self.call('world',end_world,'get_map')
        require(end_map.name==m.name and hashlib.sha256(self.call('map',end_map,'to_opendrive').encode()).hexdigest()==digest,
                'map changed during audit')
        after = scalars(self.call('world',end_world,'get_settings'))
        self.e.write('world_settings',dict(status='pass' if settings==after else 'fail',before=settings,after=after))
        require(settings==after,'WorldSettings changed during audit')
        end_actors = list(self.call('world',end_world,'get_actors'))
        end_inventory = sorted((dict(id=a.id,type_id=a.type_id,attributes=dict(a.attributes)) for a in end_actors),key=lambda a:a['id'])
        self.e.write('actor_inventory',dict(status='pass' if inventory==end_inventory else 'fail',
                                           before=inventory,after=end_inventory))
        require(inventory==end_inventory,'actor inventory changed during audit')
        props_after = self.prop_geometry(end_actors,'after')
        self.prop_gate(props_after,'after',points,corridor)
        same_props = self.prop_state(props_before)==self.prop_state(props_after)
        self.e.write('static_prop_geometry',dict(self.e.data['static_prop_geometry'],
                     status='pass' if same_props else 'fail',endpoint_state_equal=same_props,
                     transient_changes_between_endpoints='unknown',collision_complete='unknown'))
        require(same_props,'mesh actor geometry changed during audit')
        traffic_after = {}
        for a in end_actors:
            if a.type_id.startswith('traffic.'):
                tf = self.call('traffic',a,'get_transform')
                record = dict(position=vector(tf.location),rotation=scalars(tf.rotation))
                if a.type_id.startswith('traffic.traffic_light'):
                    record['state'] = str(self.call('traffic',a,'get_state'))
                if a.type_id.startswith('traffic.speed_limit.'):
                    record['speed_limit'] = speed_state(a,tf)
                traffic_after[str(a.id)] = record
        self.e.write('traffic_window',dict(before=traffic_before,after=traffic_after,
                     status='pass' if traffic_before==traffic_after else 'fail'))
        require(traffic_before==traffic_after,'traffic actor state changed during audit')
        end_weather = weather_scalars(self.call('world',end_world,'get_weather'))
        require(weather==end_weather,'weather changed during audit')
        end = snapshot(self.call('world',end_world,'get_snapshot'))
        df,dt = end['frame']-start['frame'],end['elapsed_seconds']-start['elapsed_seconds']
        require(df>=0 and dt>=0 and ((df==0 and dt==0) or (df>0 and dt>0))
                and end['platform_timestamp']>=start['platform_timestamp'],
                'nonmonotonic/inconsistent snapshot window')
        self.e.write('environment',dict(status='pass',server_version=server,client_version=client,
            map_name=m.name,episode=episode,weather_before=weather,weather_after=end_weather,
            snapshot_start=start,snapshot_end=end,single_frame_claim=False,
            note='No guarantee against transient changes between endpoint observations'))

    def regulatory(self,world,m,xml,actors,points,corridor):
        road = ET.fromstring(xml).find("./road[@id='8']")
        require(road is not None,'route road absent from XML')
        records = dict(xml=[],landmarks=[],traffic=[])
        traffic_before = {}
        speed_marks = []
        reference_cache = {}
        def save(): self.e.write('regulatory_elements',dict(status='unknown',observed=records))
        for path in ('signals/*','objects/*'):
            for item in road.findall(path):
                require('s' in item.attrib,'XML regulatory s unavailable','unknown')
                s = float(item.attrib['s'])
                require(finite(s),'XML regulatory s nonfinite','unknown')
                near = 35<=s<=270
                records['xml'].append(dict(tag=item.tag,attributes=item.attrib,near_route=near));save()
                require(not near,'XML regulatory/object near route')
        for mark in self.call('map',m,'get_all_landmarks'):
            p = vector(mark.transform.location)
            distance = min(segment_distance(p,a,b) for a,b in zip(points,points[1:]))
            records['landmarks'].append(dict(id=str(mark.id),position=p,distance_xy_m=distance));save()
            require(distance>20,'landmark near route')
            if getattr(mark,'type',None)=='274':speed_marks.append(mark)
        for a in actors:
            if not a.type_id.startswith('traffic.'):
                continue
            require(a.type_id in SPEED_TYPES or a.type_id in ('traffic.stop','traffic.yield','traffic.traffic_light'),
                    'unknown traffic regulation actor type','unknown')
            tf = self.call('traffic',a,'get_transform')
            traffic_before[str(a.id)] = dict(position=vector(tf.location),rotation=scalars(tf.rotation))
            speed = speed_state(a,tf) if a.type_id in SPEED_TYPES else None
            vertices = speed['trigger_vertices'] if speed is not None else box_vertices(a.trigger_volume,tf)
            polygon = hull([(p[0],p[1]) for p in vertices])
            record = dict(id=a.id,type_id=a.type_id,trigger_vertices=vertices,
                          z_min=min(p[2] for p in vertices),z_max=max(p[2] for p in vertices))
            records['traffic'].append(record);save()
            intersects = any(overlaps(polygon,r) for r in corridor)
            if a.type_id in SPEED_TYPES:
                state = speed
                traffic_before[str(a.id)]['speed_limit'] = state
                record.update(classification='speed_limit_application_region',speed_limit=state,
                              route_xy_overlap=intersects,physical_shape='not this trigger; Level BB/environment audit required')
                save()
                if intersects:
                    try:
                        record['reference'] = self.speed_reference(world,a,road,speed_marks,reference_cache,record)
                        record['assessment'] = 'validated_speed_regulation'
                    except Exception as exc:
                        record.update(assessment=getattr(exc,'status','unknown'),reason=str(exc));save()
                        raise
                else:record['assessment'] = 'outside_audit_corridor'
                save()
            else:
                require(not intersects,'traffic trigger overlaps route')
            if a.type_id.startswith('traffic.traffic_light'):
                record['state'] = str(self.call('traffic',a,'get_state'))
                traffic_before[str(a.id)]['state'] = record['state']
                for method in ('get_stop_waypoints','get_affected_lane_waypoints'):
                    values = self.call('traffic',a,method)
                    record[method] = []
                    for w in values:
                        p = vector(w.transform.location)
                        distance = min(segment_distance(p,u,v) for u,v in zip(points,points[1:]))
                        record[method].append(dict(position=p,pose=pose(w),distance_xy_m=distance));save()
                        require(distance>20,'traffic stop/affected waypoint near route')
        self.e.write('regulatory_elements',dict(status='pass',observed=records))
        return traffic_before

    def speed_reference(self,world,actor,road,marks,cache,record):
        """Associate via the SDK sign-id lookup, not guessed actor/sign proximity."""
        matches = []
        for mark in marks:
            if mark.road_id!=self.c.road:continue
            key = str(mark.id)
            if key not in cache:
                ranges = list(self.call('landmark',mark,'get_lane_validities'))
                require(bool(ranges) and all(isinstance(v,(tuple,list)) and len(v)==2
                        and all(type(x) is int for x in v) and v[0]<=v[1]
                        and not (v[0]<=0<=v[1]) for v in ranges),'invalid speed lane validities','unknown')
                target = self.call('world',world,'get_traffic_sign',mark) if any(lo<=self.c.lane<=hi for lo,hi in ranges) else None
                cache[key] = (mark,ranges,target.id if target is not None else None,target.type_id if target is not None else None)
            item,ranges,actor_id,type_id = cache[key]
            if actor_id==actor.id:
                require(type_id==actor.type_id,'speed reference actor type changed')
                matches.append((item,ranges))
        require(len(matches)==1,'speed sign reference missing or ambiguous','unknown')
        mark,ranges = matches[0]
        require(finite(mark.s) and mark.s>=0 and finite(mark.value) and mark.value>0,'invalid speed landmark value/position','unknown')
        length = float(road.get('length','nan'))
        require(finite(length) and mark.s<=length,'speed reference outside XML road','unknown')
        require(mark.s>=self.c.follower_s,'speed applicability not upstream of decreasing-s route','unknown')
        units = {'km/h':1.0,'mph':1.609344,'m/s':3.6}
        require(mark.unit in units,'unknown speed landmark unit','unknown')
        orientation = str(mark.orientation).rsplit('.',1)[-1]
        require(orientation in ('Positive','Negative','Both'),'unknown speed landmark orientation','unknown')
        kmh = mark.value*units[mark.unit]
        require(math.isfinite(kmh),'nonfinite normalized speed limit','unknown')
        reference = dict(landmark_id=str(mark.id),actor_id=actor.id,road_id=mark.road_id,s=mark.s,
                         value=mark.value,unit=mark.unit,limit_kmh=kmh,orientation=orientation,
                         lane_validities=ranges,position=vector(mark.transform.location))
        record['reference'] = reference  # Retain valid diagnostic values even on semantic mismatch.
        nodes = [n for path in ('signals/signal','objects/object') for n in road.findall(path) if n.get('id')==str(mark.id)]
        require(len(nodes)==1,'speed landmark/XML reference missing or ambiguous','unknown')
        node = nodes[0]
        reference['xml_source'] = dict(kind=node.tag,attributes=dict(node.attrib))
        comparison_kmh = kmh
        interpretation = 'ordinary_unit_conversion'
        require(finite(float(node.get('s','nan'))) and abs(float(node.get('s'))-mark.s)<=1e-4,
                'speed landmark/XML position mismatch')
        require(node.get('orientation') in ('+','-','none') and
                orientation=={'+':'Positive','-':'Negative','none':'Both'}[node.get('orientation')],
                'speed landmark/XML orientation mismatch')
        if node.tag=='signal':
            require(node.get('type')=='274' and node.get('unit')==mark.unit
                    and finite(float(node.get('value','nan'))) and float(node.get('value'))==mark.value,
                    'speed landmark/XML type/value/unit mismatch')
            declared = [(int(v.get('fromLane')),int(v.get('toLane'))) for v in node.findall('validity')]
            require(bool(declared) and sorted(declared)==sorted(tuple(v) for v in ranges),'speed XML lane validity mismatch')
        else:
            # 0.9.16 ObjectParser inserts mph metadata without converting Speed_*.
            # The generated SpeedLimitComponent passes that raw value to the
            # km/h Controller contract. This is source-derived, NOT a measured
            # Controller value or proof of this actor's component/spawn history.
            names = ('Speed_'+str(SPEED_TYPES[actor.type_id]),'speed_'+str(SPEED_TYPES[actor.type_id]))
            require(node.get('name') in names and mark.value==SPEED_TYPES[actor.type_id] and mark.unit=='mph',
                    'speed object/landmark value/unit mismatch')
            require(node.get('type')=='-1' and 'unit' not in node.attrib and 'value' not in node.attrib
                    and not node.findall('validity'),
                    'unreviewed speed object provenance or explicit speed metadata','unknown')
            require(mark.name==node.get('name') and mark.sub_type==str(SPEED_TYPES[actor.type_id])
                    and mark.country=='OpenDRIVE' and mark.is_dynamic is False,
                    'speed object/parser metadata mismatch','unknown')
            reference['parser_metadata'] = dict(name=mark.name,sub_type=mark.sub_type,
                                               country=mark.country,is_dynamic=mark.is_dynamic)
            comparison_kmh = mark.value
            interpretation = 'carla_0.9.16_roadrunner_object_raw_value_kmh_contract'
        reference['internal_interpretation'] = dict(
            value=comparison_kmh,unit='km/h',basis=interpretation,
            confidence=('source_derived_expected_contract' if node.tag=='object' else
                        'declared_unit_normalization_not_runtime_measurement'),measured_controller_value=None,
            actual_component_and_spawn_path='unknown',
            source_version='0.9.16',
            source_path=('ObjectParser -> TrafficLightManager generated speed sign -> '
                         'SpeedLimitComponent -> WheeledVehicleAIController') if node.tag=='object' else
                        'XML signal / Landmark declared unit normalization')
        # Preserve limit_kmh as declared-unit conversion (e.g. 90 mph=144.84096).
        # Only the separately documented object contract selects a different
        # comparison value; ordinary signals retain their full unit conversion.
        require(math.isclose(comparison_kmh,SPEED_TYPES[actor.type_id],rel_tol=0,abs_tol=1e-6),
                'speed actor/landmark unit-normalized limit mismatch')
        require(self.c.speed_limit*3.6<=min(kmh,comparison_kmh),'calibration speed cap exceeds speed regulation')
        return dict(reference,
                    route_lane=self.c.lane,route_s_direction='decreasing',
                    application='conservatively applies wherever validated trigger overlaps corridor',
                    exact_effect_region='unknown; descriptor may combine multiple triggers',
                    calibration_speed_cap_mps=self.c.speed_limit)

    def obstacles(self,world,points,corridor):
        counts,records = {},[]
        def save(status='unknown'):
            self.e.write('static_obstacles',dict(status=status,counts=counts,observed=records,
                         corridor='per-segment full lane width with conservative end caps',
                         z_screen_m=[min(p[2] for p in points)-.1,max(p[2] for p in points)+2.1],
                         mesh_complete='unknown',collision_complete='unknown'))
        def inspect(box,label,source,object_id=None,index=None):
            descriptor = raw_bbox(box)
            raw_record = dict(source=source,label=label,id=object_id,index=index,descriptor=descriptor)
            # Save raw attributes before any geometric validation, including NaN
            # as a typed token. Partial evidence survives a local/native failure.
            with (self.e.directory/'obstacle_bbox_raw.jsonl').open('a',encoding='utf-8') as raw:
                raw.write(json.dumps(raw_record,allow_nan=False)+'\n')
            try:
                vertices,interpretation = obstacle_vertices(descriptor,source)
            except Exception as exc:
                records.append(dict(raw_record,status='UNKNOWN',reason=str(exc)))
                save()
                raise
            polygon = hull([(p[0],p[1]) for p in vertices])
            zmin,zmax = min(p[2] for p in vertices),max(p[2] for p in vertices)
            xy = any(overlaps(polygon,r) for r in corridor)
            z = zmin<=max(p[2] for p in points)+2.1 and zmax>=min(p[2] for p in points)-.1
            record = dict(source=source,label=label,id=object_id,index=index,vertices=vertices,
                          raw_descriptor=descriptor,extent_interpretation=interpretation,
                          z_min=zmin,z_max=zmax,xy_overlap=xy,z_overlap=z,
                          ground_descriptor=label in GROUND_LABELS)
            records.append(record)
            if xy and z and label not in GROUND_LABELS:
                save('fail')
                raise AuditAbort('static obstacle overlaps route')
        for label in ALL_LABELS:
            boxes = list(self.call('world',world,'get_level_bbs',getattr(self.api.CityObjectLabel,label)))
            counts[label] = len(boxes)
            for i,b in enumerate(boxes): inspect(b,label,'level_bbs',index=i)
            save()
        objects = list(self.call('world',world,'get_environment_objects',self.api.CityObjectLabel.Any))
        counts['environment_Any'] = len(objects)
        for i,obj in enumerate(objects):
            label = str(obj.type).rsplit('.',1)[-1]
            inspect(obj.bounding_box,label,'environment_objects',obj.id,index=i)
        save('pass')
