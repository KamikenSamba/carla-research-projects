"""Town01-only, server-free geometry plus read-only scene checks. No tick/control APIs."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from idm_brake_config import finite

# Ground categories are counted; their descriptors never certify drivable mesh.
GROUND_LABELS = ('Roads', 'RoadLines', 'Ground', 'Terrain', 'Sky')
OBSTACLE_LABELS = ('Buildings', 'Fences', 'Other', 'Pedestrians', 'Poles', 'Sidewalks',
    'TrafficSigns', 'Vegetation', 'Car', 'Walls', 'Bridge', 'RailTrack', 'GuardRail',
    'TrafficLight', 'Static', 'Dynamic', 'Water', 'Truck', 'Motorcycle', 'Bicycle',
    'Bus', 'Rider', 'Train')
ALL_LABELS = GROUND_LABELS + OBSTACLE_LABELS


def angle(a,b):
    return (a-b+180)%360-180


def geometry_sha256(points,waypoints):
    return hashlib.sha256(json.dumps(dict(points=points,waypoints=waypoints),
                                    sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_review_geometry(c,a):
    metadata=a.get('waypoints')
    if not isinstance(metadata,list) or len(metadata)!=c.route_point_count:
        raise ValueError('Town01 review requires every waypoint pose')
    for s,p in zip(range(c.follower_s,c.end_s-1,-1),metadata):
        if (not isinstance(p,dict) or any(not finite(p.get(k)) for k in
                ('s','yaw','pitch','roll','width')) or p['s']!=s
                or any(type(p.get(k)) is not int or p[k]!=v for k,v in
                       (('road',c.road),('section',c.section),('lane',c.lane)))
                or abs(p['width']-4)>1e-6
                or abs(angle(p['pitch'],0))>.1 or abs(angle(p['roll'],0))>.1):
            raise ValueError('Town01 reviewed waypoint pose/key/width mismatch')
    points=a['points']
    length=sum(math.dist(p,q) for p,q in zip(points,points[1:]))
    if abs(length-c.planned_route_length)>.01:
        raise ValueError('Town01 reviewed coordinate distance mismatch')
    if (max(p[2] for p in points)-min(p[2] for p in points)>.02
            or any(abs(angle(p['yaw'],metadata[0]['yaw']))>.5 for p in metadata)):
        raise ValueError('Town01 reviewed height/heading change')
    for i,(p,q) in enumerate(zip(points,points[1:])):
        heading=math.degrees(math.atan2(q[1]-p[1],q[0]-p[0]))
        if abs(angle(heading,metadata[i]['yaw']))>.5:
            raise ValueError('Town01 reviewed direction inconsistent with coordinates')
    if a.get('geometry_sha256')!=geometry_sha256(points,metadata):
        raise ValueError('Town01 geometry hash mismatch')


def rotate(p,r):
    yaw,pitch,roll=(math.radians(getattr(r,k)) for k in ('yaw','pitch','roll'))
    cy,sy,cp,sp,cr,sr=math.cos(yaw),math.sin(yaw),math.cos(pitch),math.sin(pitch),math.cos(roll),math.sin(roll)
    x,y,z=p
    return (cp*cy*x+(cy*sp*sr-sy*cr)*y+(-cy*sp*cr-sy*sr)*z,
            cp*sy*x+(sy*sp*sr+cy*cr)*y+(-sy*sp*cr+cy*sr)*z,
            sp*x-cp*sr*y+cp*cr*z)


def box_vertices(box,transform=None):
    numbers=[getattr(getattr(box,k,None),axis,None)
             for k in ('location','extent') for axis in ('x','y','z')]
    rotations=[getattr(getattr(box,'rotation',None),k,None) for k in ('yaw','pitch','roll')]
    if (not all(finite(x) for x in numbers+rotations)
            or any(getattr(box.extent,k)<0 for k in ('x','y','z'))):
        raise ValueError('unknown bounding box')
    if transform is not None:
        if not all(finite(getattr(obj,k,None)) for obj,keys in
            ((transform.location,('x','y','z')),(transform.rotation,('yaw','pitch','roll'))) for k in keys):
            raise ValueError('unknown bounding box transform')
    out=[]
    for sx in (-1,1):
        for sy in (-1,1):
            for sz in (-1,1):
                v=rotate((sx*box.extent.x,sy*box.extent.y,sz*box.extent.z),box.rotation)
                v=tuple(v[i]+getattr(box.location,k) for i,k in enumerate(('x','y','z')))
                if transform is not None:
                    v=rotate(v,transform.rotation)
                    v=tuple(v[i]+getattr(transform.location,k) for i,k in enumerate(('x','y','z')))
                out.append(v)
    return out


def hull(points):
    points=sorted(set(points))
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    halves=[]
    for seq in (points,list(reversed(points))):
        out=[]
        for p in seq:
            while len(out)>1 and cross(out[-2],out[-1],p)<=0:out.pop()
            out.append(p)
        halves.append(out)
    return halves[0][:-1]+halves[1][:-1] if len(points)>1 else points


def overlaps(a,b):
    if not a or not b:return False
    axes=[(1,0),(0,1)]
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            if p!=q:axes.append((-(q[1]-p[1]),q[0]-p[0]))
    for ux,uy in axes:
        av=[x*ux+y*uy for x,y in a];bv=[x*ux+y*uy for x,y in b]
        if max(av)<min(bv)-1e-9 or max(bv)<min(av)-1e-9:return False
    return True


def ribbons(route,padding):
    pieces=[]
    for a,b in zip(route,route[1:]):
        p,q=a.transform.location,b.transform.location
        length=math.hypot(q.x-p.x,q.y-p.y)
        if not finite(length) or length<1e-6:raise ValueError('degenerate route segment')
        ux,uy=(q.x-p.x)/length,(q.y-p.y)/length
        width=max(a.lane_width,b.lane_width)/2
        # Square end caps conservatively cover joins and the initial/rear vehicle extent.
        cap=max(width,padding)
        pieces.append([(p.x-ux*cap-uy*width,p.y-uy*cap+ux*width),
                       (q.x+ux*cap-uy*width,q.y+uy*cap+ux*width),
                       (q.x+ux*cap+uy*width,q.y+uy*cap-ux*width),
                       (p.x-ux*cap+uy*width,p.y-uy*cap-ux*width)])
    return pieces


def segment_distance(point,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];square=dx*dx+dy*dy
    t=max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/square))
    return math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)


def inspect_town01(h):
    """Called before any settings/spawn; getters only. Gate handles RPC failures."""
    from idm_experiment_harness import save
    c=h.c
    live_points=[(w.transform.location.x,w.transform.location.y,w.transform.location.z) for w in h.route]
    h.fail_if(any(not all(finite(v) for v in p) for p in live_points),'live_route_geometry_unknown')
    h.fail_if(any(not .95<=math.dist(p,q)<=1.05 for p,q in zip(live_points,live_points[1:])),
              'live_route_spacing')
    for w,pose in zip(h.route,c.route_audit['waypoints']):
        r=w.transform.rotation
        h.fail_if(not finite(w.s) or abs(w.s-pose['s'])>1e-4
            or not finite(w.lane_width) or abs(w.lane_width-4)>1e-6 or any(
            not finite(getattr(r,k,None)) or abs(angle(getattr(r,k),pose[k]))>.01
            for k in ('yaw','pitch','roll')),'reviewed_waypoint_pose_mismatch')
    h.fail_if(max(w.transform.location.z for w in h.route)-min(w.transform.location.z for w in h.route)>.02,
              'route_height_span')
    root=ET.fromstring(h.opendrive_text)
    road=root.find("./road[@id='8']")
    h.fail_if(road is None,'route_xml_missing')
    regulatory=[dict(x.attrib) for path in ('signals/*','objects/*') for x in road.findall(path)
                if 's' in x.attrib and c.end_s-20<=float(x.get('s'))<=c.follower_s+20]
    h.fail_if(bool(regulatory),'regulatory_xml_near_route')
    h.fail_if(not hasattr(h.map,'get_all_landmarks'),'regulatory_inventory_unavailable')
    points=[(w.transform.location.x,w.transform.location.y) for w in h.route]
    for mark in h.call(h.map.get_all_landmarks):
        p=mark.transform.location
        h.fail_if(min(segment_distance((p.x,p.y),a,b) for a,b in zip(points,points[1:]))<=20,
                  'regulatory_landmark_near_route')
    corridor=ribbons(h.route,c.front_extent_limit)
    bounds=(min(p[0] for r in corridor for p in r),max(p[0] for r in corridor for p in r),
            min(p[1] for r in corridor for p in r),max(p[1] for r in corridor for p in r))
    h.static_route_boxes=[]
    counts={}
    # Preserve descriptors of ground categories as evidence, not a collision-free certification.
    def inspect(box,label):
        vertices=box_vertices(box)
        if (max(p[0] for p in vertices)<bounds[0] or min(p[0] for p in vertices)>bounds[1]
                or max(p[1] for p in vertices)<bounds[2] or min(p[1] for p in vertices)>bounds[3]):
            return
        poly=hull([(p[0],p[1]) for p in vertices])
        record=dict(label=label,polygon=poly,z_min=min(p[2] for p in vertices),z_max=max(p[2] for p in vertices))
        if label not in GROUND_LABELS:
            h.static_route_boxes.append(record)
            if record['z_min']<=2.1 and record['z_max']>=-.1:
                h.fail_if(any(overlaps(poly,r) for r in corridor),'static_route_obstacle')
    for label in ALL_LABELS:
        h.fail_if(not hasattr(h.api.CityObjectLabel,label),'obstacle_inventory_unavailable')
        boxes=h.call(h.world.get_level_bbs,getattr(h.api.CityObjectLabel,label))
        counts[label]=len(boxes)
        for box in boxes:inspect(box,label)
    h.fail_if(not hasattr(h.world,'get_environment_objects') or not hasattr(h.api.CityObjectLabel,'NONE'),
              'unlabelled_obstacle_inventory_unavailable')
    unknown=h.call(h.world.get_environment_objects,h.api.CityObjectLabel.NONE)
    counts['NONE']=len(unknown)
    for obj in unknown:inspect(obj.bounding_box,'NONE')
    for actor in h.preflight_actors:
        if not actor.type_id.startswith('traffic.'):continue
        h.fail_if(not hasattr(actor,'trigger_volume') or not hasattr(actor,'get_transform'),
                  'regulatory_trigger_unknown',actor.id)
        vertices=box_vertices(actor.trigger_volume,h.call(actor.get_transform))
        h.fail_if(any(overlaps(hull([(p[0],p[1]) for p in vertices]),r) for r in corridor),
                  'regulatory_trigger_near_route',actor.id)
        if actor.type_id.startswith('traffic.traffic_light'):
            for method in ('get_stop_waypoints','get_affected_lane_waypoints'):
                h.fail_if(not hasattr(actor,method),'regulatory_waypoints_unknown',actor.id)
                for w in h.call(getattr(actor,method)):
                    p=w.transform.location
                    h.fail_if(min(segment_distance((p.x,p.y),a,b) for a,b in zip(points,points[1:]))<=20,
                              'regulatory_stop_waypoint_near_route',actor.id)
    save(h.directory/'route_preflight.json',dict(mode=c.mode,point_count=len(h.route),
        measured_route_length_m=h.route_length,geometry_sha256=c.route_audit['geometry_sha256'],
        opendrive_sha256=c.route_audit['opendrive_sha256'],semantic_box_counts=counts,
        corridor='per-segment full lane width plus conservative end caps / initial rear extent',
        mesh_verified=False,collision_shapes_complete=False,surface_material=None,physical_bounds=None))


def inspect_spawn_bbox(h,actor):
    """Only the new owned actor; no tick/control, actual bbox before physics settling."""
    from idm_experiment_harness import save
    h.fail_if(any(not finite(getattr(actor.bounding_box.extent,k,None)) or
                  getattr(actor.bounding_box.extent,k)<=0 for k in ('x','y','z')),
              'spawn_bbox_unknown',actor.id)
    tf=h.call(actor.get_transform)
    vertices=box_vertices(actor.bounding_box,tf)
    poly=hull([(p[0],p[1]) for p in vertices])
    origin=h.route[0].transform
    h.fail_if(math.hypot(tf.location.x-origin.location.x,tf.location.y-origin.location.y)>.1
              or abs(angle(tf.rotation.yaw,origin.rotation.yaw))>.5,'spawn_pose_mismatch',actor.id)
    # Initial body must fit in this same lane; positions are queried, never moved.
    for x,y,z in vertices:
        location=h.api.Location(x=x,y=y,z=h.route[0].transform.location.z)
        w=h.map.get_waypoint(location,project_to_road=True,lane_type=h.api.LaneType.Driving)
        h.fail_if(w is None or (w.road_id,w.section_id,w.lane_id)!=(h.c.road,h.c.section,h.c.lane)
                  or w.is_junction,'spawn_bbox_lane_occupancy',actor.id)
        p=w.transform.location
        h.fail_if(math.hypot(x-p.x,y-p.y)>=w.lane_width/2,'spawn_bbox_outside_lane',actor.id)
    zlo,zhi=min(p[2] for p in vertices),max(p[2] for p in vertices)
    for obstacle in h.static_route_boxes:
        if zlo<=obstacle['z_max'] and zhi>=obstacle['z_min']:
            h.fail_if(overlaps(poly,obstacle['polygon']),'spawn_bbox_obstacle',actor.id)
    save(h.directory/'spawn_bbox_audit.json',dict(actor_id=actor.id,vertices=vertices,
        same_lane_occupancy=True,mesh_contact_verified=False,physics_motion_verified=False,
        frame=h.last_frame))
