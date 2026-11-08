"""Local abort facts and serialized control admission; never advances CARLA.

Admission is not RPC delivery. An already admitted call is not cancellable.
The short lock protects publication/admission, never RPC or file I/O.
"""
import copy
import math
import threading
import time

from idm_staged_stop import stopped


class AbortRejected(Exception):
    """Local rejection, deliberately not a RuntimeError/RPC poison condition."""


class AbortState:
    def __init__(self, *, pair=False, clock=time.monotonic_ns):
        self.pair=pair;self.clock=clock;self.lock=threading.Lock()
        self.requested=False;self.reason=None;self.owner_thread=None
        self.sequence=0;self.in_flight=None;self.actors={};self.events=[]
        self.last_success={};self._permits={};self._reserved=set()
        self._cleanup_finalized=False

    def _event(self,kind,actor=None,**fields):
        self.sequence+=1
        event=dict(sequence=self.sequence,kind=kind,actor_id=actor,
                   monotonic_ns=self.clock(),**fields)
        self.events.append(event)
        return event

    def _fact(self,actor,kind,**fields):
        entry=self.actors.setdefault(str(actor),dict(actor_id=actor,facts={},errors=[]))
        if kind not in entry['facts']:
            entry['facts'][kind]=self._event(kind,entry['actor_id'],**fields)

    def register(self,actor,logical=None):
        with self.lock:
            entry=self.actors.setdefault(str(actor),dict(actor_id=actor,facts={},errors=[]))
            if logical is not None:entry['logical_id']=logical
            if self.requested:self._fact(actor,'ABORT_REQUESTED',reason=self.reason)

    def request(self,reason,*,received_ns=None):
        with self.lock:
            if not self.requested:
                self.requested=True;self.reason=str(reason)
                self._event('ABORT_REQUESTED',reason=self.reason,callback_received_ns=received_ns)
            else:self._event('ABORT_REPEATED',reason=str(reason),callback_received_ns=received_ns)
            for actor in list(self.actors):self._fact(actor,'ABORT_REQUESTED',reason=self.reason)

    def _notice(self):
        for actor in list(self.actors):self._fact(actor,'DRIVE_COMMANDS_BLOCKED')

    def check(self):
        with self.lock:
            if self.requested:
                self._notice()
                raise AbortRejected(self.reason)

    def reserve_stop(self,actor,control,*,legacy=False):
        """One internal, identity-bound permit per actor, never a phase bypass."""
        with self.lock:
            if not self.requested or str(actor) not in self.actors or actor in self._reserved:
                raise AbortRejected('abort permit unavailable')
            if 'STOP_UNVERIFIED' in self.actors[str(actor)]['facts']:
                raise AbortRejected('terminal unverified stop')
            values=[control.get(k) for k in ('throttle','brake','steer')]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                raise AbortRejected('unknown abort control')
            if control['throttle']!=0 or abs(control['steer'])>.1:
                raise AbortRejected('invalid abort control')
            if legacy:
                if self.pair or control['brake']!=.7:raise AbortRejected('legacy abort not permitted')
            elif not 0<control['brake']<=.02:raise AbortRejected('unqualified bounded abort')
            expected=dict(control,hand_brake=False,reverse=False,manual_gear_shift=False)
            token=object();self._permits[token]=(actor,expected);self._reserved.add(actor)
            return token

    def admit(self,actor,frame,control,permit=None):
        with self.lock:
            thread=threading.current_thread()
            if self.owner_thread is not None and self.owner_thread is not thread:
                raise AbortRejected('control thread changed')
            if self.in_flight is not None:raise AbortRejected('concurrent control RPC prohibited')
            if permit is None:
                if self.requested:
                    self._notice();self._event('DRIVE_REJECTED',actor,frame=frame)
                    raise AbortRejected(self.reason)
            else:
                expected=self._permits.pop(permit,None)
                if expected!=(actor,dict(control)) or not self.requested:
                    raise AbortRejected('invalid or stale abort permit')
                self._notice()
            self.owner_thread=thread
            ticket=self._event('CONTROL_ADMITTED',actor,frame=frame,control=dict(control),
                               abort_control=permit is not None)
            self.in_flight=ticket['sequence']
            return ticket

    def rpc_started(self,ticket):
        with self.lock:
            if self.in_flight!=ticket['sequence']:raise AbortRejected('invalid in-flight control')
            self._event('RPC_CALL_STARTED',ticket['actor_id'],admission_sequence=ticket['sequence'])
            if ticket['abort_control']:self._fact(ticket['actor_id'],'STOP_COMMAND_ATTEMPTED')

    def finish(self,ticket,*,returned=None,error=None):
        with self.lock:
            if self.in_flight!=ticket['sequence']:raise AbortRejected('invalid control completion')
            actor=ticket['actor_id']
            self._event('RPC_CALL_COMPLETED',actor,admission_sequence=ticket['sequence'],
                        api_returned=error is None,error=error,returned=False if returned is False else None)
            self.in_flight=None
            if error is None and returned is not False:
                self.last_success[actor]=dict(frame=ticket['frame'],control=dict(ticket['control']))
                if ticket['abort_control']:
                    self._fact(actor,'STOP_COMMAND_ACKNOWLEDGED',ack_kind='api_call_returned',
                               admission_sequence=ticket['sequence'],command_frame=ticket['frame'])
            elif ticket['abort_control']:self._unverified(actor,error or 'control returned False')

    def _unverified(self,actor,reason):
        self._fact(actor,'STOP_UNVERIFIED',reason=str(reason))
        entry=self.actors[str(actor)]
        if str(reason) not in entry['errors']:entry['errors'].append(str(reason))

    def unverified(self,actor,reason):
        with self.lock:self._unverified(actor,reason)

    def observed(self,actor,first_frame,last_frame):
        with self.lock:
            facts=self.actors[str(actor)]['facts']
            if 'STOP_UNVERIFIED' in facts or 'STOP_COMMAND_ACKNOWLEDGED' not in facts:
                raise AbortRejected('unverified stop cannot be promoted')
            self._fact(actor,'STOP_OBSERVED',first_frame=first_frame,last_frame=last_frame)

    def cleanup_result(self,audit):
        with self.lock:
            # A failed/unknown terminal cleanup cannot later be upgraded by a
            # caller supplying a successful second result (there is no retry).
            if self._cleanup_finalized:return
            self._cleanup_finalized=True
            for actor in list(self.actors):
                if audit.get('attempted'):self._fact(actor,'CLEANUP_ATTEMPTED')
                if audit.get('complete') and audit.get('attempted'):
                    self._fact(actor,'CLEANUP_COMPLETED')
                if 'STOP_OBSERVED' not in self.actors[actor]['facts']:
                    self._unverified(actor,'no_authorized_post_request_frames')

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(dict(requested=self.requested,reason=self.reason,
                actors=self.actors,events=self.events,in_flight=self.in_flight,
                stop_verified=bool(self.actors) and all('STOP_OBSERVED' in a['facts'] and
                    'STOP_UNVERIFIED' not in a['facts'] for a in self.actors.values()),
                qualification='API return is not physical stop; no abort ticks added'))


def bounded_stop(row,previous,frame):
    """Existing low-speed cap/slew only. Unknown/high-speed => no command."""
    if row is None or previous is None:return None,'missing_control_evidence'
    numeric=('speed_planar_mps','v_long_mps','vz','a_long_mps2','throttle','brake','steer')
    if any(type(row.get(k)) not in (int,float) or not math.isfinite(row[k]) for k in numeric):
        return None,'unknown_stop_state'
    if row.get('frame')!=frame or type(previous.get('frame')) is not int or previous['frame']>frame:
        return None,'stale_or_unknown_control_frame'
    geometry=('roll','pitch','heading_error_deg','route_lateral_m','front_clearance_m')
    if (any(type(row.get(k)) not in (int,float) or not math.isfinite(row[k]) for k in geometry) or
        (row.get('road_id'),row.get('section_id'),row.get('lane_id'))!=(8,0,-1) or
        any(abs(row[k])>5 for k in ('roll','pitch','heading_error_deg')) or
        not 0<=row['route_lateral_m']<=.3 or row['front_clearance_m']<20):
        return None,'unqualified_route_or_pose'
    if not 0<=row['speed_planar_mps']<=.1 or abs(row['v_long_mps'])>.1:
        return None,'unqualified_speed'
    if abs(row['vz'])>=.05 or abs(row['a_long_mps2'])>5 or type(row.get('gear')) is not int or row['gear']<1:
        return None,'unqualified_motion_or_gear'
    if any(row.get(k) is not False for k in ('hand_brake','reverse','manual_gear_shift')):
        return None,'unknown_or_unsafe_control_state'
    control=previous.get('control',{})
    if any(type(control.get(k)) not in (int,float) or not math.isfinite(control[k]) or
           abs(control[k]-row[k])>1e-5 for k in ('throttle','brake','steer')):
        return None,'unobserved_latest_control'
    if not 0<=row['brake']<=.02 or not 0<=row['throttle']<=.10001 or abs(row['steer'])>.1:
        return None,'control_outside_bounded_envelope'
    elapsed=min(.05,max(0.,(frame-previous['frame'])*.05))
    brake=min(.02,control['brake']+.10*elapsed)
    if brake<=0:return None,'no_slew_budget'
    return dict(throttle=0.,brake=brake,steer=control['steer']),None


class StopWindow:
    """Evidence-only reducer. Consumes supplied samples; never obtains/ticks them."""
    def __init__(self,state,actors,request_frame):
        self.state=state;self.actors=set(actors);self.request_frame=request_frame
        self.previous=None;self.window=[];self.count=0;self.terminal=False

    def fail(self,reason):
        for actor in self.actors:self.state.unverified(actor,reason)
        self.terminal=True

    def feed(self,frame,stamp,rows):
        if self.terminal:raise AbortRejected('stop window terminal')
        if (type(frame) is not int or frame<=self.request_frame or type(stamp) not in (int,float)
            or not math.isfinite(stamp) or set(rows)!=self.actors):
            self.fail('invalid_stop_clock_or_ids');return False
        if self.previous is None and frame!=self.request_frame+1:
            self.fail('missing_first_stop_response_frame');return False
        if self.previous and (frame!=self.previous[0]+1 or abs(stamp-self.previous[1]-.05)>1e-5):
            self.fail('stop_frame_or_time_gap');return False
        self.count+=1;self.previous=(frame,stamp)
        valid=True
        for actor,row in rows.items():
            facts=self.state.snapshot()['actors'].get(str(actor),{}).get('facts',{})
            ack=facts.get('STOP_COMMAND_ACKNOWLEDGED')
            numeric=('speed_planar_mps','v_long_mps','vz','a_long_mps2','throttle','brake')
            if (not ack or frame<=ack['command_frame'] or row.get('frame')!=frame or
                row.get('control_read_at_frame')!=frame or row.get('command_based_on_frame')!=ack['command_frame'] or
                type(row.get('world_time_s')) not in (int,float) or not math.isfinite(row['world_time_s']) or
                abs(row['world_time_s']-stamp)>1e-5 or
                any(type(row.get(k)) not in (int,float) or not math.isfinite(row[k]) for k in numeric)):
                self.fail('invalid_stop_observation');return False
            expected=self.state.last_success[actor]['control']
            for key in ('throttle','brake','steer'):
                if (type(row.get(key)) not in (int,float) or not math.isfinite(row[key]) or
                    type(row.get('command_'+key)) not in (int,float) or
                    not math.isfinite(row['command_'+key]) or abs(row[key]-row['command_'+key])>1e-5 or
                    abs(row['command_'+key]-expected[key])>1e-5):
                    self.fail('stop_control_echo_missing');return False
            valid=valid and stopped(row)
        self.window=self.window+[(frame,stamp)] if valid else []
        if len(self.window)>=11 and stamp-self.window[0][1]>=.5-1e-9:
            for actor in self.actors:self.state.observed(actor,self.window[0][0],frame)
            self.terminal=True;return True
        if self.count>=100:self.fail('stop_observation_timeout')
        return False
