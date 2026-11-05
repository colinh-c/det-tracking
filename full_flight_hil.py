#!/usr/bin/env python3
"""Original-time synthetic flight through real gimbal control, explicitly bench only.

Default invocation reads manual status and prepares a report. --execute is the
separate, intentional hardware action. No homing, zeroing, prepositioning, limit
writes, flight-calibration promotion, or simulated motor plant is used here.
Counter-derived Pose.valid means usable for this relative bench computation,
never measured physical output angle or a qualified AS5600 registration.
"""
import os
# Match the measured bench setup before importing NumPy or OpenCV.
for _thread_variable in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):
    os.environ[_thread_variable]='1'
import argparse
from collections import deque
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time
import urllib.request

import cv2
cv2.setNumThreads(1)
import numpy as np

from det_types import Pose, Command, angle_error
from execution import Executor
from launch_config import load_launch_config, live_readiness
from prediction import AngularPrior
from recording import Recorder
from runtime import TrackingEngine
from simulation import SyntheticFlight

ROOT = Path(__file__).resolve().parent
QUALITY = 'unqualified_controller_counts'


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name+' must be finite')
    return float(value)


def json_get(url, timeout=.5):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read(1024*1024))


def load_profile(path=None):
    path = Path(path or ROOT/'full_flight_hil_config.json').resolve()
    profile = json.loads(path.read_text())
    if profile.get('schema_version') != 1:
        raise ValueError('Unsupported full-flight profile')
    for key in ('fps','expected_duration_s','prime_s','pose_max_age_s'):
        if number(profile.get(key), key) <= 0: raise ValueError(key+' must be positive')
    delay = number(profile.get('synthetic_frame_delay_s'), 'synthetic_frame_delay_s')
    if not 0 <= delay <= .2 or profile['prime_s'] < delay+2/profile['fps']:
        raise ValueError('Frame delay must fit the priming/history window and 0..0.2 seconds')
    if profile['pose_max_age_s'] > .3:
        raise ValueError('Counter freshness cannot exceed the executor freshness bound')
    for key in ('save_frames','allow_limit_saturation'):
        if type(profile.get(key)) is not bool: raise ValueError(key+' must be boolean')
    for key in ('launch_config','output_root'):
        profile[key] = str((path.parent/profile[key]).resolve())
    cfg = load_launch_config(profile['launch_config'])
    overrides=profile.get('executor_overrides',{})
    if not isinstance(overrides,dict) or set(overrides)-{'lease_s','max_control_gap_s'}:
        raise ValueError('HIL overrides are limited to command lease and control gap')
    cfg['executor']={**cfg.get('executor',{}),**overrides}
    executor_config=Executor(cfg['executor']).config
    for key in ('lease_s','max_control_gap_s'):
        cfg['executor'][key]=executor_config[key]
    if executor_config['max_control_gap_s']>.3:
        raise ValueError('HIL control-gap bound cannot exceed 0.3 seconds')
    if executor_config['lease_s']-cfg['motor_timeout_s']<=executor_config['max_control_gap_s']:
        raise ValueError('HIL lease must cover the control gap and outbound transport allowance')
    # Use the unchanged configured ceilings. Executor also validates these.
    if cfg.get('executor',{}).get('max_speed_deg_s',15) > 15 or cfg.get('executor',{}).get('max_accel_deg_s2',75) > 75:
        raise ValueError('Existing controller ceilings are 15 deg/s and 75 deg/s2')
    prior = AngularPrior(cfg['trajectory'],cfg['field_config'])
    if prior.start_time_s != 0 or abs(prior.end_time_s-profile['expected_duration_s']) > 1e-6:
        raise ValueError('Profile must describe the complete saved flight, starting at zero')
    return profile, cfg


def limit_bounds(limits):
    if not isinstance(limits, dict): raise ValueError('Current firmware limits were not supplied')
    raw = limits.get('soft_limits_deg', limits)
    result = {}
    for axis in ('az','el'):
        pair = raw.get(axis)
        if not isinstance(pair,(list,tuple)) or len(pair)!=2:
            raise ValueError('Missing current '+axis+' limits')
        pair = [number(x,axis+' limit') for x in pair]
        if pair[0] >= pair[1]: raise ValueError('Unordered '+axis+' limits')
        result[axis] = pair
    if limits.get('confirmation_required') is True:
        raise ValueError('The existing limit profile needs confirmation in the manual dashboard')
    return result


def position(value):
    if not isinstance(value,dict): raise ValueError('Missing controller position')
    return {axis:number(value.get(axis),axis+' count position') for axis in ('az','el')}


def trajectory_data(cfg):
    prior = AngularPrior(cfg['trajectory'],cfg['field_config'])
    # Original CSV knots plus a 10 ms grid resolve the saved high-rate ascent.
    times = np.unique(np.concatenate((prior.times,np.arange(0,prior.end_time_s,.01),[prior.end_time_s])))
    predictions = [prior.sample(float(t)) for t in times]
    if not all(p.valid for p in predictions): raise ValueError('The saved pointing prior contains a singularity')
    az = np.rad2deg(np.unwrap(np.deg2rad([p.az_deg for p in predictions])))
    el = np.array([p.el_deg for p in predictions])
    angles = np.column_stack((az,el))
    rates = np.array([[p.az_rate_deg_s,p.el_rate_deg_s] for p in predictions])
    relative = angles-angles[0]
    return times, relative, rates, [predictions[0].az_deg,predictions[0].el_deg]


def describe_trajectory(cfg, data=None):
    times, relative, rates, start = data or trajectory_data(cfg)
    peaks = np.argmax(abs(rates),axis=0)
    return {'duration_s':float(times[-1]),'wall_time_scale':1.0,'start_world_deg':dict(zip(('az','el'),start)),
            'relative_span_deg':{axis:[float(relative[:,i].min()),float(relative[:,i].max())] for i,axis in enumerate(('az','el'))},
            'peak_abs_rate_deg_s':{axis:float(abs(rates[peaks[i],i])) for i,axis in enumerate(('az','el'))},
            'peak_rate_elapsed_s':{axis:float(times[peaks[i]]) for i,axis in enumerate(('az','el'))},
            'rate_sample_resolution_s':.01,'speed_ceiling_deg_s':15.,'acceleration_ceiling_deg_s2':75.,
            'time_scaling_to_fit_limits':False,
            'renderer_note':'Existing SyntheticFlight adds 0.025 deg az and 0.020 deg el sinusoidal mismatch; stage/size changes are rendering truth only.',
            'trajectory_sha256':hashlib.sha256(Path(cfg['trajectory']).read_bytes()).hexdigest()}


def path_travel(origin, limits, data):
    bounds = limit_bounds(limits); origin = position(origin)
    times, relative, _, _ = data
    # Include the existing renderer mismatch, preserving the original target.
    target = relative+np.array([origin['az'],origin['el']])
    target[:,0] += .025*np.sin(times*.33)
    target[:,1] += .020*np.sin(times*.8)
    report = {'fits_existing_limits':True,'origin_controller_deg':origin,
              'existing_limits_deg':bounds,'axes':{},
              'qualification':'Counter-coordinate feasibility only. Target is not clipped or rescaled; actual MCU enforces the current user limits.'}
    for i,axis in enumerate(('az','el')):
        outside = (target[:,i]<bounds[axis][0]) | (target[:,i]>bounds[axis][1])
        intervals=[]; begin=None
        for n,hit in enumerate(outside):
            if hit and begin is None: begin=float(times[n])
            if begin is not None and (not hit or n==len(times)-1):
                intervals.append([begin,float(times[n])]); begin=None
        report['axes'][axis]={'target_span_controller_deg':[float(target[:,i].min()),float(target[:,i].max())],
                             'unreachable_intervals_s':intervals,
                             'approx_unreachable_s':float(np.sum(np.diff(times)*outside[:-1]))}
        if outside.any(): report['fits_existing_limits']=False
    return report


def prepare(profile,cfg,manual_status=None):
    data=trajectory_data(cfg); issues=[]; status=manual_status
    if status is None:
        try: status=json_get(profile['handoff_url'].rstrip('/')+'/motion')
        except Exception as error: issues.append('Manual connection: '+str(error)); status={}
    # Some dashboard wrappers nest the original gateway status.
    if isinstance(status,dict) and isinstance(status.get('motion'),dict): status=status['motion']
    travel=None
    try: travel=path_travel(status.get('position'),status.get('limits'),data)
    except (ValueError,AttributeError) as error: issues.append(str(error))
    if not isinstance(status,dict) or status.get('connected') is not True or status.get('available') is not True:
        issues.append('Manual motor endpoint is not available')
    elif status.get('busy') or status.get('hold_active'):
        issues.append('Manual movement is active; finish it before intentionally starting the flight test')
    if status.get('recovery_required') or status.get('limits_confirmation_required'):
        issues.append('Manual dashboard reports a recovery or limit confirmation requirement')
    if travel and not travel['fits_existing_limits'] and not profile['allow_limit_saturation']:
        issues.append('Trajectory exceeds current travel limits and allow_limit_saturation is false')
    return {'mode':'prepare_only','hardware_motion_requested':False,'ready_for_approved_execute':not issues,
            'approval_required_before_execution':True,'issues':issues,'manual_status':status,
            'trajectory':describe_trajectory(cfg,data),'travel':travel,
            'allow_limit_saturation':profile['allow_limit_saturation'],
            'feedback_quality':QUALITY,'physical_output_angle_ground_truth':False,
            'flight_calibration_valid':False,'unchanged_flight_readiness_issues':live_readiness(cfg),
            'synthetic_frame_delay_s':profile['synthetic_frame_delay_s'],
            'command_timing':{key:cfg['executor'][key] for key in ('lease_s','max_control_gap_s')},
            'input_images':'SyntheticFlight rendered from actual controller-count history, not real optical captures',
            'raw_as5600':'Cached persistent camera-gateway diagnostics, polled at most 5 Hz; source ages/read_at remain unchanged. No assumed bus-to-axis physical registration.',
            'execution':'Start Full Flight Hardware Test.command --execute (only after approval)',
            'manual_dashboard':'http://127.0.0.1:18182/'}


class CounterHistory:
    def __init__(self,origin,world_start,max_age):
        self.origin=position(origin); self.world_start=tuple(world_start); self.max_age=max_age
        self.samples=deque(maxlen=256)

    def append(self,value,received):
        age=number(value.get('age_s'),'Counter age')
        if not 0 <= age <= self.max_age: raise ValueError('Stale or future counter pose')
        pos=position(value.get('position')); stamp=received-age
        if self.samples and stamp <= self.samples[-1][0]: raise ValueError('Out-of-order counter time')
        self.samples.append((stamp,pos))
        return self.at(stamp)

    def at(self,stamp):
        if not self.samples or stamp<self.samples[0][0] or stamp>self.samples[-1][0]:
            raise ValueError('No bracketing actual counter samples for capture time')
        last=self.samples[0]
        for current in self.samples:
            if current[0]>=stamp:
                weight=0. if current[0]==last[0] else (stamp-last[0])/(current[0]-last[0])
                angles={axis:last[1][axis]+weight*(current[1][axis]-last[1][axis]) for axis in ('az','el')}
                return Pose(stamp,(self.world_start[0]+angles['az']-self.origin['az'])%360,
                            self.world_start[1]+angles['el']-self.origin['el'],True)
            last=current
        raise ValueError('Counter history lookup failed')


class RawEncoderDiagnostics:
    """Independent bounded reads. Never supply these as axis feedback."""
    def __init__(self,url):
        self.url=url; self.done=threading.Event(); self.lock=threading.Lock(); self.value=None
        self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()
    def _run(self):
        while not self.done.is_set():
            started=time.monotonic()
            try: value={'readings':json_get(self.url,.15),'error':None}
            except Exception as error: value={'readings':None,'error':str(error)}
            value.update(received_monotonic=time.monotonic(),request_duration_s=time.monotonic()-started,
                         source='persistent_camera_gateway_cache',max_poll_rate_hz=5.)
            with self.lock: self.value=value
            self.done.wait(.2)
    def snapshot(self):
        with self.lock: return deepcopy(self.value)
    def close(self): self.done.set(); self.thread.join(.2)


class FullFlightHil:
    def __init__(self,profile,cfg,handoff,manual_status,output_dir,clock=time.monotonic,
                 sleep=time.sleep,engine=None,renderer=None,diagnostics=None):
        self.profile=deepcopy(profile); self.cfg=cfg; self.handoff=handoff; self.manual_status=manual_status
        self.output_dir=Path(output_dir); self.clock=clock; self.sleep=sleep
        self.step_now=self.clock()
        # All three modules must see the same processing instant. A live clock
        # callback otherwise stamps vision a few microseconds after the now
        # passed into Predictor, which correctly rejects future observations.
        self.engine=engine or TrackingEngine(cfg,clock=lambda:self.step_now)
        self.last_prediction=None
        self.renderer=renderer or SyntheticFlight(cfg['camera_models'],cfg['trajectory'],cfg['field_config'],fps=profile['fps'])
        self.data=trajectory_data(cfg); self.diagnostics=diagnostics
        self.cancel=threading.Event(); self.launch_origin=None; self.session=None
        self.history=None; self.motor=None; self.recorder=None; self.sequence=0

    def _observation(self):
        value=self.handoff.pose(self.session); received=self.clock()
        if value.get('generation')!=self.generation: raise ValueError('Motor gateway generation changed')
        if value.get('limits') is not None and limit_bounds(value['limits'])!=self.bounds:
            raise ValueError('Travel limit profile changed during the claimed session')
        pose=self.history.append(value,received)
        return value,pose,received

    def _frame(self,now,priming):
        capture=min(now-self.profile['synthetic_frame_delay_s'],self.history.samples[-1][0])
        if capture<self.history.samples[0][0]: return None
        measured=self.history.at(capture)
        elapsed=0. if priming else max(0.,capture-self.launch_origin)
        self.renderer.t=elapsed; self.renderer.epoch=capture-elapsed
        self.renderer.pose=measured; self.renderer.sequence=self.sequence
        # Only rendering. Never invoke SyntheticFlight.send() or advance().
        frames,_=self.renderer.read()
        return frames

    def _step(self,raw,pose,observed,priming):
        frames=self._frame(observed,priming)
        if frames is None: return False
        now=self.clock(); elapsed=0. if priming else now-self.launch_origin
        if not priming and elapsed>=self.renderer.end_s: return False
        start=self.clock()
        self.step_now=now
        vision,prediction,command=self.engine.step(frames,pose,now,elapsed,not priming)
        self.last_prediction=prediction
        self.sequence+=1; command.sequence=self.sequence
        algorithm_ms=(self.clock()-start)*1000
        row={'now':now,'launch_elapsed_s':None if priming else elapsed,'priming':priming,
             'feedback_quality':QUALITY,'physical_output_angle_ground_truth':False,
             'pose':asdict(pose),'counter_sample':raw,
             'frames':{f.camera:{'timestamp':f.timestamp,'sequence':f.sequence,'pose':asdict(f.pose)} for f in frames},
             'synthetic_capture_age_s':now-frames[0].timestamp,
             'vision':asdict(vision),'prediction':asdict(prediction),'command':asdict(command),
             'algorithm_ms':algorithm_ms,'raw_as5600':self.diagnostics.snapshot() if self.diagnostics else None}
        if not priming:
            # Evaluation uses renderer truth only after all algorithm decisions.
            truth=deepcopy(self.renderer.truth)
            current=self.data[3]
            if 'az_deg' in truth and 'el_deg' in truth:
                target={'az':self.history.origin['az']+angle_error(truth['az_deg'],current[0]),
                        'el':self.history.origin['el']+truth['el_deg']-current[1]}
                row['limit_observation']={'target_controller_deg':target,
                    'target_outside':{a:not self.bounds[a][0]<=target[a]<=self.bounds[a][1] for a in ('az','el')},
                    'count_near_limit':{a:min(abs(raw['position'][a]-b) for b in self.bounds[a])<.002 for a in ('az','el')}}
            row['evaluation_only']=truth
            fresh=0<=now-pose.timestamp<=self.profile['pose_max_age_s'] and all(0<=now-f.timestamp<=.3 for f in frames)
            if not command.enabled and fresh and prediction.reason.startswith('Target loss'):
                command=Command(now,0.,0.,now+self.cfg['executor']['lease_s'],True,'Stationary; waiting for reacquisition',self.sequence)
                row['command']=asdict(command)
            if not command.enabled:
                self._write(row,frames,pose,elapsed)
                raise ValueError('Tracking command stopped: '+command.reason)
            if any(abs(v)>15+1e-9 for v in (command.az_velocity_deg_s,command.el_velocity_deg_s)):
                raise ValueError('Command exceeds existing 15 deg/s ceiling')
            # Completion or cancellation during image work must not send a late command.
            if self.cancel.is_set() or self.clock()-self.launch_origin>=self.renderer.end_s:
                row['not_sent_reason']='cancelled_or_completed_before_send'
            else:
                ack_started=self.clock()
                try: row['ack']=self.motor.send(command)
                except Exception as error:
                    row['command_error']=str(error); self._write(row,frames,pose,elapsed)
                    raise
                row['ack_duration_s']=self.clock()-ack_started
                self.command_count+=1
        self._write(row,frames,pose,None if priming else elapsed)
        return True

    def _write(self,row,frames,pose,elapsed):
        self.log.write(json.dumps(row,allow_nan=False)+'\n'); self.log.flush()
        if self.recorder:
            self.recorder.write({f.camera:f for f in frames},row['now'],pose,elapsed)

    def run(self):
        self.output_dir.mkdir(parents=True,exist_ok=False)
        self.log=(self.output_dir/'telemetry.jsonl').open('x')
        report={'scope':'real_gimbal_synthetic_images_relative_bench','completed':False,'fault':None,
                'feedback_quality':QUALITY,'physical_output_angle_ground_truth':False,
                'flight_calibration_valid':False,'full_trajectory_wall_time_scale':1.,
                'command_count':0,'release':None,'manual_control_return_confirmed':False}
        self.command_count=0
        try:
            configuration={'profile':self.profile,
                'launch_config':{k:v for k,v in self.cfg.items() if k!='camera_models'},
                'camera_models':{k:asdict(v) for k,v in self.cfg['camera_models'].items()},
                'field_config':json.loads(Path(self.cfg['field_config']).read_text())}
            (self.output_dir/'configuration.json').write_text(json.dumps(configuration,indent=2,allow_nan=False)+'\n')
            preparation=prepare(self.profile,self.cfg,self.manual_status)
            (self.output_dir/'preparation.json').write_text(json.dumps(preparation,indent=2,allow_nan=False)+'\n')
            if preparation['issues']: raise ValueError('; '.join(preparation['issues']))
            claim=self.handoff.claim(); self.session=claim['session']
            self.generation=claim.get('generation')
            if not isinstance(self.generation,str) or not self.generation: raise ValueError('Missing claimed motor generation')
            # Recheck at the actual claim origin. Never home or preposition.
            limits=claim.get('limits')
            self.bounds=limit_bounds(limits)
            report['travel_at_claim']=path_travel(claim['position'],limits,self.data)
            report['limit_saturation_allowed']=self.profile['allow_limit_saturation']
            if not report['travel_at_claim']['fits_existing_limits'] and not self.profile['allow_limit_saturation']:
                raise ValueError('Actual claim origin makes the trajectory exceed existing travel limits')
            report['claim']={key:value for key,value in claim.items() if key not in ('session','motor_base_url')}
            self.history=CounterHistory(claim['position'],self.data[3],self.profile['pose_max_age_s'])
            if self.profile['save_frames']:
                self.recorder=Recorder(self.output_dir/'synthetic-frames',self.cfg['camera_models'],self.profile['fps'])
            prime_start=self.clock(); primed=0; deadline=prime_start
            while (self.clock()-prime_start<self.profile['prime_s'] or primed<2 or
                   self.last_prediction is None or not self.last_prediction.valid):
                if self.cancel.is_set(): raise InterruptedError('Cancelled before launch')
                if self.clock()-prime_start>self.profile['prime_s']+2:
                    reason=self.last_prediction.reason if self.last_prediction else 'No prediction'
                    raise ValueError('Unable to prime a valid prediction: '+reason)
                raw,pose,now=self._observation()
                primed+=bool(self._step(raw,pose,now,True))
                deadline+=1/self.profile['fps']; self.sleep(max(0.,deadline-self.clock()))
            # Client binds the already claimed generation only after priming.
            self.motor=self.handoff.motor(self.session,timeout_s=self.cfg['motor_timeout_s'])
            if hasattr(self.engine,'executor'): self.engine.executor.reset()
            self.launch_origin=self.clock(); report['launch_origin_monotonic']=self.launch_origin
            deadline=self.launch_origin
            while self.clock()-self.launch_origin<self.renderer.end_s:
                if self.cancel.is_set(): raise InterruptedError('Cancelled by operator')
                raw,pose,now=self._observation()
                if now-self.launch_origin>=self.renderer.end_s: break
                self._step(raw,pose,now,False)
                deadline+=1/self.profile['fps']
                # Skip deadlines after an overrun; never slow the saved flight clock.
                if deadline<self.clock():
                    deadline+=math.ceil((self.clock()-deadline)*self.profile['fps'])/self.profile['fps']
                self.sleep(max(0.,min(deadline,self.launch_origin+self.renderer.end_s)-self.clock()))
            if self.cancel.is_set(): raise InterruptedError('Cancelled by operator')
            report['completed']=True
        except Exception as error:
            report['fault']=type(error).__name__+': '+str(error)
        finally:
            # This is the only cleanup motion path. It is fenced to our owner.
            # Never use a global motor.stop() after manual takeover.
            if self.session is not None:
                try:
                    report['release']=self.handoff.release(self.session)
                    report['manual_control_return_confirmed']=report['release'].get('mode')=='manual'
                except Exception as error:
                    report['release_error']=str(error); report['completed']=False
            report['command_count']=self.command_count
            report['elapsed_wall_s']=None if self.launch_origin is None else self.clock()-self.launch_origin
            if self.recorder: self.recorder.close()
            self.log.close()
            (self.output_dir/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(ROOT/'full_flight_hil_config.json'))
    parser.add_argument('--output',help='New output directory; existing directories are never overwritten')
    parser.add_argument('--execute',action='store_true',help='Explicitly claim and run the REAL gimbal after user approval')
    args=parser.parse_args(argv)
    profile,cfg=load_profile(args.config)
    output=Path(args.output) if args.output else Path(profile['output_root'])/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    readiness=prepare(profile,cfg)
    if not args.execute:
        output.mkdir(parents=True,exist_ok=False)
        (output/'preparation.json').write_text(json.dumps(readiness,indent=2,allow_nan=False)+'\n')
        print(json.dumps({'report':str(output/'preparation.json'),**readiness},indent=2))
        return 0 if readiness['ready_for_approved_execute'] else 2
    if readiness['issues']: raise SystemExit('; '.join(readiness['issues']))
    # Import/bootstrap only on explicit execution. Preparation does not rebind
    # the user's current manual connection or start an automatic service.
    from control_handoff import HandoffClient, bootstrap_handoff
    bootstrap_handoff()
    handoff=HandoffClient(profile['handoff_url'])
    diagnostics=RawEncoderDiagnostics(profile['encoder_url'])
    runner=FullFlightHil(profile,cfg,handoff,readiness['manual_status'],output,diagnostics=diagnostics)
    previous={}
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):
            previous[sig]=signal.signal(sig,lambda *_:runner.cancel.set())
        result=runner.run()
    finally:
        diagnostics.close()
        for sig,handler in previous.items(): signal.signal(sig,handler)
    print(json.dumps({'output_dir':str(output),**result},indent=2))
    return 0 if result['completed'] else 1


if __name__=='__main__': raise SystemExit(main())
