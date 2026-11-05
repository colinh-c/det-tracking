"""Bounded relative bench HIL, with injected observations and motor transport.

This module opens no hardware connections and does not alter flight calibration.
The caller owns axis registration, bounded source/transport timeouts, and motor
lease enforcement. Synthetic target truth is used only for rendering/evaluation.
Only rendered Frame objects enter the real Perception -> Predictor -> Executor
path. A counter-based pose is explicitly an estimate, not measured motion.
"""
from dataclasses import dataclass, asdict, field
from pathlib import Path
import json
import math
import time

import cv2
import numpy as np

from det_types import CameraModel, Frame, Pose, angle_error
from perception import Perception
from prediction import Predictor
from execution import Executor


class BenchFault(RuntimeError):
    pass


class BenchComplete(StopIteration):
    """The bounded trial ended normally; no further command was sent."""


@dataclass
class BenchObservation:
    pose: Pose
    axis_sources: dict = field(default_factory=dict)
    encoder_health: dict = field(default_factory=dict)
    camera_registration: dict = field(default_factory=dict)


DEFAULTS = dict(duration_s=12., fps=15., amplitude_az_deg=.1, amplitude_el_deg=.1,
                target_rate_deg_s=.2, max_command_deg_s=.25, max_accel_deg_s2=1.,
                max_excursion_deg=2., max_pose_age_s=.25, latency_s=.1,
                phase='main', allow_counter_feedback=False, save_frames=True,
                bench_registration='', seed=7)


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


class BenchHilRunner:
    """source.read() -> BenchObservation; motor.send(Command)/stop() are injected.

    axis_sources are 'as5600', 'camera_registration' or 'step_counter'. AS5600 requires healthy,
    fresh encoder_health[axis] diagnostics and a caller-verified relative axis
    registration. The latter requires explicit allow_counter_feedback=True for
    enabled trials and is excluded from measured accuracy metrics. Optical
    feedback requires camera_registration {valid, age_s, inliers, confidence,
    scale_reference, jacobian_px_per_deg:[2x2], measured_displacement_px:[x,y]}.
    Its scale is relative bench motor-reference degrees, not absolute physical
    angle calibration. Image-space residuals are reported separately. Plain Pose
    or (_, Pose) supports disabled preview only. Motor commands are velocities
    in the caller's registered bench axes; no home or absolute target is sent.
    """
    def __init__(self, source, motor=None, cameras=None, config=None, output_dir=None,
                 clock=time.monotonic, sleep=time.sleep):
        self.source,self.motor,self.clock,self.sleep=source,motor,clock,sleep
        options=dict(config or {})
        gains=options.pop('executor',{})
        perception_options=options.pop('perception',{})
        predictor_options=options.pop('predictor',{})
        unknown=set(options)-set(DEFAULTS)
        if unknown: raise ValueError('Unknown bench setting: '+', '.join(sorted(unknown)))
        self.config={**DEFAULTS,**options}
        for key in ('duration_s','fps','amplitude_az_deg','amplitude_el_deg',
                    'target_rate_deg_s','max_command_deg_s','max_accel_deg_s2',
                    'max_excursion_deg','max_pose_age_s','latency_s'):
            if not finite(self.config[key]) or self.config[key]<=0:
                raise ValueError('Use a positive finite '+key)
        if self.config['max_command_deg_s']>15 or self.config['max_accel_deg_s2']>75:
            raise ValueError('Bench command bounds exceed controller ceilings')
        if max(self.config['amplitude_az_deg'],self.config['amplitude_el_deg'])>=self.config['max_excursion_deg']:
            raise ValueError('Measured bench excursion bound must exceed target amplitude')
        for key in ('allow_counter_feedback','save_frames'):
            if type(self.config[key]) is not bool: raise ValueError(key+' must be boolean')
        if not isinstance(self.config['bench_registration'],str):
            raise ValueError('bench_registration must describe the relative axis registration')
        if self.config['phase'] not in ('boost','coast','drogue','main'):
            raise ValueError('Use an explicit bench phase')
        if type(self.config['seed']) is not int: raise ValueError('seed must be an integer')
        self.cameras=dict(cameras or {
            'narrow':CameraModel('narrow',320,240,10000.,10000.,159.5,119.5),
            'wide':CameraModel('wide',320,240,2000.,2000.,159.5,119.5)})
        if set(self.cameras)!={'narrow','wide'}:
            raise ValueError('Bench needs narrow and wide camera models')
        for name,camera in self.cameras.items():
            if (not isinstance(camera,CameraModel) or camera.name!=name or
                    any((camera.az_offset_deg,camera.el_offset_deg,camera.roll_deg))):
                raise ValueError('Bench virtual camera axes must have zero mount offsets')
        self.now=clock()
        self.perception=Perception(self.cameras,{**perception_options,'clock':lambda:self.now})
        self.predictor=Predictor(prior=None,config=predictor_options)
        self.executor=Executor({**gains,'max_speed_deg_s':self.config['max_command_deg_s'],
                                'max_accel_deg_s2':self.config['max_accel_deg_s2']})
        self.previous=None; self.origin=None; self.started=None; self.capture_origin=None
        self.last_now=None; self.last_pose=None; self.quality_contract=None
        self._optical_origin=None; self._optical_jacobian=None; self._optical_reference=None
        self.sequence=0; self.rows=[]; self.frames=[]; self.fault=None; self.closed=False
        self.completed=False
        self.rng=np.random.default_rng(self.config['seed'])
        self.output_dir=Path(output_dir) if output_dir is not None else None
        self.log=None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True,exist_ok=True)
            if any(self.output_dir.iterdir()): raise ValueError('Bench output directory must be empty')
            config_record={'scope':'bench_relative_registration_not_flight_calibration',
                           'config':self.config,'executor':self.executor.config,
                           'perception':perception_options,'predictor':predictor_options,
                           'virtual_cameras':{name:asdict(camera) for name,camera in self.cameras.items()},
                           'truth_is_evaluation_only':True}
            with (self.output_dir/'bench_config.json').open('x') as handle:
                json.dump(config_record,handle,indent=2,allow_nan=False)
            self.log=(self.output_dir/'telemetry.jsonl').open('x')

    def target(self,elapsed_s):
        """Both axes start at zero; each target rate is bounded by the configured rate."""
        if not finite(elapsed_s) or elapsed_s<0: raise ValueError('Invalid target elapsed time')
        a,b=self.config['amplitude_az_deg'],self.config['amplitude_el_deg']
        speed=self.config['target_rate_deg_s']
        return a*math.sin(speed/a*elapsed_s), b*math.sin(.73*speed/b*elapsed_s)

    def _observation(self, value, enabled):
        if isinstance(value,BenchObservation): observation=value
        else:
            pose=value[1] if isinstance(value,tuple) and len(value)==2 else value
            observation=BenchObservation(pose)
        pose=observation.pose
        if (not isinstance(pose,Pose) or pose.valid is not True or
                not all(finite(v) for v in (pose.timestamp,pose.az_deg,pose.el_deg)) or
                not -1e-6<=self.now-pose.timestamp<=self.config['max_pose_age_s']):
            raise BenchFault('Invalid, stale or future bench position')
        if self.last_pose is not None:
            if pose.timestamp<self.last_pose.timestamp:
                raise BenchFault('Regressed position timestamp')
            if (pose.timestamp==self.last_pose.timestamp and
                    (angle_error(pose.az_deg,self.last_pose.az_deg)!=0 or pose.el_deg!=self.last_pose.el_deg)):
                raise BenchFault('Position changed without a new sample timestamp')
        sources={axis:observation.axis_sources.get(axis,'unknown') for axis in ('az','el')}
        # Diagnostics must be serializable before a command can be dispatched.
        observation.encoder_health=json.loads(json.dumps(observation.encoder_health,allow_nan=False))
        observation.camera_registration=json.loads(json.dumps(observation.camera_registration,allow_nan=False))
        if self.quality_contract is not None and sources!=self.quality_contract:
            raise BenchFault('Position source changed during the bench trial')
        optical=observation.camera_registration
        optical_healthy=False
        if 'camera_registration' in sources.values():
            matrix=np.asarray(optical.get('jacobian_px_per_deg'),dtype=float)
            displacement=np.asarray(optical.get('measured_displacement_px'),dtype=float)
            optical_healthy=(optical.get('valid') is True and not optical.get('error') and
                finite(optical.get('age_s')) and 0<=optical['age_s']<=self.config['max_pose_age_s'] and
                type(optical.get('inliers')) is int and optical['inliers']>=8 and
                finite(optical.get('confidence')) and .5<=optical['confidence']<=1 and
                isinstance(optical.get('scale_reference'),str) and bool(optical['scale_reference'].strip()) and
                matrix.shape==(2,2) and displacement.shape==(2,) and
                np.isfinite(matrix).all() and np.isfinite(displacement).all())
            if optical_healthy:
                optical_healthy=abs(np.linalg.det(matrix))>1e-9 and np.linalg.cond(matrix)<=100
            if not optical_healthy:
                raise BenchFault('Optical bench registration is invalid, stale or poorly conditioned')
            if self._optical_jacobian is not None and (
                    not np.array_equal(matrix,self._optical_jacobian) or
                    optical['scale_reference']!=self._optical_reference):
                raise BenchFault('Optical scale/reference changed during trial')
            if self._optical_origin is None:
                self._optical_origin=displacement.copy(); self._optical_jacobian=matrix.copy()
                self._optical_reference=optical['scale_reference']
        measured={}
        for axis,source in sources.items():
            health=observation.encoder_health.get(axis,{})
            healthy=(source=='as5600' and health.get('valid') is True and
                     health.get('magnet_detected') is True and health.get('field_weak') is False and
                     health.get('field_strong') is False and not health.get('error') and
                     health.get('stale') is not True and finite(health.get('age_s')) and
                     0<=health['age_s']<=self.config['max_pose_age_s'])
            measured[axis]=bool(healthy)
            if enabled and not (healthy or (source=='camera_registration' and optical_healthy) or
                                (source=='step_counter' and self.config['allow_counter_feedback'])):
                raise BenchFault(axis+' has no trusted encoder or explicitly allowed counter feedback')
        if enabled and not self.config['bench_registration'].strip():
            raise BenchFault('Explicit relative bench axis registration is required')
        if enabled and self.motor is None: raise BenchFault('No injected motor transport')
        self.quality_contract=sources
        return observation,sources,measured

    def _render(self, relative, target):
        frames=[]; metadata=[]
        for name,camera in self.cameras.items():
            image=np.clip(155+self.rng.normal(0,.5,(camera.height,camera.width)),0,255).astype(np.uint8)
            point=camera.project(target[0]%360,target[1],relative)
            visible=camera.inside(point)
            if visible:
                x,y=point
                length=max(5.,camera.fy*math.radians(.25))
                thickness=max(1,int(round(camera.fx*math.radians(.04))))
                cv2.line(image,(round(x),round(y-length/2)),(round(x),round(y+length/2)),
                         40,thickness,cv2.LINE_AA)
            frames.append(Frame(name,relative.timestamp,image,relative,self.sequence))
            metadata.append({'camera':name,'timestamp':relative.timestamp,'sequence':self.sequence,
                             'size':[camera.width,camera.height],'target_visible':bool(visible),
                             'target_px':list(point) if point is not None else None})
        return frames,metadata

    def _write(self,row):
        self.rows.append(row)
        if self.log:
            self.log.write(json.dumps(row,allow_nan=False)+'\n'); self.log.flush()

    def _stop(self):
        self.executor.reset()
        if self.motor is not None: self.motor.stop()

    def _finish_if_due(self, now, row):
        if self.started is not None and now-self.started>=self.config['duration_s']:
            self._stop()
            row.update(time=now,elapsed_s=now-self.started,event='duration_completed',command_sent=False)
            self._write(row)
            self.completed=True
            raise BenchComplete('Bench duration completed')

    def step(self,enabled=False):
        if type(enabled) is not bool: raise ValueError('enabled must be boolean')
        if self.completed: raise BenchComplete('Bench duration completed')
        if self.closed or self.fault: raise BenchFault(self.fault or 'Bench runner is closed')
        row={'scope':'bench_relative_registration_not_flight_calibration','enabled_requested':enabled}
        try:
            read_started=self.clock(); value=self.source.read(); read_completed=self.clock(); self.now=read_completed
            if not finite(self.now) or (self.last_now is not None and self.now<=self.last_now):
                raise BenchFault('Nonmonotonic bench time')
            observation,sources,measured=self._observation(value,enabled)
            pose=observation.pose
            if self.origin is None:
                self.origin=pose; self.started=self.now; self.capture_origin=pose.timestamp
            elapsed=self.now-self.started
            row.update(source_read_started=read_started,source_read_completed=read_completed)
            self._finish_if_due(self.now,row)
            relative=Pose(pose.timestamp,angle_error(pose.az_deg,self.origin.az_deg),
                          pose.el_deg-self.origin.el_deg,True)
            if max(abs(relative.az_deg),abs(relative.el_deg))>self.config['max_excursion_deg']:
                raise BenchFault('Measured/estimated relative bench excursion exceeded the trial bound')
            self.sequence+=1
            target=self.target(max(0.,pose.timestamp-self.capture_origin))
            current_target=self.target(elapsed)
            self.frames,metadata=self._render(relative,target)
            self.now=self.clock()
            result=self.perception.update(self.frames,self.previous,phase_override=self.config['phase'])
            prediction=self.predictor.update(result,relative,self.now,latency_s=self.config['latency_s'])
            self.previous=prediction
            detection=result.selected
            camera=self.cameras['narrow']
            if (detection is not None and detection.camera=='narrow' and detection.confidence>=.4 and
                    detection.bbox is not None and detection.bbox[0]>1 and detection.bbox[1]>1 and
                    detection.bbox[2]<camera.width-1 and detection.bbox[3]<camera.height-1):
                prediction.image_error_az_deg=angle_error(detection.bearing_deg,relative.az_deg)
                prediction.image_error_el_deg=detection.elevation_deg-relative.el_deg
                prediction.image_timestamp=detection.timestamp
            self.now=self.clock()
            command=self.executor.update(prediction,relative,self.now,enabled)
            row.update(time=self.now,elapsed_s=elapsed,source_read_started=read_started,
                       source_read_completed=read_completed,
                       raw_pose=asdict(pose),relative_pose=asdict(relative),origin=asdict(self.origin),
                       axis_sources=sources,encoder_health=observation.encoder_health,
                       camera_registration=observation.camera_registration,
                       feedback_quality=('measured_encoder' if all(measured.values()) else
                           'optical_relative_registration' if all(v=='camera_registration' for v in sources.values()) else
                           'mixed_feedback' if len(set(sources.values()))>1 else 'estimated_position'),
                       frames=metadata,detection=asdict(detection) if detection else None,
                       vision=asdict(result),prediction=asdict(prediction),command=asdict(command),
                       command_sent=False,truth_is_evaluation_only=True,
                       evaluation={'render_target_az_deg':target[0],'render_target_el_deg':target[1],
                                   'target_now_az_deg':current_target[0],'target_now_el_deg':current_target[1],
                                   'az_error_deg':angle_error(target[0],relative.az_deg),
                                   'el_error_deg':target[1]-relative.el_deg,
                                   'physical_encoder_ground_truth':measured})
            if 'camera_registration' in sources.values():
                displacement=np.asarray(observation.camera_registration['measured_displacement_px'])-self._optical_origin
                image_error=self._optical_jacobian@np.asarray(target)-displacement
                row['evaluation']['measured_image_error_px']={'x':float(image_error[0]),'y':float(image_error[1])}
            if enabled:
                sent=self.clock()
                self._finish_if_due(sent,row)
                if not command.enabled: raise BenchFault('Controller stopped: '+command.reason)
                if sent>=command.valid_until: raise BenchFault('Command expired before dispatch')
                row['command_send_started']=sent
                self.motor.send(command)
                row['command_send_completed']=self.clock(); row['command_sent']=True
            if self.output_dir is not None and self.config['save_frames']:
                for frame,entry in zip(self.frames,metadata):
                    ok,encoded=cv2.imencode('.jpg',frame.image)
                    if not ok: raise BenchFault('Could not encode bench frame')
                    name='%06d-%s.jpg'%(self.sequence,frame.camera)
                    with (self.output_dir/name).open('xb') as handle: handle.write(encoded.tobytes())
                    entry['file']=name
            self.last_now=self.now; self.last_pose=pose
            self._write(row)
            return row
        except BenchComplete:
            raise
        except Exception as error:
            self.fault=str(error)
            row.update(error=self.fault,time=self.clock())
            try: self._stop()
            except Exception as stop_error: row['stop_error']=str(stop_error)
            self._write(row)
            raise BenchFault(self.fault) from error

    def summary(self):
        usable=[row for row in self.rows if 'evaluation' in row]
        def stats(values):
            if not values: return None
            return {'samples':len(values),'rms':float(np.sqrt(np.mean(np.square(values)))),
                    'absolute_p50_p95':np.percentile(np.abs(values),[50,95]).tolist(),
                    'maximum_absolute':float(np.max(np.abs(values)))}
        return {'scope':'bench_relative_registration_not_flight_calibration',
                'samples':len(usable),'fault':self.fault,'completed':self.completed,'truth_is_evaluation_only':True,
                'correct_physical_axis_registration_is_callers_responsibility':True,
                'virtual_camera_test_does_not_validate_real_optics':True,
                'detected_fraction':sum(row['detection'] is not None for row in usable)/len(usable) if usable else None,
                'measured_encoder_error_deg':{axis:stats([row['evaluation'][axis+'_error_deg'] for row in usable
                    if row['evaluation']['physical_encoder_ground_truth'][axis]]) for axis in ('az','el')},
                'measured_image_error_px':{axis:stats([row['evaluation']['measured_image_error_px'][axis]
                    for row in usable if 'measured_image_error_px' in row['evaluation']]) for axis in ('x','y')},
                'feedback_error_deg':{axis:stats([row['evaluation'][axis+'_error_deg'] for row in usable]) for axis in ('az','el')},
                'sent_commands':sum(row.get('command_sent',False) for row in self.rows)}

    def run(self,enabled=False):
        try:
            try:
                while self.started is None or self.clock()-self.started<self.config['duration_s']:
                    started=self.clock(); self.step(enabled)
                    remaining=1/self.config['fps']-(self.clock()-started)
                    if remaining>0: self.sleep(remaining)
            except BenchComplete:
                pass
            self.completed=True
            return self.summary()
        finally:
            self.close()

    def close(self):
        if self.closed: return
        self.closed=True
        stop_error=None
        try: self._stop()
        except Exception as error:
            stop_error=error; self.fault=self.fault or 'Final stop failed: '+str(error)
        finally:
            if self.log: self.log.close(); self.log=None
            if self.output_dir is not None:
                with (self.output_dir/'summary.json').open('x') as handle:
                    json.dump(self.summary(),handle,indent=2,allow_nan=False)
        if stop_error is not None: raise BenchFault('Final stop failed: '+str(stop_error)) from stop_error
