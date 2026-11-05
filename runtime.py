"""One processing path for synthetic, replay and real Pi observations."""
from dataclasses import asdict,replace
from copy import deepcopy
from pathlib import Path
import json
import threading
import time
import math
import cv2
from det_types import Prediction, Pose, Command
from perception import Perception
from prediction import AngularPrior, Predictor
from execution import Executor
from launch_config import live_readiness


class TrackingEngine:
    def __init__(self,cfg,clock=time.monotonic):
        self.cfg=cfg; self.clock=clock
        self.perception=Perception(cfg['camera_models'],{**cfg.get('perception',{}),'clock':clock})
        self.prior=AngularPrior(cfg['trajectory'],cfg['field_config'])
        self.predictor=Predictor(self.prior,cfg.get('predictor'))
        self.executor=Executor(cfg.get('executor'))
        self.previous=None; self.click_hint=None

    def step(self,frames,pose,now,elapsed=None,enabled=False,phase_override=None):
        hint=self.previous
        if self.click_hint is not None and now-self.click_hint.timestamp<1:
            hint=self.click_hint
        # An expired control prediction ends prior-only motor search, but it
        # must not remove the independent trajectory hint from image matching.
        # Perception still has to find supported visual evidence to reacquire.
        if (hint is None or not hint.valid) and elapsed is not None:
            hint=self.prior.sample(elapsed)
            hint.timestamp=now
        result=self.perception.update(frames,hint,phase_override,elapsed)
        prediction=self.predictor.update(result,pose,now,elapsed,self.cfg.get('latency_s',.1))
        self.previous=prediction  # Perception hints stay in world coordinates.
        control=replace(prediction)
        if prediction.valid:
            camera=self.cfg['camera_models']['narrow']
            try:
                az,el=camera.mount_for_boresight(prediction.az_deg,prediction.el_deg)
                future=camera.mount_for_boresight(prediction.az_deg+prediction.az_rate_deg_s*.01,prediction.el_deg+prediction.el_rate_deg_s*.01)
                from det_types import angle_error
                control.az_deg,control.el_deg=az,el
                control.az_rate_deg_s=angle_error(future[0],az)/.01
                control.el_rate_deg_s=(future[1]-el)/.01
                detection=result.selected
                frame=next((f for f in frames if detection is not None and f.camera==detection.camera and abs(f.timestamp-detection.timestamp)<1e-6),None)
                if detection is not None and detection.camera=='narrow' and frame is not None and frame.pose.valid and detection.confidence>=.4:
                    bbox=detection.bbox
                    contained=bbox is not None and bbox[0]>1 and bbox[1]>1 and bbox[2]<camera.width-1 and bbox[3]<camera.height-1
                    if contained:
                        mount=camera.mount_for_boresight(detection.bearing_deg,detection.elevation_deg)
                        control.image_error_az_deg=angle_error(mount[0],frame.pose.az_deg)
                        control.image_error_el_deg=mount[1]-frame.pose.el_deg
                        control.image_timestamp=frame.timestamp
            except ValueError as error:
                control.valid=False;control.reason='Camera/mount transform: '+str(error)
        command=self.executor.update(control,pose,now,enabled)
        return result,control,command


class TrackingApp:
    def __init__(self,cfg,mode,source,allow_motion=False,run_dir=None):
        self.cfg=cfg; self.mode=mode; self.source=source; self.allow_motion=allow_motion
        self.now=source.now if mode in ('simulate','replay') else time.monotonic()
        self.engine=TrackingEngine(cfg,clock=lambda:self.now)
        self.lock=threading.RLock(); self.frames={}; self.status={}; self.error=None
        self.enabled=mode=='simulate'; self.phase_override=None; self.launch_origin=None
        self.stopped=False; self.motor=None; self.run_dir=Path(run_dir) if run_dir else None
        self.events=[]; self.last_result=None; self.last_prediction=None; self.command_sequence=0
        self._gateway_generation=None
        self.log=None
        if self.run_dir:
            self.run_dir.mkdir(parents=True,exist_ok=True)
            self.log=(self.run_dir/'telemetry.jsonl').open('w')

    def action(self,action,payload=None):
        payload=payload or {}
        with self.lock:
            if self.stopped:
                if action in ('stop','manual'):return {'enabled':False,'stopped':True}
                raise ValueError('Tracker is closed; restart it before control')
            if action in ('stop','manual'):
                self.enabled=False
                self.engine.executor.reset()
                if self.motor:
                    try:self.motor.stop()
                    finally:self.motor=None
            elif action=='start':
                if self.error: raise ValueError('Clear the fault by restarting after checking the link')
                if self.mode=='live':
                    issues=live_readiness(self.cfg)
                    if not self.allow_motion: issues.insert(0,'Start with --allow-motion to permit automatic tracking')
                    if getattr(self.source,'generation_verified',False) is not True:
                        issues.append('Capture gateway generation must be verified before automatic tracking')
                    if issues: raise ValueError('; '.join(issues))
                    from transport import HttpMotor
                    self.motor=HttpMotor(self.cfg['motor_base_url'],timeout_s=self.cfg.get('motor_timeout_s',.05))
                elif self.mode=='replay': raise ValueError('Replay is read only')
                self.enabled=True
            elif action=='launch':
                self.launch_origin=self.now
            elif action=='phase':
                phase=payload.get('phase')
                if phase not in (None,'unknown','boost','coast','drogue','main','landed'):
                    raise ValueError('Unknown phase')
                self.phase_override=None if phase in (None,'unknown') else phase
            elif action=='acquire':
                name=payload.get('camera'); frame=self.frames.get(name)
                if frame is None: raise ValueError('No current camera frame')
                if self.now-frame.timestamp>.5: raise ValueError('Camera frame is stale')
                camera=self.cfg['camera_models'][name]
                x,y=float(payload['x']),float(payload['y'])
                if not math.isfinite(x+y) or not camera.inside((x,y)): raise ValueError('Click inside the image')
                az,el=camera.unproject(x,y,frame.pose)
                self.engine=TrackingEngine(self.cfg,clock=lambda:self.now)
                self.engine.click_hint=Prediction(self.now,az,el,valid=True,source='manual_acquisition',uncertainty_deg=.3)
            else: raise ValueError('Unknown action')
            self.events.append({'time':self.now,'action':action,'payload':payload})
            return {'enabled':self.enabled,'phase_override':self.phase_override}

    def _capture_generation(self):
        """Disarm once per verified process transition before using new input."""
        if self.mode!='live':return False
        generation=getattr(self.source,'generation',None)
        verified=getattr(self.source,'generation_verified',False) is True
        if not verified or not isinstance(generation,str) or not generation:return False
        previous=self._gateway_generation
        changed=(generation!=previous and (previous is not None or
                 getattr(self.source,'generation_changed',False) is True))
        self._gateway_generation=generation
        if not changed:return False
        self.enabled=False
        motor,self.motor=self.motor,None
        if motor is not None:
            try:motor.stop()
            except Exception as error:
                self.error='Capture gateway restarted; motor Stop failed: '+str(error)
        # Global command_sequence deliberately belongs to the app, not the
        # recreated filters, so remote command anti-replay remains monotonic.
        self.engine=TrackingEngine(self.cfg,clock=lambda:self.now)
        self.last_result=None;self.last_prediction=None
        self.events.append({'time':self.now,'action':'capture_gateway_restarted',
                            'payload':{'previous_generation':previous,'generation':generation}})
        return True

    def _source_status(self,frames,changed):
        if self.mode!='live':
            return {'gateway_generation':None,'generation_verified':False,'generation_revision':0,
                    'generation_changed':False,'encoder_readings':[],'source_errors':{},
                    'camera_info':{},'connection_status':'not_live'}
        generation=getattr(self.source,'generation',None)
        verified=getattr(self.source,'generation_verified',False) is True
        legacy=(not verified and generation is None and bool(frames) and
                self.cfg.get('encoders',{}).get('allow_legacy_gateway') is True)
        return {'gateway_generation':generation,'generation_verified':verified,
                'generation_revision':getattr(self.source,'generation_revision',0),
                'generation_changed':changed,
                'encoder_readings':deepcopy(getattr(self.source,'encoder_readings',[])),
                'source_errors':deepcopy(getattr(self.source,'errors',{})),
                'camera_info':deepcopy(getattr(self.source,'camera_info',{})),
                'connection_status':'verified' if verified else 'legacy_preview' if legacy else 'unverified'}

    def tick(self):
        with self.lock:
            if self.stopped:raise RuntimeError('Tracker is closed')
        frames,pose=self.source.read()
        with self.lock:
            if self.stopped:raise RuntimeError('Tracker closed while reading observations')
            self.now=self.source.now if self.mode in ('simulate','replay') else time.monotonic()
            elapsed=self.source.t if self.mode=='simulate' else (getattr(self.source,'launch_elapsed_s',None) if self.mode=='replay' else (self.now-self.launch_origin if self.launch_origin is not None else None))
            generation_changed=self._capture_generation()
            if generation_changed:
                pose=replace(pose,valid=False)
                frames=[replace(frame,pose=replace(frame.pose,valid=False)) for frame in frames]
            self.frames={frame.camera:frame for frame in frames}
            readiness=live_readiness(self.cfg)
            if self.mode=='live' and self.enabled and readiness:
                self.error='Live prerequisites changed: '+'; '.join(readiness)
                self.action('stop')
            start=time.perf_counter()
            result,prediction,command=self.engine.step(frames,pose,self.now,elapsed,self.enabled,self.phase_override)
            self.command_sequence+=1
            command.sequence=self.command_sequence
            algorithm_ms=(time.perf_counter()-start)*1000
            if self.mode=='simulate': self.source.send(command)
            elif self.motor:
                # Keep a zero-velocity lease while fresh observations support
                # waiting for reacquisition. Stale input/faults still disarm.
                fresh_pose=pose.valid and 0 <= self.now-pose.timestamp <= .3
                fresh_frames=any(0 <= self.now-frame.timestamp <= .3 for frame in frames)
                if (not command.enabled and self.enabled and fresh_pose and fresh_frames
                        and prediction.reason.startswith('Target loss')):
                    command=Command(self.now,0.,0.,self.now+.25,True,'Stationary; waiting for reacquisition',command.sequence)
                try:
                    if command.enabled:self.motor.send(command)
                    else:
                        self.motor.stop(); self.motor=None; self.enabled=False
                except Exception as error:
                    self.error='Motor transport fault: '+str(error); self.enabled=False
                    try:self.motor.stop()
                    except Exception:pass
                    self.motor=None
            self.last_result=result; self.last_prediction=prediction
            self.status={'mode':self.mode,'automatic_enabled':self.enabled,'launch_elapsed_s':elapsed,'phase':asdict(result.phase),'selected':asdict(result.selected) if result.selected else None,'prediction':asdict(prediction),'command':asdict(command),'pose':asdict(pose),'narrow_found':result.narrow_found,'wide_found':result.wide_found,'warnings':result.warnings,'algorithm_ms':algorithm_ms,'readiness_issues':readiness,'error':self.error,'phase_override':self.phase_override,'frame_time':self.now,'camera_sizes':{n:[c.width,c.height] for n,c in self.cfg['camera_models'].items()}}
            self.status.update(self._source_status(frames,generation_changed))
            if self.log:
                self.log.write(json.dumps(self.status,allow_nan=False)+'\n')
                self.log.flush()
            return result,prediction,command,algorithm_ms

    def image(self,name):
        with self.lock:
            frame=self.frames.get(name)
            if frame is None:return None
            image=cv2.cvtColor(frame.image,cv2.COLOR_GRAY2BGR) if frame.image.ndim==2 else frame.image.copy()
            h,w=image.shape[:2]
            cv2.drawMarker(image,(w//2,h//2),(0,220,220),cv2.MARKER_CROSS,20,1)
            if self.last_result:
                for candidate in self.last_result.candidates.get(name,[]):
                    cv2.drawMarker(image,(round(candidate.x),round(candidate.y)),(0,180,0),cv2.MARKER_CROSS,12,1)
                selected=self.last_result.selected
                if selected and selected.camera==name:
                    cv2.drawMarker(image,(round(selected.x),round(selected.y)),(0,0,255),cv2.MARKER_CROSS,24,2)
                    if selected.bbox:
                        x0,y0,x1,y1=selected.bbox; cv2.rectangle(image,(x0,y0),(x1,y1),(0,180,0),1)
            return image

    def close(self):
        with self.lock:
            if self.stopped:return
            self.stopped=True; self.enabled=False
            if self.motor:
                try:self.motor.close()
                except Exception as error:self.error=str(error)
                self.motor=None
            try:self.source.close()
            finally:
                if self.log:self.log.close();self.log=None
