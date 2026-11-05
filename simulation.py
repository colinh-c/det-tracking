"""Synthetic camera observations and a delayed gimbal plant; truth is evaluation-only."""
from dataclasses import asdict
from pathlib import Path
import csv
import math
import json
import numpy as np
import cv2
from det_types import Pose, Frame, angle_error
from field_config import load_config


class SyntheticFlight:
    def __init__(self, cameras, trajectory, field_config, fps=15, start_s=0, seed=7,
                 angular_bias_deg=0, time_bias_s=0, dropout=None, encoder_step_deg=0):
        self.cameras=cameras
        self.fps=float(fps)
        if self.fps <= 0: raise ValueError('FPS must be positive')
        with open(trajectory,newline='') as handle:
            rows=list(csv.DictReader(handle))
        self.times=np.array([float(row['Time']) for row in rows])
        pad=np.array(load_config(field_config).pad_position_enu())
        self.positions=np.array([[float(row['Position East of launch']),float(row['Position North of launch']),float(row['Altitude'])] for row in rows])+pad
        self.t=float(start_s); self.end_s=float(self.times[-1]); self.epoch=1000.0
        self.rng=np.random.default_rng(seed); self.bias=float(angular_bias_deg); self.time_bias=float(time_bias_s)
        self.dropout=dropout or []; self.encoder_step_deg=float(encoder_step_deg)
        az,el,_=self.target(self.t)
        self.pose=Pose(self.now,az+.1,el-.08)
        self.velocity=np.zeros(2); self.pending=[]; self.command=None; self.sequence=0

    @property
    def now(self): return self.epoch+self.t

    def target(self,t):
        t=float(np.clip(t+self.time_bias,self.times[0],self.times[-1]))
        pos=np.array([np.interp(t,self.times,self.positions[:,i]) for i in range(3)])
        horizontal=math.hypot(pos[0],pos[1]); distance=float(np.linalg.norm(pos))
        az=math.degrees(math.atan2(pos[0],pos[1]))+self.bias+.025*math.sin(t*.33)
        el=math.degrees(math.atan2(pos[2],horizontal))+.02*math.sin(t*.8)
        return az%360,el,distance

    def phase(self):
        # Evaluation/rendering only. Never returned as an algorithm observation.
        if self.t < 4.264: return 'boost'
        if self.t < 25.514: return 'coast'
        if self.t < 110.877: return 'drogue'
        if self.t < 139.738: return 'main'
        return 'landed'

    def read(self):
        az,el,distance=self.target(self.t)
        measured=self.pose
        if self.encoder_step_deg > 0:
            step=self.encoder_step_deg
            measured=Pose(self.now,round(self.pose.az_deg/step)*step%360,round(self.pose.el_deg/step)*step)
        phase=self.phase(); frames=[]; truth={'time_s':self.t,'phase':phase,'az_deg':az,'el_deg':el,'visible':{},'target_px':{},'boresight_inside_body':{}}
        for name,camera in self.cameras.items():
            # Low-order sky gradient, sensor noise, stationary bright distractor.
            h,w=camera.height,camera.width
            row=np.linspace(146,164,h,dtype=np.float32)[:,None]
            image=np.clip(row+self.rng.normal(0,1.5,(h,w)),0,255).astype(np.uint8)
            point=camera.project(az,el,self.pose)
            visible=camera.inside(point)
            hidden=any(a<=self.t<b and name in names for a,b,names in self.dropout)
            truth['visible'][name]=bool(visible and not hidden)
            truth['target_px'][name]=list(point) if point is not None else None
            truth['boresight_inside_body'][name]=False
            # Vary body swing, independently of its center. Body is never rendered at prior coordinates by the detector.
            if point is not None and not hidden:
                x,y=point; length=max(3,camera.fy*2.4516/max(distance,1)); thick=max(1,int(camera.fx*.1567/max(distance,1)))
                swing=.25*math.sin(self.t*2) if phase in ('main','drogue') else -.08*math.sin(self.t)
                # Render COM at point; CG fraction not midpoint.
                direction=np.array([math.sin(swing),math.cos(swing)])
                a=np.array(point)-direction*(length*.62653); b=np.array(point)+direction*(length*.37347)
                segment=b-a
                q=np.array([camera.cx,camera.cy])
                along=float(np.clip((q-a)@segment/max(float(segment@segment),1e-12),0,1))
                truth['boresight_inside_body'][name]=bool(np.linalg.norm(q-(a+along*segment)) <= max(.5,thick/2))
                a=np.clip(a,-5000,5000).astype(int); b=np.clip(b,-5000,5000).astype(int)
                cv2.line(image,tuple(a),tuple(b),45,thick,cv2.LINE_AA)
                if phase in ('main','drogue'):
                    diameter=2.1336 if phase=='main' else .61
                    radius=max(2,int(camera.fx*diameter/max(distance,1)/2))
                    canopy_y=int(y-camera.fy*4/max(distance,1))
                    # Canopy intentionally brighter and larger than rocket body.
                    cv2.ellipse(image,(int(x),canopy_y),(radius,max(2,radius//2)),0,180,360,235,-1)
            cv2.circle(image,(int(w*.82),int(h*.28)),3,235,-1)
            frames.append(Frame(name,self.now,image,measured,self.sequence))
        self.truth=truth
        return frames,measured

    def send(self,command):
        # Model a 70 ms transport/actuation delay and lease on target plant clock.
        self.pending.append((self.now+.07,command))

    def advance(self):
        dt=1/self.fps
        while self.pending and self.pending[0][0]<=self.now:
            _,self.command=self.pending.pop(0)
        desired=np.zeros(2)
        if self.command and self.command.enabled and self.command.valid_until>=self.now:
            desired=np.array([self.command.az_velocity_deg_s,self.command.el_velocity_deg_s])
        desired=np.clip(desired,-15,15)
        self.velocity += np.clip(desired-self.velocity,-75*dt,75*dt)
        self.t+=dt; self.sequence+=1
        self.pose=Pose(self.now,(self.pose.az_deg+self.velocity[0]*dt)%360,self.pose.el_deg+self.velocity[1]*dt)

    def close(self): pass


class Metrics:
    def __init__(self): self.rows=[]

    def observe(self,truth,result,prediction,pose,elapsed_ms):
        target=result.selected
        error=None
        if target:
            error=math.hypot(angle_error(target.bearing_deg,truth['az_deg'])*math.cos(math.radians(truth['el_deg'])),target.elevation_deg-truth['el_deg'])
        pointing=math.hypot(angle_error(pose.az_deg,truth['az_deg'])*math.cos(math.radians(truth['el_deg'])),pose.el_deg-truth['el_deg'])
        self.rows.append({**truth,'selected_camera':target.camera if target else None,'detection_error_deg':error,'correct_detection':bool(error is not None and error<.2),'pointing_error_deg':pointing,'inferred_phase':result.phase.name,'phase_source':result.phase.source,'algorithm_ms':elapsed_ms})

    def summary(self,fps):
        def part(rows):
            n=len(rows)
            if not n: return {}
            errors=[r['pointing_error_deg'] for r in rows]
            longest=run=0
            for row in rows:
                run=0 if row['correct_detection'] else run+1
                longest=max(longest,run)
            return {'frames':n,'duration_s':n/fps,'correct_detection_fraction_within_0_2_deg':sum(r['correct_detection'] for r in rows)/n,'narrow_visible_fraction':sum(r['visible'].get('narrow',False) for r in rows)/n,'either_camera_visible_fraction':sum(any(r['visible'].values()) for r in rows)/n,'longest_detection_loss_s':longest/fps,'wrong_target_detection_fraction':sum(r['selected_camera'] is not None and not r['correct_detection'] for r in rows)/n,'narrow_boresight_inside_body_s':sum(r.get('boresight_inside_body',{}).get('narrow',False) for r in rows)/fps,'pointing_error_deg_p50_p95':np.percentile(errors,[50,95]).tolist(),'time_within_pointing_tolerance_s':{str(t):sum(e<=t for e in errors)/fps for t in (.01,.025,.05,.1,.25,.5,1,5)},'algorithm_ms_p50_p95':np.percentile([r['algorithm_ms'] for r in rows],[50,95]).tolist()}
        return {'status':'synthetic_only_not_physical_validation','all_flight':part(self.rows),'main':part([r for r in self.rows if r['phase']=='main']),'evaluation_threshold_deg':.2,'threshold_is_not_a_user_acceptance_requirement':True,'truth_is_evaluation_only':True,'assumptions':['pinhole cameras, ideal calibration','saved trajectory with optional angular/time mismatch','simplified canopy and body with swing','delayed motor plant, configured speed/acceleration ceilings','not a photorealistic or real flight test']}
