"""Relative optical bench observations, with no motor or flight-calibration API.

The caller supplies a physical reference JPEG and a measured scene-displacement
Jacobian (pixels per controller degree). Positive scene displacement is used as
measured, without an invented sign reversal. Position is solve(J, displacement).
HTTP publication age is conservative timing, not a synchronized exposure time.
"""
from concurrent.futures import ThreadPoolExecutor, TimeoutError
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from bench_hil import BenchObservation
from det_types import Pose
from transport import _http, _json


class OpticalBenchSource:
    def __init__(self, baseline_jpeg, jacobian_px_per_deg, *, scale_reference,
                 camera_url='http://127.0.0.1:18082/frame/0.jpg',
                 encoder_url='http://127.0.0.1:18085/encoders', timeout_s=.15,
                 max_age_s=.25, min_inliers=16, max_fit_error_px=1.5,
                 diagnostics_path=None, fetch=None, clock=time.monotonic):
        self.clock=clock;self.fetch=fetch or _http
        self.camera_url=camera_url;self.encoder_url=encoder_url
        self.timeout=float(timeout_s);self.max_age=float(max_age_s)
        self.min_inliers=int(min_inliers);self.max_fit_error=float(max_fit_error_px)
        if (not all(math.isfinite(v) and v>0 for v in (self.timeout,self.max_age,self.max_fit_error))
                or self.timeout>1 or self.min_inliers<8):
            raise ValueError('Use positive bounded timeout/age/fit settings and at least eight inliers')
        if not isinstance(scale_reference,str) or not scale_reference.strip():
            raise ValueError('An explicit physical scale reference is required')
        self.scale_reference=scale_reference
        self.jacobian=np.array(jacobian_px_per_deg,dtype=float,copy=True)
        if (self.jacobian.shape!=(2,2) or not np.isfinite(self.jacobian).all()
                or abs(np.linalg.det(self.jacobian))<=1e-9 or np.linalg.cond(self.jacobian)>100):
            raise ValueError('Jacobian must be a finite nonsingular well-conditioned 2x2 matrix')
        self.jacobian.flags.writeable=False
        raw=bytes(baseline_jpeg) if isinstance(baseline_jpeg,(bytes,bytearray)) else Path(baseline_jpeg).read_bytes()
        if not raw.startswith(b'\xff\xd8'):
            raise ValueError('The baseline must be a JPEG')
        self.baseline=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_GRAYSCALE)
        if self.baseline is None:raise ValueError('Cannot decode baseline JPEG')
        self.baseline_sha256=hashlib.sha256(raw).hexdigest()
        self.height,self.width=self.baseline.shape
        self.scale=min(1.,728/self.width)
        self.small=cv2.resize(self.baseline,None,fx=self.scale,fy=self.scale,interpolation=cv2.INTER_AREA)
        self.features=cv2.goodFeaturesToTrack(self.small,maxCorners=400,qualityLevel=.015,minDistance=6,blockSize=7)
        if self.features is None or len(self.features)<self.min_inliers:
            raise ValueError('Baseline has insufficient trackable scene features')
        self.window=cv2.createHanningWindow((self.small.shape[1],self.small.shape[0]),cv2.CV_32F)
        self._sequence=-1;self._generation=None
        self._lock=threading.Lock();self._closed=False;self._pending=[]
        self.diagnostics={};self._log=None
        self.last_frame_jpeg=None;self.last_image=None;self.last_frame_timestamp=None;self.last_frame_sequence=None
        if diagnostics_path is not None:
            path=Path(diagnostics_path);path.parent.mkdir(parents=True,exist_ok=True)
            self._log=path.open('x')
        self._pool=ThreadPoolExecutor(max_workers=2,thread_name_prefix='det-optical-bench')

    def _register(self,image):
        if image.shape!=self.baseline.shape:raise ValueError('Camera dimensions differ from physical baseline')
        current=cv2.resize(image,(self.small.shape[1],self.small.shape[0]),interpolation=cv2.INTER_AREA)
        shift,response=cv2.phaseCorrelate(self.small.astype(np.float32),current.astype(np.float32),self.window)
        guess=np.array(shift,dtype=np.float32) if math.isfinite(response) and response>.05 else np.zeros(2,np.float32)
        if not np.isfinite(guess).all():guess[:]=0
        initial=self.features+guess.reshape(1,1,2)
        options=dict(winSize=(25,25),maxLevel=4,
                     criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        moved,status,_=cv2.calcOpticalFlowPyrLK(self.small,current,self.features,initial,
                                             flags=cv2.OPTFLOW_USE_INITIAL_FLOW,**options)
        if moved is None or status is None:raise ValueError('Optical feature tracking failed')
        finite=np.isfinite(moved).all(axis=(1,2))
        moved=np.nan_to_num(moved)
        back,back_status,_=cv2.calcOpticalFlowPyrLK(current,self.small,moved,None,**options)
        if back is None or back_status is None:raise ValueError('Reverse optical tracking failed')
        error=np.linalg.norm(back-self.features,axis=2).ravel()
        points=moved.reshape(-1,2)
        good=(status.ravel()==1)&(back_status.ravel()==1)&finite&(error<=max(.5,self.scale))
        good&=(points[:,0]>=1)&(points[:,0]<current.shape[1]-1)&(points[:,1]>=1)&(points[:,1]<current.shape[0]-1)
        reference=self.features.reshape(-1,2)[good];tracked=points[good]
        if len(reference)<self.min_inliers:raise ValueError('Insufficient consistent optical features')
        matrix,mask=cv2.estimateAffine2D(reference,tracked,method=cv2.RANSAC,
            ransacReprojThreshold=max(.5,self.max_fit_error*self.scale),maxIters=1500,confidence=.99,refineIters=10)
        if matrix is None or mask is None or not np.isfinite(matrix).all():
            raise ValueError('Robust affine scene registration failed')
        inlier=mask.ravel().astype(bool);count=int(inlier.sum());ratio=count/len(reference)
        if count<self.min_inliers or ratio<.5:raise ValueError('Too few robust affine inliers')
        projected=reference@matrix[:,:2].T+matrix[:,2]
        rms=float(np.sqrt(np.mean(np.sum((projected[inlier]-tracked[inlier])**2,axis=1)))/self.scale)
        singular=np.linalg.svd(matrix[:,:2],compute_uv=False)
        coverage=float(cv2.contourArea(cv2.convexHull(reference[inlier].astype(np.float32))))/self.small.size
        confidence=float(min(1.,count/30)*ratio*math.exp(-rms/(2*self.max_fit_error)))
        if rms>self.max_fit_error or coverage<.02 or np.max(np.abs(singular-1))>.05 or confidence<.5:
            raise ValueError('Poor affine fit, spatial coverage, or confidence')
        # Uniform image scaling preserves the affine linear term. Pixel-center
        # transforms account for the half-pixel resize convention in translation.
        center=np.array([(self.width-1)/2,(self.height-1)/2])
        small_center=(center+.5)*self.scale-.5
        displacement=(matrix[:,:2]@small_center+matrix[:,2]-small_center)/self.scale
        return {'inliers':count,'tracked_features':len(reference),'confidence':confidence,
                'fit_error_px':rms,'spatial_coverage':coverage,
                'phase_correlation_response':float(response) if math.isfinite(response) else 0.,
                'measured_displacement_px':displacement.tolist()}

    def _frame(self,result):
        raw,headers,started,received=result
        if headers.get('content-type','').split(';')[0].strip()!='image/jpeg':
            raise ValueError('Camera response is not a JPEG')
        info=_json(headers.get('x-frame-info','{}'))
        if not isinstance(info,dict) or info.get('error'):raise ValueError('Camera metadata reports an error')
        sequence=info.get('sequence');age=info.get('age_s')
        if type(sequence) is not int or sequence<=0 or sequence<=self._sequence:
            raise ValueError('Camera sequence repeated or regressed')
        if (isinstance(age,bool) or not isinstance(age,(int,float)) or not math.isfinite(age) or age<0
                or not all(math.isfinite(v) for v in (started,received)) or received<started):
            raise ValueError('Invalid camera timing metadata')
        stamp=received-age-(received-started)
        if not -.01<=self.clock()-stamp<=self.max_age:raise ValueError('Camera frame is stale or future dated')
        if info.get('size')!=[self.width,self.height]:raise ValueError('Camera metadata dimensions differ from baseline')
        generation=headers.get('x-gateway-generation')
        if generation is not None and info.get('gateway_generation')!=generation:
            raise ValueError('Camera generation metadata disagrees')
        if self._generation is not None and generation!=self._generation:
            raise ValueError('Camera process changed during optical bench trial')
        image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_GRAYSCALE)
        if image is None:raise ValueError('Invalid camera JPEG')
        if image.shape!=self.baseline.shape:raise ValueError('Camera dimensions differ from physical baseline')
        self._sequence=sequence
        self._generation=generation
        self.last_frame_jpeg=bytes(raw);self.last_image=image.copy()
        self.last_frame_timestamp=stamp;self.last_frame_sequence=sequence
        return image,stamp,{'request_started':started,'request_completed':received,
                           'round_trip_s':received-started,'reported_age_s':age,
                           'capture_time_estimate':stamp,'sequence':sequence,
                           'gateway_generation':generation,'timing_kind':'conservative_publication_age'}

    def read(self):
        with self._lock:
            if self._closed:raise RuntimeError('Optical bench source is closed')
            self.last_frame_jpeg=None;self.last_image=None;self.last_frame_timestamp=None;self.last_frame_sequence=None
            started=self.clock();wall_started=time.perf_counter()
            registration={'valid':False,'age_s':0.,'inliers':0,'confidence':0.,
                'scale_reference':self.scale_reference,'jacobian_px_per_deg':self.jacobian.tolist(),
                'measured_displacement_px':[0.,0.],'baseline_sha256':self.baseline_sha256,
                'scope':'relative_optical_bench_not_flight_calibration','error':None}
            health={'readings':[],'error':None};timing={};stamp=started;angles=np.zeros(2)
            try:
                if any(not future.done() for future in self._pending):
                    raise ValueError('Previous HTTP request is still pending; no queued retry')
                deadline=time.monotonic()+self.timeout+.02
                camera=self._pool.submit(self.fetch,self.camera_url,self.timeout,limit=8*1024*1024)
                encoders=self._pool.submit(self.fetch,self.encoder_url,self.timeout,limit=65536)
                self._pending=[camera,encoders]
                camera_result=None
                try:camera_result=camera.result(timeout=max(0.,deadline-time.monotonic()))
                except TimeoutError:registration['error']='Camera HTTP request timed out'
                except Exception as error:registration['error']='Camera fetch: '+str(error)
                try:
                    raw,_,enc_started,enc_received=encoders.result(timeout=max(0.,deadline-time.monotonic()))
                    readings=_json(raw)
                    if not isinstance(readings,list) or not all(isinstance(row,dict) for row in readings):
                        raise ValueError('Encoder diagnostics must be a list of objects')
                    json.dumps(readings,allow_nan=False)
                    health.update(readings=readings,request_started=enc_started,request_completed=enc_received)
                except TimeoutError:health['error']='Encoder HTTP request timed out'
                except Exception as error:health['error']='Encoder diagnostics: '+str(error)
                if camera_result is None:raise ValueError(registration['error'] or 'No camera response')
                image,stamp,timing=self._frame(camera_result)
                registration.update(self._register(image))
                angles=np.linalg.solve(self.jacobian,np.asarray(registration['measured_displacement_px']))
                age=self.clock()-stamp
                if not np.isfinite(angles).all() or not 0<=age<=self.max_age:
                    raise ValueError('Optical position is nonfinite or stale after registration')
                registration.update(valid=True,age_s=age,error=None)
            except Exception as error:
                registration.update(valid=False,error=str(error),age_s=max(0.,self.clock()-stamp))
            registration['processing_ms']=(time.perf_counter()-wall_started)*1000
            registration['camera_timing']=dict(timing)
            self.diagnostics={'read_started':started,'read_completed':self.clock(),
                              'camera_timing':timing,'encoder_health':health,'registration':registration}
            if self._log:
                self._log.write(json.dumps(self.diagnostics,allow_nan=False)+'\n');self._log.flush()
            return BenchObservation(Pose(stamp,float(angles[0]),float(angles[1]),registration['valid']),
                {'az':'camera_registration','el':'camera_registration'},health,registration)

    def close(self):
        self._closed=True
        self._pool.shutdown(wait=False,cancel_futures=True)
        if self._log:
            self._log.close();self._log=None
