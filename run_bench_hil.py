#!/usr/bin/env python3
"""Mac bench runner, synthetic observations with real measured camera feedback.

Requires a local image registration, reviewed leased motion gateway, and a
separate read-only encoder endpoint. This cannot arm the flight runtime or
mark camera/encoder flight calibration complete.
"""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):
    os.environ[name]='1'
import argparse
import json
import time
from pathlib import Path
import cv2
from bench_hil import BenchHilRunner
from optical_bench_source import OpticalBenchSource
from transport import HttpMotor
from det_types import CameraModel


class LoggedMotor:
    def __init__(self,motor,path):
        self.motor=motor
        self.log=path.open('x')
    def send(self,command):
        started=time.monotonic()
        result=self.motor.send(command)
        self.log.write(json.dumps({'started':started,'completed':time.monotonic(),
                                  'sequence':command.sequence,'status':result})+'\n')
        self.log.flush()
        return result
    def stop(self):
        return self.motor.stop()
    def close(self):
        self.log.close()


class RecordingSource:
    def __init__(self,source,directory):
        self.source=source
        self.directory=directory
        self.sequence=0
    def read(self):
        observation=self.source.read()
        raw=self.source.last_frame_jpeg
        if raw is not None:
            self.sequence+=1
            (self.directory/('physical-%06d.jpg'%self.sequence)).write_bytes(raw)
        return observation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registration',type=Path,required=True)
    parser.add_argument('--settings',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--execute',action='store_true',help='Command real relative bench motion')
    parser.add_argument('--warmup-frames',type=int,default=4)
    parser.add_argument('--motor-url',default='http://127.0.0.1:18084')
    parser.add_argument('--motor-timeout',type=float,default=.05,
                        help='Measured HTTP transport allowance; command expiry stays configured')
    parser.add_argument('--camera-profile',type=Path,
                        default=Path(__file__).with_name('live_native_config.json'))
    args=parser.parse_args()
    if not 2<=args.warmup_frames<=30:
        parser.error('Use 2..30 acquisition frames before motion')
    cv2.setNumThreads(1)
    registration=json.loads(args.registration.read_text())
    settings=json.loads(args.settings.read_text()) if args.settings else {}
    settings['bench_registration']=registration['scale_reference']
    camera_config=json.loads(args.camera_profile.read_text())['cameras']
    cameras={}
    for name,original in camera_config.items():
        sx,sy=320/original['width'],240/original['height']
        cameras[name]=CameraModel(name,320,240,original['fx']*sx,original['fy']*sy,
                                 (original['cx']+.5)*sx-.5,(original['cy']+.5)*sy-.5)
    source=OpticalBenchSource(registration['baseline'],registration['jacobian_px_per_deg'],
                             scale_reference=registration['scale_reference'],
                             diagnostics_path=Path(str(args.out)+'.source.jsonl'))
    motor=None
    runner=None
    try:
        if args.execute:
            motor=LoggedMotor(HttpMotor(args.motor_url,timeout_s=args.motor_timeout),
                              Path(str(args.out)+'.motor.jsonl'))
        runner=BenchHilRunner(RecordingSource(source,args.out),motor,
                              cameras=cameras,config=settings,output_dir=args.out)
        if args.execute:
            for _ in range(args.warmup_frames):
                runner.step(enabled=False)
                time.sleep(1/runner.config['fps'])
        summary=runner.run(enabled=args.execute)
        print(json.dumps(summary,indent=2,allow_nan=False),flush=True)
    finally:
        try:
            if runner is not None:runner.close()
            elif motor is not None:motor.stop()
        finally:
            try:
                if motor is not None:motor.close()
            finally:
                source.close()


if __name__=='__main__':
    main()
