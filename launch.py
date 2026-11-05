#!/usr/bin/env python3
"""DET Mac stack. Default synthetic flight; live actuation needs explicit arming."""
import os
# Small matrices and image components do not need competing worker pools.
for _thread_variable in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","VECLIB_MAXIMUM_THREADS"):
    os.environ[_thread_variable]="1"
from dataclasses import asdict
from pathlib import Path
import argparse
import json
import math
import signal
import threading
import time
import cv2
cv2.setNumThreads(1)
import numpy as np
from launch_config import load_launch_config,live_readiness
from runtime import TrackingApp
from simulation import SyntheticFlight,Metrics
from dashboard import create_server


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(Path(__file__).with_name('launch_config.json')))
    parser.add_argument('--mode',choices=['simulate','replay','live'],default='simulate')
    parser.add_argument('--replay-dir')
    parser.add_argument('--loop',action='store_true',help='Repeat an unrecorded displayed synthetic trial')
    parser.add_argument('--serve',action='store_true',help='Show local dual-camera dashboard')
    parser.add_argument('--port',type=int,default=18183)
    parser.add_argument('--allow-motion',action='store_true',help='Permit live automatic arming after final calibration checks')
    parser.add_argument('--start-s',type=float,default=0)
    parser.add_argument('--duration-s',type=float)
    parser.add_argument('--fps',type=float,default=15)
    parser.add_argument('--encoder-step-deg',type=float,default=0,help='Synthetic encoder quantization, AS5600 ideal step is .087890625 deg')
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--angular-bias-deg',type=float,default=0)
    parser.add_argument('--time-bias-s',type=float,default=0)
    parser.add_argument('--occlusion',action='store_true',help='Synthetic 0.5 s narrow dropout during main')
    parser.add_argument('--run-dir',default=None)
    parser.add_argument('--record',action='store_true',help='Write paired raw replay videos and annotated preview')
    parser.add_argument('--check',action='store_true',help='Report live prerequisites without connecting')
    args=parser.parse_args(argv)
    cfg=load_launch_config(args.config)
    if args.check:
        issues=live_readiness(cfg)
        print(json.dumps({'software_config_valid':True,'live_automatic_prerequisites':issues},indent=2))
        return 0
    numeric=[args.fps,args.start_s,args.angular_bias_deg,args.time_bias_s,args.encoder_step_deg]+([] if args.duration_s is None else [args.duration_s])
    if not all(math.isfinite(value) for value in numeric) or args.encoder_step_deg<0 or args.fps<=0 or (args.duration_s is not None and args.duration_s<=0) or args.start_s<0:
        parser.error('Use positive fps/duration and nonnegative start time')
    if args.loop and (args.mode!='simulate' or args.record or not args.serve):parser.error('--loop requires an unrecorded simulated display')
    if args.allow_motion and args.mode!='live':parser.error('--allow-motion applies only to live mode')
    if args.mode=='simulate':
        dropout=[(117,117.5,['narrow'])] if args.occlusion else []
        source=SyntheticFlight(cfg['camera_models'],cfg['trajectory'],cfg['field_config'],args.fps,args.start_s,args.seed,args.angular_bias_deg,args.time_bias_s,dropout,args.encoder_step_deg)
    elif args.mode=='replay':
        if not args.replay_dir:parser.error('--replay-dir is required')
        from replay import ReplaySource
        source=ReplaySource(args.replay_dir,cfg['camera_models'])
    else:
        if args.allow_motion and live_readiness(cfg):parser.error('Live calibration prerequisites are pending; run --check')
        from transport import PiSource
        source=PiSource(cfg['camera_base_url'],cfg['camera_models'],cfg['encoders'],timeout_s=.15)
    directory=Path(args.run_dir) if args.run_dir else Path(__file__).with_name('runs')/time.strftime('%Y%m%d-%H%M%S')
    if directory.exists() and any(directory.iterdir()):parser.error('Run directory is not empty; choose another directory')
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'run_config.json').write_text(json.dumps({k:v for k,v in cfg.items() if k!='camera_models'},indent=2))
    (directory/'run_arguments.json').write_text(json.dumps(vars(args),indent=2))
    import hashlib
    hashes={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in Path(__file__).parent.glob('*.py')}
    (directory/'software_hashes.json').write_text(json.dumps(hashes,indent=2))
    app=TrackingApp(cfg,args.mode,source,args.allow_motion,directory)
    server=None;server_thread=None;recorder=None;stopping=threading.Event();metrics=Metrics()
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,lambda *_:stopping.set())
    wall_start=time.monotonic();count=0;last_notice=wall_start
    try:
        if args.serve:
            server=create_server(app,args.port)
            server_thread=threading.Thread(target=server.serve_forever,daemon=True);server_thread.start()
            print(f'DET {args.mode} display: http://127.0.0.1:{server.server_port}/',flush=True)
        if args.record:
            from recording import Recorder
            recorder=Recorder(directory,cfg['camera_models'],args.fps)
        while not stopping.is_set():
            if args.mode=='simulate' and source.t>source.end_s:
                if not args.loop:break
                from runtime import TrackingEngine
                with app.lock:
                    source=SyntheticFlight(cfg['camera_models'],cfg['trajectory'],cfg['field_config'],args.fps,args.start_s,args.seed,args.angular_bias_deg,args.time_bias_s,dropout,args.encoder_step_deg)
                    app.source=source;app.now=source.now;app.engine=TrackingEngine(cfg,clock=lambda:app.now)
                wall_start=time.monotonic();count=0;metrics=Metrics()
            if args.duration_s is not None:
                duration=source.t-args.start_s if args.mode=='simulate' else time.monotonic()-wall_start
                if duration>=args.duration_s:break
            try:result,prediction,command,elapsed=app.tick()
            except StopIteration:break
            except Exception as error:
                app.action('stop');app.error='Processing fault: '+str(error)
                raise
            count+=1
            if args.mode=='simulate':metrics.observe(source.truth,result,prediction,source.pose,elapsed)
            if recorder:
                images=[app.image(name) for name in ('narrow','wide')]
                preview=None
                if all(image is not None for image in images) and images[0].shape[0]==images[1].shape[0]:
                    preview=np.hstack(images)
                    cv2.putText(preview,f'{args.mode.upper()}  {app.status["launch_elapsed_s"] or 0:.1f}s  {result.phase.name} ({result.phase.source})',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,240),1)
                from det_types import Pose
                recorder.write(app.frames,app.now,Pose(**app.status['pose']),app.status['launch_elapsed_s'],preview)
            if args.mode=='simulate':source.advance()
            if time.monotonic()-last_notice>20:
                print(f'{count} frames processed; selected={result.selected.camera if result.selected else "none"}; phase={result.phase.name}',flush=True);last_notice=time.monotonic()
            if args.serve or args.mode=='live':
                remaining=1/args.fps-(time.monotonic()-(wall_start+((count-1)/args.fps)))
                if remaining>0:stopping.wait(min(remaining,1/args.fps))
    finally:
        try:app.close()
        finally:
            try:
                if server:
                    if server_thread and server_thread.is_alive():server.shutdown()
                    server.server_close()
            finally:
                if recorder:recorder.close()
    summary=metrics.summary(args.fps) if args.mode=='simulate' else {'mode':args.mode,'frames':count,'physical_operation_verified':False}
    summary.update({'frames_processed':count,'wall_time_s':time.monotonic()-wall_start,'mode':args.mode,'allow_motion':args.allow_motion})
    (directory/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    if metrics.rows:
        with (directory/'evaluation_truth.jsonl').open('w') as handle:
            for row in metrics.rows:handle.write(json.dumps(row,allow_nan=False)+'\n')
    print(json.dumps({'run':str(directory),'summary':summary},indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
