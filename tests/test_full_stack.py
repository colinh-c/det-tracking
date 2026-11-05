import unittest
import tempfile
import threading
import json
import time
from urllib.request import urlopen,Request
from launch_config import load_launch_config
from runtime import TrackingApp,TrackingEngine
from simulation import SyntheticFlight,Metrics
from dashboard import create_server
from det_types import Pose,Frame

class FullStackTests(unittest.TestCase):
    def app(self,start=112):
        cfg=load_launch_config()
        source=SyntheticFlight(cfg['camera_models'],cfg['trajectory'],cfg['field_config'],fps=15,start_s=start)
        return TrackingApp(cfg,'simulate',source)
    def test_same_path_closed_loop_main_trial_and_truth_is_not_observation(self):
        app=self.app();metrics=Metrics()
        try:
            for _ in range(45):
                result,prediction,command,elapsed=app.tick()
                metrics.observe(app.source.truth,result,prediction,app.source.pose,elapsed)
                self.assertFalse(hasattr(result,'truth'))
                self.assertFalse(hasattr(next(iter(app.frames.values())),'phase'))
                app.source.advance()
            summary=metrics.summary(15)
            self.assertGreater(summary['main']['correct_detection_fraction_within_0_2_deg'],.8)
            self.assertLess(summary['main']['pointing_error_deg_p50_p95'][1],.5)
            self.assertIn(result.phase.source,('simulation_prior','visual','manual','unknown','vision','observed','visual_heuristic'))
        finally:app.close()
    def test_dashboard_real_http_stop_and_click(self):
        app=self.app();app.tick()
        server=create_server(app,0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base='http://127.0.0.1:'+str(server.server_port)
        def post(data,header=True):
            return urlopen(Request(base+'/action',data=json.dumps(data).encode(),headers={'Content-Type':'application/json',**({'X-DET-Control':'1'} if header else {})}),timeout=2)
        try:
            self.assertIn(b'DET launch tracking',urlopen(base).read())
            status=json.load(urlopen(base+'/status'));self.assertEqual(status['mode'],'simulate')
            self.assertTrue(urlopen(base+'/frame/narrow.jpg').read().startswith(b'\xff\xd8'))
            json.load(post({'action':'acquire','payload':{'camera':'narrow','x':320,'y':240}}))
            self.assertEqual(app.engine.click_hint.source,'manual_acquisition')
            json.load(post({'action':'stop'}));app.tick()
            self.assertFalse(app.status['command']['enabled'])
            json.load(post({'action':'phase','payload':{'phase':'main'}}));app.tick()
            self.assertEqual(app.status['phase']['source'],'manual')
            with self.assertRaises(Exception):post({'action':'start'},False)
        finally:server.shutdown();server.server_close();app.close()
    def test_live_cannot_arm_with_nominal_calibration(self):
        app=self.app();app.mode='live';app.allow_motion=True
        try:
            with self.assertRaisesRegex(ValueError,'calibration'):app.action('start')
            self.assertFalse(app.enabled and app.motor is not None)
        finally:app.close()

    def test_main_with_quantized_encoders_retains_correct_body(self):
        app=self.app(start=115);app.source.encoder_step_deg=360/4096
        metrics=Metrics()
        try:
            for _ in range(45):
                result,prediction,command,elapsed=app.tick()
                metrics.observe(app.source.truth,result,prediction,app.source.pose,elapsed)
                app.source.advance()
            summary=metrics.summary(15)['main']
            self.assertGreater(summary['correct_detection_fraction_within_0_2_deg'],.8)
            self.assertGreater(summary['either_camera_visible_fraction'],.95)
        finally:app.close()

    def test_acquisition_click_clears_old_identity_filter_and_integral(self):
        app=self.app();app.tick()
        try:
            previous=app.engine
            previous_sequence=app.status['command']['sequence']
            self.assertIsNotNone(previous.perception.track)
            app.action('acquire',{'camera':'narrow','x':400,'y':250})
            self.assertIsNot(app.engine,previous)
            self.assertIsNone(app.engine.perception.track)
            self.assertIsNone(app.engine.previous)
            self.assertEqual(app.engine.click_hint.source,'manual_acquisition')
            app.source.advance();app.tick()
            self.assertGreater(app.status['command']['sequence'],previous_sequence)
        finally:app.close()
    def test_closed_app_cannot_rearm_or_process(self):
        app=self.app();app.close()
        with self.assertRaises(ValueError):app.action('start')
        with self.assertRaises(RuntimeError):app.tick()
        self.assertFalse(app.action('stop')['enabled'])
    def test_armed_live_prerequisite_change_stops_before_sending(self):
        from unittest.mock import patch
        app=self.app();app.mode='live';app.enabled=True
        class Motor:
            def __init__(self):self.stopped=False
            def stop(self):self.stopped=True
            def close(self):pass
            def send(self,command):raise AssertionError('Cannot command after invalid calibration')
        motor=Motor();app.motor=motor
        try:
            with patch('runtime.live_readiness',return_value=['Field geometry changed']):app.tick()
            self.assertTrue(motor.stopped);self.assertFalse(app.enabled)
            self.assertIn('prerequisites changed',app.error)
        finally:app.close()
    def test_nonzero_camera_boresight_is_used_for_mount_commands(self):
        cfg=load_launch_config();camera=cfg['camera_models']['narrow']
        camera.az_offset_deg=.8;camera.el_offset_deg=-.4
        source=SyntheticFlight(cfg['camera_models'],cfg['trajectory'],cfg['field_config'],fps=15,start_s=115)
        az,el,_=source.target(source.t)
        mount=camera.mount_for_boresight(az,el)
        source.pose=Pose(source.now,*mount)
        app=TrackingApp(cfg,'simulate',source)
        try:
            for _ in range(45):
                app.tick();source.advance()
            az,el,_=source.target(source.t)
            projected=camera.project(az,el,source.pose)
            self.assertTrue(camera.inside(projected))
            self.assertLess(abs(app.last_prediction.az_deg-az),1.5)
            self.assertGreater(abs(app.last_prediction.az_deg-az),.4)
        finally:app.close()
