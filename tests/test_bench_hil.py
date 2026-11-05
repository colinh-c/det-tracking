import math
from pathlib import Path
import tempfile
import unittest

from det_types import Pose
from bench_hil import BenchHilRunner, BenchObservation, BenchFault, BenchComplete


class Clock:
    def __init__(self): self.now = 1000.
    def __call__(self): return self.now
    def sleep(self, dt): self.now += dt


class Source:
    def __init__(self, clock, kind='as5600'):
        self.clock = clock
        self.az = 23.
        self.el = -4.
        self.age = 0.
        self.kind = kind
        self.weak = False
    def read(self):
        health = {axis: {'valid':True, 'magnet_detected':True, 'field_weak':self.weak,
                         'field_strong':False, 'age_s':self.age} for axis in ('az','el')}
        return BenchObservation(Pose(self.clock()-self.age,self.az,self.el),
                                {'az':self.kind,'el':self.kind},health)


class Motor:
    def __init__(self): self.commands=[]; self.stops=0; self.fail=False
    def send(self, command):
        self.commands.append(command)
        if self.fail: raise RuntimeError('injected motor failure')
    def stop(self): self.stops += 1


class BenchTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.source=Source(self.clock); self.motor=Motor()
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.directory=Path(self.temp.name)/'run'
    def runner(self, **config):
        return BenchHilRunner(self.source,self.motor,config={
            'bench_registration':'test fixture relative axes, not flight calibration',
            'save_frames':False, **config},output_dir=self.directory,
            clock=self.clock,sleep=self.clock.sleep)

    def test_waveform_is_bounded_smooth_and_starts_at_origin(self):
        runner=self.runner()
        self.addCleanup(runner.close)
        self.assertEqual(runner.target(0),(0.,0.))
        previous=runner.target(0)
        for i in range(1,2001):
            current=runner.target(i*.01)
            self.assertTrue(all(abs(v)<=1+1e-12 for v in current))
            self.assertTrue(all(abs(a-b)/.01<=.2+1e-6 for a,b in zip(current,previous)))
            previous=current

    def test_frames_are_rendered_from_actual_pose_not_command_integration(self):
        runner=self.runner()
        self.addCleanup(runner.close)
        runner.step(enabled=False)
        self.clock.sleep(.1); self.source.az += .25
        row=runner.step(enabled=False)
        self.assertAlmostEqual(row['relative_pose']['az_deg'],.25)
        self.assertEqual(self.motor.commands,[])
        self.assertGreater(abs(row['evaluation']['az_error_deg']),.2)
        self.assertTrue(row['evaluation']['physical_encoder_ground_truth']['az'])

    def test_actual_detector_predictor_executor_pipeline_and_logs(self):
        runner=self.runner()
        self.addCleanup(runner.close)
        rows=[]
        for _ in range(15):
            rows.append(runner.step(enabled=True)); self.clock.sleep(1/15)
        self.assertTrue(any(row['detection'] is not None for row in rows))
        self.assertTrue(any(row['prediction']['valid'] for row in rows))
        self.assertTrue(any(command.enabled for command in self.motor.commands))
        self.assertTrue(all(abs(command.az_velocity_deg_s)<=.25 for command in self.motor.commands))
        self.assertTrue(all(abs(command.el_velocity_deg_s)<=.25 for command in self.motor.commands))
        self.assertEqual(len((self.directory/'telemetry.jsonl').read_text().splitlines()),15)

    def test_weak_encoder_cannot_be_promoted_by_pose_valid(self):
        runner=self.runner(); self.addCleanup(runner.close)
        self.source.weak=True
        with self.assertRaises(BenchFault): runner.step(enabled=True)
        self.assertEqual(self.motor.commands,[])
        self.assertGreater(self.motor.stops,0)

    def test_counter_fallback_is_explicit_and_not_measured_accuracy(self):
        self.source.kind='step_counter'
        runner=self.runner(allow_counter_feedback=True); self.addCleanup(runner.close)
        row=runner.step(enabled=True)
        self.assertEqual(row['evaluation']['physical_encoder_ground_truth'],{'az':False,'el':False})
        self.assertEqual(row['feedback_quality'],'estimated_position')
        self.assertIsNone(runner.summary()['measured_encoder_error_deg']['az'])

    def test_counter_feedback_rejected_without_explicit_optin(self):
        self.source.kind='step_counter'
        runner=self.runner(); self.addCleanup(runner.close)
        with self.assertRaises(BenchFault): runner.step(enabled=True)
        self.assertEqual(self.motor.commands,[])

    def test_stale_pose_and_excursion_latch_stop(self):
        runner=self.runner(); self.addCleanup(runner.close)
        runner.step(enabled=True)
        self.clock.sleep(.1); self.source.az += 3
        with self.assertRaises(BenchFault): runner.step(enabled=True)
        previous=len(self.motor.commands)
        self.source.az -= 3; self.clock.sleep(.1)
        with self.assertRaises(BenchFault): runner.step(enabled=True)
        self.assertEqual(len(self.motor.commands),previous)

    def test_run_always_stops_after_send_failure(self):
        runner=self.runner(); self.addCleanup(runner.close)
        self.motor.fail=True
        with self.assertRaises(BenchFault): runner.run(enabled=True)
        self.assertGreater(self.motor.stops,0)
        self.assertTrue((self.directory/'summary.json').exists())

    def test_source_read_crossing_duration_completes_without_an_extra_command(self):
        original=self.source.read
        calls=[0]
        def slow_read():
            calls[0]+=1
            if calls[0]>1: self.clock.sleep(.08)
            return original()
        self.source.read=slow_read
        runner=self.runner(duration_s=.1)
        summary=runner.run(enabled=True)
        self.assertIsNone(summary['fault'])
        self.assertEqual(len(self.motor.commands),1)
        self.assertGreater(self.motor.stops,0)
        self.assertEqual(runner.rows[-1]['event'],'duration_completed')
        self.assertTrue(runner.closed)

    def test_single_step_duration_completion_stops_without_becoming_fault(self):
        runner=self.runner(duration_s=.1); self.addCleanup(runner.close)
        runner.step(enabled=True); self.clock.sleep(.11)
        with self.assertRaises(BenchComplete): runner.step(enabled=True)
        self.assertIsNone(runner.fault)
        self.assertEqual(len(self.motor.commands),1)
        self.assertGreater(self.motor.stops,0)

    def test_processing_crossing_duration_cannot_dispatch_late_command(self):
        runner=self.runner(duration_s=.1); self.addCleanup(runner.close)
        original=runner.perception.update
        def slow_process(*args,**kwargs):
            result=original(*args,**kwargs); self.clock.sleep(.11); return result
        runner.perception.update=slow_process
        with self.assertRaises(BenchComplete): runner.step(enabled=True)
        self.assertIsNone(runner.fault)
        self.assertEqual(self.motor.commands,[])
        self.assertGreater(self.motor.stops,0)

    def test_stale_pose_blocks_before_any_send(self):
        runner=self.runner(); self.addCleanup(runner.close)
        self.source.age=1
        with self.assertRaises(BenchFault): runner.step(enabled=True)
        self.assertEqual(self.motor.commands,[])

    def test_optical_feedback_is_measured_in_pixels_not_encoder_ground_truth(self):
        self.source.kind='camera_registration'
        original=self.source.read
        def optical():
            observation=original()
            observation.encoder_health['az']['field_weak']=True
            observation.camera_registration={'valid':True,'age_s':0.,'inliers':30,'confidence':.9,
                'scale_reference':'bounded motor moves, bench relative only',
                'jacobian_px_per_deg':[[0.,20.],[70.,0.]],'measured_displacement_px':[0.,0.]}
            return observation
        self.source.read=optical
        runner=self.runner(); self.addCleanup(runner.close)
        runner.step(enabled=True); self.clock.sleep(.1)
        row=runner.step(enabled=True)
        self.assertEqual(row['feedback_quality'],'optical_relative_registration')
        self.assertEqual(row['evaluation']['physical_encoder_ground_truth'],{'az':False,'el':False})
        self.assertTrue(row['encoder_health']['az']['field_weak'])
        self.assertGreater(row['evaluation']['measured_image_error_px']['y'],0)
        self.assertIsNotNone(runner.summary()['measured_image_error_px']['y'])
        self.assertIsNone(runner.summary()['measured_encoder_error_deg']['az'])

    def test_bad_optical_fit_blocks_motion(self):
        self.source.kind='camera_registration'
        original=self.source.read
        def optical():
            observation=original()
            observation.camera_registration={'valid':True,'age_s':0.,'inliers':3,'confidence':.9,
                'scale_reference':'bench','jacobian_px_per_deg':[[1.,0.],[0.,1.]],
                'measured_displacement_px':[0.,0.]}
            return observation
        self.source.read=optical
        runner=self.runner(); self.addCleanup(runner.close)
        with self.assertRaises(BenchFault): runner.step(enabled=True)
        self.assertEqual(self.motor.commands,[])

    def test_delayed_offline_plant_closes_loop_through_images(self):
        def trial(enabled,name):
            clock=Clock(); source=Source(clock); motor=Motor()
            runner=BenchHilRunner(source,motor,config={
                'bench_registration':'offline plant fixture, not hardware',
                'save_frames':False,'duration_s':10.,'target_rate_deg_s':.1},
                output_dir=Path(self.temp.name)/name,clock=clock,sleep=clock.sleep)
            pending=[]; active=None; sent=0
            try:
                for _ in range(160):
                    runner.step(enabled=enabled)
                    if len(motor.commands)>sent:
                        pending.append((clock()+.075,motor.commands[-1])); sent=len(motor.commands)
                    clock.sleep(.05)
                    while pending and pending[0][0]<=clock(): _,active=pending.pop(0)
                    if active is not None and active.valid_until>clock():
                        source.az += active.az_velocity_deg_s*.05
                        source.el += active.el_velocity_deg_s*.05
                return runner.summary()
            finally: runner.close()
        stationary=trial(False,'stationary')
        controlled=trial(True,'controlled')
        self.assertGreater(controlled['detected_fraction'],.95)
        for axis in ('az','el'):
            self.assertLess(controlled['measured_encoder_error_deg'][axis]['rms'],
                            stationary['measured_encoder_error_deg'][axis]['rms']*.65)


if __name__=='__main__': unittest.main()
