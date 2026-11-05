import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from det_types import Command, Frame, Pose, Prediction, VisionResult
from full_flight_hil import FullFlightHil, CounterHistory, prepare, load_profile, main
from runtime import TrackingEngine
from simulation import SyntheticFlight


class Clock:
    def __init__(self): self.t = 1000.
    def __call__(self): return self.t
    def sleep(self, dt): self.t += max(0., dt)


class RealClock:
    def __call__(self): return time.monotonic()
    def sleep(self,dt): time.sleep(max(0.,dt))


class Motor:
    def __init__(self): self.commands = []
    def send(self, command): self.commands.append(command); return {'active': True}


class Handoff:
    def __init__(self, clock):
        self.clock = clock; self.claims = 0; self.releases = []; self.device = Motor()
        self.generation = 'test-generation'; self.position = {'az': 12., 'el': 2.}
    def claim(self):
        self.claims += 1
        return {'session': 'owned', 'generation': self.generation, 'position': dict(self.position),
                'limits': {'az':[-180,180], 'el':[-10,95]}}
    def pose(self, session):
        self.clock.sleep(.005)
        return {'position': dict(self.position), 'age_s': .005, 'generation': self.generation}
    def motor(self, session, timeout_s): return self.device
    def release(self, session): self.releases.append(session); return {'released': True,'mode':'manual'}


class Renderer:
    def __init__(self):
        self.t = 0.; self.end_s = .4; self.epoch = 0.; self.sequence = 0
        self.times = np.array([0., .4]); self.truth = {}; self.rendered = []
    def read(self):
        self.rendered.append((self.t, self.pose))
        self.truth = {'evaluation_only_marker': 987, 'time_s': self.t}
        return [Frame('narrow', self.epoch+self.t, np.zeros((4,4), np.uint8),
                      self.pose, self.sequence)], self.pose
    def advance(self): raise AssertionError('A hardware replay must never advance a software plant')
    def send(self, command): raise AssertionError('A hardware replay must never command a software plant')


class Engine:
    def __init__(self, clock): self.clock = clock; self.calls = []; self.extra_work = 0.
    def step(self, frames, pose, now, elapsed=None, enabled=False, phase_override=None):
        self.calls.append((now, elapsed, enabled, phase_override, frames, pose))
        self.clock.sleep(self.extra_work)
        return (VisionResult(now), Prediction(now, valid=True),
                Command(now, .1, .1, now+.25, enabled, 'fixture', len(self.calls)))


class FullFlightTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.handoff = Handoff(self.clock)
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.profile, self.cfg = load_profile()
        self.profile.update(prime_s=.2, save_frames=False, synthetic_frame_delay_s=.08)
        self.engine = Engine(self.clock); self.renderer = Renderer()
        self.manual = {'available': True, 'initialized': True, 'connected': True, 'busy': False,
                       'hold_active': False, 'position': {'az':12., 'el':2.},
                       'limits': {'soft_limits_deg': {'az':[-180,180], 'el':[-10,95]},
                                  'max_velocity_deg_s':15, 'acceleration_deg_s2':75}}
    def runner(self):
        return FullFlightHil(self.profile, self.cfg, self.handoff, self.manual,
                             Path(self.temp.name)/'run', clock=self.clock, sleep=self.clock.sleep,
                             engine=self.engine, renderer=self.renderer)

    def test_prepare_is_read_only_and_reports_original_demand_and_limits(self):
        result = prepare(self.profile, self.cfg, self.manual)
        self.assertAlmostEqual(result['trajectory']['duration_s'], 139.738)
        self.assertGreater(result['trajectory']['peak_abs_rate_deg_s']['el'], 21.7)
        self.assertTrue(result['travel']['fits_existing_limits'])
        self.assertFalse(result['flight_calibration_valid'])
        self.assertEqual(self.handoff.claims, 0)
        self.manual['position']['el'] = 20.
        self.assertFalse(prepare(self.profile,self.cfg,self.manual)['travel']['fits_existing_limits'])

    def test_hil_timing_overrides_leave_base_flight_config_unchanged(self):
        from launch_config import load_launch_config
        base=load_launch_config(self.profile['launch_config'])
        self.assertNotIn('lease_s',base['executor'])
        self.assertEqual(self.cfg['executor']['lease_s'],.4)
        self.assertEqual(self.cfg['executor']['max_control_gap_s'],.3)
        self.assertEqual(self.cfg['executor']['max_speed_deg_s'],15.)
        self.assertEqual(self.cfg['executor']['max_accel_deg_s2'],75.)
        self.assertGreater(self.cfg['executor']['lease_s']-self.cfg['motor_timeout_s'],
                           self.cfg['executor']['max_control_gap_s'])
        for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):
            self.assertEqual(os.environ[name],'1')
        # OpenCV's GCD backend does not expose the requested limit through
        # getNumThreads(); do not treat its CPU-count return as verification.
        self.assertEqual(self.profile['encoder_url'],'http://127.0.0.1:18082/encoders')

    def test_loss_wait_uses_same_hil_lease(self):
        original=self.engine.step
        def loss(*args,**kwargs):
            vision,prediction,command=original(*args,**kwargs)
            if command.enabled:
                prediction.valid=False; prediction.reason='Target loss beyond bounded search'
                command.enabled=False
            return vision,prediction,command
        self.engine.step=loss
        result=self.runner().run()
        self.assertTrue(result['completed'])
        self.assertTrue(self.handoff.device.commands)
        for command in self.handoff.device.commands:
            self.assertAlmostEqual(command.valid_until-command.timestamp,.4)
            self.assertEqual(command.az_velocity_deg_s,0.)
            self.assertEqual(command.el_velocity_deg_s,0.)

    def test_history_interpolates_counts_not_wrapped_world_or_commands(self):
        h = CounterHistory({'az':179., 'el':2.}, (359., 0.), .3)
        h.append({'position':{'az':179.,'el':2.},'age_s':0},1000.)
        h.append({'position':{'az':181.,'el':4.},'age_s':0},1000.2)
        p = h.at(1000.1)
        self.assertAlmostEqual(p.az_deg,0.); self.assertAlmostEqual(p.el_deg,1.)
        with self.assertRaises(ValueError): h.at(1000.3)
        with self.assertRaises(ValueError): h.append({'position':{'az':1.,'el':2.},'age_s':.5},1001.)

    def test_real_counts_frames_prime_then_walltime_no_software_plant(self):
        runner = self.runner(); report = runner.run()
        self.assertTrue(report['completed']); self.assertEqual(self.handoff.releases,['owned'])
        self.assertTrue(self.handoff.device.commands)
        first_on = next(i for i,c in enumerate(self.engine.calls) if c[2])
        self.assertGreater(first_on,1)
        self.assertTrue(all(c[3] is None for c in self.engine.calls))
        for now, elapsed, enabled, _, frames, pose in self.engine.calls[first_on:]:
            self.assertAlmostEqual(elapsed,now-runner.launch_origin)
            self.assertLessEqual(frames[0].timestamp,now-.08+1e-7)
            self.assertAlmostEqual(pose.az_deg,0.)  # Claim count12 maps to pad bearing0.
        rows=[json.loads(s) for s in (runner.output_dir/'telemetry.jsonl').read_text().splitlines()]
        self.assertTrue(all(r['feedback_quality']=='unqualified_controller_counts' for r in rows))
        self.assertTrue(all('evaluation_only_marker' not in r['vision'] for r in rows))

    def test_processing_overrun_skips_time_and_final_source_boundary_is_normal(self):
        self.engine.extra_work=.09
        runner=self.runner(); report=runner.run()
        self.assertTrue(report['completed'])
        active=[c for c in self.engine.calls if c[2]]
        self.assertTrue(all(c[1]==c[0]-runner.launch_origin for c in active))
        self.assertLess(len(active),6)
        self.assertEqual(self.handoff.releases,['owned'])

    def test_generation_change_and_send_failure_release_without_global_stop(self):
        runner=self.runner()
        original=self.handoff.pose
        def changed(session):
            out=original(session); out['generation']='other'; return out
        self.handoff.pose=changed
        result=runner.run()
        self.assertFalse(result['completed']); self.assertIn('generation', result['fault'])
        self.assertEqual(self.handoff.releases,['owned']); self.assertEqual(self.handoff.device.commands,[])

    def test_actual_claim_origin_rechecked_before_any_command(self):
        self.profile['allow_limit_saturation']=False
        self.handoff.position['el']=25.
        report=self.runner().run()
        self.assertFalse(report['completed']); self.assertIn('travel',report['fault'])
        self.assertEqual(self.handoff.releases,['owned']); self.assertEqual(self.handoff.device.commands,[])

    def test_user_selected_limits_log_unreachable_target_without_rescaling(self):
        self.handoff.position={'az':30.19992,'el':-5.16230}
        original=self.handoff.claim
        def claim():
            result=original()
            result['limits']={'az':[-.1357,32.461868],'el':[-5.162340,42.300701]}
            return result
        self.handoff.claim=claim
        report=self.runner().run()
        self.assertTrue(report['completed'])
        self.assertFalse(report['travel_at_claim']['fits_existing_limits'])
        self.assertGreater(report['travel_at_claim']['axes']['el']['approx_unreachable_s'],0)
        self.assertEqual(report['full_trajectory_wall_time_scale'],1.)
        self.assertEqual(self.handoff.releases,['owned'])

    def test_cancellation_and_transport_error_restore_only_the_owned_session(self):
        runner=self.runner()
        def cancel_send(command):
            self.handoff.device.commands.append(command)
            runner.cancel.set()
        self.handoff.device.send=cancel_send
        result=runner.run()
        self.assertFalse(result['completed']); self.assertIn('Cancelled',result['fault'])
        self.assertEqual(self.handoff.releases,['owned'])

    def test_send_failure_is_logged_and_owner_released(self):
        def fail(command): raise RuntimeError('manual takeover')
        self.handoff.device.send=fail
        runner=self.runner(); result=runner.run()
        self.assertIn('manual takeover',result['fault'])
        self.assertEqual(self.handoff.releases,['owned'])
        rows=[json.loads(s) for s in (runner.output_dir/'telemetry.jsonl').read_text().splitlines()]
        self.assertEqual(rows[-1]['command_error'],'manual takeover')

    def test_real_three_module_pipeline_and_renderer_short_fixture(self):
        self.engine=TrackingEngine(self.cfg,clock=self.clock)
        self.renderer=SyntheticFlight(self.cfg['camera_models'],self.cfg['trajectory'],self.cfg['field_config'])
        self.renderer.end_s=.6  # Short injected fixture, never an execute option.
        self.renderer.advance=lambda: self.fail('Software plant advance was called')
        self.renderer.send=lambda _: self.fail('Software plant send was called')
        report=self.runner().run()
        self.assertTrue(report['completed'],report)
        self.assertTrue(self.handoff.device.commands)
        self.assertTrue(all(abs(c.az_velocity_deg_s)<=15 and abs(c.el_velocity_deg_s)<=15
                            for c in self.handoff.device.commands))
        self.assertFalse(self.cfg['encoders']['calibrated'])
        self.assertEqual(self.cfg['calibration']['status'],'nominal_for_simulation')

    def test_actual_monotonic_clock_is_frozen_within_each_engine_step(self):
        # A constant fake clock concealed the five-microsecond future vision
        # timestamp in the real attempt. This fixture uses real wall time but
        # only a fake handoff/motor, never a device or network connection.
        self.clock=RealClock(); self.handoff=Handoff(self.clock)
        renderer=SyntheticFlight(self.cfg['camera_models'],self.cfg['trajectory'],self.cfg['field_config'])
        renderer.end_s=.25
        runner=FullFlightHil(self.profile,self.cfg,self.handoff,self.manual,
                            Path(self.temp.name)/'real-clock',clock=self.clock,sleep=self.clock.sleep,
                            renderer=renderer)
        result=runner.run()
        self.assertTrue(result['completed'],result)
        rows=[json.loads(s) for s in (runner.output_dir/'telemetry.jsonl').read_text().splitlines()]
        self.assertTrue(rows)
        for row in rows:
            self.assertEqual(row['vision']['timestamp'],row['now'])
            self.assertTrue(row['prediction']['valid'],row['prediction']['reason'])
        self.assertTrue(self.handoff.device.commands)

    def test_invalid_priming_never_constructs_motor_or_starts_launch_clock(self):
        original=self.engine.step
        def invalid(*args,**kwargs):
            vision,prediction,command=original(*args,**kwargs)
            prediction.valid=False; prediction.reason='fixture: blind initialization'
            return vision,prediction,command
        self.engine.step=invalid
        runner=self.runner()
        with patch.object(self.handoff,'motor',side_effect=AssertionError('Must not arm')) as motor:
            result=runner.run()
        self.assertFalse(result['completed'])
        self.assertIn('prime',result['fault'])
        self.assertIsNone(runner.launch_origin)
        motor.assert_not_called()
        self.assertEqual(self.handoff.device.commands,[])
        self.assertEqual(self.handoff.releases,['owned'])

    def test_default_cli_only_reads_manual_status(self):
        path=Path(self.temp.name)/'preparation'
        with patch('full_flight_hil.json_get',return_value=self.manual) as get, \
             patch('full_flight_hil.FullFlightHil.run',side_effect=AssertionError('Not approved')), \
             patch('builtins.print'):
            self.assertEqual(main(['--output',str(path)]),0)
        get.assert_called_once_with('http://127.0.0.1:18083/motion')
        report=json.loads((path/'preparation.json').read_text())
        self.assertFalse(report['hardware_motion_requested'])
        self.assertTrue(report['approval_required_before_execution'])

    def test_revoked_release_does_not_claim_manual_was_restored(self):
        self.handoff.release=lambda _: {'released':False,'mode':'automatic'}
        result=self.runner().run()
        self.assertFalse(result['manual_control_return_confirmed'])


if __name__=='__main__': unittest.main()
