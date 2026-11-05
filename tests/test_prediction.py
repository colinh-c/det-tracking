import csv
import math
from pathlib import Path
import tempfile
import unittest

from det_types import Detection, Phase, Pose, VisionResult
from field_config import FieldConfig, save_config
from prediction import AngularPrior, Predictor

ROOT = Path(__file__).resolve().parents[1]


def observation(t, az=0, el=10, phase='ascent', sigma=.02):
    return VisionResult(t, Detection(t, 'narrow', 100, 100, az, el, 1,
                                    angular_sigma_deg=sigma), phase=Phase(phase, 1, 'test'))


class PriorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.csv = self.path / 'flight.csv'
        with self.csv.open('w') as stream:
            stream.write('Time,Position East of launch,Position North of launch,Altitude\n'
                         '0,0,0,0\n1,0,0,100\n2,0,0,200\n')
        self.config = self.path / 'field.json'
        save_config(FieldConfig(), self.config)

    def test_actual_enu_angles_rates_and_elapsed_timestamp(self):
        prior = AngularPrior(self.csv, self.config)
        result = prior.sample(1)
        self.assertTrue(result.valid)
        self.assertEqual(result.timestamp, 1)
        self.assertAlmostEqual(result.az_deg, 0)
        self.assertAlmostEqual(result.el_deg, math.degrees(math.atan2(100, 500)))
        self.assertAlmostEqual(result.el_rate_deg_s, math.degrees(500 * 100 / (500**2 + 100**2)))

    def test_changed_geometry_and_signed_height_do_not_rotate_saved_flight(self):
        save_config(FieldConfig(pad_bearing_deg=450, pad_height_relative_to_tracker_m=-20,
                                rail_tilt_from_vertical_deg=45, rail_bearing_deg=180), self.config)
        result = AngularPrior(self.csv, self.config).sample(1)
        self.assertAlmostEqual(result.az_deg, 90)
        self.assertAlmostEqual(result.el_deg, math.degrees(math.atan2(80, 500)))

    def test_saved_real_export_is_usable_but_not_extrapolated_past_end(self):
        prior = AngularPrior(ROOT / 'openrocket_trajectory.csv', ROOT / 'field_config.json')
        self.assertTrue(prior.sample(20).valid)
        self.assertFalse(prior.sample(10000).valid)
        self.assertFalse(prior.sample(-1).valid)
        self.assertFalse(prior.sample(float('nan')).valid)

    def test_invalid_required_columns_or_time_order_are_rejected(self):
        for text in ('Time,Altitude\n0,0\n',
                     'Time,Position East of launch,Position North of launch,Altitude\n0,0,0,0\n0,0,0,1\n',
                     'Time,Position East of launch,Position North of launch,Altitude\n0,0,0,NaN\n1,0,0,1\n'):
            self.csv.write_text(text)
            with self.assertRaises(ValueError):
                AngularPrior(self.csv, self.config)

    def test_prior_innovation_tracks_offset_instead_of_claiming_simulation_as_measurement(self):
        prior = AngularPrior(self.csv, self.config)
        predictor = Predictor(prior)
        model = prior.sample(1)
        value = predictor.update(observation(1001, model.az_deg + 3, model.el_deg + 2),
                                 Pose(1001, 0, 0), 1001, launch_elapsed_s=1, latency_s=0)
        self.assertTrue(value.valid)
        self.assertAlmostEqual(value.az_deg, model.az_deg + 3)
        self.assertAlmostEqual(value.el_deg, model.el_deg + 2)
        self.assertIn('vision', value.source)


class PredictorTests(unittest.TestCase):
    def test_temporal_rate_and_latency_lead_follow_observed_motion(self):
        predictor = Predictor()
        for i in range(12):
            t = 1000 + i * .05
            value = predictor.update(observation(t, 2 * i * .05, 10), Pose(t, 0, 0), t, latency_s=.1)
        self.assertTrue(value.valid)
        self.assertAlmostEqual(value.az_rate_deg_s, 2, delta=.25)
        self.assertGreater(value.az_deg, 1.2)
        self.assertLess(value.az_deg, 1.4)
        self.assertEqual(value.timestamp, t)
        self.assertEqual(value.lead_time_s, .1)

    def test_wraparound_uses_short_angular_innovation(self):
        predictor = Predictor()
        for i in range(12):
            t = 1000 + i * .05
            value = predictor.update(observation(t, (359 + .15 * i) % 360), Pose(t, 359, 0), t, latency_s=0)
        self.assertTrue(value.valid)
        self.assertAlmostEqual(value.az_rate_deg_s, 3, delta=.3)
        self.assertLess(value.az_deg, 2)

    def test_loss_uncertainty_grows_then_stops_and_duplicates_do_not_refresh(self):
        predictor = Predictor()
        initial = observation(10)
        first = predictor.update(initial, Pose(10, 0, 0), 10, latency_s=0)
        coast = predictor.update(VisionResult(10.2), Pose(10.2, 0, 0), 10.2, latency_s=0)
        self.assertTrue(coast.valid)
        self.assertGreater(coast.uncertainty_deg, first.uncertainty_deg)
        self.assertIn('coast', coast.source)
        duplicate = VisionResult(10.3, initial.selected)
        self.assertTrue(predictor.update(duplicate, Pose(10.3, 0, 0), 10.3, latency_s=0).valid)
        lost = predictor.update(VisionResult(10.7), Pose(10.7, 0, 0), 10.7, latency_s=0)
        self.assertFalse(lost.valid)
        self.assertIn('loss', lost.reason.lower())

    def test_main_has_tighter_coast_policy_and_preserves_phase(self):
        predictor = Predictor()
        predictor.update(observation(10, phase='main'), Pose(10, 0, 0), 10, latency_s=0)
        result = predictor.update(VisionResult(10.4, phase=Phase('main', 1, 'manual')),
                                  Pose(10.4, 0, 0), 10.4, latency_s=0)
        self.assertFalse(result.valid)
        self.assertEqual(result.phase, 'main')

    def test_stale_invalid_future_and_out_of_order_inputs_are_rejected(self):
        cases = [(observation(10), Pose(9, 0, 0), 10),
                 (observation(10), Pose(10, 0, 0, False), 10),
                 (observation(10, float('nan')), Pose(10, 0, 0), 10),
                 (observation(11), Pose(10, 0, 0), 10),
                 (observation(9), Pose(10, 0, 0), 10)]
        for vision, pose, now in cases:
            with self.subTest(vision=vision, pose=pose):
                self.assertFalse(Predictor().update(vision, pose, now).valid)
        predictor = Predictor()
        predictor.update(observation(10), Pose(10, 0, 0), 10)
        self.assertFalse(predictor.update(observation(9.9), Pose(9.9, 0, 0), 9.9).valid)

    def test_prior_only_acquisition_is_bounded(self):
        prior = AngularPrior(ROOT / 'openrocket_trajectory.csv', ROOT / 'field_config.json')
        predictor = Predictor(prior)
        initial = predictor.update(VisionResult(1000), Pose(1000, 0, 0), 1000, launch_elapsed_s=0)
        self.assertTrue(initial.valid)
        self.assertIn('prior', initial.source)
        self.assertEqual(initial.lead_time_s, .1)
        self.assertFalse(predictor.update(VisionResult(1001), Pose(1001, 0, 0), 1001,
                                          launch_elapsed_s=1).valid)

    def test_encoder_time_reversal_and_invalid_phase_are_rejected(self):
        predictor = Predictor()
        predictor.update(observation(10), Pose(10, 0, 0), 10)
        self.assertFalse(predictor.update(observation(10.1), Pose(9.99, 0, 0), 10.1).valid)
        vision = observation(10)
        vision.phase.confidence = float('nan')
        self.assertFalse(Predictor().update(vision, Pose(10, 0, 0), 10).valid)

    def test_bounded_prior_search_preserves_last_validated_offset_and_wrap(self):
        prior = AngularPrior(ROOT / 'openrocket_trajectory.csv', ROOT / 'field_config.json')
        predictor = Predictor(prior)
        initial = prior.sample(1)
        predictor.update(observation(1001, initial.az_deg - 2, initial.el_deg + .4),
                         Pose(1001, 0, 0), 1001, 1, latency_s=0)
        lost = VisionResult(1003)
        value = predictor.update(lost, Pose(1003, 0, 0), 1003, 3, latency_s=.1)
        expected = prior.sample(3.1)
        self.assertTrue(value.valid)
        self.assertEqual(value.source, 'openrocket_reacquisition')
        self.assertIsNone(lost.selected)
        self.assertAlmostEqual(value.az_deg, (expected.az_deg - 2) % 360)
        self.assertAlmostEqual(value.el_deg, expected.el_deg + .4)
        self.assertAlmostEqual(value.el_rate_deg_s, expected.el_rate_deg_s)
        self.assertEqual(value.lead_time_s, .1)
        self.assertIn('heuristic', value.reason)
        self.assertFalse(predictor.update(VisionResult(1009.1), Pose(1009.1, 0, 0),
                                          1009.1, 9.1).valid)

    def test_prior_search_requires_same_launch_epoch_valid_pose_and_saved_range(self):
        prior = AngularPrior(ROOT / 'openrocket_trajectory.csv', ROOT / 'field_config.json')
        for bad_pose, elapsed in ((Pose(1002, 0, 0, False), 2),
                                  (Pose(1002, 0, 0), None),
                                  (Pose(1002, 0, 0), 3)):
            predictor = Predictor(prior)
            initial = prior.sample(1)
            predictor.update(observation(1001, initial.az_deg, initial.el_deg),
                             Pose(1001, 0, 0), 1001, 1)
            self.assertFalse(predictor.update(VisionResult(1002), bad_pose,
                                              1002, elapsed).valid)
        predictor = Predictor(prior)
        initial = prior.sample(139)
        predictor.update(observation(1139, initial.az_deg, initial.el_deg),
                         Pose(1139, 0, 0), 1139, 139)
        self.assertFalse(predictor.update(VisionResult(1140), Pose(1140, 0, 0),
                                          1140, 140).valid)

    def test_saved_ascent_reacquires_after_target_outruns_motor_ceiling(self):
        # Uses actual rendered images through perception/prediction/execution.
        # Ground truth remains in the evaluator, never in algorithm messages.
        from launch_config import load_launch_config
        from runtime import TrackingApp
        from simulation import SyntheticFlight, Metrics
        cfg = load_launch_config()
        source = SyntheticFlight(cfg['camera_models'], cfg['trajectory'], cfg['field_config'], fps=15)
        app = TrackingApp(cfg, 'simulate', source)
        metrics = Metrics()
        lost_at = regained_at = None
        search_count = 0
        try:
            for _ in range(180):
                result, prediction, command, elapsed = app.tick()
                metrics.observe(source.truth, result, prediction, source.pose, elapsed)
                if result.selected is None and source.t > 2.5 and lost_at is None:
                    lost_at = source.t
                if lost_at is not None and result.selected is not None and regained_at is None:
                    regained_at = source.t
                if prediction.source == 'openrocket_reacquisition':
                    self.assertIsNone(result.selected)
                    search_count += 1
                self.assertLessEqual(abs(command.el_velocity_deg_s), 15)
                source.advance()
            self.assertIsNotNone(lost_at)
            self.assertIsNotNone(regained_at)
            self.assertGreater(search_count, 0)
            self.assertLess(regained_at - lost_at, 8)
            self.assertEqual(metrics.summary(15)['all_flight']['wrong_target_detection_fraction'], 0)
            self.assertIsNotNone(result.selected)
        finally:
            app.close()

    def test_main_prior_search_is_shorter_and_can_be_disabled(self):
        prior = AngularPrior(ROOT / 'openrocket_trajectory.csv', ROOT / 'field_config.json')
        for config in ({}, {'main_prior_search_s': 0}):
            predictor = Predictor(prior, config)
            initial = prior.sample(112)
            predictor.update(observation(1112, initial.az_deg, initial.el_deg, phase='main'),
                             Pose(1112, 0, 0), 1112, 112, latency_s=0)
            value = predictor.update(VisionResult(1113), Pose(1113, 0, 0), 1113, 113)
            self.assertEqual(value.valid, not bool(config))
            self.assertFalse(predictor.update(VisionResult(1114.1), Pose(1114.1, 0, 0),
                                              1114.1, 114.1).valid)

    def test_reacquisition_after_loss_reinitializes_from_fresh_measurement(self):
        predictor = Predictor()
        predictor.update(observation(10), Pose(10, 0, 0), 10)
        self.assertFalse(predictor.update(VisionResult(11), Pose(11, 0, 0), 11).valid)
        value = predictor.update(observation(11.1, 20), Pose(11.1, 0, 0), 11.1, latency_s=0)
        self.assertTrue(value.valid)
        self.assertAlmostEqual(value.az_deg, 20)


if __name__ == '__main__':
    unittest.main()
