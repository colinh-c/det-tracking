"""Offline plant contracts: commands must obey latency, leases, and travel."""
import importlib.util
import json
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import patch

from det_types import Command

ROOT = Path(__file__).resolve().parents[1]
SAVED_RUN = ROOT / 'tests/fixtures/bounded-flight'


class PortableSnapshotTests(unittest.TestCase):
    def test_relative_input_paths_are_resolved_against_snapshot_directory(self):
        from offline_bounded_flight import OfflineFlight
        try:
            flight = OfflineFlight(SAVED_RUN)
        except FileNotFoundError as error:
            self.fail('Snapshot paths must resolve independently of the working directory: ' + str(error))
        self.assertEqual(ROOT/'openrocket_trajectory.csv', Path(flight.cfg['trajectory']))
        self.assertEqual(ROOT/'field_config.json', Path(flight.cfg['field_config']))
        self.assertAlmostEqual(139.738, flight.renderer.end_s)

    def test_absolute_input_paths_remain_supported(self):
        from offline_bounded_flight import OfflineFlight
        configuration = json.loads((SAVED_RUN/'configuration.json').read_text())
        configuration['launch_config']['trajectory'] = str(ROOT/'openrocket_trajectory.csv')
        configuration['launch_config']['field_config'] = str(ROOT/'field_config.json')
        with tempfile.TemporaryDirectory() as directory:
            snapshot = Path(directory)
            (snapshot/'configuration.json').write_text(json.dumps(configuration))
            (snapshot/'summary.json').write_text((SAVED_RUN/'summary.json').read_text())
            flight = OfflineFlight(snapshot)
        self.assertEqual(ROOT/'openrocket_trajectory.csv', Path(flight.cfg['trajectory']))
        self.assertAlmostEqual(139.738, flight.renderer.end_s)
        self.assertAlmostEqual(39.60839114135742,
                               flight.plant.bounds['el'][1]-flight.plant.origin['el'])


class BoundedPlantTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('offline_bounded_flight'),
                             'The offline bounded evaluator is missing')
        from offline_bounded_flight import BoundedPlant
        self.plant = BoundedPlant({'az': 0., 'el': 0.},
                                  {'az': [-1., 1.], 'el': [-1., 1.]},
                                  (0., 0.), start_time=10.)

    def command(self, az, el, stamp=10., expiry=20.):
        return Command(stamp, az, el, expiry, True, 'test', 1)

    def test_delayed_motion_has_real_acceleration_and_retains_capture_history(self):
        # Removing transport delay or teleporting to requested velocity must fail.
        self.plant.send(self.command(15., 0.))
        self.plant.advance_to(10.07)
        self.assertAlmostEqual(0., self.plant.position['az'])
        self.plant.advance_to(10.12)
        self.assertAlmostEqual(.09375, self.plant.position['az'], places=5)
        self.assertAlmostEqual(3.75, self.plant.velocity['az'], places=5)
        self.assertAlmostEqual(0., self.plant.pose_at(10.04).az_deg)
        self.assertLess(self.plant.pose_at(10.10).az_deg, self.plant.pose_at(10.12).az_deg)

    def test_outward_motion_stops_at_bound_then_inward_command_releases(self):
        # Unbounded SyntheticFlight.advance previously let this move past travel.
        self.plant.send(self.command(15., -15.))
        self.plant.advance_to(11.)
        self.assertEqual({'az': 1., 'el': -1.}, self.plant.position)
        self.assertEqual({'az': 0., 'el': 0.}, self.plant.velocity)
        self.plant.send(self.command(-3., 3., stamp=11.))
        self.plant.advance_to(11.2)
        self.assertLess(self.plant.position['az'], 1.)
        self.assertGreater(self.plant.position['el'], -1.)

    def test_expired_lease_brakes_without_a_new_command(self):
        self.plant.send(self.command(1., 0., expiry=10.15))
        self.plant.advance_to(10.3)
        self.assertAlmostEqual(0., self.plant.velocity['az'])
        stopped = self.plant.position['az']
        self.plant.advance_to(10.8)
        self.assertAlmostEqual(stopped, self.plant.position['az'])

    def test_saved_origin_maps_to_world_without_clipping_target(self):
        from offline_bounded_flight import OfflineFlight
        flight = OfflineFlight(SAVED_RUN)
        self.assertAlmostEqual(0., flight.plant.current_pose().az_deg)
        self.assertAlmostEqual(0., flight.plant.current_pose().el_deg)
        self.assertAlmostEqual(39.60839114135742,
                               flight.plant.bounds['el'][1] - flight.plant.origin['el'])
        self.assertGreater(flight.renderer.target(25.)[1], 80.)
        self.assertAlmostEqual(139.738, flight.renderer.end_s)

    def test_real_engine_short_run_is_network_free_with_delayed_capture(self):
        from offline_bounded_flight import OfflineFlight
        # Tripwire catches any accidental connector/transport use, while the real
        # renderer, predictor, perception, executor, and plant all run normally.
        with patch.object(socket, 'socket', side_effect=AssertionError('No network allowed')):
            flight = OfflineFlight(SAVED_RUN)
            rows = list(flight.observations(stop_after_s=.3))
        live = [row for row in rows if not row['priming']]
        self.assertGreater(len(live), 2)
        self.assertTrue(any(row['vision']['selected'] for row in live))
        self.assertAlmostEqual(.08, live[-1]['now'] - live[-1]['frames']['narrow']['timestamp'])
        self.assertGreater(live[-1]['counter_position']['el'], 2.69231)
        self.assertLess(live[-1]['evaluation_only']['time_s'], live[-1]['launch_elapsed_s'])


class MainGeometryMetricsTests(unittest.TestCase):
    def test_main_metrics_distinguish_frame_visibility_from_boresight_body(self):
        import offline_bounded_flight as module
        self.assertTrue(hasattr(module, 'main_geometry_metrics'),
                        'Main time-on-target geometry metrics are missing')
        rows = []
        for error, narrow, wide, body in [(.005, True, True, True),
                                           (.02, True, True, False),
                                           (.06, False, True, False)]:
            rows.append({'evaluation_only': {'phase': 'main',
                         'visible': {'narrow': narrow, 'wide': wide},
                         'boresight_inside_body': {'narrow': body, 'wide': False}},
                         'pointing_error_deg': error})
        metrics = module.main_geometry_metrics(rows, 15.)
        self.assertEqual(3, metrics['main_rendered_rows'])
        self.assertEqual(2, metrics['main_narrow_com_in_frame_rows'])
        self.assertAlmostEqual(2/3, metrics['main_narrow_com_in_frame_fraction'])
        self.assertEqual(3, metrics['main_either_com_in_frame_rows'])
        self.assertAlmostEqual(1/15, metrics['main_narrow_boresight_inside_body_s'])
        self.assertAlmostEqual(.02, metrics['main_pointing_error_percentiles_deg']['p50'])
        self.assertAlmostEqual(1/15, metrics['main_pointing_time_within_deg_s']['0.01'])
        self.assertAlmostEqual(2/15, metrics['main_pointing_time_within_deg_s']['0.025'])
        self.assertAlmostEqual(2/15, metrics['main_pointing_time_within_deg_s']['0.05'])
        self.assertAlmostEqual(3/15, metrics['main_pointing_time_within_deg_s']['0.1'])

    def test_summary_uses_rendered_main_phase_when_inferred_phase_disagrees(self):
        from copy import deepcopy
        from offline_bounded_flight import OfflineFlight
        source = OfflineFlight(SAVED_RUN)
        sample = next(row for row in source.observations(stop_after_s=.2) if not row['priming'])
        drogue = deepcopy(sample)
        drogue['evaluation_only']['phase'] = 'drogue'
        drogue['vision']['phase']['name'] = 'main_descent'
        drogue['vision']['phase']['source'] = 'visual_heuristic'
        main = deepcopy(sample)
        main['evaluation_only']['phase'] = 'main'
        main['vision']['phase']['name'] = 'drogue_descent'
        main['vision']['selected'] = None
        main['correct_selection'] = False
        main['selected_error_deg'] = None
        flight = OfflineFlight(SAVED_RUN)
        # This boundary supplies a compact known observation stream to exercise
        # the real artifact aggregation without another full rendered flight.
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(flight, 'observations', return_value=iter([drogue, main])):
                summary = flight.run(Path(directory)/'run')
        self.assertEqual(1, summary['main_rows'])
        self.assertEqual(0, summary['main_correct_selection_rows'])
        self.assertEqual(1, summary.get('inferred_main_rows'))
        self.assertEqual(1, summary.get('early_visual_main_rows'))
        self.assertEqual(1, summary['main_rendered_rows'])


class BoundedFlightIntegrationTests(unittest.TestCase):
    def test_full_flight_preserves_long_loss_and_evaluates_main_reacquisition(self):
        self.assertIsNotNone(importlib.util.find_spec('offline_bounded_flight'),
                             'The offline bounded evaluator is missing')
        from offline_bounded_flight import OfflineFlight
        with patch.object(socket, 'socket', side_effect=AssertionError('No network allowed')):
            flight = OfflineFlight(SAVED_RUN)
            with tempfile.TemporaryDirectory() as directory:
                destination = Path(directory) / 'run'
                summary = flight.run(destination)
                self.assertTrue(summary['completed'])
                self.assertAlmostEqual(139.738, summary['trajectory_duration_s'])
                self.assertGreater(summary['target_outside_travel_rows'], 1400)
                self.assertGreater(summary['no_selection_rows'], 1000)
                self.assertGreater(summary['main_rows'], 400)
                self.assertEqual(summary['flight_rows'], summary['correct_selection_rows'] +
                                 summary['incorrect_selection_rows'] + summary['no_selection_rows'])
                self.assertEqual(0, summary['travel_violation_rows'])
                self.assertLessEqual(summary['peak_actual_speed_deg_s'], 15.)
                self.assertLessEqual(summary['peak_free_actual_accel_deg_s2'], 75.000001)
                # Regression thresholds apply to this known ideal fixture,
                # not physical flight acceptance requirements.
                self.assertGreater(summary['main_correct_selection_rows']/summary['main_rows'], .95)
                self.assertGreater(summary['main_narrow_com_in_frame_fraction'], .95)
                self.assertGreater(summary['main_narrow_boresight_inside_body_fraction'], .9)
                self.assertEqual(0, summary['fixed_narrow_distractor_rows'])
                self.assertEqual(0, summary['early_visual_main_rows'])
                self.assertGreater(summary['first_reacquisition_after_initial_loss_s'], 100.)
                self.assertLess(summary['first_reacquisition_after_initial_loss_s'], 110.)
                self.assertTrue((destination / 'telemetry.jsonl').is_file())
                self.assertTrue((destination / 'summary.json').is_file())
                # Protect recoverable outputs from accidental overwrite.
                with self.assertRaises(FileExistsError):
                    flight.run(destination)


if __name__ == '__main__':
    unittest.main()
