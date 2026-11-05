"""Network boundary tests using a real local HTTP server, never the Pi."""
import json
import io
import threading
import time
import unittest
import uuid
from urllib.error import HTTPError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

import cv2
import numpy as np

from det_types import CameraModel, Command


class CameraHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        self.server.paths.append(self.path)
        if self.path in self.server.failures:
            self.send_response(503)
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        if self.path == '/encoders':
            data = json.dumps(self.server.encoders).encode()
            kind, info = 'application/json', None
        else:
            data = self.server.jpeg
            kind, info = 'image/jpeg', dict(self.server.frame_info)
            generation = self.server.generations.get(self.path, self.server.generation)
            if generation is not None:
                info.setdefault('gateway_generation', generation)
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(data)))
        generation = self.server.generations.get(self.path, self.server.generation)
        if generation is not None:
            self.send_header('X-Gateway-Generation', generation)
        if info is not None:
            self.send_header('X-Frame-Info', json.dumps(info))
        self.end_headers()
        self.wfile.write(data)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), CameraHandler)
        self.server.paths = []
        self.server.generation = str(uuid.uuid4())
        self.server.generations = {}
        self.server.failures = set()
        self.server.frame_info = dict(sequence=4, age_s=.02, size=[24, 16], error=None,
                                      received_at=9000000000.0)
        self.server.jpeg = cv2.imencode('.jpg', np.full((16, 24), 150, np.uint8))[1].tobytes()
        self.server.encoders = [dict(bus=b, enabled=True, connected=True, valid=True,
                                    magnet_detected=True, field_weak=False, field_strong=False,
                                    stale=False, age_s=.03, read_at=9000000001.0,
                                    degrees=d) for b, d in [(1, 20.0), (3, 350.0)]]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = 'http://127.0.0.1:%d' % self.server.server_port
        self.cameras = {n: CameraModel(n, 24, 16, 100, 100, 12, 8, calibrated=True)
                        for n in ('narrow', 'wide')}
        self.config = dict(calibrated=True, max_age_s=.5,
                           axes={'az': dict(bus=3, zero_deg=10, sign=-1, offset_deg=15),
                                 'el': dict(bus=1, zero_deg=30, sign=1, offset_deg=5)},
                           camera_indices={'narrow': 1, 'wide': 0})
        self.source = None

    def tearDown(self):
        if self.source:
            self.source.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(1)

    def make_source(self):
        from transport import PiSource
        self.source = PiSource(self.url, self.cameras, self.config)
        return self.source

    def test_frame_uses_local_age_bus_mapping_and_no_duplicate_refresh(self):
        source = self.make_source()
        before = time.monotonic()
        frames, pose = source.read()
        self.assertEqual({f.camera for f in frames}, {'narrow', 'wide'})
        self.assertTrue(pose.valid)
        self.assertAlmostEqual(pose.az_deg, 35)
        self.assertAlmostEqual(pose.el_deg, -5)
        self.assertTrue(all(before - .2 < f.timestamp <= time.monotonic() - .019 for f in frames))
        self.assertLess(pose.timestamp, time.monotonic() - .029)
        self.assertIn('/frame/1.jpg', self.server.paths)
        again, same_pose = source.read()
        self.assertEqual(again, [])
        self.assertLessEqual(same_pose.timestamp, pose.timestamp)
        self.server.frame_info['sequence'] = 2
        self.assertEqual(source.read()[0], [])

    def test_bad_images_dimensions_and_metadata_cannot_be_frames(self):
        source = self.make_source()
        for bad in (b'not a jpeg', cv2.imencode('.jpg', np.zeros((9, 12), np.uint8))[1].tobytes()):
            self.server.jpeg = bad
            self.assertEqual(source.read()[0], [])
        self.server.jpeg = cv2.imencode('.jpg', np.zeros((16, 24), np.uint8))[1].tobytes()
        for key, val in [('age_s', float('nan')), ('sequence', True), ('error', 'stale')]:
            old = self.server.frame_info[key]
            self.server.frame_info[key] = val
            self.assertEqual(source.read()[0], [])
            self.server.frame_info[key] = old

    def test_invalid_or_unregistered_encoder_never_becomes_valid_pose(self):
        source = self.make_source()
        for key, value in [('magnet_detected', False), ('field_weak', True), ('valid', False),
                           ('stale', True), ('age_s', 9), ('error', 'bad angle')]:
            original = dict(self.server.encoders[0])
            self.server.encoders[0][key] = value
            self.assertFalse(source.read()[1].valid, key)
            self.server.encoders[0] = original
        source.close()
        self.config['calibrated'] = False
        self.assertFalse(self.make_source().read()[1].valid)

    def test_duplicate_encoder_cannot_reset_sample_age(self):
        source = self.make_source()
        first = source.read()[1]
        # A repeated sample with an erroneously reset advertised age is not new.
        for row in self.server.encoders:
            row['age_s'] = 0
        self.assertLessEqual(source.read()[1].timestamp, first.timestamp)

    def test_frame_pose_needs_a_matching_encoder_history_not_response_pose(self):
        source = self.make_source()
        frames, pose = source.read()
        self.assertTrue(pose.valid)
        self.assertTrue(all(not f.pose.valid for f in frames))
        time.sleep(.04)
        for row in self.server.encoders:
            row['read_at'] += 1
            row['degrees'] += 1
            row['age_s'] = .01
        self.server.frame_info.update(sequence=5, age_s=.025)
        frames, pose = source.read()
        self.assertTrue(all(f.pose.valid for f in frames))
        self.assertTrue(all(f.pose.timestamp == f.timestamp for f in frames))
        self.server.frame_info.update(sequence=6, age_s=.4)
        self.assertTrue(all(not f.pose.valid for f in source.read()[0]))

    def test_registered_config_accepts_canonical_direct_axis_keys(self):
        self.config.update(self.config.pop('axes'))
        self.assertTrue(self.make_source().read()[1].valid)

    def test_verified_process_restart_resets_sequences_and_encoder_history(self):
        source = self.make_source()
        self.server.frame_info['sequence'] = 50000
        self.assertEqual(len(source.read()[0]), 2)
        old_generation = source.generation
        self.assertEqual(source.generation_revision, 1)
        self.assertTrue(source.generation_verified)
        self.server.generation = str(uuid.uuid4())
        self.server.frame_info['sequence'] = 1
        frames, pose = source.read()
        self.assertEqual(len(frames), 2)
        self.assertEqual(source.generation_revision, 2)
        self.assertTrue(source.generation_changed)
        self.assertFalse(pose.valid)
        self.assertTrue(all(not frame.pose.valid for frame in frames))
        self.assertTrue(all(len(history) == 1 for history in source._axis_history.values()))
        self.server.frame_info['sequence'] = 2
        self.assertEqual(len(source.read()[0]), 2)
        self.assertFalse(source.generation_changed)
        self.server.generation = old_generation
        self.server.frame_info['sequence'] = 99999
        self.assertEqual(source.read()[0], [])
        self.assertIn('retired', source.errors['generation'])
        self.assertEqual(source.generation_revision, 2)

    def test_mixed_generation_batch_does_not_mutate_encoder_or_sequence_state(self):
        source = self.make_source()
        source.read()
        old_samples = dict(source._encoder_samples)
        old_history = {axis:list(history) for axis,history in source._axis_history.items()}
        old_sequences = dict(source._sequences)
        self.server.generations['/frame/0.jpg'] = str(uuid.uuid4())
        self.server.frame_info['sequence'] += 1
        for row in self.server.encoders:
            row['read_at'] += 1
            row['degrees'] += 1
        frames, pose = source.read()
        self.assertEqual(frames, [])
        self.assertFalse(pose.valid)
        self.assertFalse(source.generation_verified)
        self.assertEqual(source._encoder_samples, old_samples)
        self.assertEqual(source._axis_history, old_history)
        self.assertEqual(source._sequences, old_sequences)

    def test_missing_malformed_and_disagreeing_generation_fail_closed(self):
        source = self.make_source()
        for generation in (None, '', 'not-a-uuid'):
            self.server.generation = generation
            self.assertEqual(source.read()[0], [])
            self.assertFalse(source.generation_verified)
            self.assertEqual(source.generation_revision, 0)
        self.server.generation = str(uuid.uuid4())
        self.server.frame_info['gateway_generation'] = str(uuid.uuid4())
        self.assertEqual(source.read()[0], [])
        self.assertEqual(source._encoder_samples, {})
        self.assertEqual(source.generation_revision, 0)

    def test_legacy_preview_is_explicit_and_never_world_valid_or_downgraded(self):
        self.config['allow_legacy_gateway'] = True
        self.server.generation = None
        source = self.make_source()
        frames, pose = source.read()
        self.assertEqual(len(frames), 2)
        self.assertFalse(pose.valid)
        self.assertTrue(all(not frame.pose.valid for frame in frames))
        self.assertFalse(source.generation_verified)
        self.assertEqual(source.generation_revision, 0)
        self.assertEqual(source.read()[0], [])
        self.server.generation = str(uuid.uuid4())
        self.server.frame_info['sequence'] = 1
        self.assertEqual(len(source.read()[0]), 2)
        self.assertTrue(source.generation_changed)
        self.assertEqual(source.generation_revision, 1)
        self.server.generation = None
        self.server.frame_info['sequence'] = 1000
        self.assertEqual(source.read()[0], [])
        self.assertIn('downgrade', source.errors['generation'])

    def test_uncalibrated_preview_exposes_public_diagnostics(self):
        self.config['calibrated'] = False
        source = self.make_source()
        self.server.encoders[0].update(status=55, field_weak=True, agc=128, magnitude=1647)
        self.server.encoders[0]['unexpected_private'] = 'not-public'
        frames, pose = source.read()
        self.assertEqual(len(frames), 2)
        self.assertFalse(pose.valid)
        self.assertEqual(source.encoder_readings[0]['status'], 55)
        self.assertEqual(source.encoder_readings[0]['agc'], 128)
        self.assertNotIn('unexpected_private', source.encoder_readings[0])
        self.assertEqual(source.camera_info['narrow']['size'], [24,16])

    def test_generation_retirement_storage_is_bounded(self):
        source = self.make_source()
        source.read()
        source._retired_generations = {str(uuid.uuid4()) for _ in range(64)}
        self.server.generation = str(uuid.uuid4())
        self.server.frame_info['sequence'] = 1
        self.assertEqual(source.read()[0], [])
        self.assertIn('limit', source.errors['generation'])
        self.assertEqual(len(source._retired_generations), 64)

    def test_first_start_and_restart_accept_wide_with_failed_narrow(self):
        source = self.make_source()
        self.server.failures.add('/frame/1.jpg')  # Config maps narrow to 1.
        frames, pose = source.read()
        self.assertEqual([frame.camera for frame in frames], ['wide'])
        self.assertTrue(source.generation_verified)
        self.assertEqual(source.generation_revision, 1)
        self.assertIn('narrow', source.errors)
        self.server.generation = str(uuid.uuid4())
        self.server.frame_info['sequence'] = 1
        frames, pose = source.read()
        self.assertEqual([frame.camera for frame in frames], ['wide'])
        self.assertEqual(source.generation_revision, 2)
        self.assertTrue(source.generation_changed)
        self.assertFalse(pose.valid)

    def test_encoder_failure_preserves_established_raw_preview_without_valid_pose(self):
        source = self.make_source()
        source.read()
        self.server.failures.add('/encoders')
        self.server.frame_info['sequence'] += 1
        frames, pose = source.read()
        self.assertEqual(len(frames), 2)
        self.assertFalse(pose.valid)
        self.assertTrue(all(not frame.pose.valid for frame in frames))
        self.assertEqual(source.encoder_readings, [])
        self.assertIn('encoders', source.errors)
        self.server.generation = str(uuid.uuid4())
        self.server.frame_info['sequence'] = 1
        self.assertEqual(source.read()[0], [])
        self.assertEqual(source.generation_revision, 1)


class AlignmentTests(unittest.TestCase):
    def test_unavailable_http_response_is_closed_before_retry_by_caller(self):
        from transport import _http, TransportError
        body = io.BytesIO(b'unavailable')
        error = HTTPError('http://127.0.0.1:1/frame/0.jpg', 503, 'Unavailable', {}, body)
        with patch('transport.urlopen', side_effect=error):
            with self.assertRaises(TransportError):
                _http(error.url, .1)
        self.assertTrue(body.closed)

    def test_angle_alignment_wrap_and_strict_gap_horizon(self):
        from transport import align_angle
        self.assertAlmostEqual(align_angle([(10.0, 359), (10.1, 1)], 10.05, True), 0)
        self.assertAlmostEqual(align_angle([(10.0, 5), (10.1, 7)], 10.15), 8)
        with self.assertRaises(ValueError): align_angle([(10.0, 0), (10.5, 10)], 10.25)
        with self.assertRaises(ValueError): align_angle([(10.0, 0), (10.1, 1)], 10.151)
        with self.assertRaises(ValueError): align_angle([(10.0, 0)], 10.01)


class MotorTests(unittest.TestCase):
    def test_stale_command_is_rejected_before_network(self):
        from transport import HttpMotor, TransportError
        ready = json.dumps(dict(initialized=True, fault=None, generation='session', firmware_watchdog=True)).encode()
        with patch('transport._http', return_value=(ready, {}, 1, 1)) as network:
            motor = HttpMotor('http://127.0.0.1:1', timeout_s=.05)
            before = network.call_count
            with self.assertRaises(TransportError):
                motor.send(Command(time.monotonic()-1, 1, 0, time.monotonic()-.1, True))
            self.assertEqual(network.call_count, before)

    def test_client_requires_firmware_watchdog_status_before_arming(self):
        from transport import HttpMotor, TransportError
        old = json.dumps(dict(initialized=True, fault=None, generation='session', firmware_watchdog=False)).encode()
        with patch('transport._http', return_value=(old, {}, 1, 1)):
            with self.assertRaises(TransportError):
                HttpMotor('http://127.0.0.1:1', timeout_s=.05)


if __name__ == '__main__':
    unittest.main()
