import json
import tempfile
import threading
import time
from pathlib import Path
import unittest

import cv2
import numpy as np

from optical_bench_source import OpticalBenchSource


class OpticalBenchSourceTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(45)
        self.baseline = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), .6)
        self.jpeg = cv2.imencode('.jpg', self.baseline, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
        self.image = self.baseline.copy()
        self.now = 100.
        self.sequence = 1
        self.age = .01
        self.jacobian = np.array([[80., -10.], [15., -100.]])
        self.raw = [dict(bus=1, valid=False, magnet_detected=True, field_weak=True,
                         field_strong=False, status=55, agc=128, magnitude=1647, age_s=.02)]

    def fetch(self, url, timeout, **kwargs):
        if url.endswith('/encoders'):
            return json.dumps(self.raw).encode(), {}, 99.98, 100.
        encoded = cv2.imencode('.jpg', self.image, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
        info = dict(sequence=self.sequence, age_s=self.age, size=[320, 240], error=None)
        return encoded, {'content-type': 'image/jpeg', 'x-frame-info': json.dumps(info)}, 99.98, 100.

    def source(self, **kwargs):
        source = OpticalBenchSource(self.jpeg, self.jacobian, scale_reference='offline known translation fixture',
                                    fetch=self.fetch, clock=lambda: self.now, **kwargs)
        self.addCleanup(source.close)
        return source

    def test_translation_recovers_relative_controller_axes_and_preserves_raw_health(self):
        displacement = np.array([8., -6.])
        self.image = cv2.warpAffine(self.baseline, np.array([[1., 0., 8.], [0., 1., -6.]]), (320, 240))
        observation = self.source().read()
        self.assertTrue(observation.pose.valid, observation.camera_registration)
        expected = np.linalg.solve(self.jacobian, displacement)
        np.testing.assert_allclose([observation.pose.az_deg, observation.pose.el_deg], expected, atol=.003)
        self.assertEqual(observation.axis_sources, {'az': 'camera_registration', 'el': 'camera_registration'})
        self.assertEqual(observation.encoder_health['readings'], self.raw)
        self.assertFalse(observation.encoder_health['readings'][0]['valid'])
        self.assertGreaterEqual(observation.camera_registration['inliers'], 16)
        self.assertGreaterEqual(observation.camera_registration['confidence'], .5)
        self.assertEqual(observation.pose.timestamp, 99.97)
        json.dumps(observation.camera_registration, allow_nan=False)

    def test_affine_rotation_uses_displacement_at_image_center(self):
        center = (159.5, 119.5)
        matrix = cv2.getRotationMatrix2D(center, 1.2, 1.005)
        matrix[:, 2] += [5., 3.]
        self.image = cv2.warpAffine(self.baseline, matrix, (320, 240))
        observation = self.source().read()
        self.assertTrue(observation.pose.valid, observation.camera_registration)
        np.testing.assert_allclose(observation.camera_registration['measured_displacement_px'], [5., 3.], atol=.3)

    def test_blank_or_unrelated_scene_is_invalid(self):
        for image in (np.full_like(self.baseline, 150), np.random.default_rng(8).integers(0, 256, self.baseline.shape, dtype=np.uint8)):
            self.image = image
            observation = self.source().read()
            self.assertFalse(observation.pose.valid)
            self.assertFalse(observation.camera_registration['valid'])
            self.assertTrue(observation.camera_registration['error'])

    def test_stale_and_repeated_frames_never_refresh_position(self):
        source = self.source()
        self.assertTrue(source.read().pose.valid)
        self.assertTrue(source.last_frame_jpeg.startswith(b'\xff\xd8'))
        self.assertEqual(source.last_image.shape, self.baseline.shape)
        self.assertEqual(source.last_frame_sequence, 1)
        repeated = source.read()
        self.assertFalse(repeated.pose.valid)
        self.assertIsNone(source.last_frame_jpeg)
        self.assertIsNone(source.last_image)
        self.assertIn('sequence', repeated.camera_registration['error'].lower())
        self.sequence = 2
        self.age = 2.
        stale = source.read()
        self.assertFalse(stale.pose.valid)
        self.assertIn('stale', stale.camera_registration['error'].lower())

    def test_singular_scale_and_textureless_reference_rejected_before_fetch(self):
        with self.assertRaises(ValueError):
            OpticalBenchSource(self.jpeg, [[1., 1.], [2., 2.]], scale_reference='fixture', fetch=self.fetch)
        blank = cv2.imencode('.jpg', np.full_like(self.baseline, 150))[1].tobytes()
        with self.assertRaisesRegex(ValueError, 'features'):
            OpticalBenchSource(blank, self.jacobian, scale_reference='fixture', fetch=self.fetch)

    def test_diagnostics_are_saved_and_fetch_wait_is_bounded(self):
        gate = threading.Event()
        original = self.fetch
        def blocked(url, timeout, **kwargs):
            if url.endswith('/encoders'):
                gate.wait(2)
            return original(url, timeout, **kwargs)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'source.jsonl'
            source = OpticalBenchSource(self.jpeg, self.jacobian, scale_reference='fixture', fetch=blocked,
                                        clock=lambda: self.now, timeout_s=.03, diagnostics_path=path)
            try:
                started = time.monotonic()
                observation = source.read()
                self.assertLess(time.monotonic()-started, .5)
                self.assertTrue(observation.pose.valid, observation.camera_registration)
                self.assertTrue(observation.encoder_health['error'])
                row = json.loads(path.read_text().splitlines()[0])
                self.assertIn('camera_timing', row)
                self.assertIn('registration', row)
            finally:
                gate.set()
                source.close()


if __name__ == '__main__':
    unittest.main()
