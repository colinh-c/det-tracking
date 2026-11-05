"""Real JPEG codec tests on native-size fixtures, without camera hardware."""
import unittest
import importlib.util
from unittest.mock import patch, Mock

import cv2
import numpy as np

from pi_capture_gateway import JpegEncoder


class CaptureJpegTests(unittest.TestCase):
    @staticmethod
    def decode(data):
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise AssertionError('Encoded output is not a decodable JPEG')
        return image

    @unittest.skipUnless(importlib.util.find_spec('simplejpeg'), 'Optional simplejpeg codec is not installed')
    def test_native_noncontiguous_rgb_preserves_dimensions_and_channel_order(self):
        storage = np.zeros((1088, 2912, 3), np.uint8)
        rgb = storage[:, ::2]
        rgb[:] = [25, 40, 210]
        rgb[150:900, 200:1200] = [225, 35, 20]
        self.assertFalse(rgb.flags.c_contiguous)
        encoder = JpegEncoder(85)
        image = self.decode(encoder.encode(rgb))
        self.assertEqual(encoder.backend, 'simplejpeg_rgb420_fastdct')
        self.assertIsNone(encoder.fallback_reason)
        self.assertEqual(image.shape, (1088, 1456, 3))
        # OpenCV decodes BGR, while Picamera2 BGR888 buffers contain RGB bytes.
        self.assertGreater(int(image[500, 600, 2]), 200)
        self.assertLess(int(image[500, 600, 0]), 40)
        self.assertGreater(int(image[50, 50, 0]), 190)

    @unittest.skipUnless(importlib.util.find_spec('PIL'), 'Pillow fallback codec is not installed')
    def test_missing_optional_codec_uses_decodable_pillow_fallback(self):
        with patch('pi_capture_gateway._load_simplejpeg', side_effect=ImportError('not installed')):
            encoder = JpegEncoder(85)
        image = self.decode(encoder.encode(np.full((1088, 1456, 3), 145, np.uint8)))
        self.assertEqual(image.shape, (1088, 1456, 3))
        self.assertEqual(encoder.backend, 'pillow_rgb420')
        self.assertIn('not installed', encoder.fallback_reason)

    @unittest.skipUnless(importlib.util.find_spec('PIL'), 'Pillow fallback codec is not installed')
    def test_codec_failure_falls_back_once_without_per_frame_retry(self):
        module = Mock()
        module.encode_jpeg.side_effect = RuntimeError('codec unavailable')
        with patch('pi_capture_gateway._load_simplejpeg', return_value=module):
            encoder = JpegEncoder(85)
        fixture = np.full((64, 96, 3), 150, np.uint8)
        self.decode(encoder.encode(fixture))
        self.decode(encoder.encode(fixture))
        self.assertEqual(module.encode_jpeg.call_count, 1)
        self.assertEqual(encoder.backend, 'pillow_rgb420')
        self.assertIn('codec unavailable', encoder.fallback_reason)

    def test_invalid_input_is_rejected_without_silent_image_conversion(self):
        encoder = JpegEncoder(85)
        for image in (np.zeros((64, 96), np.uint8), np.zeros((64, 96, 3), np.float32),
                      np.zeros((64, 96, 4), np.uint8), np.zeros((0, 96, 3), np.uint8)):
            with self.assertRaises(ValueError):
                encoder.encode(image)
        for quality in (0, 96, True, 85.5):
            with self.assertRaises(ValueError):
                JpegEncoder(quality)


if __name__ == '__main__':
    unittest.main()
