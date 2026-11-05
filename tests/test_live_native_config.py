import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from det_types import Pose, angle_error
from launch_config import ROOT, load_launch_config, live_readiness
from transport import PiSource


class NativeLiveConfigTests(unittest.TestCase):
    def config(self):
        return load_launch_config(ROOT/'live_native_config.json')

    def test_native_profile_preserves_unverified_calibration_and_simulation_default(self):
        cfg = self.config()
        self.assertEqual(cfg['camera_base_url'], 'http://127.0.0.1:18082')
        self.assertEqual(cfg['motor_base_url'], 'http://127.0.0.1:18084')
        self.assertFalse(cfg['encoders']['calibrated'])
        self.assertIs(cfg['encoders']['allow_legacy_gateway'], True)
        self.assertEqual(cfg['encoders']['mapping_status'], 'unverified')
        self.assertEqual((cfg['encoders']['az']['bus'], cfg['encoders']['el']['bus']), (1, 3))
        self.assertIn('16 mm wide placeholder', cfg['calibration']['notes'])
        self.assertTrue(live_readiness(cfg))
        self.assertFalse(cfg['flight_configuration_confirmed'])
        self.assertFalse(cfg['payload_deployment_enabled'])
        for camera in cfg['camera_models'].values():
            self.assertEqual((camera.width, camera.height), (1456, 1088))
            self.assertEqual((camera.cx, camera.cy), (727.5, 543.5))
            self.assertFalse(camera.calibrated)
        default = load_launch_config()
        self.assertIsNot(default['encoders'].get('allow_legacy_gateway'), True)
        self.assertEqual(default['camera_models']['narrow'].width, 640)
        self.assertEqual(default['camera_models']['narrow'].height, 480)

    def test_scaled_projection_uses_pixel_centers_and_roundtrips(self):
        native = self.config()['camera_models']
        original = load_launch_config()['camera_models']
        pose = Pose(100, 12, 18)
        for name in native:
            for az, el in ((12, 18), (12.08, 18.05), (11.9, 17.92)):
                old = original[name].project(az, el, pose)
                point = native[name].project(az, el, pose)
                self.assertAlmostEqual(point[0], (old[0]+.5)*1456/640-.5, places=8)
                self.assertAlmostEqual(point[1], (old[1]+.5)*1088/480-.5, places=8)
                recovered = native[name].unproject(*point, pose)
                self.assertAlmostEqual(angle_error(recovered[0], az), 0, places=8)
                self.assertAlmostEqual(recovered[1], el, places=8)

    def test_native_jpegs_are_accepted_but_unregistered_pose_stays_invalid(self):
        cfg = self.config()
        encoded = cv2.imencode('.jpg', np.full((1088, 1456), 128, np.uint8))[1].tobytes()
        requested = []

        def fake_http(url, timeout, data=None, limit=None):
            self.assertIsNone(data)
            self.assertTrue(url.startswith('http://127.0.0.1:18082/'))
            requested.append(url)
            if url.endswith('/encoders'):
                readings = [dict(bus=bus, enabled=True, connected=True, valid=True,
                                 magnet_detected=True, field_weak=False, field_strong=False,
                                 stale=False, age_s=.01, read_at=50, degrees=20)
                            for bus in (1, 3)]
                return json.dumps(readings).encode(), {}, 99.98, 100.
            info = dict(sequence=7, age_s=.01, size=[1456, 1088], error=None)
            return encoded, {'content-type': 'image/jpeg', 'x-frame-info': json.dumps(info)}, 99.98, 100.

        with patch('transport._http', side_effect=fake_http), patch('transport.time.monotonic', return_value=100.):
            source = PiSource(cfg['camera_base_url'], cfg['camera_models'], cfg['encoders'])
            try:
                frames, pose = source.read()
                self.assertEqual({frame.camera for frame in frames}, {'narrow', 'wide'})
                self.assertTrue(all(frame.image.shape == (1088, 1456) for frame in frames))
                self.assertTrue(all(frame.sequence == 7 and not frame.pose.valid for frame in frames))
                self.assertFalse(pose.valid)
                self.assertFalse(source.generation_verified)
                self.assertIsNone(source.generation)
            finally:
                source.close()
        self.assertEqual(len(requested), 3)

    def test_launcher_invokes_only_native_readonly_live_view(self):
        launcher = ROOT/'Start Native Live View.command'
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            command = root/launcher.name
            command.write_text(launcher.read_text())
            python = root/'.venv/bin/python'
            python.parent.mkdir(parents=True)
            python.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
            python.chmod(0o755)
            result = subprocess.run(['/bin/zsh', str(command)], cwd='/', check=True,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.stdout.splitlines(),
                             ['launch.py', '--mode', 'live', '--serve', '--config', 'live_native_config.json'])
        self.assertNotIn('--allow-motion', launcher.read_text())
        self.assertNotIn('open ', launcher.read_text())


if __name__ == '__main__':
    unittest.main()
