import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from det_types import CameraModel, Frame, Pose
from launch_config import load_launch_config
from recording import Recorder

try:
    import offline_hil_replay as replay
except ModuleNotFoundError:
    replay = None


class OfflineHilReplayTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(replay, 'offline HIL replay evaluator must exist')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.run = self.root/'saved'
        self.run.mkdir()
        self.cameras = {name: CameraModel(name, 64, 48, 100, 100, 31.5, 23.5)
                        for name in ('narrow', 'wide')}
        cfg = load_launch_config()
        cfg['cameras'] = {name: {key: value for key, value in vars(camera).items() if key != 'name'}
                          for name, camera in self.cameras.items()}
        cfg.pop('camera_models')
        cfg['perception']['association_gate_deg'] = 10.
        configuration = {'launch_config': cfg, 'camera_models': {n: vars(c) for n,c in self.cameras.items()},
                         'field_config': json.loads(Path(cfg['field_config']).read_text()),
                         'profile': {'fps': 15.}}
        (self.run/'configuration.json').write_text(json.dumps(configuration))
        (self.run/'summary.json').write_text(json.dumps({'launch_origin_monotonic': 10.}))
        recorder = Recorder(self.run/'synthetic-frames', self.cameras, 15)
        self.rows = []
        for index, (now, elapsed, capture) in enumerate(((9.9, None, 9.8), (10.1, .1, 10.02), (11., 1., 10.92))):
            pose = Pose(now-.01, 2+index, 1)
            # Distinct camera poses and sequences catch accidental global metadata substitution.
            frames = {n: Frame(n, capture, self.body_image(index),
                               Pose(capture-.005, 2+index+j, 1), index+j*100)
                      for j,n in enumerate(self.cameras)}
            recorder.write(frames, now, pose, elapsed)
            row = {'now': now, 'launch_elapsed_s': elapsed, 'priming': elapsed is None,
                   'pose': vars(pose),
                   'frames': {n: {'timestamp': f.timestamp, 'sequence': f.sequence, 'pose': vars(f.pose)} for n,f in frames.items()},
                   'vision': {'selected': None, 'candidates': {}, 'phase': {'name': 'unknown'}}}
            if elapsed is not None:
                row['evaluation_only'] = {'time_s': capture-10., 'phase': 'boost', 'az_deg': 0., 'el_deg': 0.,
                                          'visible': {'narrow': True, 'wide': True}}
            self.rows.append(row)
        recorder.close()
        self.save_rows()

    @staticmethod
    def body_image(index):
        image = np.full((48,64), 120+index*20, np.uint8)
        image[12:36, 30:34] = 35
        return image

    def save_rows(self):
        (self.run/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))

    def test_exact_metadata_mapping_rejects_shifted_time_sequence_pose_and_row_count(self):
        observations = [json.loads(x) for x in (self.run/'synthetic-frames'/'observations.jsonl').read_text().splitlines()]
        replay.validate_alignment(self.rows, observations, self.cameras, 10.)
        for field in ('now', 'launch_elapsed_s', 'sequence', 'timestamp', 'pose', 'count'):
            bad = copy.deepcopy(observations)
            if field in ('now', 'launch_elapsed_s'):
                bad[1][field] += .001
            elif field == 'count':
                bad.pop()
            elif field == 'pose':
                bad[1]['frames']['wide']['pose']['az_deg'] += .01
            else:
                bad[1]['frames']['wide'][field] += 1
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'mapping|count'):
                replay.validate_alignment(self.rows, bad, self.cameras, 10.)

    def test_replay_preserves_clock_capture_pose_elapsed_and_disables_commands(self):
        result = replay.evaluate(self.run, self.root/'result')
        rows = [json.loads(x) for x in (self.root/'result'/'telemetry.jsonl').read_text().splitlines()]
        self.assertTrue(result['completed'])
        self.assertEqual(result['flight']['frames'], 2)
        self.assertEqual([r['now'] for r in rows], [9.9, 10.1, 11.])
        self.assertEqual([r['engine_elapsed_s'] for r in rows], [0., .1, 1.])
        self.assertEqual(rows[1]['frames']['wide']['sequence'], 101)
        self.assertEqual(rows[1]['frames']['wide']['pose']['az_deg'], 4)
        self.assertTrue(all(not r['command']['enabled'] for r in rows))
        self.assertFalse(result['hardware_access'])
        self.assertIn('MJPEG', ' '.join(result['limitations']))
        self.assertEqual(len(result['input_sha256']['synthetic-frames/narrow.avi']), 64)
        with self.assertRaises(FileExistsError):
            replay.evaluate(self.run, self.root/'result')

    def test_truth_changes_metrics_without_changing_engine_decisions(self):
        replay.evaluate(self.run, self.root/'first')
        for row in self.rows:
            if 'evaluation_only' in row:
                row['evaluation_only']['az_deg'] = 90.
        self.save_rows()
        replay.evaluate(self.run, self.root/'second')
        def read(name):
            return [json.loads(x) for x in (self.root/name/'telemetry.jsonl').read_text().splitlines()]
        first, second = read('first'), read('second')
        self.assertTrue(any(r['vision']['selected'] is not None for r in first))
        self.assertEqual([r['vision'] for r in first], [r['vision'] for r in second])
        self.assertNotEqual(first[1]['evaluation']['selected_error_deg'], second[1]['evaluation']['selected_error_deg'])
        self.assertEqual(first[1]['evaluation']['truth_by_camera']['wide']['az_deg'], 0)
        self.assertEqual(second[1]['evaluation']['truth_by_camera']['wide']['az_deg'], 90)

    def test_offline_run_cannot_import_hardware_or_open_network_sockets(self):
        # Exercise the complete real replay path with forbidden transport imports and socket creation.
        code = '''
import builtins, socket, sys
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in {'transport','control_handoff','full_flight_hil','pi_capture_gateway','pi_tracking_gateway','encoder_reader'}:
        raise AssertionError('hardware import: '+name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
def denied(*args, **kwargs): raise AssertionError('network socket creation')
socket.socket = denied
import offline_hil_replay
result = offline_hil_replay.evaluate(sys.argv[1], sys.argv[2])
assert result['completed'] and result['hardware_access'] is False
'''
        result = subprocess.run([sys.executable, '-c', code, str(self.run), str(self.root/'guarded')],
                                cwd=Path(__file__).resolve().parents[1], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
