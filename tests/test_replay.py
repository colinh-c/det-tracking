import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from det_types import CameraModel, Frame, Pose
from recording import Recorder
from replay import ReplaySource


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.cameras = {name: CameraModel(name, 64, 48, 200, 200, 32, 24)
                        for name in ('narrow', 'wide')}

    def frame(self, name, timestamp, sequence, value, az=5):
        return Frame(name, timestamp, np.full((48, 64), value, np.uint8),
                     Pose(timestamp-.01, az, 3, False), sequence)

    def record_pair(self):
        recorder = Recorder(self.root, self.cameras, 15)
        try:
            recorder.write({'narrow': self.frame('narrow', 9.8, 7, 60)},
                           10., Pose(9.99, 1, 2), None)
            recorder.write({'narrow': self.frame('narrow', 10.8, 8, 120),
                            'wide': self.frame('wide', 10.6, 71, 210, az=9)},
                           11., Pose(10.99, 2, 4), 1.25)
        finally:
            recorder.close()

    def rows(self):
        return [json.loads(line) for line in (self.root/'observations.jsonl').read_text().splitlines()]

    def replace_rows(self, rows):
        (self.root/'observations.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))

    def test_raw_roundtrip_preserves_individual_metadata_and_missing_slot(self):
        self.record_pair()
        rows = self.rows()
        self.assertEqual(rows[0]['schema_version'], 2)
        self.assertNotIn('wide', rows[0]['frames'])
        self.assertEqual(set(rows[0]), {'schema_version', 'now', 'pose', 'launch_elapsed_s', 'frames'})
        source = ReplaySource(self.root, self.cameras)
        self.addCleanup(source.close)
        frames, pose = source.read()
        self.assertEqual(source.now, 10)
        self.assertIsNone(source.launch_elapsed_s)
        self.assertEqual([frame.camera for frame in frames], ['narrow'])
        self.assertEqual(frames[0].timestamp, 9.8)
        self.assertEqual(frames[0].sequence, 7)
        self.assertEqual(frames[0].pose, Pose(9.8-.01, 5, 3, False))
        self.assertEqual(pose, Pose(9.99, 1, 2))
        frames, pose = source.read()
        frames = {frame.camera: frame for frame in frames}
        self.assertEqual(source.now, 11)
        self.assertEqual(source.launch_elapsed_s, 1.25)
        self.assertEqual(frames['wide'].timestamp, 10.6)
        self.assertEqual(frames['wide'].sequence, 71)
        self.assertEqual(frames['wide'].pose.az_deg, 9)
        self.assertAlmostEqual(float(frames['wide'].image.mean()), 210, delta=3)
        self.assertAlmostEqual(float(frames['narrow'].image.mean()), 120, delta=3)
        with self.assertRaises(StopIteration):
            source.read()

    def test_duplicate_capture_is_preserved_for_downstream_staleness(self):
        recorder = Recorder(self.root, self.cameras, 15)
        frame = self.frame('narrow', 9.8, 7, 80)
        recorder.write({'narrow': frame}, 10, Pose(10, 0, 0), None)
        recorder.write({'narrow': frame}, 11, Pose(11, 0, 0), None)
        recorder.close()
        source = ReplaySource(self.root, self.cameras)
        self.addCleanup(source.close)
        first, _ = source.read()
        second, _ = source.read()
        self.assertEqual((first[0].timestamp, first[0].sequence),
                         (second[0].timestamp, second[0].sequence))

    def test_invalid_write_is_rejected_before_any_video_slot_is_written(self):
        recorder = Recorder(self.root, self.cameras, 15)
        try:
            bad = self.frame('wide', 9.8, 1, 50)
            bad.image = np.zeros((10, 10), np.uint8)
            with self.assertRaisesRegex(ValueError, 'geometry'):
                recorder.write({'narrow': self.frame('narrow', 9.8, 1, 60), 'wide': bad},
                               10, Pose(10, 0, 0), None)
            recorder.write({'narrow': self.frame('narrow', 9.8, 1, 100)}, 10, Pose(10, 0, 0), None)
        finally:
            recorder.close()
        source = ReplaySource(self.root, self.cameras)
        self.addCleanup(source.close)
        frames, _ = source.read()
        self.assertAlmostEqual(float(frames[0].image.mean()), 100, delta=3)
        with self.assertRaises(StopIteration):
            source.read()

    def test_initialization_failure_releases_already_open_writers(self):
        class Writer:
            def __init__(self, opened): self.opened, self.released = opened, False
            def isOpened(self): return self.opened
            def release(self): self.released = True
        writers = [Writer(True), Writer(False)]
        with patch('recording.cv2.VideoWriter', side_effect=writers):
            with self.assertRaisesRegex(RuntimeError, 'writer'):
                Recorder(self.root, self.cameras, 15)
        self.assertTrue(all(writer.released for writer in writers))

    def test_legacy_rows_are_explicitly_rejected(self):
        self.record_pair()
        self.replace_rows([{'timestamp': 10, 'sequence': 1, 'pose': {'timestamp': 10, 'az_deg': 0, 'el_deg': 0, 'valid': True}}])
        source = ReplaySource(self.root, self.cameras)
        self.addCleanup(source.close)
        with self.assertRaisesRegex(ValueError, 'schema_version 2'):
            source.read()

    def test_malformed_nonfinite_and_reversed_metadata_rejected(self):
        self.record_pair()
        good = self.rows()
        for mutation in ('nan', 'global_order', 'frame_order', 'sequence', 'missing_pose'):
            rows = json.loads(json.dumps(good))
            if mutation == 'nan': rows[0]['pose']['az_deg'] = float('nan')
            elif mutation == 'global_order': rows[1]['now'] = 9
            elif mutation == 'frame_order': rows[1]['frames']['narrow']['timestamp'] = 8
            elif mutation == 'sequence': rows[0]['frames']['narrow']['sequence'] = True
            else: del rows[0]['frames']['narrow']['pose']
            self.replace_rows(rows)
            with self.subTest(mutation=mutation):
                source = ReplaySource(self.root, self.cameras)
                try:
                    with self.assertRaises(ValueError):
                        source.read(); source.read()
                finally:
                    source.close()

    def test_early_video_eof_is_corruption_not_clean_replay_end(self):
        self.record_pair()
        rows = self.rows()
        rows.append({**rows[-1], 'now': 12})
        self.replace_rows(rows)
        source = ReplaySource(self.root, self.cameras)
        self.addCleanup(source.close)
        source.read(); source.read()
        with self.assertRaisesRegex(ValueError, 'ended before'):
            source.read()

    def test_extra_video_slots_without_sidecar_rows_are_rejected(self):
        self.record_pair()
        self.replace_rows(self.rows()[:1])
        source = ReplaySource(self.root, self.cameras)
        self.addCleanup(source.close)
        source.read()
        with self.assertRaisesRegex(ValueError, 'without metadata'):
            source.read()

    def test_geometry_mismatch_is_rejected(self):
        self.record_pair()
        cameras = dict(self.cameras)
        cameras['narrow'] = CameraModel('narrow', 80, 48, 200, 200, 40, 24)
        source = ReplaySource(self.root, cameras)
        self.addCleanup(source.close)
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            source.read()

    def test_existing_recording_is_not_overwritten(self):
        self.record_pair()
        original = (self.root/'observations.jsonl').read_bytes()
        with self.assertRaises(FileExistsError):
            Recorder(self.root, self.cameras, 15)
        self.assertEqual((self.root/'observations.jsonl').read_bytes(), original)

    def test_optional_preview_preserves_slot_count_when_absent(self):
        import cv2
        recorder = Recorder(self.root, self.cameras, 15)
        recorder.write({}, 10, Pose(10, 0, 0), None)
        recorder.write({}, 11, Pose(11, 0, 0), None, np.full((48, 128, 3), 170, np.uint8))
        recorder.write({}, 12, Pose(12, 0, 0), None)
        recorder.close()
        capture = cv2.VideoCapture(str(self.root/'preview.mp4'))
        try:
            images = [capture.read() for _ in range(3)]
            self.assertTrue(all(ok for ok, _ in images))
            self.assertLess(images[0][1].mean(), 5)
            self.assertGreater(images[1][1].mean(), 160)
            self.assertLess(images[2][1].mean(), 5)
            self.assertFalse(capture.read()[0])
        finally:
            capture.release()
