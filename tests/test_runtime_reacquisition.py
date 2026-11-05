"""Saved-flight loss boundaries, using real images and the production engine."""
import unittest

import cv2
import numpy as np

from det_types import Frame, Pose, Prediction
from launch_config import load_launch_config
from runtime import TrackingEngine


class RuntimeReacquisitionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_launch_config()
        self.now = 1000.
        self.engine = TrackingEngine(self.cfg, clock=lambda: self.now)
        image = np.full((480, 640), 150, np.uint8)
        image[160:320, 317:322] = 45
        self.engine.step([Frame('narrow', self.now, image, Pose(self.now, 0, 0), 0)],
                         Pose(self.now, 0, 0), self.now, 0)
        self.now = 1102.
        self.engine.previous = Prediction(self.now - 1, valid=False,
                                         reason='Target loss exceeded bounded coast')

    def test_invalid_motion_prediction_does_not_remove_visual_search_prior(self):
        pose = Pose(self.now, 15, 39.6)
        narrow = np.full((480, 640), 150, np.uint8)
        wide = narrow.copy()
        cv2.circle(narrow, (524, 134), 3, 235, -1)
        cv2.circle(wide, (524, 134), 3, 235, -1)
        # Independent fixture: the saved 102-second target projects near
        # (102.35, .48) in wide, far above the narrow camera's field of view.
        wide[0:4, 101:104] = 45
        frames = [Frame('narrow', self.now, narrow, pose, 1),
                  Frame('wide', self.now, wide, pose, 1)]
        result, prediction, _ = self.engine.step(frames, pose, self.now, 102)
        self.assertIsNotNone(result.selected)
        self.assertEqual('wide', result.selected.camera)
        self.assertLess(result.selected.y, 4)
        self.assertTrue(prediction.valid)
        self.assertIn('vision', prediction.source)

    def test_association_prior_does_not_extend_expired_motor_prediction(self):
        pose = Pose(self.now, 15, 39.6)
        result, prediction, command = self.engine.step([], pose, self.now, 102, True)
        self.assertIsNone(result.selected)
        self.assertFalse(prediction.valid)
        self.assertFalse(command.enabled)


if __name__ == '__main__':
    unittest.main()
