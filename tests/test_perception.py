import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from det_types import CameraModel, Frame, Pose, Prediction
from perception import Perception


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.now = 1000.0
        self.cameras = {
            'narrow': CameraModel('narrow', 320, 240, 1200, 1200, 160, 120, calibrated=True),
            'wide': CameraModel('wide', 320, 240, 300, 300, 160, 120, calibrated=True),
        }
        self.config = {'clock': lambda: self.now, 'max_frame_age_s': .5}
        self.perception = Perception(self.cameras, self.config)

    def image(self, body=(156, 90, 8, 60), value=230, background=150):
        image = np.full((240, 320), background, dtype=np.uint8)
        if body:
            x, y, w, h = body
            image[y:y+h, x:x+w] = value
        return image

    def frame(self, image, camera='narrow', sequence=1, timestamp=None, pose=None):
        timestamp = self.now if timestamp is None else timestamp
        return Frame(camera, timestamp, image, pose or Pose(timestamp, 0, 0), sequence)

    def hint(self, camera='narrow', x=160, y=120):
        az, el = self.cameras[camera].unproject(x, y, Pose(self.now, 0, 0))
        return Prediction(self.now, az, el, valid=True, source='acquisition', uncertainty_deg=.3)

    def test_detects_bright_and_dark_body_without_trained_classifier(self):
        for value in (230, 45):
            with self.subTest(value=value):
                detector = Perception(self.cameras, self.config)
                result = detector.update([self.frame(self.image(value=value))], self.hint())
                self.assertIsNotNone(result.selected)
                self.assertTrue(result.narrow_found)
                self.assertAlmostEqual(159.5, result.selected.x, delta=1)
                self.assertEqual('rocket', result.selected.identity)
                self.assertEqual('unknown', result.phase.name)

    def test_temporal_association_rejects_brighter_distractor_and_bad_prior(self):
        self.perception.update([self.frame(self.image())], self.hint())
        self.now += .05
        image = self.image((158, 90, 8, 60))
        cv2.circle(image, (205, 120), 12, 255, -1)
        result = self.perception.update([self.frame(image, sequence=2)], self.hint(x=205))
        self.assertIsNotNone(result.selected)
        self.assertLess(result.selected.x, 170)

    def test_wide_reacquisition_then_narrow_preference(self):
        wide = self.frame(self.image((157, 110, 5, 20)), 'wide')
        result = self.perception.update([self.frame(self.image(None)), wide], self.hint())
        self.assertEqual('wide', result.selected.camera)
        self.assertFalse(result.narrow_found)
        self.now += .05
        result = self.perception.update([
            self.frame(self.image(), sequence=2),
            self.frame(self.image((157, 110, 5, 20)), 'wide', 2)], self.hint())
        self.assertEqual('narrow', result.selected.camera)

    def test_missing_detection_is_not_declared_out_of_frame(self):
        self.perception.update([self.frame(self.image())], self.hint())
        self.now += .05
        result = self.perception.update([self.frame(self.image(None), sequence=2)], self.hint())
        self.assertIsNone(result.selected)
        self.assertFalse(result.narrow_found)
        self.assertTrue(any('no_detection' in warning for warning in result.warnings))
        self.assertFalse(any('out_of_frame' in warning for warning in result.warnings))

    def test_duplicate_stale_and_invalid_pose_do_not_refresh_detection(self):
        frame = self.frame(self.image())
        self.perception.update([frame], self.hint())
        duplicate = self.perception.update([frame], self.hint())
        self.assertIsNone(duplicate.selected)
        self.assertTrue(any('duplicate' in warning for warning in duplicate.warnings))
        self.now += 1
        stale = self.perception.update([self.frame(self.image(), sequence=2, timestamp=1000)], self.hint())
        self.assertIsNone(stale.selected)
        self.assertTrue(any('stale' in warning for warning in stale.warnings))
        invalid = self.perception.update([self.frame(self.image(), sequence=3, pose=Pose(self.now, 0, 0, False))])
        self.assertIsNone(invalid.selected)

    def test_main_targets_body_instead_of_larger_canopy(self):
        image = self.image((157, 135, 6, 65))
        cv2.ellipse(image, (160, 65), (45, 16), 0, 0, 360, 240, -1)
        result = self.perception.update([self.frame(image)], self.hint(y=167), phase_override='main_descent')
        self.assertIsNotNone(result.selected)
        self.assertGreater(result.selected.y, 140)
        self.assertEqual('body_centroid_uncertain_com', result.selected.aim_kind)
        self.assertEqual('manual', result.phase.source)
        self.assertFalse(any(item.identity == 'payload' for values in result.candidates.values() for item in values))

    def test_main_canopy_only_does_not_become_rocket_com(self):
        image = self.image(None)
        cv2.ellipse(image, (160, 100), (45, 16), 0, 0, 360, 240, -1)
        result = self.perception.update([self.frame(image)], self.hint(y=100), phase_override='main_descent')
        self.assertIsNone(result.selected)

    def test_thin_cord_does_not_merge_body_and_canopy_aim_point(self):
        image = self.image((157, 135, 6, 65))
        cv2.ellipse(image, (160, 65), (45, 16), 0, 0, 360, 230, -1)
        cv2.line(image, (160, 80), (160, 135), 230, 1)
        result = self.perception.update([self.frame(image)], self.hint(y=167), phase_override='main_descent')
        self.assertIsNotNone(result.selected)
        self.assertGreater(result.selected.y, 140)

    def test_configured_nose_direction_produces_labeled_com_offset(self):
        detector = Perception(self.cameras, {**self.config, 'nose_direction_by_camera': {'narrow': [0, -1]}})
        result = detector.update([self.frame(self.image())], self.hint(), phase_override='main_descent')
        expected = 90 + (1.536 / 2.4516) * 59
        self.assertAlmostEqual(expected, result.selected.y, delta=1)
        self.assertEqual('body_com_configured_nose', result.selected.aim_kind)

    def test_timeline_is_labeled_prior_and_truth_attributes_are_ignored(self):
        frame = self.frame(self.image())
        frame.truth_phase = 'main_descent'
        frame.truth_target = (1, 1)
        unknown = self.perception.update([frame], self.hint())
        self.assertEqual('unknown', unknown.phase.name)
        self.now += .05
        result = self.perception.update([self.frame(self.image(), sequence=2)], self.hint(), launch_elapsed_s=115)
        self.assertEqual('main_descent', result.phase.name)
        self.assertEqual('simulation_prior', result.phase.source)
        self.assertLessEqual(result.phase.confidence, .4)
        self.assertGreater(result.selected.x, 100)

    def test_gimbal_pose_motion_is_not_mistaken_for_target_identity_motion(self):
        self.perception.update([self.frame(self.image())], self.hint())
        self.now += .05
        pose = Pose(self.now, .5, 0)
        pixel = self.cameras['narrow'].project(0, 0, pose)
        image = self.image((round(pixel[0])-4, 90, 8, 60))
        result = self.perception.update([self.frame(image, sequence=2, pose=pose)])
        self.assertIsNotNone(result.selected)
        self.assertAlmostEqual(0, (result.selected.bearing_deg+180)%360-180, delta=.08)

    def test_older_cross_camera_frame_cannot_rewind_track(self):
        self.perception.update([self.frame(self.image((157, 110, 5, 20)), 'wide')], self.hint())
        self.now += .1
        result = self.perception.update([self.frame(self.image(), timestamp=999.95)], self.hint())
        self.assertIsNone(result.selected)
        self.assertTrue(any('older_than_track' in warning for warning in result.warnings))

    def test_visual_canopy_cue_respects_camera_roll_and_stays_heuristic(self):
        self.cameras['narrow'].roll_deg = 90
        detector = Perception(self.cameras, self.config)
        for index in range(5):
            self.now = 1000 + .15 * index
            image = self.image((130-index, 116, 60, 8))
            cv2.ellipse(image, (245-index, 120), (16, 40), 0, 0, 360, 240, -1)
            result = detector.update([self.frame(image, sequence=index+1)], self.hint())
        self.assertIsNotNone(result.selected)
        self.assertEqual('main_descent', result.phase.name)
        self.assertEqual('visual_heuristic', result.phase.source)
        self.assertLess(result.phase.confidence, .7)

    def test_resolved_body_cannot_turn_into_tiny_interior_hot_spot(self):
        self.perception.update([self.frame(self.image())], self.hint())
        self.now += .05
        result = self.perception.update([
            self.frame(self.image((162, 115, 5, 5)), sequence=2),
            self.frame(self.image((159, 112, 2, 15)), 'wide', 2)], self.hint())
        self.assertIsNotNone(result.selected)
        self.assertEqual('wide', result.selected.camera)
        self.assertFalse(result.narrow_found)

    def test_cross_camera_disagreement_prefers_temporal_body_over_narrow(self):
        self.perception.update([
            self.frame(self.image()),
            self.frame(self.image((159, 112, 2, 15)), 'wide')], self.hint())
        self.now += .05
        result = self.perception.update([
            self.frame(self.image((178, 90, 8, 60)), sequence=2),
            self.frame(self.image((159, 112, 2, 15)), 'wide', 2)], self.hint())
        self.assertEqual('wide', result.selected.camera)
        self.assertFalse(result.narrow_found)

    def test_clipped_body_not_rejected_only_for_shrinking(self):
        self.perception.update([self.frame(self.image((156, 0, 8, 60)))], self.hint(y=30))
        self.now += .05
        # Gimbal motion projects the same body mostly beyond the top edge.
        pose = Pose(self.now, 0, -2.3)
        result = self.perception.update([
            self.frame(self.image((156, 0, 8, 12)), sequence=2, pose=pose)])
        self.assertIsNotNone(result.selected)

    def test_unresolved_distant_body_can_follow_wide_acquisition(self):
        wide = self.frame(self.image((159, 119, 2, 2)), 'wide')
        self.perception.update([wide], self.hint())
        self.now += .05
        result = self.perception.update([
            self.frame(self.image((158, 117, 4, 6)), sequence=2),
            self.frame(self.image((159, 119, 2, 2)), 'wide', 2)], self.hint())
        self.assertIsNotNone(result.selected)
        self.assertEqual('narrow', result.selected.camera)

    def test_bbox_is_xyxy_and_prior_is_deled_to_exposure_time(self):
        prior = self.hint()
        prior.az_rate_deg_s = 10
        prior.az_deg = 2
        prior.lead_time_s = .2
        result = self.perception.update([self.frame(self.image())], prior)
        self.assertIsNotNone(result.selected)
        self.assertEqual((156, 90, 164, 150), result.selected.bbox)

    def test_canopy_alone_is_not_relabelled_as_rocket_before_main(self):
        image = self.image(None)
        cv2.ellipse(image, (160, 100), (45, 16), 0, 0, 360, 240, -1)
        result = self.perception.update([self.frame(image)], self.hint(y=100), phase_override='drogue')
        self.assertIsNone(result.selected)

    def test_small_resolved_canopy_cannot_inherit_rocket_identity(self):
        image = self.image(None)
        cv2.ellipse(image, (160, 120), (2, 2), 0, 180, 360, 235, -1)
        result = self.perception.update([self.frame(image)], self.hint(), phase_override='drogue')
        self.assertIsNone(result.selected)

    def test_pixel_scale_drogue_cannot_prove_main_deployment(self):
        for index in range(6):
            self.now = 1000 + .15 * index
            image = self.image((159, 110 + index, 3, 8), value=45)
            cv2.ellipse(image, (160, 100 + index), (2, 2), 0, 180, 360, 235, -1)
            result = self.perception.update([self.frame(image, sequence=index + 1)],
                self.hint(y=114 + index), launch_elapsed_s=102 + .15 * index)
        self.assertIsNotNone(result.selected)
        self.assertEqual('drogue_descent', result.phase.name)
        self.assertEqual('simulation_prior', result.phase.source)

    def test_unanchored_singleton_cannot_win_when_other_camera_is_ambiguous(self):
        # A hot spot is the only narrow feature. Two equally plausible wide
        # features must not make that singleton inherit rocket identity.
        narrow = self.image((240, 60, 7, 7))
        wide = self.image((60, 60, 7, 7))
        wide[170:177, 240:247] = 230
        result = self.perception.update([
            self.frame(narrow), self.frame(wide, 'wide')])
        self.assertIsNone(result.selected)
        self.assertTrue(result.candidates['narrow'])

    def test_prior_outside_narrow_view_does_not_confirm_interior_spot(self):
        # The expected ray is visible in the wide camera but far above narrow.
        # The ten-degree prior gate alone cannot confirm a narrow hot spot.
        prior = Prediction(self.now, 0, 8, valid=True,
                           source='openrocket_prior', uncertainty_deg=5)
        result = self.perception.update([
            self.frame(self.image((240, 60, 7, 7)))], prior)
        self.assertIsNone(result.selected)

    def test_prior_outside_view_still_allows_body_clipped_at_expected_edge(self):
        prior = Prediction(self.now, 0, 6, valid=True,
                           source='openrocket_prior', uncertainty_deg=5)
        result = self.perception.update([
            self.frame(self.image((156, 0, 8, 10)))], prior)
        self.assertIsNotNone(result.selected)
        self.assertEqual('narrow', result.selected.camera)

    def test_corner_entry_requires_both_expected_boundaries(self):
        prior = Prediction(self.now, 352, 6, valid=True,
                           source='openrocket_prior', uncertainty_deg=5)
        result = self.perception.update([
            self.frame(self.image((156, 0, 8, 10)))], prior)
        self.assertIsNone(result.selected)
        self.now += .05
        prior.timestamp = self.now
        result = self.perception.update([
            self.frame(self.image((0, 0, 8, 10)), sequence=2)], prior)
        self.assertIsNotNone(result.selected)

    def test_independent_camera_agreement_can_acquire_with_mismatched_prior(self):
        prior = Prediction(self.now, 0, 8, valid=True,
                           source='openrocket_prior', uncertainty_deg=5)
        # Narrow's expected ray is outside its image. Agreement with the
        # independent wide observation can still support acquisition.
        result = self.perception.update([
            self.frame(self.image()),
            self.frame(self.image((159, 112, 2, 15)), 'wide')], prior)
        self.assertIsNotNone(result.selected)
        self.assertEqual('narrow', result.selected.camera)

    def test_two_agreeing_cameras_can_acquire_without_prior_or_track(self):
        result = self.perception.update([
            self.frame(self.image()),
            self.frame(self.image((159, 112, 2, 15)), 'wide')])
        self.assertIsNotNone(result.selected)
        self.assertTrue(result.narrow_found)
        self.assertTrue(result.wide_found)

    def test_expired_track_rates_cannot_manufacture_cross_camera_agreement(self):
        self.perception.track = {'timestamp': self.now - 2, 'az': 0, 'el': 0,
                                 'az_rate': 10, 'el_rate': 0}
        result = self.perception.update([
            self.frame(self.image((177, 90, 8, 60))),
            self.frame(self.image((159, 112, 2, 15)), 'wide',
                       timestamp=self.now - .1)])
        self.assertIsNone(result.selected)

    def test_closed_loop_early_ascent_does_not_lock_sensor_hot_spot(self):
        from launch_config import load_launch_config
        from runtime import TrackingApp
        from simulation import SyntheticFlight
        from det_types import angle_error
        cfg = load_launch_config()
        source = SyntheticFlight(cfg['camera_models'], cfg['trajectory'], cfg['field_config'], fps=15)
        app = TrackingApp(cfg, 'simulate', source)
        errors = []
        try:
            for _ in range(46):
                result, _, _, _ = app.tick()
                if result.selected:
                    errors.append(abs(angle_error(result.selected.elevation_deg, source.truth['el_deg'])))
                source.advance()
            self.assertGreater(len(errors), 35)
            self.assertLess(max(errors), .2)
        finally:
            app.close()


if __name__ == '__main__':
    unittest.main()
