import math
from collections import deque
import unittest
from det_types import Pose, Prediction
from execution import Executor


def target(t, az=0, el=0, rate=0, phase='ascent'):
    return Prediction(t, az, el, rate, 0, True, 'vision', .05, '', phase)


class ExecutorTests(unittest.TestCase):
    def test_feedback_uses_measured_pose_and_wraps_bearing(self):
        executor = Executor({'kp': 1, 'ki': 0, 'kd': 0, 'feedforward_gain': 0})
        first = executor.update(target(10, 1), Pose(10, 359, 0), 10, True)
        self.assertGreater(first.az_velocity_deg_s, 0)
        second = executor.update(target(10.1, 1), Pose(10.1, 1, 0), 10.1, True)
        self.assertAlmostEqual(second.az_velocity_deg_s, 0)

    def test_feedforward_and_existing_speed_acceleration_limits(self):
        executor = Executor({'kp': 0, 'ki': 0, 'kd': 0})
        previous = 0
        for i in range(20):
            t = 10 + i * .02
            command = executor.update(target(t, rate=100), Pose(t, 0, 0), t, True)
            self.assertLessEqual(abs(command.az_velocity_deg_s), 15)
            self.assertLessEqual(abs(command.az_velocity_deg_s - previous), 75 * .02 + 1e-9)
            self.assertAlmostEqual(command.valid_until - t, .25)
            previous = command.az_velocity_deg_s
        self.assertAlmostEqual(previous, 15)

    def test_disabled_and_invalid_inputs_stop_immediately_without_ramp(self):
        cases = [(target(11, 50), Pose(11, 0, 0), False),
                 (target(11, 50), Pose(10, 0, 0), True),
                 (target(10, 50), Pose(11, 0, 0), True),
                 (target(11, float('nan')), Pose(11, 0, 0), True),
                 (target(11, 50), Pose(11, 0, 0, False), True)]
        for prediction, pose, enabled in cases:
            executor = Executor()
            executor.update(target(10.99, 50), Pose(10.99, 0, 0), 10.99, True)
            command = executor.update(prediction, pose, 11, enabled)
            self.assertFalse(command.enabled)
            self.assertEqual((command.az_velocity_deg_s, command.el_velocity_deg_s), (0, 0))
            self.assertTrue(command.reason)

    def test_future_out_of_order_and_nonfinite_time_stop(self):
        executor = Executor()
        executor.update(target(10), Pose(10, 0, 0), 10, True)
        self.assertFalse(executor.update(target(9), Pose(9, 0, 0), 9, True).enabled)
        self.assertFalse(Executor().update(target(11), Pose(10, 0, 0), 10, True).enabled)
        bad = Executor().update(target(10), Pose(10, 0, 0), float('nan'), True)
        self.assertFalse(bad.enabled)
        self.assertTrue(math.isfinite(bad.timestamp))

    def test_derivative_uses_measured_rate_without_position_setpoint_kick(self):
        executor = Executor({'kp': 0, 'ki': 0, 'kd': 1, 'feedforward_gain': 0, 'derivative_tau_s': 0})
        executor.update(target(10), Pose(10, 0, 0), 10, True)
        changed_target = executor.update(target(10.1, 100), Pose(10.1, 0, 0), 10.1, True)
        self.assertEqual(changed_target.az_velocity_deg_s, 0)
        moving_pose = executor.update(target(10.2, 100), Pose(10.2, 1, 0), 10.2, True)
        self.assertLess(moving_pose.az_velocity_deg_s, 0)

    def test_integral_does_not_wind_up_against_saturation(self):
        executor = Executor({'kp': 1, 'ki': 1, 'kd': 0, 'feedforward_gain': 0})
        for i in range(100):
            t = 10 + .02 * i
            executor.update(target(t, 100), Pose(t, 0, 0), t, True)
        for i in range(20):
            t = 12 + .02 * i
            command = executor.update(target(t, 0), Pose(t, 0, 0), t, True)
        self.assertAlmostEqual(command.az_velocity_deg_s, 0)

    def test_main_handover_discards_unreachable_target_integral_without_image_feedback(self):
        executor = Executor({'kp': 0, 'ki': 1, 'kd': 0,
                             'main_kp': 0, 'main_ki': 2, 'main_kd': 0,
                             'feedforward_gain': 0})
        # A downstream travel stop can hold pose fixed while commands remain
        # below this executor's speed limit, so its normal antiwindup cannot see it.
        for index in range(100):
            now = 10 + .02 * index
            before = executor.update(target(now, el=.5), Pose(now, 0, 0), now, True)
        self.assertGreater(before.el_velocity_deg_s, .9)
        # Main starts with the target at current pose and no narrow image error.
        # Stored ascent effort must not be doubled by the main integral gain.
        now += .02
        after = executor.update(target(now, el=0, phase='main'), Pose(now, 0, 0), now, True)
        self.assertTrue(after.enabled)
        self.assertAlmostEqual(after.el_velocity_deg_s, 0)
        self.assertLessEqual(abs(after.el_velocity_deg_s - before.el_velocity_deg_s), 75 * .02)

    def test_main_tighter_than_premain_and_no_zeroing_requirement(self):
        pre, main = Executor(), Executor()
        for i in range(10):
            t = 10 + i * .02
            a = pre.update(target(t, 123.2), Pose(t, 123, 20), t, True)
            b = main.update(target(t, 123.2, phase='main'), Pose(t, 123, 20), t, True)
        self.assertTrue(a.enabled and b.enabled)
        self.assertGreater(b.az_velocity_deg_s, a.az_velocity_deg_s)

    def test_reset_clears_controller_memory_and_rejects_invalid_configuration(self):
        executor = Executor()
        executor.update(target(10, 50), Pose(10, 0, 0), 10, True)
        executor.reset()
        result = executor.update(target(1), Pose(1, 0, 0), 1, True)
        self.assertTrue(result.enabled)
        self.assertEqual(result.az_velocity_deg_s, 0)
        for config in ({'max_speed_deg_s': 16}, {'max_accel_deg_s2': 76}, {'kp': float('nan')}, {'lease_s': 0}):
            with self.assertRaises(ValueError):
                Executor(config)

    def test_duplicate_encoder_timestamp_does_not_redifferentiate_or_accept_changed_data(self):
        executor = Executor({'kp': 0, 'ki': 0, 'kd': 1, 'feedforward_gain': 0, 'derivative_tau_s': 0})
        executor.update(target(10), Pose(10, 0, 0), 10, True)
        executor.update(target(10.1), Pose(10.1, .5, 0), 10.1, True)
        duplicate = executor.update(target(10.2), Pose(10.1, .5, 0), 10.2, True)
        self.assertAlmostEqual(duplicate.az_velocity_deg_s, -5)
        changed = executor.update(target(10.21), Pose(10.1, 1, 0), 10.21, True)
        self.assertFalse(changed.enabled)

    def test_finite_inputs_that_overflow_calculation_stop_instead_of_saturating(self):
        executor = Executor({'kp': 1e308})
        command = executor.update(target(10, 100), Pose(10, 0, 0), 10, True)
        self.assertFalse(command.enabled)
        self.assertEqual(command.az_velocity_deg_s, 0)

    def test_delayed_constant_velocity_plant_has_no_persistent_position_lead_bias(self):
        executor = Executor({'kp': 2.5, 'ki': 0, 'kd': 0, 'deadband_deg': 0})
        pending = deque()
        angle, velocity, dt, lead = 359., 0., .02, .1
        errors = []
        for index in range(1000):
            elapsed = index * dt
            now = 1000 + elapsed
            while pending and pending[0][0] <= elapsed + 1e-9:
                _, velocity = pending.popleft()
            angle += velocity * dt
            current_target = 359 + elapsed
            prediction = target(now, (current_target + lead) % 360, rate=1)
            prediction.lead_time_s = lead
            command = executor.update(prediction, Pose(now, angle % 360, 0), now, True)
            pending.append((elapsed + lead, command.az_velocity_deg_s))
            if elapsed > 15:
                errors.append((angle - current_target + 180) % 360 - 180)
        self.assertLess(abs(sum(errors) / len(errors)), .005)

    def test_main_image_error_uses_capture_residual_and_keeps_velocity_feedforward(self):
        config = {'main_kp': 2, 'main_ki': 0, 'main_kd': 0, 'initial_dt_s': .1}
        prediction = target(10, 40, rate=.2, phase='main')
        prediction.lead_time_s = .1
        prediction.image_error_az_deg = .01
        prediction.image_error_el_deg = -.02
        prediction.image_timestamp = 9.95
        command = Executor(config).update(prediction, Pose(10, 0, 0), 10, True)
        self.assertTrue(command.enabled)
        self.assertAlmostEqual(command.az_velocity_deg_s, .22)
        self.assertAlmostEqual(command.el_velocity_deg_s, -.04)
        self.assertIn('image', command.reason)
        prediction.phase = 'ascent'
        coarse = Executor(config).update(prediction, Pose(10, 0, 0), 10, True)
        self.assertGreater(coarse.az_velocity_deg_s, 1)

    def test_stale_or_missing_image_error_falls_back_but_malformed_metadata_stops(self):
        config = {'main_kp': 1, 'main_ki': 0, 'main_kd': 0, 'feedforward_gain': 0}
        for stamp in (None, 9.):
            prediction = target(10, .1, phase='main')
            if stamp is not None:
                prediction.image_error_az_deg = 5.
                prediction.image_error_el_deg = 0.
                prediction.image_timestamp = stamp
            command = Executor(config).update(prediction, Pose(10, 0, 0), 10, True)
            self.assertTrue(command.enabled)
            self.assertAlmostEqual(command.az_velocity_deg_s, .1)
            self.assertNotIn('image', command.reason)
        for values in ((.1, None, 10), (.1, .1, float('nan')),
                       (float('nan'), .1, 10), (.1, .1, 11)):
            prediction = target(10, .1, phase='main')
            prediction.image_error_az_deg, prediction.image_error_el_deg, prediction.image_timestamp = values
            command = Executor(config).update(prediction, Pose(10, 0, 0), 10, True)
            self.assertFalse(command.enabled)

    def test_image_derivative_uses_image_motion_instead_of_quantized_encoder_rate(self):
        executor = Executor({'main_kp': 0, 'main_ki': 0, 'main_kd': 1,
                             'feedforward_gain': 0, 'derivative_tau_s': 0})
        for index in range(3):
            now = 10 + index * .1
            prediction = target(now, phase='main')
            prediction.image_error_az_deg = .01
            prediction.image_error_el_deg = 0
            prediction.image_timestamp = now
            command = executor.update(prediction, Pose(now, index * .1, 0), now, True)
            self.assertAlmostEqual(command.az_velocity_deg_s, 0)
        prediction = target(10.3, phase='main')
        prediction.image_error_az_deg = .02
        prediction.image_error_el_deg = 0
        prediction.image_timestamp = 10.3
        command = executor.update(prediction, Pose(10.3, .3, 0), 10.3, True)
        self.assertAlmostEqual(command.az_velocity_deg_s, .1)

    def test_regressed_or_changed_duplicate_image_feedback_stops(self):
        for stamp, error in ((9.99, .01), (10., .02)):
            executor = Executor()
            first = target(10., phase='main')
            first.image_error_az_deg, first.image_error_el_deg, first.image_timestamp = .01, 0., 10.
            self.assertTrue(executor.update(first, Pose(10., 0., 0.), 10., True).enabled)
            second = target(10.1, phase='main')
            second.image_error_az_deg, second.image_error_el_deg, second.image_timestamp = error, 0., stamp
            self.assertFalse(executor.update(second, Pose(10.1, 0., 0.), 10.1, True).enabled)

    def test_main_image_feedback_reduces_delayed_quantized_encoder_error(self):
        def run(with_images):
            executor = Executor()
            pending = deque()
            image_history = deque()
            angle, velocity, dt, lead = .04, 0., .02, .1
            encoder_step = 360 / 4096
            errors = []
            for index in range(1000):
                elapsed = index * dt
                now = 1000 + elapsed
                while pending and pending[0][0] <= elapsed + 1e-9:
                    _, velocity = pending.popleft()
                angle += velocity * dt
                current_target = .12 * elapsed
                measured = round(angle / encoder_step) * encoder_step
                prediction = target(now, current_target + .12 * lead, rate=.12, phase='main')
                prediction.lead_time_s = lead
                image_history.append((now, current_target - angle))
                if with_images and len(image_history) > 3:
                    capture_time, residual = image_history.popleft()
                    prediction.image_error_az_deg = residual
                    prediction.image_error_el_deg = 0
                    prediction.image_timestamp = capture_time
                command = executor.update(prediction, Pose(now, measured, 0), now, True)
                self.assertTrue(command.enabled)
                pending.append((elapsed + lead, command.az_velocity_deg_s))
                if elapsed > 10:
                    errors.append(abs(angle - current_target))
            return sum(errors) / len(errors)
        coarse, fine = run(False), run(True)
        self.assertLess(fine, coarse * .5)
        self.assertLess(fine, .01)

    def test_invalid_lead_metadata_is_rejected(self):
        for lead in (float('nan'), -.1, 10):
            prediction = target(10)
            prediction.lead_time_s = lead
            self.assertFalse(Executor().update(prediction, Pose(10, 0, 0), 10, True).enabled)


if __name__ == '__main__':
    unittest.main()
