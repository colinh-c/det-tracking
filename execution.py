"""Measured-pose and main-phase image PID commands, with no hardware interface.

Configured speed/acceleration ceilings are existing controller limits, not
mechanical travel bounds. Live calibration and transport enforcement belong to
the runtime/gateway. Fresh main-phase image residuals bypass encoder quantization
for position/rate correction while valid encoders remain mandatory. All
timestamps are on the Mac monotonic clock.
"""
import math
from numbers import Real
import numpy as np
from det_types import Command, Pose, Prediction, angle_error


DEFAULTS = {
    'kp': 2.5, 'ki': .2, 'kd': .08,
    'main_kp': 4., 'main_ki': .4, 'main_kd': .12,
    'feedforward_gain': 1., 'max_speed_deg_s': 15., 'max_accel_deg_s2': 75.,
    'pose_max_age_s': .3, 'prediction_max_age_s': .3,
    'max_lead_time_s': .5, 'image_max_age_s': .2,
    'max_uncertainty_deg': 12., 'main_max_uncertainty_deg': 3.,
    'lease_s': .25, 'initial_dt_s': .02, 'max_control_gap_s': .3,
    'integral_limit': 10., 'derivative_tau_s': .08,
    'deadband_deg': .02, 'main_deadband_deg': .005,
}


def finite(*values):
    return all(isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value) for value in values)


class Executor:
    def __init__(self, config: dict = None):
        self.config = dict(DEFAULTS)
        if config:
            unknown = set(config) - set(DEFAULTS)
            if unknown:
                raise ValueError(f'Unknown executor configuration: {sorted(unknown)}')
            self.config.update(config)
        positive = {'max_speed_deg_s', 'max_accel_deg_s2', 'pose_max_age_s', 'prediction_max_age_s',
                    'max_lead_time_s', 'image_max_age_s',
                    'max_uncertainty_deg', 'main_max_uncertainty_deg', 'lease_s', 'initial_dt_s',
                    'max_control_gap_s', 'integral_limit'}
        for name, value in self.config.items():
            if not finite(value) or value < 0 or (name in positive and value == 0):
                raise ValueError(f'Invalid executor setting {name}')
        if self.config['max_speed_deg_s'] > 15 or self.config['max_accel_deg_s2'] > 75:
            raise ValueError('Requested limits exceed configured controller ceilings of 15 deg/s and 75 deg/s^2')
        if self.config['lease_s'] > 1 or self.config['initial_dt_s'] > self.config['max_control_gap_s']:
            raise ValueError('Use a short lease and initial step within the control-gap bound')
        self._sequence = 0
        self.reset()

    def reset(self):
        self._integral = np.zeros(2)
        self._velocity = np.zeros(2)
        self._measured_rate = np.zeros(2)
        self._last_now = self._last_pose = None
        self._pose_watermark = self._prediction_watermark = None
        self._image_watermark = None
        self._image_history = None
        self._image_rate = np.zeros(2)
        self._using_image = False
        self._using_main = False

    def _stop(self, now, reason):
        timestamp = max(float(now), self._last_now or 0.0) if finite(now) and now >= 0 else (self._last_now or 0.0)
        self._integral[:] = 0
        self._velocity[:] = 0
        self._measured_rate[:] = 0
        self._last_pose = None
        self._image_history = None
        self._image_rate[:] = 0
        self._using_image = False
        self._using_main = False
        self._sequence += 1
        return Command(timestamp, 0., 0., timestamp + self.config['lease_s'], False, reason, self._sequence)

    def update(self, prediction: Prediction, pose: Pose, now: float, enabled: bool = False) -> Command:
        if not finite(now) or now < 0:
            return self._stop(now, 'Invalid monotonic time')
        if self._last_now is not None and now <= self._last_now:
            return self._stop(now, 'Out-of-order control time')
        dt = self.config['initial_dt_s'] if self._last_now is None else now - self._last_now
        self._last_now = now
        if enabled is not True:
            return self._stop(now, 'Automatic control disabled or manually overridden')
        if dt > self.config['max_control_gap_s']:
            return self._stop(now, 'Control update gap exceeded configured bound')
        if (not pose.valid or not finite(pose.timestamp, pose.az_deg, pose.el_deg) or
                not -1e-6 <= now - pose.timestamp <= self.config['pose_max_age_s']):
            return self._stop(now, 'Invalid, stale or future encoder pose')
        values = (prediction.timestamp, prediction.az_deg, prediction.el_deg, prediction.az_rate_deg_s,
                  prediction.el_rate_deg_s, prediction.uncertainty_deg, prediction.lead_time_s)
        if (not prediction.valid or not finite(*values) or prediction.uncertainty_deg < 0 or
                not 0 <= prediction.lead_time_s <= self.config['max_lead_time_s'] or
                not -1e-6 <= now - prediction.timestamp <= self.config['prediction_max_age_s']):
            return self._stop(now, 'Invalid, stale or future prediction')
        if ((self._pose_watermark is not None and pose.timestamp < self._pose_watermark) or
                (self._prediction_watermark is not None and prediction.timestamp < self._prediction_watermark)):
            return self._stop(now, 'Out-of-order pose or prediction')
        if (self._last_pose is not None and pose.timestamp == self._last_pose.timestamp and
                (abs(angle_error(pose.az_deg, self._last_pose.az_deg)) > 1e-9 or
                 abs(pose.el_deg - self._last_pose.el_deg) > 1e-9)):
            return self._stop(now, 'Encoder angle changed without a new timestamp')
        main = prediction.phase in ('main', 'main_descent', 'under_main')
        prefix = 'main_' if main else ''
        if prediction.uncertainty_deg > self.config[prefix + 'max_uncertainty_deg']:
            return self._stop(now, 'Prediction uncertainty exceeds configured limit')
        use_image = False
        image_values = (prediction.image_error_az_deg, prediction.image_error_el_deg,
                        prediction.image_timestamp)
        if main and any(value is not None for value in image_values):
            if (not finite(*image_values) or prediction.image_timestamp > now + 1e-6 or
                    prediction.image_timestamp > prediction.timestamp + 1e-6):
                return self._stop(now, 'Invalid or incomplete image feedback metadata')
            use_image = now - prediction.image_timestamp <= self.config['image_max_age_s']
            if use_image and self._image_watermark is not None and prediction.image_timestamp < self._image_watermark:
                return self._stop(now, 'Out-of-order image feedback')
        self._pose_watermark, self._prediction_watermark = pose.timestamp, prediction.timestamp
        if self._last_pose is not None and pose.timestamp > self._last_pose.timestamp:
            pose_dt = pose.timestamp - self._last_pose.timestamp
            measured = np.array([angle_error(pose.az_deg, self._last_pose.az_deg),
                                 pose.el_deg - self._last_pose.el_deg]) / pose_dt
            if not np.all(np.isfinite(measured)):
                return self._stop(now, 'Nonfinite measured encoder velocity')
            weight = pose_dt / (self.config['derivative_tau_s'] + pose_dt)
            self._measured_rate += weight * (measured - self._measured_rate)
        self._last_pose = Pose(pose.timestamp, pose.az_deg, pose.el_deg, pose.valid)
        desired_rate = np.array([prediction.az_rate_deg_s, prediction.el_rate_deg_s])
        # Both sides must refer to the same actuation time. Comparing a led
        # target to current pose produces a steady +rate*latency position bias.
        # Propagate only from measured encoder velocity, never commanded motion.
        prediction_age = max(0., now - prediction.timestamp)
        pose_horizon = max(0., now - pose.timestamp) + prediction.lead_time_s
        future_target = np.array([prediction.az_deg, prediction.el_deg]) + desired_rate * prediction_age
        future_pose = np.array([pose.az_deg, pose.el_deg]) + self._measured_rate * pose_horizon
        error = np.array([angle_error(future_target[0], future_pose[0]), future_target[1] - future_pose[1]])
        derivative = desired_rate - self._measured_rate
        if use_image:
            # This residual is measured in the captured image. Adding encoder
            # propagation would reintroduce the quantization that cancels when
            # the image ray and captured pose are subtracted at the same time.
            error = np.array([angle_error(prediction.image_error_az_deg, 0.),
                              prediction.image_error_el_deg])
            stamp = prediction.image_timestamp
            if self._image_history is not None:
                previous_stamp, previous_error = self._image_history
                delta = np.array([angle_error(error[0], previous_error[0]), error[1] - previous_error[1]])
                if stamp == previous_stamp:
                    if np.any(np.abs(delta) > 1e-9):
                        return self._stop(now, 'Image residual changed without a new timestamp')
                else:
                    image_dt = stamp - previous_stamp
                    image_rate = delta / image_dt
                    if not np.all(np.isfinite(image_rate)):
                        return self._stop(now, 'Nonfinite image angular velocity')
                    weight = image_dt / (self.config['derivative_tau_s'] + image_dt)
                    self._image_rate += weight * (image_rate - self._image_rate)
            self._image_watermark = stamp
            self._image_history = (stamp, error.copy())
            derivative = self._image_rate
        else:
            self._image_history = None
            self._image_rate[:] = 0
        # Integral effort belongs to its feedback/gain mode. In particular,
        # effort accumulated against a travel stop before main must not be
        # multiplied by the main gain while the narrow body is still offscreen.
        # Keep measured rate and output velocity for a continuous ramp.
        if use_image != self._using_image or main != self._using_main:
            self._integral[:] = 0
        self._using_image = use_image
        self._using_main = main
        error[np.abs(error) <= self.config[prefix + 'deadband_deg']] = 0
        kp, ki, kd = (self.config[prefix + name] for name in ('kp', 'ki', 'kd'))
        integral = np.clip(self._integral + error * dt, -self.config['integral_limit'], self.config['integral_limit']) if ki else np.zeros(2)
        with np.errstate(over='ignore', invalid='ignore'):
            base = self.config['feedforward_gain'] * desired_rate + kp * error + kd * derivative
            raw = base + ki * integral
        if not np.all(np.isfinite(raw)):
            return self._stop(now, 'Nonfinite controller calculation')
        speed, acceleration = self.config['max_speed_deg_s'], self.config['max_accel_deg_s2']

        def limit(value):
            return np.clip(np.clip(value, -speed, speed), self._velocity - acceleration * dt, self._velocity + acceleration * dt)

        limited = limit(raw)
        # Freeze only integration that pushes further into speed OR acceleration
        # saturation; an opposing error may unwind the stored integral.
        pushing = ((raw > limited + 1e-12) & (error > 0)) | ((raw < limited - 1e-12) & (error < 0))
        if ki:
            integral[pushing] = self._integral[pushing]
        raw = base + ki * integral
        velocity = limit(raw)
        if not np.all(np.isfinite(velocity)):
            return self._stop(now, 'Nonfinite controller calculation')
        self._integral, self._velocity = integral, velocity
        self._sequence += 1
        reason = 'Tracking main descent' if main else 'Tracking target retention'
        if use_image:
            reason += '; fresh narrow image feedback'
        if np.any(np.abs(raw - velocity) > 1e-9):
            reason += '; controller velocity/acceleration limited'
        return Command(now, float(velocity[0]), float(velocity[1]), now + self.config['lease_s'],
                       True, reason, self._sequence)
