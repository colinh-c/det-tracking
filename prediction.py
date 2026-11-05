"""Observed angular filtering with an explicitly labeled saved-flight prior.

All runtime times are Mac monotonic seconds. AngularPrior.sample alone accepts
launch elapsed seconds. Prediction.timestamp is generation time; its angles
are led to now + latency_s. No synthetic truth or motor commands are consumed.
"""
import csv
import math
from numbers import Real
from pathlib import Path
from typing import Optional
import numpy as np

from det_types import Prediction, Pose, VisionResult, angle_error
from field_config import load_config


def finite(*values):
    return all(isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value) for value in values)


def is_main(phase):
    return phase in ('main', 'main_descent', 'under_main')


class AngularPrior:
    """Interpolate saved Cartesian flight positions; site edits do not rerun physics."""
    def __init__(self, trajectory_path, field_config_path):
        self.field_config = load_config(field_config_path)
        required = ('Time', 'Position East of launch', 'Position North of launch', 'Altitude')
        points = []
        with Path(trajectory_path).open(newline='', encoding='utf-8-sig') as source:
            reader = csv.DictReader(source)
            if not set(required).issubset(reader.fieldnames or []):
                raise ValueError('Trajectory requires Time, East/North of launch and Altitude columns')
            for line, row in enumerate(reader, 2):
                try:
                    point = [float(row[name]) for name in required]
                except (TypeError, ValueError) as error:
                    raise ValueError(f'Invalid required trajectory value at line {line}') from error
                if not finite(*point) or point[0] < 0:
                    raise ValueError(f'Nonfinite or negative-time trajectory value at line {line}')
                if points and point[0] <= points[-1][0]:
                    raise ValueError('Trajectory times must increase strictly')
                points.append(point)
        if len(points) < 2:
            raise ValueError('Trajectory needs at least two samples')
        values = np.asarray(points, dtype=float)
        self.times = values[:, 0]
        self.positions = values[:, 1:] + np.asarray(self.field_config.pad_position_enu())
        self.start_time_s, self.end_time_s = float(self.times[0]), float(self.times[-1])

    def sample(self, elapsed_s: float) -> Prediction:
        timestamp = float(elapsed_s) if finite(elapsed_s) else 0.0
        if not finite(elapsed_s) or not self.start_time_s <= elapsed_s <= self.end_time_s:
            return Prediction(timestamp, reason='Outside saved simulation time range', source='openrocket_prior')
        index = min(max(int(np.searchsorted(self.times, elapsed_s, side='right')) - 1, 0), len(self.times) - 2)
        dt = self.times[index + 1] - self.times[index]
        velocity = (self.positions[index + 1] - self.positions[index]) / dt
        east, north, up = self.positions[index] + velocity * (elapsed_s - self.times[index])
        ve, vn, vu = velocity
        horizontal = math.hypot(east, north)
        radius2 = horizontal * horizontal + up * up
        if horizontal < 1e-9 or not finite(radius2) or radius2 <= 0:
            return Prediction(timestamp, reason='Prior bearing is singular at tracker zenith/origin', source='openrocket_prior')
        horizontal_rate = (east * ve + north * vn) / horizontal
        return Prediction(timestamp,
                          math.degrees(math.atan2(east, north)) % 360,
                          math.degrees(math.atan2(up, horizontal)),
                          math.degrees((north * ve - east * vn) / horizontal**2),
                          math.degrees((horizontal * vu - up * horizontal_rate) / radius2),
                          True, 'openrocket_prior', 5.0,
                          'Saved simulation; 5 degree uncertainty is a heuristic, not measured accuracy')


DEFAULTS = {
    'measurement_max_age_s': .35, 'pose_max_age_s': .3, 'max_latency_s': .5,
    'max_coast_s': .6, 'main_max_coast_s': .35,
    'process_accel_deg_s2': 20., 'main_process_accel_deg_s2': 5.,
    'initial_rate_sigma_deg_s': 10., 'min_measurement_sigma_deg': .01,
    'innovation_gate_sigma': 8., 'confidence_min': .15,
    'max_uncertainty_deg': 12., 'main_max_uncertainty_deg': 3.,
    'prior_acquisition_s': .6, 'prior_uncertainty_deg': 5.,
    'prior_search_s': 8., 'main_prior_search_s': 2.,
    'prior_search_uncertainty_deg': 1., 'prior_search_growth_deg_s': .5,
}


class Predictor:
    """Constant-velocity angular Kalman filter with prior curvature corrections.

The state remains in measured bearing/elevation coordinates. The prior contributes
changes of rate/curvature, never replaces observed angular innovations. Defaults
are initial tuning values; covariance/uncertainty is not a calibrated error bound.
"""
    def __init__(self, prior: Optional[AngularPrior] = None, config: dict = None):
        self.config = dict(DEFAULTS)
        if config:
            unknown = set(config) - set(DEFAULTS)
            if unknown:
                raise ValueError(f'Unknown predictor configuration: {sorted(unknown)}')
            self.config.update(config)
        for name, value in self.config.items():
            if not finite(value) or value < 0 or (name not in ('prior_acquisition_s', 'prior_search_s', 'main_prior_search_s', 'confidence_min') and value == 0):
                raise ValueError(f'{name} must be finite and positive (acquisition/confidence may be zero)')
            if not math.isfinite(value * value):
                raise ValueError(f'{name} is too large for finite covariance calculations')
        if self.config['confidence_min'] > 1:
            raise ValueError('confidence_min cannot exceed one')
        self.prior = prior
        self.reset()

    def reset(self):
        self._state = self._covariance = None
        self._time = self._epoch = self._last_now = self._last_vision_time = self._last_pose_time = None
        self._phase = 'unknown'
        self._acquire_start = None
        self._prior_offset = None
        self._prior_offset_sigma = None

    def _invalid(self, now, reason):
        stamp = float(now) if finite(now) else (self._last_now or 0.0)
        return Prediction(stamp, reason=reason, phase=self._phase)

    def _prior_at(self, timestamp, epoch):
        if self.prior is None or epoch is None:
            return None
        result = self.prior.sample(timestamp - epoch)
        return result if result.valid else None

    def _project(self, timestamp, epoch):
        dt = timestamp - self._time
        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = dt
        state = transition @ self._state
        acceleration = self.config['main_process_accel_deg_s2' if is_main(self._phase) else 'process_accel_deg_s2']
        influence = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
        covariance = transition @ self._covariance @ transition.T + acceleration**2 * influence @ influence.T
        used_prior = False
        if epoch is not None and self._epoch is not None and abs(epoch - self._epoch) <= 1e-6:
            before, after = self._prior_at(self._time, epoch), self._prior_at(timestamp, epoch)
            if before is not None and after is not None:
                state += np.array([angle_error(after.az_deg, before.az_deg) - before.az_rate_deg_s * dt,
                                   after.el_deg - before.el_deg - before.el_rate_deg_s * dt,
                                   after.az_rate_deg_s - before.az_rate_deg_s,
                                   after.el_rate_deg_s - before.el_rate_deg_s])
                used_prior = True
        return state, covariance, used_prior

    def _search_prior(self, now, epoch, latency_s, age, uncertainty_limit):
        """Bounded trajectory search, anchored only to an accepted observation.

        This is an explicitly heuristic search estimate, not a vision detection
        or continuation of calibrated Kalman covariance. Changing the launch
        clock invalidates its saved offset instead of silently moving the search.
        """
        limit = self.config['main_prior_search_s' if is_main(self._phase) else 'prior_search_s']
        if (limit <= 0 or age > limit or self._prior_offset is None or epoch is None or
                self._epoch is None or abs(epoch - self._epoch) > 1e-6):
            return None
        prior = self._prior_at(now + latency_s, epoch)
        if prior is None:
            return None
        uncertainty = max(self.config['prior_search_uncertainty_deg'], self._prior_offset_sigma)
        uncertainty += self.config['prior_search_growth_deg_s'] * (age + latency_s)
        if not finite(uncertainty) or uncertainty > uncertainty_limit:
            return None
        return Prediction(now, (prior.az_deg + self._prior_offset[0]) % 360,
                          prior.el_deg + self._prior_offset[1], prior.az_rate_deg_s,
                          prior.el_rate_deg_s, True, 'openrocket_reacquisition', uncertainty,
                          f'Bounded saved-trajectory search; no current visual target; observation age {age:.3f}s; '
                          f'limit {limit:.3f}s; uncertainty is heuristic', self._phase, latency_s)

    def update(self, vision: VisionResult, pose: Pose, now: float,
               launch_elapsed_s: Optional[float] = None, latency_s: float = .1) -> Prediction:
        if not finite(now) or now < 0:
            return self._invalid(now, 'Invalid monotonic time')
        if self._last_now is not None and now <= self._last_now:
            return self._invalid(now, 'Out-of-order update time')
        self._last_now = now
        if not finite(latency_s) or not 0 <= latency_s <= self.config['max_latency_s']:
            return self._invalid(now, 'Invalid or excessive latency lead')
        if (not pose.valid or not finite(pose.timestamp, pose.az_deg, pose.el_deg) or
                not -1e-6 <= now - pose.timestamp <= self.config['pose_max_age_s']):
            return self._invalid(now, 'Invalid, stale or future encoder pose')
        if self._last_pose_time is not None and pose.timestamp < self._last_pose_time:
            return self._invalid(now, 'Out-of-order encoder pose')
        self._last_pose_time = pose.timestamp
        if (not finite(vision.timestamp) or not -1e-6 <= now - vision.timestamp <= self.config['measurement_max_age_s'] or
                (self._last_vision_time is not None and vision.timestamp < self._last_vision_time)):
            return self._invalid(now, 'Invalid, stale or out-of-order vision timestamp')
        self._last_vision_time = vision.timestamp
        if launch_elapsed_s is not None and not finite(launch_elapsed_s):
            return self._invalid(now, 'Invalid launch elapsed time')
        epoch = None if launch_elapsed_s is None else now - launch_elapsed_s
        if (not isinstance(vision.phase.name, str) or not finite(vision.phase.confidence) or
                not 0 <= vision.phase.confidence <= 1):
            return self._invalid(now, 'Invalid phase estimate')
        if isinstance(vision.phase.name, str) and vision.phase.name != 'unknown':
            self._phase = vision.phase.name
        coast_limit = self.config['main_max_coast_s' if is_main(self._phase) else 'max_coast_s']
        uncertainty_limit = self.config['main_max_uncertainty_deg' if is_main(self._phase) else 'max_uncertainty_deg']
        if self._acquire_start is None:
            self._acquire_start = now
        measurement = vision.selected
        fresh = False
        if measurement is not None:
            if (not finite(measurement.timestamp, measurement.bearing_deg, measurement.elevation_deg,
                           measurement.x, measurement.y, measurement.confidence, measurement.angular_sigma_deg) or
                    not self.config['confidence_min'] <= measurement.confidence <= 1 or measurement.angular_sigma_deg < 0):
                return self._invalid(now, 'Invalid or low-confidence angular detection')
            if not -1e-6 <= now - measurement.timestamp <= self.config['measurement_max_age_s']:
                return self._invalid(now, 'Stale or future angular detection')
            if measurement.timestamp > vision.timestamp + 1e-6:
                return self._invalid(now, 'Detection time exceeds its vision result time')
            if self._time is not None and measurement.timestamp < self._time:
                return self._invalid(now, 'Out-of-order angular detection')
            fresh = self._time is None or measurement.timestamp > self._time
            if fresh:
                sigma = max(self.config['min_measurement_sigma_deg'], measurement.angular_sigma_deg) / math.sqrt(max(measurement.confidence, .01))
                if not finite(sigma * sigma):
                    return self._invalid(now, 'Detection uncertainty exceeds numeric range')
                observed = np.array([measurement.bearing_deg % 360, measurement.elevation_deg])
                if self._state is None or measurement.timestamp - self._time > coast_limit:
                    prior = self._prior_at(measurement.timestamp, epoch)
                    rates = [prior.az_rate_deg_s, prior.el_rate_deg_s] if prior is not None else [0., 0.]
                    self._state = np.array([*observed, *rates])
                    self._covariance = np.diag([sigma*sigma, sigma*sigma,
                                               self.config['initial_rate_sigma_deg_s']**2,
                                               self.config['initial_rate_sigma_deg_s']**2])
                else:
                    state, covariance, _ = self._project(measurement.timestamp, epoch)
                    innovation = np.array([angle_error(observed[0], state[0]), observed[1] - state[1]])
                    noise = np.eye(2) * sigma**2
                    innovation_covariance = covariance[:2, :2] + noise
                    distance2 = float(innovation @ np.linalg.solve(innovation_covariance, innovation))
                    if distance2 > self.config['innovation_gate_sigma']**2:
                        return self._invalid(now, 'Angular innovation rejected; last observation not refreshed')
                    gain = np.linalg.solve(innovation_covariance, covariance[:2, :]).T
                    self._state = state + gain @ innovation
                    residual = np.eye(4)
                    residual[:, :2] -= gain
                    self._covariance = residual @ covariance @ residual.T + gain @ noise @ gain.T
                self._time, self._epoch = measurement.timestamp, epoch
                measured_prior = self._prior_at(measurement.timestamp, epoch)
                self._prior_offset = (angle_error(observed[0], measured_prior.az_deg),
                                      observed[1] - measured_prior.el_deg) if measured_prior is not None else None
                self._prior_offset_sigma = sigma if measured_prior is not None else None
        if self._state is None:
            prior = self._prior_at(now + latency_s, epoch)
            allowed = self.config['prior_acquisition_s']
            if prior is not None and allowed > 0 and now - self._acquire_start <= allowed:
                uncertainty = self.config['prior_uncertainty_deg']
                if uncertainty <= uncertainty_limit:
                    return Prediction(now, prior.az_deg, prior.el_deg, prior.az_rate_deg_s,
                                      prior.el_rate_deg_s, True, 'prior_acquisition', uncertainty,
                                      'Bounded saved-simulation acquisition; no observed target yet', self._phase, latency_s)
            return self._invalid(now, 'No fresh target observation; prior acquisition unavailable or expired')
        age = now - self._time
        if age > coast_limit:
            search = self._search_prior(now, epoch, latency_s, age, uncertainty_limit)
            if search is not None:
                return search
            return self._invalid(now, 'Target loss exceeded bounded coast and available prior search intervals')
        state, covariance, used_prior = self._project(now + latency_s, epoch)
        uncertainty = math.sqrt(max(float(covariance[0, 0]), float(covariance[1, 1]), 0))
        if not np.all(np.isfinite(state)) or not finite(uncertainty) or uncertainty > uncertainty_limit:
            return self._invalid(now, 'Angular prediction uncertainty exceeded configured limit')
        source = 'vision' if fresh else 'coast'
        if used_prior:
            source += '+openrocket_prior'
        return Prediction(now, float(state[0] % 360), float(state[1]), float(state[2]), float(state[3]),
                          True, source, uncertainty,
                          f'Lead {latency_s:.3f}s; observation age {age:.3f}s; uncertainty is model-based', self._phase, latency_s)
