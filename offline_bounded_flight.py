#!/usr/bin/env python3
"""Offline closed-loop evaluator using SyntheticFlight and TrackingEngine.

This is an ideal software plant, not measured hardware behavior or flight
validation. Nothing in this module opens a transport, socket, serial device, or
service. Targets keep the original trajectory and clock; only the simulated
mount is stopped at the user's saved controller-coordinate travel bounds.
"""
import argparse
from bisect import bisect_right
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from det_types import CameraModel, Pose, angle_error
from runtime import TrackingEngine
from simulation import SyntheticFlight


class BoundedPlant:
    """Ideal count-coordinate velocity plant with delayed leased commands.

    Integrates acceleration at at most 5 ms steps and lands exactly on transport
    and lease boundaries. A hard travel stop zeros outward velocity. That abrupt
    stop is separate from the free-motion acceleration ceiling of 75 deg/s^2.
    """
    def __init__(self, origin, bounds, start_world, start_time=1000.,
                 transport_delay_s=.07, max_speed_deg_s=15., max_accel_deg_s2=75.):
        self.origin = {axis: float(origin[axis]) for axis in ('az', 'el')}
        self.bounds = {axis: tuple(map(float, bounds[axis])) for axis in ('az', 'el')}
        self.world_start = tuple(map(float, start_world))
        self.position = dict(self.origin)
        self.velocity = {'az': 0., 'el': 0.}
        self.now = float(start_time)
        self.delay = float(transport_delay_s)
        self.speed = float(max_speed_deg_s)
        self.acceleration = float(max_accel_deg_s2)
        if not (0 <= self.delay <= 1 and 0 < self.speed <= 15 and 0 < self.acceleration <= 75):
            raise ValueError('Invalid ideal plant timing or motion ceilings')
        for axis in self.bounds:
            low, high = self.bounds[axis]
            if not (math.isfinite(low) and math.isfinite(high) and low < high and low <= self.position[axis] <= high):
                raise ValueError('Saved origin must be inside finite travel limits')
        self.pending = []
        self.active = None
        self.times = [self.now]
        self.history = [(self.position['az'], self.position['el'])]
        self.peak_speed = self.peak_free_acceleration = 0.
        self.boundary_stop_count = 0

    def send(self, command):
        """Queue an in-memory command; this does not communicate externally."""
        if command.timestamp < self.now - 1e-8:
            raise ValueError('Cannot queue a command from before the plant time')
        self.pending.append((float(command.timestamp) + self.delay, replace(command)))
        self.pending.sort(key=lambda item: item[0])

    def current_pose(self):
        return self.pose_at(self.now)

    def pose_at(self, timestamp):
        if timestamp < self.times[0] - 1e-8 or timestamp > self.now + 1e-8:
            raise ValueError('Capture pose is outside recorded plant history')
        index = max(0, min(len(self.times)-1, bisect_right(self.times, timestamp)-1))
        position = self.history[index]
        if index + 1 < len(self.times):
            weight = (timestamp-self.times[index]) / (self.times[index+1]-self.times[index])
            position = tuple(a + weight*(b-a) for a, b in zip(position, self.history[index+1]))
        return Pose(float(timestamp), (self.world_start[0] + position[0]-self.origin['az']) % 360.,
                    self.world_start[1] + position[1]-self.origin['el'])

    def advance_to(self, timestamp):
        timestamp = float(timestamp)
        if timestamp < self.now - 1e-8:
            raise ValueError('Plant time cannot move backwards')
        while self.now < timestamp - 1e-10:
            while self.pending and self.pending[0][0] <= self.now + 1e-10:
                _, self.active = self.pending.pop(0)
            end = min(timestamp, self.now + .005)
            if self.pending:
                end = min(end, self.pending[0][0])
            if self.active is not None and self.active.valid_until > self.now + 1e-10:
                end = min(end, self.active.valid_until)
            dt = end - self.now
            if dt <= 1e-12:
                self.now = end
                continue
            enabled = self.active is not None and self.active.enabled and self.active.valid_until > self.now + 1e-10
            for axis, field in (('az', 'az_velocity_deg_s'), ('el', 'el_velocity_deg_s')):
                wanted = float(np.clip(getattr(self.active, field), -self.speed, self.speed)) if enabled else 0.
                old_velocity = self.velocity[axis]
                delta = float(np.clip(wanted-old_velocity, -self.acceleration*dt, self.acceleration*dt))
                new_velocity = old_velocity + delta
                next_position = self.position[axis] + .5*(old_velocity+new_velocity)*dt
                low, high = self.bounds[axis]
                hit = (next_position >= high and new_velocity > 0) or (next_position <= low and new_velocity < 0)
                if hit:
                    next_position = min(high, max(low, next_position))
                    new_velocity = 0.
                    self.boundary_stop_count += 1
                else:
                    self.peak_free_acceleration = max(self.peak_free_acceleration, abs(delta)/dt)
                self.position[axis] = min(high, max(low, next_position))
                self.velocity[axis] = new_velocity
                self.peak_speed = max(self.peak_speed, abs(new_velocity))
            self.now = end
            self.times.append(self.now)
            self.history.append((self.position['az'], self.position['el']))


def main_geometry_metrics(rows, fps):
    """Time-on-target measures use rendered main truth after engine decisions.

    COM visibility means the center is in the image. Boresight-in-body means the
    optical axis intersects the rendered body; these are separate conditions.
    All durations are frame counts divided by the unchanged simulation cadence.
    """
    main = [row for row in rows if row.get('evaluation_only', {}).get('phase') == 'main']
    count = len(main)
    narrow = sum(row['evaluation_only']['visible']['narrow'] for row in main)
    either = sum(any(row['evaluation_only']['visible'].values()) for row in main)
    body = sum(row['evaluation_only']['boresight_inside_body']['narrow'] for row in main)
    errors = [row['pointing_error_deg'] for row in main]
    return {'main_geometry_phase_basis': 'rendered_target_phase_at_capture_time',
            'main_rendered_rows': count, 'main_rendered_duration_s': count/fps,
            'main_narrow_com_in_frame_rows': narrow,
            'main_narrow_com_in_frame_fraction': narrow/count if count else None,
            'main_narrow_com_in_frame_s': narrow/fps,
            'main_either_com_in_frame_rows': either,
            'main_either_com_in_frame_fraction': either/count if count else None,
            'main_either_com_in_frame_s': either/fps,
            'main_narrow_boresight_inside_body_rows': body,
            'main_narrow_boresight_inside_body_s': body/fps,
            'main_narrow_boresight_inside_body_fraction': body/count if count else None,
            'main_pointing_time_within_deg_s': {str(threshold): sum(error <= threshold for error in errors)/fps for threshold in (.01, .025, .05, .1)},
            'main_pointing_error_percentiles_deg': {name: float(np.percentile(errors, percentile)) if errors else None for name, percentile in [('p50',50), ('p90',90), ('p95',95), ('p99',99), ('max',100)]}}


class OfflineFlight:
    def __init__(self, saved_run, seed=7):
        self.saved_run = Path(saved_run).resolve()
        self.configuration = json.loads((self.saved_run/'configuration.json').read_text())
        self.saved_summary = json.loads((self.saved_run/'summary.json').read_text())
        self.cfg = deepcopy(self.configuration['launch_config'])
        # Portable snapshots locate their inputs beside the snapshot directory;
        # absolute paths in existing local run snapshots remain supported.
        for key in ('trajectory', 'field_config'):
            self.cfg[key] = str((self.saved_run/Path(self.cfg[key])).resolve())
        self.cfg['camera_models'] = {name: CameraModel(**data) for name, data in self.configuration['camera_models'].items()}
        self.profile = self.configuration['profile']
        self.fps = float(self.profile['fps'])
        self.image_delay = float(self.profile['synthetic_frame_delay_s'])
        self.seed = int(seed)
        self.renderer = SyntheticFlight(self.cfg['camera_models'], self.cfg['trajectory'], self.cfg['field_config'],
                                        fps=self.fps, seed=self.seed)
        self.now = 1000.
        self.engine = TrackingEngine(self.cfg, clock=lambda: self.now)
        prior_start = self.engine.prior.sample(0.)
        claim = self.saved_summary['claim']
        bounds = {axis: claim['limits'][axis] for axis in ('az', 'el')}
        self.plant = BoundedPlant(claim['position'], bounds, (prior_start.az_deg, prior_start.el_deg), self.now)
        self.launch_origin = self.now + float(self.profile['prime_s'])
        self._consumed = False

    def observations(self, stop_after_s=None):
        if self._consumed:
            raise ValueError('Create a new OfflineFlight instance for each run')
        self._consumed = True
        dt = 1./self.fps
        sequence = 0
        duration = self.renderer.end_s if stop_after_s is None else min(float(stop_after_s), self.renderer.end_s)
        tick = 0
        launched = False
        while True:
            self.now = 1000. + tick*dt
            priming = self.now < self.launch_origin-1e-9
            elapsed = 0. if priming else max(0., self.now-self.launch_origin)
            if not priming and elapsed >= duration-1e-9:
                break
            self.plant.advance_to(self.now)
            capture = self.now-self.image_delay
            if capture < self.plant.times[0]-1e-9:
                tick += 1
                continue
            if not priming and not launched:
                self.engine.executor.reset()
                launched = True
            pose = self.plant.current_pose()
            captured_pose = self.plant.pose_at(capture)
            self.renderer.t = 0. if priming else max(0., capture-self.launch_origin)
            self.renderer.epoch = capture-self.renderer.t
            self.renderer.pose = captured_pose
            self.renderer.sequence = sequence
            frames, _ = self.renderer.read()
            vision, prediction, command = self.engine.step(frames, pose, self.now, elapsed, not priming)
            # The HIL runner permits stationary waiting on fresh input after
            # bounded target loss. Preserve this policy without adding a target.
            if not priming and not command.enabled and prediction.reason.startswith('Target loss'):
                command = replace(command, enabled=True, reason='Stationary; waiting for reacquisition')
            if not priming:
                self.plant.send(command)
            row = {'now': self.now, 'launch_elapsed_s': None if priming else elapsed, 'priming': priming,
                   'pose': asdict(pose), 'counter_position': dict(self.plant.position), 'actual_velocity': dict(self.plant.velocity),
                   'frames': {frame.camera: {'timestamp': frame.timestamp, 'sequence': frame.sequence, 'pose': asdict(frame.pose)} for frame in frames},
                   'vision': asdict(vision), 'prediction': asdict(prediction), 'command': asdict(command)}
            if not priming:
                # Ground truth is attached only after perception/prediction/
                # command decisions. It is never an engine input or track seed.
                truth = deepcopy(self.renderer.truth)
                target = {'az': self.plant.origin['az'] + angle_error(truth['az_deg'], self.plant.world_start[0]),
                          'el': self.plant.origin['el'] + truth['el_deg']-self.plant.world_start[1]}
                row['evaluation_only'] = truth
                row['target_controller_deg'] = target
                row['target_outside_travel'] = {axis: not self.plant.bounds[axis][0] <= target[axis] <= self.plant.bounds[axis][1] for axis in ('az', 'el')}
                detection = vision.selected
                error = None if detection is None else math.hypot(angle_error(detection.bearing_deg, truth['az_deg'])*math.cos(math.radians(truth['el_deg'])), detection.elevation_deg-truth['el_deg'])
                row['selected_error_deg'] = error
                row['correct_selection'] = error is not None and error < .2
                row['pointing_error_deg'] = math.hypot(angle_error(captured_pose.az_deg, truth['az_deg'])*math.cos(math.radians(truth['el_deg'])), captured_pose.el_deg-truth['el_deg'])
            yield row
            sequence += 1
            tick += 1
        self.plant.advance_to(self.launch_origin+duration)

    def run(self, output_dir):
        output_dir = Path(output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
        summary = {'scope': 'offline_ideal_bounded_plant_not_hardware_validation', 'completed': False,
                   'trajectory_duration_s': self.renderer.end_s, 'flight_rows': 0, 'priming_rows': 0,
                   'correct_selection_rows': 0, 'incorrect_selection_rows': 0, 'no_selection_rows': 0,
                   'main_rows': 0, 'main_correct_selection_rows': 0, 'main_incorrect_selection_rows': 0, 'inferred_main_rows': 0, 'early_visual_main_rows': 0,
                   'target_outside_travel_rows': 0, 'travel_violation_rows': 0, 'stationary_wait_rows': 0,
                   'first_reacquisition_after_initial_loss_s': None, 'last_correct_selection_s': None,
                   'fixed_narrow_distractor_rows': 0, 'seed': self.seed,
                   'assumptions': ['Ideal controller counts, not physical angle measurements',
                                   '15 deg/s velocity, 75 deg/s^2 free-motion acceleration',
                                   'Hard travel stop zeros outward velocity; inward motion is allowed',
                                   '80 ms image capture delay with interpolated past pose',
                                   '70 ms command transport delay, command lease enforced',
                                   'No trajectory clipping, rescaling, time slowing, or truth-seeded perception'],
                   'origin_controller_deg': dict(self.plant.origin), 'travel_bounds_controller_deg': self.plant.bounds,
                   'source_saved_run': str(self.saved_run), 'source_modules_sha256': {}}
        for name in ('offline_bounded_flight.py','runtime.py','perception.py','prediction.py','execution.py','simulation.py'):
            path = Path(__file__).resolve().parent/name
            summary['source_modules_sha256'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
        lost = False
        main_errors = []
        geometry_rows = []
        with (output_dir/'telemetry.jsonl').open('x') as handle:
            for row in self.observations():
                handle.write(json.dumps(row, allow_nan=False)+'\n')
                if row['priming']:
                    summary['priming_rows'] += 1
                    continue
                summary['flight_rows'] += 1
                if row['evaluation_only']['phase']=='main':
                    geometry_rows.append(row)
                selected = row['vision']['selected']
                if row['correct_selection']:
                    summary['correct_selection_rows'] += 1
                    summary['last_correct_selection_s'] = row['launch_elapsed_s']
                    if lost and summary['first_reacquisition_after_initial_loss_s'] is None:
                        summary['first_reacquisition_after_initial_loss_s'] = row['launch_elapsed_s']
                elif selected is None:
                    summary['no_selection_rows'] += 1
                    lost = True
                else:
                    summary['incorrect_selection_rows'] += 1
                if selected is not None and selected['camera']=='narrow' and selected['x']==524 and selected['y']==134:
                    summary['fixed_narrow_distractor_rows'] += 1
                summary['inferred_main_rows'] += row['vision']['phase']['name']=='main_descent'
                summary['early_visual_main_rows'] += (row['vision']['phase']['name']=='main_descent' and
                    row['vision']['phase']['source']=='visual_heuristic' and row['evaluation_only']['phase']!='main')
                if row['evaluation_only']['phase']=='main':
                    summary['main_rows'] += 1
                    if row['correct_selection']:
                        summary['main_correct_selection_rows'] += 1
                        main_errors.append(row['selected_error_deg'])
                    elif selected is not None:
                        summary['main_incorrect_selection_rows'] += 1
                summary['target_outside_travel_rows'] += any(row['target_outside_travel'].values())
                summary['travel_violation_rows'] += any(not self.plant.bounds[axis][0]-1e-9 <= row['counter_position'][axis] <= self.plant.bounds[axis][1]+1e-9 for axis in ('az','el'))
                summary['stationary_wait_rows'] += row['command']['reason']=='Stationary; waiting for reacquisition'
        summary.update(completed=True, simulated_elapsed_s=self.plant.now-self.launch_origin,
                       peak_actual_speed_deg_s=self.plant.peak_speed,
                       peak_free_actual_accel_deg_s2=self.plant.peak_free_acceleration,
                       boundary_stop_count=self.plant.boundary_stop_count,
                       final_controller_deg=dict(self.plant.position),
                       main_correct_error_median_deg=float(np.median(main_errors)) if main_errors else None)
        summary.update(main_geometry_metrics(geometry_rows, self.fps))
        (output_dir/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--saved-run', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='New output directory; existing data is never overwritten')
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()
    # CLI tripwire: an accidental future external interface fails immediately.
    import socket
    def deny_network(*args, **kwargs):
        raise RuntimeError('Offline bounded evaluation prohibits network access')
    socket.socket = deny_network
    socket.create_connection = deny_network
    print(json.dumps(OfflineFlight(args.saved_run, args.seed).run(args.output), indent=2))


if __name__ == '__main__':
    main()
