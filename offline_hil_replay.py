#!/usr/bin/env python3
"""Evaluate saved HIL MJPEG slots offline. No services, transports or motor calls.

The recorded processing clock, acquisition poses, individual frame timestamps,
sequences and launch clock remain authoritative. Truth is evaluated only after
TrackingEngine.step returns. This is an image replay with a fixed recorded
camera path, not a new closed-loop plant or physical validation.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import time

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_name, '1')
import cv2
import numpy as np

from det_types import CameraModel, angle_error
from launch_config import load_launch_config
from recording import validate_row
from replay import ReplaySource
from runtime import TrackingEngine
from simulation import SyntheticFlight

cv2.setNumThreads(1)
ROOT = Path(__file__).resolve().parent


def read_jsonl(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate_alignment(telemetry, observations, cameras, launch_origin):
    """Reject sidecar/telemetry drift before decoding or making engine decisions."""
    if not telemetry or len(telemetry) != len(observations):
        raise ValueError('Telemetry/observation row count mismatch')
    previous_now, previous_frames = None, {}
    launched = False
    for index, (original, raw) in enumerate(zip(telemetry, observations)):
        row = validate_row(raw, cameras, previous_now, previous_frames)
        if any(row[key] != original.get(key) for key in ('now', 'pose', 'frames', 'launch_elapsed_s')):
            raise ValueError('Telemetry/observation mapping mismatch at row '+str(index))
        elapsed = row['launch_elapsed_s']
        priming = original.get('priming')
        if type(priming) is not bool or priming != (elapsed is None) or (launched and priming):
            raise ValueError('Unrecognized HIL priming/launch mapping at row '+str(index))
        if not priming:
            launched = True
            if not math.isclose(row['now']-elapsed, launch_origin, abs_tol=1e-8, rel_tol=0):
                raise ValueError('Launch clock mapping differs from saved origin')
        for metadata in row['frames'].values():
            if metadata['timestamp'] > row['now'] or metadata['pose']['timestamp'] > metadata['timestamp']:
                raise ValueError('Future capture/pose timestamp mapping')
        previous_now = row['now']
        previous_frames.update(row['frames'])
    if not launched:
        raise ValueError('No flight rows in saved HIL recording')


def recorded_configuration(run):
    snapshot = json.loads((run/'configuration.json').read_text())
    cfg = deepcopy(snapshot['launch_config'])
    cfg['camera_models'] = {name: CameraModel(**data) for name, data in snapshot['camera_models'].items()}
    for name in ('trajectory', 'field_config'):
        if not Path(cfg[name]).is_file():
            raise ValueError('Recorded local '+name+' is unavailable')
    if json.loads(Path(cfg['field_config']).read_text()) != snapshot['field_config']:
        raise ValueError('Recorded field geometry differs from its saved snapshot')
    return cfg, snapshot


def truth_for_frames(frames, original, renderer, launch_origin):
    """Called after engine.step. Never feeds renderer truth into TrackingEngine."""
    truth = {}
    saved = original.get('evaluation_only')
    for frame in frames:
        elapsed = max(0., frame.timestamp-launch_origin)
        if saved is not None and math.isclose(saved['time_s'], elapsed, abs_tol=1e-8, rel_tol=0):
            item = {'az_deg': saved['az_deg'], 'el_deg': saved['el_deg'], 'phase': saved['phase'],
                    'visible': bool(saved['visible'][frame.camera]), 'source': 'saved_evaluation_only'}
        else:
            az, el, _ = renderer.target(elapsed)
            renderer.t = elapsed
            camera = renderer.cameras[frame.camera]
            item = {'az_deg': az, 'el_deg': el, 'phase': renderer.phase(),
                    'visible': bool(camera.inside(camera.project(az, el, frame.pose))),
                    'source': 'SyntheticFlight.target_at_capture'}
        if not all(isinstance(item[key], (int,float)) and math.isfinite(item[key]) for key in ('az_deg','el_deg')):
            raise ValueError('Nonfinite evaluation truth')
        truth[frame.camera] = {**item, 'capture_elapsed_s': elapsed, 'timestamp': frame.timestamp}
    return truth


def detection_error(detection, truth):
    if detection is None:
        return None
    target = truth.get(detection['camera'])
    if target is None or detection['timestamp'] != target['timestamp']:
        raise ValueError('Detection does not map to a recorded camera capture')
    return math.hypot(angle_error(detection['bearing_deg'], target['az_deg'])*math.cos(math.radians(target['el_deg'])),
                      detection['elevation_deg']-target['el_deg'])


def assess(vision, truth, threshold):
    selected = vision.get('selected')
    error = detection_error(selected, truth)
    correct = bool(selected is not None and truth[selected['camera']]['visible'] and error < threshold)
    candidates = {}
    for camera, detections in vision.get('candidates', {}).items():
        errors = [detection_error(detection, truth) for detection in detections]
        candidates[camera] = {'count': len(detections), 'angular_errors_deg': errors,
                              'best_error_deg': min(errors) if errors else None,
                              'correct_candidate': bool(truth.get(camera, {}).get('visible') and any(e < threshold for e in errors))}
    return {'selected_camera': selected['camera'] if selected else None, 'selected_error_deg': error,
            'selected_correct': correct, 'false_lock': selected is not None and not correct,
            'visible': any(item['visible'] for item in truth.values()),
            'selected_while_camera_invisible': bool(selected and not truth[selected['camera']]['visible']),
            'correct_candidate_available': any(item['correct_candidate'] for item in candidates.values()),
            'candidates': candidates, 'inferred_phase': vision.get('phase', {}).get('name', 'unknown')}


def aggregate(rows):
    if not rows:
        return {'frames': 0}
    errors = [row['selected_error_deg'] for row in rows if row['selected_error_deg'] is not None]
    visible = sum(row['visible'] for row in rows)
    correct = sum(row['selected_correct'] for row in rows)
    selected = sum(row['selected_camera'] is not None for row in rows)
    locks = sum(row['false_lock'] for row in rows)
    return {'frames': len(rows), 'visible_frames': visible, 'selected_frames': selected,
            'correct_selections': correct, 'false_lock_frames': locks,
            'selected_while_camera_invisible': sum(row['selected_while_camera_invisible'] for row in rows),
            'correct_fraction_all_frames': correct/len(rows),
            'correct_fraction_visible_frames': correct/visible if visible else None,
            'false_lock_fraction_selected_frames': locks/selected if selected else None,
            'visible_without_selection': sum(row['visible'] and row['selected_camera'] is None for row in rows),
            'correct_candidate_frames': sum(row['correct_candidate_available'] for row in rows),
            'association_misses_with_correct_candidate': sum(row['correct_candidate_available'] and not row['selected_correct'] for row in rows),
            'selected_error_deg_p50_p95': np.percentile(errors, [50,95]).tolist() if errors else None}


def phase_metrics(rows):
    return {phase: aggregate([row for row in rows if row['truth_phase'] == phase])
            for phase in sorted({row['truth_phase'] for row in rows})}


def false_lock_intervals(rows):
    intervals, current = [], None
    for row in rows:
        if row['false_lock']:
            if current is None:
                current = {'start_elapsed_s': row['elapsed_s'], 'end_elapsed_s': row['elapsed_s'], 'frames': 0}
            current['end_elapsed_s'] = row['elapsed_s']
            current['frames'] += 1
        elif current is not None:
            intervals.append(current)
            current = None
    if current is not None:
        intervals.append(current)
    return intervals


def evaluate(run_dir, output_dir, config_path=None, threshold_deg=.2):
    run, output = Path(run_dir).resolve(), Path(output_dir).resolve()
    if not math.isfinite(threshold_deg) or threshold_deg <= 0:
        raise ValueError('Evaluation threshold must be finite and positive')
    if output.exists():
        raise FileExistsError('Output directory already exists: '+str(output))
    original_cfg, snapshot = recorded_configuration(run)
    cfg = load_launch_config(config_path) if config_path else original_cfg
    summary_path = run/'summary.json'
    launch_origin = json.loads(summary_path.read_text())['launch_origin_monotonic']
    if isinstance(launch_origin, bool) or not isinstance(launch_origin, (int,float)) or not math.isfinite(launch_origin):
        raise ValueError('Missing finite saved launch origin')
    telemetry = read_jsonl(run/'telemetry.jsonl')
    observations = read_jsonl(run/'synthetic-frames'/'observations.jsonl')
    validate_alignment(telemetry, observations, cfg['camera_models'], launch_origin)
    inputs = [run/'telemetry.jsonl', run/'configuration.json', summary_path, run/'synthetic-frames'/'observations.jsonl']
    inputs += [run/'synthetic-frames'/(name+'.avi') for name in cfg['camera_models']]
    if any(not path.is_file() for path in inputs):
        raise ValueError('Replay requires local regular input files')
    hashes = {str(path.relative_to(run)): sha256(path) for path in inputs}
    hashes['trajectory'] = sha256(original_cfg['trajectory'])
    hashes['field_config'] = sha256(original_cfg['field_config'])
    if config_path:
        hashes['override_config'] = sha256(config_path)
    software = {path.name: sha256(path) for path in ROOT.glob('*.py')}
    source = ReplaySource(run/'synthetic-frames', cfg['camera_models'])
    now = [0.]
    engine = TrackingEngine(cfg, clock=lambda: now[0])
    # This renderer is a target lookup only. read(), send() and advance() are never called.
    renderer = SyntheticFlight(original_cfg['camera_models'], original_cfg['trajectory'], original_cfg['field_config'], fps=snapshot['profile']['fps'])
    output.mkdir(parents=True, exist_ok=False)
    report = {'completed': False, 'mode': 'offline_saved_hil_image_replay', 'hardware_access': False,
              'input_directory': str(run), 'input_sha256': hashes, 'software_sha256': software,
              'evaluation_threshold_deg': threshold_deg, 'threshold_is_not_a_user_acceptance_requirement': True,
              'clock_mapping': 'Original processing/capture timestamps and poses. Priming null launch_elapsed maps to engine elapsed=0; executor resets at first flight row, matching full_flight_hil._step/run.',
              'truth_is_evaluation_only_after_engine_step': True,
              'limitations': ['Saved MJPEG images are lossy; replay detection can differ from the original uncompressed synthetic frames.',
                              'Fixed recorded camera poses do not respond to replay commands. This does not predict a new closed-loop physical flight.',
                              'Synthetic imagery, nominal camera calibration and unqualified controller counts are not physical angle ground truth.']}
    flight, original_metrics = [], []
    started = time.perf_counter()
    try:
        with (output/'telemetry.jsonl').open('x') as stream:
            first_flight = True
            for index, original in enumerate(telemetry):
                frames, pose = source.read()
                now[0] = source.now
                elapsed = source.launch_elapsed_s
                if elapsed is not None and first_flight:
                    engine.executor.reset()
                    first_flight = False
                engine_elapsed = 0. if original['priming'] else elapsed
                tick = time.perf_counter()
                vision, prediction, command = engine.step(frames, pose, now[0], engine_elapsed, enabled=False)
                algorithm_ms = (time.perf_counter()-tick)*1000
                # Every evaluation access below occurs after the complete engine decision.
                truth = truth_for_frames(frames, original, renderer, launch_origin)
                evaluation = assess(asdict(vision), truth, threshold_deg)
                baseline = assess(original['vision'], truth, threshold_deg)
                phase = next(iter(truth.values()))['phase'] if truth else 'unknown'
                for item in (evaluation, baseline):
                    item.update(elapsed_s=elapsed, truth_phase=phase)
                row = {'row_index': index, 'now': now[0], 'pose': asdict(pose), 'frames': observations[index]['frames'],
                       'launch_elapsed_s': elapsed, 'engine_elapsed_s': engine_elapsed, 'priming': original['priming'],
                       'vision': asdict(vision), 'prediction': asdict(prediction), 'command': asdict(command),
                       'algorithm_ms': algorithm_ms,
                       'evaluation': {**evaluation, 'truth_by_camera': truth}, 'original_evaluation': baseline}
                stream.write(json.dumps(row, allow_nan=False)+'\n')
                if elapsed is not None:
                    flight.append(evaluation)
                    original_metrics.append(baseline)
            try:
                source.read()
            except StopIteration:
                pass
            else:
                raise ValueError('Replay yielded extra rows')
        report.update(completed=True, rows=len(telemetry), flight=aggregate(flight), phases=phase_metrics(flight),
                      false_lock_intervals=false_lock_intervals(flight),
                      original_telemetry={'flight': aggregate(original_metrics), 'phases': phase_metrics(original_metrics),
                                          'false_lock_intervals': false_lock_intervals(original_metrics)})
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        source.close()
        report['processing_wall_s'] = time.perf_counter()-started
        (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        effective = {k: v for k,v in cfg.items() if k != 'camera_models'}
        (output/'effective_configuration.json').write_text(json.dumps(effective, indent=2, allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='Saved HIL directory, containing telemetry and synthetic-frames')
    parser.add_argument('--output', required=True, help='New output directory; never overwritten')
    parser.add_argument('--config', help='Optional launch configuration for an offline experiment')
    parser.add_argument('--threshold-deg', type=float, default=.2, help='Evaluation tolerance only, default 0.2 degree')
    args = parser.parse_args(argv)
    result = evaluate(args.input, args.output, args.config, args.threshold_deg)
    print(json.dumps({'output': str(Path(args.output).resolve()), 'completed': result['completed'],
                      'flight': result['flight'], 'phases': result['phases']}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
