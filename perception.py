"""Image-only blob association with explicit phase and COM uncertainty.

This is a heuristic detector, not a trained rocket/phase classifier. A blank or
unassociated frame is a missed observation, not evidence of an off-screen target.
The observer consumes only Frame imagery, acquisition metadata, pose and the
explicit angular prior. It never reads synthetic target labels or coordinates.
"""
import math
import time
from typing import Optional

import cv2
import numpy as np

from det_types import CameraModel, Detection, Frame, Phase, Prediction, VisionResult, angle_error


class Perception:
    def __init__(self, cameras: dict, config: dict = None):
        if not cameras:
            raise ValueError('At least one camera model is required')
        self.cameras = dict(cameras)
        self.config = dict(config or {})
        self.clock = self.config.get('clock', time.monotonic)
        self.max_age = float(self.config.get('max_frame_age_s', .5))
        self.max_pose_age = float(self.config.get('max_pose_age_s', .25))
        self.coast_s = float(self.config.get('association_memory_s', .8))
        self.gate_deg = float(self.config.get('association_gate_deg', .8))
        self.min_contrast = float(self.config.get('min_contrast', 16))
        self.min_area = int(self.config.get('min_area_px', 2))
        if self.min_area < 1:
            raise ValueError('Minimum blob area must be positive')
        self.cg_fraction = float(self.config.get('cg_fraction_from_nose',
                                  self.config.get('body_cg_fraction', 1.536 / 2.4516)))
        if not 0 <= self.cg_fraction <= 1:
            raise ValueError('CG fraction must be between nose and tail')
        if not all(math.isfinite(value) and value > 0 for value in
                   (self.max_age, self.max_pose_age, self.coast_s, self.gate_deg, self.min_contrast)):
            raise ValueError('Age, association, and contrast settings must be positive and finite')
        self.narrow = self.config.get('narrow_camera', 'narrow' if 'narrow' in cameras else max(cameras, key=lambda name: cameras[name].fx))
        if self.narrow not in cameras:
            raise ValueError('Unknown narrow camera')
        self.events = {'burnout': 4.264, 'apogee': 25.514, 'main': 110.877, 'ground': 139.738}
        self.events.update(self.config.get('phase_events', {}))
        values = [float(self.events[key]) for key in ('burnout', 'apogee', 'main', 'ground')]
        if not all(math.isfinite(value) and value >= 0 for value in values) or values != sorted(values):
            raise ValueError('Simulation phase events must be finite and ordered')
        self.events = dict(zip(('burnout', 'apogee', 'main', 'ground'), values))
        self.seen = {}
        self.track = None
        self.appearance = {}
        self.visual_main_since = None
        self.visual_main_last = None
        self.visual_votes = 0

    def _phase(self, now, override, elapsed):
        if override is not None:
            name = {'main': 'main_descent', 'drogue': 'drogue_descent'}.get(override, override)
            if name not in {'unknown', 'pre_main', 'boost', 'coast', 'drogue_descent', 'main_descent', 'landed'}:
                raise ValueError('Unsupported phase override; payload handoff is disabled')
            return Phase(name, 1.0, 'manual')
        if (self.visual_main_last is not None and now - self.visual_main_last < 1.0
                and self.visual_votes >= 3 and self.visual_main_last - self.visual_main_since >= .3):
            return Phase('main_descent', .55, 'visual_heuristic')
        if elapsed is not None:
            if isinstance(elapsed, bool) or not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0:
                raise ValueError('Launch elapsed time must be finite and nonnegative')
            name = ('boost' if elapsed < self.events['burnout'] else
                    'coast' if elapsed < self.events['apogee'] else
                    'drogue_descent' if elapsed < self.events['main'] else
                    'main_descent' if elapsed < self.events['ground'] else 'landed')
            return Phase(name, .30, 'simulation_prior')
        return Phase()

    def _gray(self, frame, model):
        image = np.asarray(frame.image)
        if image.shape[:2] != (model.height, model.width):
            raise ValueError('image_geometry_mismatch')
        if image.ndim not in (2, 3) or (image.ndim == 3 and image.shape[2] not in (3, 4)):
            raise ValueError('unsupported_image_shape')
        if image.dtype != np.uint8:
            if not np.issubdtype(image.dtype, np.number) or not np.isfinite(image).all():
                raise ValueError('invalid_image_values')
            scale = 255 if np.issubdtype(image.dtype, np.floating) and image.max() <= 1 else 1
            image = np.clip(image * scale, 0, 255).astype(np.uint8)
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY if image.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
        return image

    def _components(self, mask, offset=(0, 0)):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        result = []
        for label in range(1, count):
            x, y, width, height, area = stats[label]
            if self.min_area <= area <= mask.size * .35:
                ys, xs = np.nonzero(labels[y:y+height, x:x+width] == label)
                points = np.column_stack((xs + x + offset[0], ys + y + offset[1])).astype(float)
                result.append(points)
        return sorted(result, key=len, reverse=True)[:64]

    def _blobs(self, gray, model, pose):
        background = float(np.median(gray))
        residual = gray.astype(np.float32) - background
        threshold = max(self.min_contrast, 2.8 * float(np.median(np.abs(residual))))
        components = []
        for sign in (1, -1):
            mask = np.asarray(sign * residual >= threshold, dtype=np.uint8)
            for points in self._components(mask):
                # A one-pixel cord can otherwise move the centroid onto the
                # larger canopy. Split only when opening leaves several bodies.
                low = points.min(axis=0).astype(int)
                high = points.max(axis=0).astype(int) + 1
                roi = mask[low[1]:high[1], low[0]:high[0]]
                opened = cv2.morphologyEx(roi, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8),
                                          borderType=cv2.BORDER_CONSTANT, borderValue=0)
                parts = self._components(opened, tuple(low))
                components.extend(parts if len(parts) >= 2 else [points])
        blobs = []
        up_point = model.project(pose.az_deg, min(89.9, pose.el_deg + .1), pose)
        center = model.project(pose.az_deg, pose.el_deg, pose)
        up = np.array(up_point) - np.array(center) if up_point is not None and center is not None else np.array([0., -1.])
        up = up / np.linalg.norm(up) if np.linalg.norm(up) > 1e-6 else np.array([0., -1.])
        for points in components:
            mean = points.mean(axis=0)
            covariance = (points - mean).T @ (points - mean) / max(1, len(points) - 1)
            _, vectors = np.linalg.eigh(covariance)
            axis = vectors[:, -1]
            along = (points - mean) @ axis
            across = (points - mean) @ np.array([-axis[1], axis[0]])
            length = float(np.ptp(along) + 1)
            width = float(np.ptp(across) + 1)
            aspect = length / max(1, width)
            alignment = abs(float(axis @ up))
            xs, ys = points[:, 0].astype(int), points[:, 1].astype(int)
            contrast = float(np.mean(np.abs(residual[ys, xs])))
            bbox = (int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1))
            clipped = bbox[0] == 0 or bbox[1] == 0 or bbox[2] == model.width or bbox[3] == model.height
            # Recovery features can be only five pixels across in the wide
            # view. The old eight-pixel floor let a small resolved canopy
            # inherit rocket identity during wide-camera reacquisition.
            canopy = length >= 5 and alignment < .45 and aspect >= 1.4
            body = aspect >= 2.1 and alignment >= .45
            blobs.append({'mean': mean, 'axis': axis, 'along': along, 'across': across,
                          'points': points, 'length': length, 'width': width,
                          'aspect': aspect, 'alignment': alignment, 'area': len(points),
                          'polarity': 1 if residual[ys, xs].mean() > 0 else -1,
                          'contrast': contrast, 'bbox': bbox, 'canopy': canopy, 'clipped': clipped,
                          'main_body': body or length <= 6, 'resolved_body': body and length >= 8,
                          'image_up': up})
        return blobs

    def _nose(self, blob, camera):
        direction = self.config.get('nose_direction_by_camera', {}).get(camera)
        if direction is None:
            hint = self.config.get('body_nose_hint', 'unknown')
            direction = {'image_up': [0, -1], 'image_down': [0, 1]}.get(hint)
        if direction is not None:
            direction = np.asarray(direction, dtype=float)
            if direction.shape != (2,) or not np.isfinite(direction).all() or np.linalg.norm(direction) == 0:
                raise ValueError('Nose direction must be a finite nonzero image vector')
            projection = float(blob['axis'] @ (direction / np.linalg.norm(direction)))
            if abs(projection) >= .5:
                return (1 if projection > 0 else -1), 'configured_nose'
        # Do not infer the nose from ascent/descent direction: recovery pose and
        # motion need not align. A clear asymmetric visible taper is optional.
        if self.config.get('infer_visible_nose_taper', False) and blob['length'] >= 20:
            along, across = blob['along'], blob['across']
            span = float(np.ptp(along))
            ends = [across[along <= along.min() + .18 * span], across[along >= along.max() - .18 * span]]
            widths = [float(np.ptp(end) + 1) if len(end) >= 3 else 0 for end in ends]
            if min(widths) >= 1 and max(widths) >= 4 and min(widths) / max(widths) < .5:
                return (-1 if widths[0] < widths[1] else 1), 'visible_taper'
        return None, None

    def _detection(self, blob, frame, model, phase):
        point = blob['mean'].copy()
        kind = 'body_centroid_uncertain_com'
        pixel_sigma = max(.8, blob['width'] / 4)
        if phase.name == 'main_descent' and blob['resolved_body']:
            nose, source = self._nose(blob, frame.camera)
            if nose is not None:
                start = blob['along'].max() if nose > 0 else blob['along'].min()
                end = blob['along'].min() if nose > 0 else blob['along'].max()
                point += blob['axis'] * (start + self.cg_fraction * (end - start))
                kind = 'body_com_' + source
                pixel_sigma += .05 * blob['length']
            else:
                pixel_sigma += (abs(self.cg_fraction - .5) + .08) * blob['length']
        az, el = model.unproject(float(point[0]), float(point[1]), frame.pose)
        confidence = min(.9, .25 + .35 * min(1, blob['contrast'] / 80) + .2 * min(1, blob['aspect'] / 6))
        sigma = math.degrees(math.atan(pixel_sigma / min(model.fx, model.fy)))
        if not model.calibrated:
            sigma = max(sigma, .2)
        return Detection(frame.timestamp, frame.camera, float(point[0]), float(point[1]), az, el,
                         confidence, 'recovery_canopy' if blob['canopy'] else 'rocket_candidate',
                         kind, blob['bbox'], max(.001, sigma))

    def _anchor(self, frame, prior):
        if self.track is not None:
            dt = frame.timestamp - self.track['timestamp']
            if 0 <= dt <= self.coast_s:
                return ((self.track['az'] + self.track['az_rate'] * dt) % 360,
                        float(np.clip(self.track['el'] + self.track['el_rate'] * dt, -89.9, 89.9)),
                        self.gate_deg + 6 * dt, 'temporal')
        if prior is not None and prior.valid and prior.source not in {'truth', 'ground_truth', 'simulator_truth', 'synthetic_truth'}:
            values = (prior.timestamp, prior.az_deg, prior.el_deg, prior.az_rate_deg_s,
                      prior.el_rate_deg_s, prior.uncertainty_deg, prior.lead_time_s)
            if all(math.isfinite(value) for value in values) and abs(frame.timestamp - prior.timestamp) <= 2:
                dt = frame.timestamp - prior.timestamp - prior.lead_time_s
                return ((prior.az_deg + prior.az_rate_deg_s * dt) % 360,
                        float(np.clip(prior.el_deg + prior.el_rate_deg_s * dt, -89.9, 89.9)),
                        max(self.gate_deg, min(10., 3 * max(0., prior.uncertainty_deg))), 'prior')
        return None

    def _choose(self, records, frame, phase, prior):
        model = self.cameras[frame.camera]
        anchor = self._anchor(frame, prior)
        point = model.project(anchor[0], anchor[1], frame.pose) if anchor else None
        previous = self.appearance.get(frame.camera)
        scored = []
        for detection, blob in records:
            # A classified canopy remains a separate recovery observation in
            # every first-flight phase. It must never inherit rocket identity.
            if blob['canopy']:
                continue
            if phase.name == 'main_descent' and not blob['main_body']:
                continue
            fresh_shape = previous and 0 <= frame.timestamp - previous['timestamp'] <= self.coast_s
            if fresh_shape and previous['resolved_body'] and not blob['clipped']:
                # Perspective scale cannot collapse by orders of magnitude in
                # one ordinary frame. Do not let an interior hot pixel replace
                # a recently resolved rocket. Edge clipping is handled below.
                if blob['length'] < .35 * previous['length'] or blob['area'] < .08 * previous['area']:
                    continue
            score = .2 / max(1, blob['aspect']) + (.4 if blob['canopy'] else 0)
            distance = 0.
            if anchor:
                if point is None:
                    continue
                anchor_point = np.array(point)
                if blob['clipped']:
                    # A partially visible body's centroid moves inward from
                    # its full projected centroid. Compare to the visible edge.
                    anchor_point = np.clip(anchor_point, [0, 0], [model.width-1, model.height-1])
                dx = math.degrees(math.atan((blob['mean'][0] - anchor_point[0]) / model.fx))
                dy = math.degrees(math.atan((blob['mean'][1] - anchor_point[1]) / model.fy))
                distance = math.hypot(dx, dy)
                if distance > anchor[2]:
                    continue
                score += 2.0 * distance / anchor[2]
            if fresh_shape:
                score += .35 * abs(math.log(max(1, blob['area']) / previous['area']))
                score += .15 * abs(math.log(max(1, blob['aspect']) / previous['aspect']))
                score += .12 * (blob['polarity'] != previous['polarity'])
            score -= .1 * detection.confidence
            blob['association_distance'] = distance
            blob['association_score'] = score
            blob['anchored'] = anchor is not None
            # A broad trajectory uncertainty can exceed this camera's entire
            # field of view. It cannot, alone, turn an interior sensor feature
            # into an acquired rocket while the expected ray is elsewhere.
            expected_edge = False
            if point is not None and not model.inside(point):
                left, top, right, bottom = blob['bbox']
                expected_edge = ((point[0] >= 0 or left == 0) and
                                 (point[0] < model.width or right == model.width) and
                                 (point[1] >= 0 or top == 0) and
                                 (point[1] < model.height or bottom == model.height))
            blob['requires_support'] = (anchor is None or
                (anchor[3] == 'prior' and not model.inside(point) and not expected_edge))
            scored.append((score, detection, blob))
        scored.sort(key=lambda item: item[0])
        if not scored:
            return None
        if len(scored) > 1 and scored[1][0] - scored[0][0] < (.05 if anchor else .15):
            return None
        score, detection, blob = scored[0]
        detection.confidence *= .85 if anchor else .6
        return detection, blob

    def _camera_agreement(self, first, second):
        dt = first.timestamp - second.timestamp
        rates = {'az_rate': 0., 'el_rate': 0.}
        if self.track and all(0 <= detection.timestamp - self.track['timestamp'] <= self.coast_s
                              for detection in (first, second)):
            rates = self.track
        az = (second.bearing_deg + rates['az_rate'] * dt) % 360
        el = second.elevation_deg + rates['el_rate'] * dt
        separation = math.hypot(angle_error(first.bearing_deg, az) * math.cos(math.radians(el)),
                                first.elevation_deg - el)
        model = min((self.cameras[first.camera], self.cameras[second.camera]),
                    key=lambda camera: min(camera.fx, camera.fy))
        tolerance = max(.08, math.degrees(math.atan(3 / min(model.fx, model.fy))))
        return separation <= tolerance

    def _reconcile_cameras(self, found, result):
        """Only prefer narrow resolution after independent angular agreement."""
        unsupported = [name for name, (detection, blob) in found.items()
            if blob['requires_support'] and not any(
                name != other and self._camera_agreement(detection, measurement)
                for other, (measurement, _) in found.items())]
        for name in unsupported:
            found.pop(name)
            result.warnings.append('acquisition_requires_spatial_support:' + name)
        if self.narrow not in found:
            return
        narrow, nb = found[self.narrow]
        for name in list(found):
            if name == self.narrow or self.narrow not in found:
                continue
            wide, wb = found[name]
            if self._camera_agreement(narrow, wide):
                continue
            result.warnings.append('cross_camera_association_disagreement:' + name)
            # Clipping biases body centroids. Otherwise use temporal/prior
            # residual, with appearance as a tie-breaker, not lens preference.
            n_support = nb['association_distance'] + .1 * nb['association_score'] + (.2 if nb['clipped'] else 0)
            w_support = wb['association_distance'] + .1 * wb['association_score'] + (.2 if wb['clipped'] else 0)
            if not nb['anchored'] and not wb['anchored']:
                found.pop(self.narrow, None)
                found.pop(name, None)
            elif n_support < w_support:
                found.pop(name, None)
            else:
                found.pop(self.narrow, None)

    def update(self, frames: list, prior_hint: Optional[Prediction] = None,
               phase_override: Optional[str] = None, launch_elapsed_s: Optional[float] = None) -> VisionResult:
        now = float(self.clock())
        phase = self._phase(now, phase_override, launch_elapsed_s)
        result = VisionResult(now, phase=phase)
        found, observed = {}, {}
        newest = {}
        for frame in frames:
            if frame.camera not in newest or frame.timestamp > newest[frame.camera].timestamp:
                newest[frame.camera] = frame
        for name, frame in newest.items():
            if name not in self.cameras:
                result.warnings.append('unknown_camera:' + str(name))
                continue
            if not math.isfinite(frame.timestamp) or now - frame.timestamp > self.max_age or frame.timestamp - now > .1:
                result.warnings.append('stale_or_future_frame:' + name)
                continue
            if self.track and frame.timestamp < self.track['timestamp']:
                result.warnings.append('frame_older_than_track:' + name)
                continue
            if (not frame.pose.valid or not all(math.isfinite(value) for value in
                    (frame.pose.timestamp, frame.pose.az_deg, frame.pose.el_deg))
                    or abs(frame.timestamp - frame.pose.timestamp) > self.max_pose_age):
                result.warnings.append('invalid_or_stale_pose:' + name)
                continue
            previous = self.seen.get(name)
            if previous and (frame.sequence <= previous[0] or frame.timestamp <= previous[1]):
                result.warnings.append('duplicate_or_out_of_order_frame:' + name)
                continue
            model = self.cameras[name]
            try:
                gray = self._gray(frame, model)
            except ValueError as error:
                result.warnings.append(str(error) + ':' + name)
                continue
            self.seen[name] = (frame.sequence, frame.timestamp)
            blobs = self._blobs(gray, model, frame.pose)
            records = [(self._detection(blob, frame, model, phase), blob) for blob in blobs]
            result.candidates[name] = [detection for detection, _ in records]
            observed[name] = records
            chosen = self._choose(records, frame, phase, prior_hint)
            if chosen:
                found[name] = chosen
            else:
                result.warnings.append('no_detection_or_ambiguous_association:' + name)
            if not model.calibrated:
                result.warnings.append('nominal_camera_calibration:' + name)
        self._reconcile_cameras(found, result)
        for detection, _ in found.values():
            detection.identity = 'rocket'
        result.narrow_found = self.narrow in found
        result.wide_found = any(name != self.narrow for name in found)
        selected_name = self.narrow if self.narrow in found else next(iter(found), None)
        if selected_name is not None:
            detection, blob = found[selected_name]
            result.selected = detection
            az_rate = el_rate = 0.
            if self.track and 0 < detection.timestamp - self.track['timestamp'] <= self.coast_s:
                dt = detection.timestamp - self.track['timestamp']
                az_rate = .55 * self.track['az_rate'] + .45 * angle_error(detection.bearing_deg, self.track['az']) / dt
                el_rate = .55 * self.track['el_rate'] + .45 * (detection.elevation_deg - self.track['el']) / dt
            self.track = {'timestamp': detection.timestamp, 'az': detection.bearing_deg,
                          'el': detection.elevation_deg, 'az_rate': float(np.clip(az_rate, -100, 100)),
                          'el_rate': float(np.clip(el_rate, -100, 100))}
            for name, (measurement, candidate) in found.items():
                previous = self.appearance.get(name)
                if candidate['clipped'] and previous and measurement.timestamp - previous['timestamp'] <= self.coast_s:
                    continue
                self.appearance[name] = {key: candidate[key] for key in ('area', 'aspect', 'polarity', 'length', 'resolved_body')}
                self.appearance[name]['timestamp'] = measurement.timestamp
            if detection.aim_kind == 'body_centroid_uncertain_com':
                result.warnings.append('rocket_com_uncertain_nose_or_pose_unverified')
            # A large broad feature above an associated descending body can
            # support a low-confidence phase hypothesis, never payload handoff.
            canopy_seen = False
            # A few pixels or a clipped body do not resolve the canopy/body
            # scale ratio. They may support identity, not main deployment.
            if blob['resolved_body'] and blob['length'] >= 20 and not blob['clipped'] and el_rate < -.02:
                for _, feature in observed[selected_name]:
                    relative = feature['mean'] - blob['mean']
                    if (feature['canopy'] and not feature['clipped'] and
                            feature['length'] >= .45 * blob['length']
                            and float(relative @ blob['image_up']) > 0
                            and np.linalg.norm(relative) < 4 * blob['length']):
                        canopy_seen = True
            if canopy_seen:
                if self.visual_main_last is None or now - self.visual_main_last > .4:
                    self.visual_main_since, self.visual_votes = now, 0
                self.visual_main_last, self.visual_votes = now, self.visual_votes + 1
            result.phase = self._phase(now, phase_override, launch_elapsed_s)
        return result
