"""Raw MJPG observations and capture metadata. Preview is never replay input.

AVI rate is a playback convenience; sidecar capture/processing times are
authoritative Mac-monotonic timestamps. MJPG images are lossy and unannotated.
"""
import json
import math
import numbers
import os
from pathlib import Path
import re
from typing import Optional
import cv2
import numpy as np
from det_types import Frame, Pose


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value):
        raise ValueError(label + ' must be finite')
    return float(value)


def _pose(value):
    if not isinstance(value, dict) or set(value) != {'timestamp', 'az_deg', 'el_deg', 'valid'}:
        raise ValueError('Pose metadata is missing or malformed')
    if not isinstance(value['valid'], bool):
        raise ValueError('Pose valid flag must be boolean')
    return {**{key: _finite(value[key], 'Pose '+key) for key in ('timestamp', 'az_deg', 'el_deg')},
            'valid': value['valid']}


def _pose_dict(pose):
    if not isinstance(pose, Pose):
        raise ValueError('A Pose is required')
    return _pose({key: getattr(pose, key) for key in ('timestamp', 'az_deg', 'el_deg', 'valid')})


def validate_row(row, cameras, previous_now, previous_frames):
    """Validate recording/replay without substituting global capture metadata."""
    if not isinstance(row, dict) or type(row.get('schema_version')) is not int or row['schema_version'] != 2:
        raise ValueError('Replay requires schema_version 2 with individual frame capture metadata')
    if set(row) != {'schema_version', 'now', 'pose', 'launch_elapsed_s', 'frames'}:
        raise ValueError('Recording row fields are missing or unknown')
    now = _finite(row['now'], 'Processing time')
    if previous_now is not None and now <= previous_now:
        raise ValueError('Processing timestamps must increase')
    elapsed = row['launch_elapsed_s']
    if elapsed is not None:
        elapsed = _finite(elapsed, 'Launch elapsed time')
        if elapsed < 0:
            raise ValueError('Launch elapsed time must be nonnegative')
    if not isinstance(row['frames'], dict) or set(row['frames']) - set(cameras):
        raise ValueError('Unknown or malformed camera metadata')
    frames = {}
    for name, metadata in row['frames'].items():
        if not isinstance(metadata, dict) or set(metadata) != {'timestamp', 'sequence', 'pose'}:
            raise ValueError('Individual frame metadata is missing or malformed')
        timestamp = _finite(metadata['timestamp'], 'Frame timestamp')
        sequence = metadata['sequence']
        if isinstance(sequence, bool) or not isinstance(sequence, numbers.Integral) or sequence < 0:
            raise ValueError('Frame sequence must be a nonnegative integer')
        prior = previous_frames.get(name)
        if prior and (timestamp < prior['timestamp'] or sequence < prior['sequence']):
            raise ValueError('Frame timestamp or sequence moved backwards: '+name)
        frames[name] = {'timestamp': timestamp, 'sequence': int(sequence), 'pose': _pose(metadata['pose'])}
    return {'schema_version': 2, 'now': now, 'pose': _pose(row['pose']),
            'launch_elapsed_s': elapsed, 'frames': frames}


def _camera_names(cameras):
    if not cameras or any(not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_]+', name)
                          or name != camera.name for name, camera in cameras.items()):
        raise ValueError('Camera names must match the models and be safe file names')


def _bgr(image, shape):
    array = np.asarray(image)
    if array.dtype != np.uint8 or array.shape[:2] != shape:
        raise ValueError('Recording image geometry or uint8 format differs from the camera')
    if array.ndim == 2:
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    if array.ndim == 3 and array.shape[2] == 3:
        return np.ascontiguousarray(array)
    raise ValueError('Recording needs grayscale or BGR camera images')


class Recorder:
    def __init__(self, directory, cameras, fps):
        self.root = Path(directory)
        self.cameras = dict(cameras)
        _camera_names(self.cameras)
        self.fps = _finite(fps, 'Recording frame rate')
        if self.fps <= 0:
            raise ValueError('Recording frame rate must be positive')
        if any(camera.width % 2 or camera.height % 2 for camera in self.cameras.values()):
            raise ValueError('MJPG recording requires even image dimensions')
        self.writers = {}
        self.sidecar = None
        self.preview = None
        self.preview_shape = None
        self.previous_now = None
        self.previous_frames = {}
        self.rows_written = 0
        self.closed = False
        self.root.mkdir(parents=True, exist_ok=True)
        try:
            self.sidecar = (self.root/'observations.jsonl').open('x')
            for name, camera in self.cameras.items():
                self.writers[name] = self._open_video(name+'.avi', 'MJPG', (camera.height, camera.width))
        except Exception:
            self.close()
            raise

    def _open_video(self, filename, codec, shape):
        path = self.root/filename
        descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.close(descriptor)
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), self.fps, (shape[1], shape[0]))
        if not writer.isOpened():
            writer.release()
            raise RuntimeError('Video writer unavailable: '+filename)
        return writer

    def write(self, frames: dict, now: float, pose: Pose, launch_elapsed_s: Optional[float],
              preview: Optional[np.ndarray] = None):
        if self.closed:
            raise RuntimeError('Recorder is closed')
        if not isinstance(frames, dict) or set(frames) - set(self.cameras):
            raise ValueError('Unexpected recording camera')
        metadata, images = {}, {}
        # Validate the whole row before appending any video, preventing shifts
        # if a later camera in the row has malformed metadata or geometry.
        for name, camera in self.cameras.items():
            frame = frames.get(name)
            if frame is None:
                images[name] = np.zeros((camera.height, camera.width, 3), np.uint8)
                continue
            if not isinstance(frame, Frame) or frame.camera != name:
                raise ValueError('Frame camera does not match its recording slot')
            metadata[name] = {'timestamp': frame.timestamp, 'sequence': frame.sequence,
                              'pose': _pose_dict(frame.pose)}
            images[name] = _bgr(frame.image, (camera.height, camera.width))
        row = validate_row({'schema_version': 2, 'now': now, 'pose': _pose_dict(pose),
                            'launch_elapsed_s': launch_elapsed_s, 'frames': metadata},
                           self.cameras, self.previous_now, self.previous_frames)
        preview_image = None
        if preview is not None:
            shape = np.asarray(preview).shape[:2]
            if len(shape) != 2 or min(shape) <= 0 or any(value % 2 for value in shape):
                raise ValueError('Preview requires positive even image dimensions')
            preview_image = _bgr(preview, self.preview_shape or shape)
        encoded = json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n'
        try:
            if preview_image is not None and self.preview is None:
                self.preview_shape = preview_image.shape[:2]
                self.preview = self._open_video('preview.mp4', 'mp4v', self.preview_shape)
                for _ in range(self.rows_written):
                    self.preview.write(np.zeros((*self.preview_shape, 3), np.uint8))
            for name, writer in self.writers.items():
                writer.write(images[name])
            if self.preview is not None:
                self.preview.write(preview_image if preview_image is not None else
                                   np.zeros((*self.preview_shape, 3), np.uint8))
            self.sidecar.write(encoded)
            self.sidecar.flush()
        except Exception:
            self.close()
            raise
        self.previous_now = row['now']
        self.previous_frames.update(row['frames'])
        self.rows_written += 1

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            for writer in self.writers.values():
                writer.release()
            if self.preview is not None:
                self.preview.release()
        finally:
            if self.sidecar is not None:
                self.sidecar.close()
