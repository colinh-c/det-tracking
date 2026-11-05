"""Replay raw video slots using authoritative individual capture metadata."""
from pathlib import Path
import json
import cv2
from det_types import Frame, Pose
from recording import _camera_names, validate_row

class ReplaySource:
    def __init__(self, directory, cameras):
        self.root = Path(directory)
        self.cameras = dict(cameras)
        _camera_names(self.cameras)
        self.records = None
        self.captures = {}
        self.now = 0.
        self.launch_elapsed_s = None
        self.previous_now = None
        self.previous_frames = {}
        self.closed = False
        self.finished = False
        try:
            self.records = (self.root/'observations.jsonl').open()
            for name in self.cameras:
                capture = cv2.VideoCapture(str(self.root/(name+'.avi')))
                self.captures[name] = capture
                if not capture.isOpened():
                    raise ValueError('Replay cannot open raw camera video: '+name)
        except Exception:
            self.close()
            raise

    def read(self):
        if self.closed or self.finished:
            raise StopIteration
        try:
            line = self.records.readline()
            if not line:
                for name, capture in self.captures.items():
                    ok, _ = capture.read()
                    if ok:
                        raise ValueError('Replay video has frames without metadata: '+name)
                self.finished = True
                raise StopIteration
            try:
                row = json.loads(line)
            except (ValueError, TypeError) as error:
                raise ValueError('Malformed observations.jsonl row') from error
            row = validate_row(row, self.cameras, self.previous_now, self.previous_frames)
            frames = []
            for name, capture in self.captures.items():
                ok, image = capture.read()
                if not ok or image is None:
                    raise ValueError('Replay video ended before its metadata: '+name)
                camera = self.cameras[name]
                if image.shape[:2] != (camera.height, camera.width):
                    raise ValueError('Replay dimensions differ from calibration: '+name)
                metadata = row['frames'].get(name)
                if metadata is not None:
                    frames.append(Frame(name, metadata['timestamp'], cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                        Pose(**metadata['pose']), metadata['sequence']))
            self.now = row['now']
            self.launch_elapsed_s = row['launch_elapsed_s']
            self.previous_now = self.now
            self.previous_frames.update(row['frames'])
            return frames, Pose(**row['pose'])
        except StopIteration:
            raise
        except Exception:
            # A partial row cannot be retried against advanced video cursors.
            self.close()
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            for capture in self.captures.values():
                capture.release()
        finally:
            if self.records is not None:
                self.records.close()
