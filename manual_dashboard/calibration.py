"""Wall reference records. This module never controls motion or emitters."""
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import uuid

class CalibrationError(ValueError):
    pass


def map_between_views(point, source, destination):
    a, b = source["points"]["a"], source["points"]["b"]
    c, d = destination["points"]["a"], destination["points"]["b"]
    u = complex(b[0] - a[0], b[1] - a[1])
    if abs(u) < 1:
        raise CalibrationError("Dot A and Dot B must be distinct")
    scale = complex(d[0] - c[0], d[1] - c[1]) / u
    mapped = complex(c[0], c[1]) + scale * complex(point[0] - a[0], point[1] - a[1])
    return [mapped.real, mapped.imag]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def build_reference(data):
    try:
        result = deepcopy(data)
        if len(result["views"]) != 2:
            raise CalibrationError("Two camera views are required")
        measurements = result["measurements"]
        for name in ("dot_spacing_m", "distance_to_a_m", "vertical_offset_m"):
            value = measurements[name]
            if not finite(value) or (name != "vertical_offset_m" and value <= 0):
                raise CalibrationError("Enter finite wall measurements; spacing and distance must be positive")
        for view in result["views"]:
            width, height = view["size"]
            if not all(finite(v) and v > 0 for v in (width, height)):
                raise CalibrationError("Invalid camera dimensions")
            for name in ("a", "b", "pointer"):
                x, y = view["points"][name]
                if not all(finite(v) for v in (x, y)) or not (0 <= x < width and 0 <= y < height):
                    raise CalibrationError("Mark Dot A, Dot B and the pointer reference inside each image")
            a, b = view["points"]["a"], view["points"]["b"]
            span = math.hypot(a[0] - b[0], a[1] - b[1])
            if span < 2:
                raise CalibrationError("Dot A and Dot B are too close together")
            view["metres_per_pixel_along_ab"] = measurements["dot_spacing_m"] / span
        result.update(schema_version=1, status="wall_reference_only", axis_calibrated=False,
                      created_at=datetime.now(timezone.utc).isoformat(),
                      limitations=["Two marks establish a local similarity at this wall pose and distance.",
                                   "Lens distortion, full perspective, axis direction and motion scale are not calibrated."])
        return result
    except (KeyError, TypeError, ValueError) as error:
        if isinstance(error, CalibrationError):
            raise
        raise CalibrationError("Missing or malformed reference data") from error


def save_reference(root, reference, frames):
    destination = Path(root) / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8])
    destination.mkdir(parents=True, exist_ok=False)
    for index, frame in enumerate(frames):
        (destination / f"camera_{index + 1}.jpg").write_bytes(frame)
    (destination / "reference.json").write_text(json.dumps(reference, indent=2, allow_nan=False) + "\n")
    return destination
