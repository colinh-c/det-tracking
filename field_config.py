#!/usr/bin/env python3
"""Editable DET site geometry, using only the Python standard library.

This is a configuration foundation for Mac-side tracking work, not a validated
flight tracker or a motor command interface. Origin: tracker. ENU axes: east,
north, up. Bearings: degrees clockwise from true North. Rail tilt: angle from
vertical, 0 degrees up, 90 horizontal, 180 down. No gimbal travel limits apply.
Rail settings describe geometry; they do not rerun saved OpenRocket flight
physics. Substantial rail or wind changes require an updated simulation.

Load/inspect: python3 field_config.py --config field_config.json
Import with backup: python3 field_config.py --config field_config.json --import-json downloaded.json
"""
import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile

SCHEMA_VERSION = 1
FIELDS = ("pad_horizontal_distance_m", "pad_bearing_deg", "pad_height_relative_to_tracker_m",
          "rail_tilt_from_vertical_deg", "rail_bearing_deg")
HEADERS = {"schema_version": SCHEMA_VERSION, "coordinate_frame": "ENU",
           "heading_reference": "true_north_clockwise"}


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def bearing_to_enu_azimuth_deg(bearing):
    """Convert North-clockwise bearing to East-counterclockwise ENU azimuth."""
    return (90.0 - finite_number(bearing, "bearing") % 360.0) % 360.0


def clean_zero(value):
    return 0.0 if abs(value) < 1e-12 else value


@dataclass(frozen=True)
class FieldConfig:
    pad_horizontal_distance_m: float = 500.0
    pad_bearing_deg: float = 0.0
    pad_height_relative_to_tracker_m: float = 0.0
    rail_tilt_from_vertical_deg: float = 0.0
    rail_bearing_deg: float = 0.0
    measurement_status: dict = field(default_factory=lambda: {name: "nominal" for name in FIELDS})
    notes: str = "Nominal, unmeasured setup. Replace with on-site measurements before relying on geometry."

    def __post_init__(self):
        for name in FIELDS:
            object.__setattr__(self, name, finite_number(getattr(self, name), name))
        if self.pad_horizontal_distance_m < 0:
            raise ValueError("pad_horizontal_distance_m must be nonnegative")
        if not 0 <= self.rail_tilt_from_vertical_deg <= 180:
            raise ValueError("rail_tilt_from_vertical_deg must be between 0 and 180")
        for name in ("pad_bearing_deg", "rail_bearing_deg"):
            object.__setattr__(self, name, getattr(self, name) % 360.0)
        if not isinstance(self.measurement_status, dict) or set(self.measurement_status) != set(FIELDS):
            raise ValueError("measurement_status must specify every geometry field exactly once")
        if any(value not in ("nominal", "measured") for value in self.measurement_status.values()):
            raise ValueError("measurement_status values must be nominal or measured")
        object.__setattr__(self, "measurement_status", dict(self.measurement_status))
        if not isinstance(self.notes, str):
            raise ValueError("notes must be text")

    def to_dict(self):
        return {**HEADERS, "geometry": {name: getattr(self, name) for name in FIELDS},
                "measurement_status": dict(self.measurement_status), "notes": self.notes}

    def pad_position_enu(self):
        """Pad position in metres relative to the tracker, not a gimbal angle."""
        bearing = math.radians(self.pad_bearing_deg)
        return (clean_zero(self.pad_horizontal_distance_m * math.sin(bearing)),
                clean_zero(self.pad_horizontal_distance_m * math.cos(bearing)),
                self.pad_height_relative_to_tracker_m)

    def rail_unit_enu(self):
        """Unit vector pointing along the rail in ENU; bearing is moot at zero tilt."""
        bearing = math.radians(self.rail_bearing_deg)
        tilt = math.radians(self.rail_tilt_from_vertical_deg)
        horizontal = math.sin(tilt)
        return (clean_zero(horizontal * math.sin(bearing)),
                clean_zero(horizontal * math.cos(bearing)), clean_zero(math.cos(tilt)))


def validate_config(data):
    if not isinstance(data, dict):
        raise ValueError("Configuration must be a JSON object")
    required = {*HEADERS, "geometry", "measurement_status"}
    if set(data) - required - {"notes"} or required - set(data):
        raise ValueError("Configuration has missing or unknown fields")
    for name, expected in HEADERS.items():
        if data[name] != expected or isinstance(data[name], bool):
            raise ValueError(f"{name} must be {expected!r}")
    geometry = data["geometry"]
    if not isinstance(geometry, dict) or set(geometry) != set(FIELDS):
        raise ValueError("geometry must contain exactly the five documented fields")
    return FieldConfig(**geometry, measurement_status=data["measurement_status"], notes=data.get("notes", ""))


def reject_constant(value):
    raise ValueError(f"Non-finite JSON value is not allowed: {value}")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def load_config(path):
    with Path(path).open(encoding="utf-8") as source:
        return validate_config(json.load(source, parse_constant=reject_constant, object_pairs_hook=unique_object))


def save_config(config, path):
    """Validate, back up existing bytes, then replace atomically. Return backup path."""
    config = validate_config(config.to_dict() if isinstance(config, FieldConfig) else config)
    path = Path(path)
    if path.is_symlink():
        raise ValueError("Choose the configuration file itself, not a symbolic link")
    path.parent.mkdir(parents=True, exist_ok=True)
    backup = None
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix="." + path.name + ".", suffix=".tmp", delete=False) as output:
            temporary = Path(output.name)
            json.dump(config.to_dict(), output, indent=2, allow_nan=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        if path.exists():
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            backup = path.with_name(path.name + ".backup-" + stamp)
            with path.open("rb") as source, backup.open("xb") as output:
                shutil.copyfileobj(source, output)
                output.flush()
                os.fsync(output.fileno())
        os.replace(temporary, path)
        temporary = None
        return backup
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("field_config.json"))
    parser.add_argument("--import-json", type=Path, help="Validate this JSON and replace --config after backing it up")
    args = parser.parse_args(argv)
    try:
        backup = None
        if args.import_json is not None:
            backup = save_config(load_config(args.import_json), args.config)
        config = load_config(args.config)
        report = {"config_path": str(args.config.resolve()), "config": config.to_dict(),
                  "derived": {"pad_position_enu_m": config.pad_position_enu(),
                              "rail_unit_enu": config.rail_unit_enu(),
                              "pad_azimuth_enu_deg": bearing_to_enu_azimuth_deg(config.pad_bearing_deg),
                              "rail_azimuth_enu_deg": bearing_to_enu_azimuth_deg(config.rail_bearing_deg)}}
        if backup is not None:
            report["backup_path"] = str(backup.resolve())
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError) as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
