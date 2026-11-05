#!/usr/bin/env python3
"""Translate a stored OpenRocket flight into site-relative pointing geometry.

This is a nominal prior, not a live estimator or motor-command generator.
Editing rail or weather settings does not recompute the stored flight physics.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def camera_angles(east, north, up):
    horizontal = math.hypot(east, north)
    distance = math.hypot(horizontal, up)
    if distance == 0:
        return None, None, 0.
    bearing = math.degrees(math.atan2(east, north)) % 360 if horizontal > 1e-9 else None
    return bearing, math.degrees(math.atan2(up, horizontal)), distance


def unwrap_bearings(values):
    result, previous = [], None
    for value in values:
        if value is None:
            result.append(None)
            continue
        value = value if previous is None else previous + (value-previous+180) % 360-180
        result.append(value)
        previous = value
    return result


def derivative(times, values):
    result = []
    for i, value in enumerate(values):
        if value is None:
            result.append(None)
            continue
        left, right = i-1, i+1
        while left >= 0 and (times[left] >= times[i] or values[left] is None):
            left -= 1
        while right < len(times) and (times[right] <= times[i] or values[right] is None):
            right += 1
        if left < 0:
            left = i
        if right >= len(times):
            right = i
        result.append((values[right]-values[left])/(times[right]-times[left]) if times[right] > times[left] else None)
    return result


def main():
    from field_config import load_config
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root/'field_config.json')
    parser.add_argument('--trajectory', type=Path, default=root/'openrocket_trajectory.csv')
    parser.add_argument('--output', type=Path, default=root/'pointing_prior.csv')
    args = parser.parse_args()
    config = load_config(args.config)
    pad = config.pad_position_enu()
    with args.trajectory.open(newline='') as handle:
        data = list(csv.DictReader(handle))
    result = []
    for row in data:
        time_s, east, north, up = [float(row[key]) for key in ('Time', 'Position East of launch', 'Position North of launch', 'Altitude')]
        if not all(math.isfinite(v) for v in (time_s, east, north, up)):
            raise ValueError('The saved trajectory needs finite time and position values')
        east, north, up = east+pad[0], north+pad[1], up+pad[2]
        bearing, elevation, distance = camera_angles(east, north, up)
        result.append({'time_s': time_s, 'east_from_tracker_m': east,
                       'north_from_tracker_m': north, 'up_from_tracker_m': up,
                       'range_m': distance, 'bearing_clockwise_from_north_deg': bearing,
                       'elevation_deg': elevation})
    times = [row['time_s'] for row in result]
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError('Saved trajectory times must be nondecreasing')
    bearing_rates = derivative(times, unwrap_bearings([row['bearing_clockwise_from_north_deg'] for row in result]))
    elevation_rates = derivative(times, [row['elevation_deg'] for row in result])
    for row, az_rate, el_rate in zip(result, bearing_rates, elevation_rates):
        row.update(nominal_bearing_rate_deg_s=az_rate, nominal_elevation_rate_deg_s=el_rate)
    with args.output.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(result[0]))
        writer.writeheader()
        writer.writerows(result)
    summary = {'status': 'saved_simulation_geometry_prior_only', 'rows': len(result),
               'config': str(args.config.resolve()),
               'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
               'trajectory_sha256': hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
               'pad_position_enu_m': list(pad),
               'range_min_max_m': [min(x['range_m'] for x in result), max(x['range_m'] for x in result)],
               'max_abs_nominal_bearing_rate_deg_s': max(abs(x) for x in bearing_rates if x is not None),
               'max_abs_nominal_elevation_rate_deg_s': max(abs(x) for x in elevation_rates if x is not None),
               'warning': 'Rail and wind changes require a new flight simulation. This export only translates the stored launch-relative trajectory; it never commands motion.'}
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
