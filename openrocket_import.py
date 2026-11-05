#!/usr/bin/env python3
"""Read saved OpenRocket data without executing a simulation or changing its archive."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile


DEFAULT_SOURCE = Path('rocket.ork')
UNITS = {
    'Time': 's', 'Altitude': 'm', 'Altitude above sea level': 'm',
    'Vertical velocity': 'm/s', 'Total velocity': 'm/s',
    'Vertical acceleration': 'm/s^2', 'Total acceleration': 'm/s^2',
    'Position East of launch': 'm', 'Position North of launch': 'm',
    'Lateral distance': 'm', 'Lateral direction': 'rad',
    'Lateral velocity': 'm/s', 'Lateral acceleration': 'm/s^2',
    'Latitude': 'deg', 'Longitude': 'deg', 'Angle of attack': 'rad',
    'Roll rate': 'rad/s', 'Pitch rate': 'rad/s', 'Yaw rate': 'rad/s',
    'Vertical orientation (zenith)': 'rad', 'Lateral orientation (azimuth)': 'rad',
    'Mass': 'kg', 'Motor mass': 'kg', 'Longitudinal moment of inertia': 'kg*m^2',
    'Rotational moment of inertia': 'kg*m^2', 'Gravitational acceleration': 'm/s^2',
    'CP location': 'm', 'CG location': 'm', 'Stability margin calibers': 'calibers',
    'Thrust': 'N', 'Thrust-to-weight ratio': 'dimensionless', 'Drag force': 'N',
    'Wind velocity': 'm/s', 'Wind direction': 'rad', 'Air temperature': 'K',
    'Air pressure': 'Pa', 'Air density': 'kg/m^3', 'Speed of sound': 'm/s',
    'Mach number': 'dimensionless', 'Reynolds number': 'dimensionless',
    'Reference length': 'm', 'Reference area': 'm^2', 'Simulation time step': 's',
    'Computation time': 's', 'Coriolis acceleration': 'm/s^2',
}
CONDITION_UNITS = {
    'launchrodlength': 'm', 'launchrodangle': 'deg from vertical',
    'launchroddirection': 'deg, OpenRocket launch bearing',
    'windaverage': 'm/s', 'windturbulence': 'dimensionless', 'winddirection': 'rad',
    'launchaltitude': 'm above sea level', 'launchlatitude': 'deg',
    'launchlongitude': 'deg', 'timestep': 's', 'maxtime': 's',
}


def leaf_values(element):
    return {child.tag: (child.text or '').strip() for child in element if len(child) == 0}


def xml_record(element):
    return {'tag': element.tag, 'attributes': dict(element.attrib),
            'text': (element.text or '').strip(),
            'children': [xml_record(child) for child in element]}


def field_manifest(columns):
    fields = []
    meanings = {
        'Altitude': 'Altitude relative to the launch level.',
        'Altitude above sea level': 'Saved altitude above sea level.',
        'Position East of launch': 'East displacement from launch, as explicitly named in the archive.',
        'Position North of launch': 'North displacement from launch, as explicitly named in the archive.',
        'CG location': 'Raw OpenRocket axial CG location; no suspended-body geometry is inferred.',
        'Reference length': 'Aerodynamic reference length, not total rocket length.',
    }
    for name in columns:
        unit = UNITS.get(name)
        if unit is None and name.endswith('coefficient'):
            unit = 'dimensionless'
        if unit is None:
            raise ValueError('Unknown field units; refusing to guess: ' + name)
        field = {'name': name, 'unit': unit, 'meaning': meanings.get(name, 'Original OpenRocket field, unchanged.')}
        if unit in ('rad', 'rad/s'):
            field['angle_convention'] = 'Preserved OpenRocket convention; no bearing, zenith, elevation, or camera transform applied.'
        fields.append(field)
    return fields


def load_design(source, config_id=None, simulation_name=None):
    source = Path(source).resolve()
    original = source.read_bytes()
    with zipfile.ZipFile(io.BytesIO(original)) as archive:
        info = archive.getinfo('rocket.ork')
        if info.file_size > 20_000_000:
            raise ValueError('OpenRocket XML exceeds the bounded import size')
        xml = archive.read(info)
    if b'<!DOCTYPE' in xml.upper() or b'<!ENTITY' in xml.upper():
        raise ValueError('XML document declarations/entities are not accepted')
    root = ET.fromstring(xml)
    rocket = root.find('rocket')
    if root.tag != 'openrocket' or rocket is None:
        raise ValueError('Not an OpenRocket design')
    motors = {}
    for motor in rocket.findall('.//motormount/motor'):
        motors.setdefault(motor.attrib['configid'], []).append(leaf_values(motor))
    configurations = [
        {'config_id': item.attrib['configid'], 'default': item.get('default') == 'true',
         'stages': [dict(stage.attrib) for stage in item.findall('stage')],
         'motors': motors.get(item.attrib['configid'], [])}
        for item in rocket.findall('motorconfiguration')]
    simulations = root.findall('./simulations/simulation')
    if simulation_name is not None:
        candidates = [item for item in simulations if item.findtext('name') == simulation_name]
    else:
        if config_id is None:
            defaults = [item['config_id'] for item in configurations if item['default']]
            if len(defaults) != 1:
                raise ValueError('Select a configuration explicitly; there is no unique default')
            config_id = defaults[0]
        candidates = [item for item in simulations if item.findtext('conditions/configid') == config_id]
        current = [item for item in candidates if item.get('status') == 'uptodate']
        if len(current) == 1:
            candidates = current
    if len(candidates) != 1:
        raise ValueError('Select a unique saved simulation explicitly')
    selected = candidates[0]
    selected_id = selected.findtext('conditions/configid')
    if config_id is not None and selected_id != config_id:
        raise ValueError('Simulation and configuration selections conflict')
    if selected_id not in {item['config_id'] for item in configurations}:
        raise ValueError('Simulation references an unknown configuration')
    data = selected.find('flightdata')
    branches = data.findall('databranch') if data is not None else []
    if len(branches) != 1:
        raise ValueError('This importer requires one explicitly selected saved trajectory branch')
    branch = branches[0]
    columns = branch.attrib['types'].split(',')
    if len(set(columns)) != len(columns) or 'Time' not in columns:
        raise ValueError('Invalid or duplicate trajectory columns')
    rows = [(point.text or '').strip().split(',') for point in branch.findall('datapoint')]
    if not rows or any(len(row) != len(columns) for row in rows):
        raise ValueError('Trajectory datapoint columns do not match the field manifest')
    numeric = [[float(value) for value in row] for row in rows]
    if any(math.isinf(value) for row in numeric for value in row):
        raise ValueError('Infinite trajectory values are not supported')
    times = [row[columns.index('Time')] for row in numeric]
    if not all(math.isfinite(t) for t in times) or any(b < a for a, b in zip(times, times[1:])):
        raise ValueError('Trajectory time must be finite and nondecreasing')
    events = [{**event.attrib, 'time_s': float(event.attrib['time'])} for event in branch.findall('event')]
    recovery = []
    for chute in rocket.iter('parachute'):
        values = leaf_values(chute)
        recovery.append({'id': values['id'], 'name': values['name'],
                         'diameter_m': float(values['diameter']), 'cd': values['cd'],
                         'deployment_event': values['deployevent'],
                         'deployment_altitude_m': float(values['deployaltitude']),
                         'deployment_delay_s': float(values['deploydelay']), 'source_values': values})
    main_ids = {item['id'] for item in recovery if item['deployment_event'] == 'altitude'}
    main_events = [event for event in events if event['type'] == 'recoverydevicedeployment' and event.get('source') in main_ids]
    main_time = main_events[0]['time_s'] if len(main_events) == 1 else None
    ground_time = next((event['time_s'] for event in events if event['type'] == 'groundhit'), None)
    apogee_time = next((event['time_s'] for event in events if event['type'] == 'apogee'), None)
    outer = rocket.findall('./subcomponents/stage/subcomponents/*')
    radii = [float(item.findtext('radius', item.findtext('aftradius', '0')).split()[-1]) for item in outer]
    conditions = selected.find('conditions')
    scalar_conditions = leaf_values(conditions)
    first, last = dict(zip(columns, numeric[0])), dict(zip(columns, numeric[-1]))
    def last_finite(name):
        if name not in columns:
            return None
        index = columns.index(name)
        return next((row[index] for row in reversed(numeric) if math.isfinite(row[index])), None)
    windows = {'apogee_time_s': apogee_time, 'main_deployment_time_s': main_time,
               'ground_hit_time_s': ground_time,
               'main_to_ground_s': ground_time - main_time if ground_time is not None and main_time is not None else None,
               'apogee_to_ground_s': ground_time - apogee_time if ground_time is not None and apogee_time is not None else None}
    if main_time is not None:
        nearest = min(numeric, key=lambda row: abs(row[columns.index('Time')] - main_time))
        windows['nearest_main_event_sample'] = {name: nearest[columns.index(name)] for name in ('Time', 'Altitude', 'Total velocity')}
    baseline = {
        'schema_version': 1,
        'source': {'path': str(source), 'sha256': hashlib.sha256(original).hexdigest(),
                   'archive_member': 'rocket.ork', 'creator': root.get('creator'),
                   'format_version': root.get('version'), 'simulation_executed': False},
        'selection': {'config_id': selected_id, 'simulation_name': selected.findtext('name'),
                      'saved_status': selected.get('status'), 'branch_name': branch.get('name'),
                      'is_saved_default': any(item['config_id'] == selected_id and item['default'] for item in configurations),
                      'motors': motors.get(selected_id, []),
                      'assumption': 'Use the saved default configuration and matching up-to-date simulation unless explicitly overridden; actual flight configuration remains user-controlled.'},
        'configurations': configurations,
        'saved_simulations': [{'name': item.findtext('name'), 'saved_status': item.get('status'),
                               'config_id': item.findtext('conditions/configid'),
                               'summary': dict(item.find('flightdata').attrib) if item.find('flightdata') is not None else {}}
                              for item in simulations],
        'selected_summary': dict(data.attrib),
        'conditions': {'original_scalar_values': scalar_conditions,
                       'units': CONDITION_UNITS,
                       'wind_models': [xml_record(item) for item in conditions.findall('wind')],
                       'atmosphere': dict(conditions.find('atmosphere').attrib) if conditions.find('atmosphere') is not None else {},
                       'is_confirmed_launch_day_configuration': False},
        'geometry': {'outer_component_length_sum_m': sum(float(item.findtext('length', '0')) for item in outer),
                     'maximum_outer_body_diameter_m': 2 * max(radii),
                     'outer_components': [{'type': item.tag, **leaf_values(item)} for item in outer],
                     'fin_sets': [leaf_values(item) for item in rocket.iter('trapezoidfinset')],
                     'note': 'Length is the sum of this design\'s sequential outer components; earlier user length was approximate 2.75 m. Confirm as-flown geometry before interpreting physical image scale.'},
        'mass_and_cg': {'launch_mass_kg': first.get('Mass'), 'post_burn_mass_kg': last_finite('Mass'),
                        'launch_motor_mass_kg': first.get('Motor mass'), 'spent_motor_mass_kg': last_finite('Motor mass'),
                        'launch_cg_location_m': first.get('CG location'), 'post_burn_cg_location_m': last_finite('CG location'),
                        'post_burn_value_policy': 'Last available finite value in the saved trajectory; NaNs in CSV remain unchanged.',
                        'cg_reference': 'Original OpenRocket axial CG coordinate; no inference of suspended assembly geometry.'},
        'internal_payload_components': [leaf_values(item) for item in rocket.iter('masscomponent') if item.findtext('masscomponenttype') == 'payload'],
        'recovery': recovery,
        'shock_cords': [leaf_values(item) for item in rocket.iter('shockcord')],
        'events': events,
        'warnings': [{'type': item.get('type'), **leaf_values(item)} for item in data.findall('warning')],
        'tracking_windows': windows,
        'trajectory': {'file': 'openrocket_trajectory.csv', 'point_count': len(rows), 'column_count': len(columns),
                       'fields': field_manifest(columns), 'time_range_s': [times[0], times[-1]],
                       'sample_interval_range_s': [min(b - a for a, b in zip(times, times[1:])), max(b - a for a, b in zip(times, times[1:]))],
                       'nan_policy': 'Original NaN strings are retained as unavailable data, never replaced by zero.',
                       'camera_offset_applied': False,
                       'coordinate_policy': 'Original named launch-relative East/North/Altitude columns are unchanged. No camera position, rotation, orientation convention conversion, resampling, or interpolation is applied.'},
        'validation_status': 'Saved simulation data only; not observed flight or physical tracking validation.',
    }
    def json_measurements(value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: json_measurements(item) for key, item in value.items()}
        if isinstance(value, list):
            return [json_measurements(item) for item in value]
        return value
    return json_measurements(baseline), columns, rows


def review_text(b):
    s, w, g, m = b['selection'], b['tracking_windows'], b['geometry'], b['mass_and_cg']
    motors = ', '.join(item['manufacturer'] + ' ' + item['designation'] for item in s['motors'])
    conditions = b['conditions']['original_scalar_values']
    selection_basis = ('It is the design\'s saved default.' if s['is_saved_default'] else
                       'This is an explicit override of the saved default; outdated results may reference earlier component IDs.')
    def number(value, digits=3):
        return 'unresolved' if value is None else f'{float(value):.{digits}f}'
    warning_text = '\n'.join('- ' + warning.get('description', warning['type']) +
                             ' (stored simulation warning, not observed flight validation).'
                             for warning in b['warnings']) or '- No saved simulation warning.'
    main = next((item for item in b['recovery'] if item['deployment_event'] == 'altitude'), None)
    drogue = next((item for item in b['recovery'] if item['deployment_event'] == 'apogee'), None)
    main_sample_altitude = w.get('nearest_main_event_sample', {}).get('Altitude')
    burnout = next((item['time_s'] for item in b['events'] if item['type'] == 'burnout'), None)
    max_altitude = float(b['selected_summary']['maxaltitude'])
    return f'''# SSI DET OpenRocket baseline review

Date: 2026-11-02

Use **{s['simulation_name']}**, {motors}, configuration `{s['config_id']}`, as the selected offline baseline. {selection_basis} The saved simulation is marked `{s['saved_status']}`. This selection is an explicit working assumption; use the importer's `--simulation` or `--config-id` options to select a different saved case.

Source: [{Path(b['source']['path']).name}](<{b['source']['path']}>), archive member `rocket.ork`, created by {b['source']['creator']}. SHA256: `{b['source']['sha256']}`. The archive was read without execution or modification. No flight simulation, image synthesis, hardware contact, or deployment occurred.

The export preserves every saved datapoint: **{b['trajectory']['point_count']} rows and {b['trajectory']['column_count']} original fields**. `openrocket_trajectory.csv` retains the original field names, numeric strings and NaNs. `openrocket_baseline.json` contains field units, all events and warnings, recovery details, conditions and configuration choices. Sampling varies from about 0.001 to 0.5 seconds, so consumers must use the recorded timestamps. The original launch-relative East, North and altitude fields are retained; no camera coordinate transform is applied. OpenRocket's orientation labels and radians are preserved without interpreting the label “zenith” as a camera or rail angle.

The user's latest **500 m nominal flat-site camera offset** belongs in field setup and projection. It does not change the saved OpenRocket trajectory. The saved simulation's latitude, longitude, wind and atmosphere are model inputs, not confirmed launch-day measurements.

## Recovery and tracking interval

The file contains one Sustainer trajectory and no payload separation event. The internal 2 kg component named Payload stays part of the rocket model; no-deployment does not mean its mass should be deleted.

- Current-design drogue: {number(drogue['diameter_m'] if drogue else None)} m diameter, configured for apogee. Saved apogee time: {number(w['apogee_time_s'])} s; exact recovery-event timestamps remain in the JSON.
- Current-design main: {number(main['diameter_m'] if main else None, 4)} m diameter, configured Cd {main['cd'] if main else 'unresolved'} and deployment altitude {number(main['deployment_altitude_m'] if main else None, 0)} m. Saved deployment time associated with that component: {number(w['main_deployment_time_s'])} s; corresponding sample altitude: {number(main_sample_altitude)} m. Preserve saved event timing. An unresolved association in an outdated simulation is not guessed.
- Ground hit: {number(w['ground_hit_time_s'])} s. The main-parachute tracking window is **{number(w['main_to_ground_s'])} s**; apogee-to-ground descent spans {number(w['apogee_to_ground_s'])} s.
{warning_text}

The stated objective is to keep the rocket in frame before main deployment, without requiring tight centering, and then maximize descent time on target while aiming at the estimated rocket center of mass under main. Future payload-parachute centering is a separate later flight. A single saved rocket branch does not provide separate canopy, cord, nose-section and airframe kinematics. Its saved descent orientation must not be treated as a validated hanging or swinging recovery pose. Synthetic pose, canopy offset, swing and appearance parameters should remain clearly marked assumptions until the recovery arrangement is confirmed.

## Dimensions, mass and conditions

The design's sequential outer-component lengths total **{g['outer_component_length_sum_m']:.4f} m**, with maximum body diameter **{g['maximum_outer_body_diameter_m'] * 1000:.3f} mm**. This differs from the earlier approximate 2.75 m length and nominal 6-inch diameter; use the supplied design for synthetic geometry and confirm the as-flown dimensions before physical image-scale calibration. The four fins have 0.30 m root chord, 0.10 m tip chord and 0.16 m radial height. The trajectory's 0.157 m “Reference length” is an aerodynamic reference dimension, not total vehicle length.

Saved launch mass is {number(m['launch_mass_kg'])} kg and post-burn mass is {number(m['post_burn_mass_kg'])} kg, including the spent motor. Motor mass changes from {number(m['launch_motor_mass_kg'])} to {number(m['spent_motor_mass_kg'])} kg. Saved axial CG location changes from {number(m['launch_cg_location_m'])} to {number(m['post_burn_cg_location_m'])} m. Post-burn values use the last available finite samples; unavailable JSON measurements are null and original CSV NaNs remain unchanged. These remain OpenRocket axial coordinates, not a measured center of a suspended recovery assembly.

Selected conditions: rail tilt {conditions.get('launchrodangle')} degrees from vertical, rail length {conditions.get('launchrodlength')} m, average wind {conditions.get('windaverage')} m/s, turbulence ratio {conditions.get('windturbulence')}, atmosphere {b['conditions']['atmosphere'].get('model')}, launch altitude {conditions.get('launchaltitude')} m, and saved latitude/longitude {conditions.get('launchlatitude')}/{conditions.get('launchlongitude')}. Full wind models, including standard deviation, are retained in JSON. Burnout is {number(burnout)} s; apogee is {max_altitude:.3f} m at {number(w['apogee_time_s'])} s, approximately {max_altitude / .3048:,.0f} ft. The selected trajectory should determine the synthetic test envelope rather than being clipped to the earlier approximate 10,000 ft requirement. All four saved motor cases and their current/outdated status remain listed in the JSON.

## Remaining inputs

The design is sufficient to begin the offline trajectory adapter and synthetic tracking work. Before treating the scenario as flight-ready, confirm the actual motor/configuration, as-flown recovery rig and mass, and launch-day site/rail/wind conditions. The field configuration can retain the user's editable 500 m offset. A real flight video is not required to build the synthetic baseline. Star-based pointing calibration is later work and is not supplied by this archive.

Format references: [OpenRocket 24.12 flight data types](https://github.com/openrocket/openrocket/blob/release-24.12/core/src/main/java/info/openrocket/core/simulation/FlightDataType.java), [RK4 stepper](https://github.com/openrocket/openrocket/blob/release-24.12/core/src/main/java/info/openrocket/core/simulation/RK4SimulationStepper.java), and [recovery drag calculation](https://github.com/openrocket/openrocket/blob/release-24.12/core/src/main/java/info/openrocket/core/simulation/BasicLandingStepper.java).
'''


def write_exports(source, output_dir, config_id=None, simulation_name=None):
    baseline, columns, rows = load_design(source, config_id, simulation_name)
    review = review_text(baseline)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / 'openrocket_trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)
    (output_dir / 'openrocket_baseline.json').write_text(json.dumps(baseline, indent=2, allow_nan=False) + '\n')
    (output_dir / 'OPENROCKET_REVIEW.md').write_text(review)
    return baseline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--config-id', help='Explicit saved motor configuration UUID')
    parser.add_argument('--simulation', dest='simulation_name', help='Exact saved simulation name')
    args = parser.parse_args()
    baseline = write_exports(args.source, args.output_dir, args.config_id, args.simulation_name)
    print(json.dumps({'selection': baseline['selection']['simulation_name'],
                      'rows': baseline['trajectory']['point_count'],
                      'columns': baseline['trajectory']['column_count'],
                      'source_sha256': baseline['source']['sha256']}))


if __name__ == '__main__':
    main()
