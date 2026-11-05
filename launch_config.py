"""Validated, editable launch configuration. Nominal optics are for development."""
from pathlib import Path
import json
import math
import hashlib
from det_types import CameraModel
from field_config import load_config

ROOT = Path(__file__).resolve().parent


def load_launch_config(path=None):
    path = Path(path or ROOT/'launch_config.json').resolve()
    def finite(value):
        if value in ('NaN','Infinity','-Infinity'):
            raise ValueError('Nonfinite configuration value')
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result: raise ValueError('Duplicate config key: '+key)
            result[key]=value
        return result
    cfg=json.loads(path.read_text(),parse_constant=finite,object_pairs_hook=unique)
    if cfg.get('schema_version') != 1: raise ValueError('Unsupported launch configuration')
    cfg['_root']=str(path.parent)
    for key in ('field_config','trajectory'):
        cfg[key]=str((path.parent/cfg[key]).resolve())
        if not Path(cfg[key]).is_file(): raise ValueError('Missing '+key)
    load_config(cfg['field_config'])
    cfg['_field_sha256']=hashlib.sha256(Path(cfg['field_config']).read_bytes()).hexdigest()
    if cfg.get('payload_deployment_enabled') is not False: raise ValueError('First-flight payload deployment must be disabled')
    for key in ('latency_s','motor_timeout_s'):
        value=cfg.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0 < value <= .5:
            raise ValueError(key+' must be finite and within 0..0.5 seconds')
    cfg['camera_models']={name:CameraModel(name=name,**data) for name,data in cfg['cameras'].items()}
    if set(cfg['camera_models']) != {'narrow','wide'}: raise ValueError('Both narrow and wide cameras are required')
    for section in ('executor','predictor'):
        for name,value in cfg.get(section,{}).items():
            if isinstance(value,(int,float)) and (isinstance(value,bool) or not math.isfinite(value)):
                raise ValueError('Invalid '+section+'.'+name)
    return cfg


def live_readiness(cfg):
    issues=[]
    calibration=cfg.get('calibration',{})
    if calibration.get('status') != 'final': issues.append('Final camera and axis calibration is pending')
    if not calibration.get('reference'): issues.append('Calibration reference is missing')
    if not all(camera.calibrated for camera in cfg['camera_models'].values()):
        issues.append('Both camera models must be calibrated')
    if not cfg.get('encoders',{}).get('calibrated'): issues.append('Encoder axis registration is pending')
    for axis in ('az','el'):
        values=cfg.get('encoders',{}).get(axis,{})
        if values.get('sign') not in (-1,1): issues.append('Encoder sign missing for '+axis)
        for key in ('zero_deg','offset_deg'):
            value=values.get(key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                issues.append('Encoder '+key+' missing for '+axis)
    field=load_config(cfg['field_config']).to_dict()
    current_hash=hashlib.sha256(Path(cfg['field_config']).read_bytes()).hexdigest()
    if current_hash != cfg.get('_field_sha256'):
        issues.append('Field geometry changed after loading; restart the tracker to consume it')
    for key in ('pad_horizontal_distance_m','pad_bearing_deg','pad_height_relative_to_tracker_m'):
        if field['measurement_status'][key] != 'measured': issues.append('Measure field '+key)
    if cfg.get('flight_configuration_confirmed') is not True: issues.append('Confirm as-flown OpenRocket configuration')
    return issues
