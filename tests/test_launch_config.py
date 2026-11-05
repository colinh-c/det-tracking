import unittest
import tempfile
import json
import hashlib
from pathlib import Path
from launch_config import ROOT,load_launch_config,live_readiness

class ConfigTests(unittest.TestCase):
    def test_nominal_configuration_is_explicitly_not_live_ready(self):
        cfg=load_launch_config()
        issues=live_readiness(cfg)
        self.assertTrue(issues)
        self.assertTrue(any('calibration' in item.lower() for item in issues))
        self.assertEqual(cfg['camera_models']['narrow'].width,640)
    def test_live_readiness_consumes_measured_geometry_and_calibration(self):
        cfg=load_launch_config()
        with tempfile.TemporaryDirectory() as tmp:
            field=json.loads(Path(cfg['field_config']).read_text())
            field['measurement_status']={key:'measured' for key in field['measurement_status']}
            path=Path(tmp)/'field.json';path.write_text(json.dumps(field));cfg['field_config']=str(path)
            cfg['_field_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
            cfg['calibration']={'status':'final','reference':'bench-and-stars-example-not-real'}
            cfg['flight_configuration_confirmed']=True;cfg['encoders']['calibrated']=True
            for camera in cfg['camera_models'].values():camera.calibrated=True
            self.assertEqual(live_readiness(cfg),[])
    def test_duplicate_config_keys_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'bad.json';path.write_text('{"schema_version":1,"schema_version":1}')
            with self.assertRaises(ValueError):load_launch_config(path)

    def test_geometry_change_after_engine_load_requires_restart(self):
        cfg=load_launch_config()
        with tempfile.TemporaryDirectory() as tmp:
            field=json.loads(Path(cfg['field_config']).read_text())
            field['geometry']['pad_horizontal_distance_m']=650
            path=Path(tmp)/'changed.json';path.write_text(json.dumps(field));cfg['field_config']=str(path)
            self.assertTrue(any('restart' in item for item in live_readiness(cfg)))
