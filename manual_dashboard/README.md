# Manual dual viewfinder

The Mac runs viewfinder.py with geometry.js, viewfinder.html and calibration.py. The Pi runs motion_gateway.py for manual finite/continuous jogs and explicit user-defined travel limits, and either the native csi_camera_gateway.py or the tracker capture gateway for camera/encoder data. Do not run both capture services simultaneously.

Use the root README and docs/DEPLOYMENT.md for configuration. Tests are local fakes and do not require a device. Raw captures and calibration results are intentionally excluded.
