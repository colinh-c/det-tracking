# Pi capture provenance

pi_capture_gateway.py is derived from the existing DET_Calibration_2026-10-27/csi_camera_gateway.py. Changes are the launch description and configurable encoder interval/stale threshold, defaulting to .02/.25 seconds. encoder_reader.py is an unchanged copy of the existing bounded subprocess reader. No original file was changed.

The .02 second setting is a wait after the two bus reads, not a guaranteed 50 Hz encoder rate. Actual subprocess and I2C timing must be benchmarked. The Mac rejects frames without sufficiently aligned valid encoder history. Camera age begins at capture_array receipt, before JPEG encoding. It is not a synchronized physical exposure timestamp. Measured capture/exposure and transport latency must be included in final calibration.
