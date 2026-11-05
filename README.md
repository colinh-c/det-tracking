# DET rocket tracking

Dual-camera rocket tracking for a Raspberry Pi 5 and SKR Pico gimbal. The Mac runs perception, prediction and control; the Pi captures images and encoders; the Pico generates motor steps.

Before main deployment, the tracker prioritizes keeping the correct rocket in view and reacquiring it with the wide camera. During main descent it aims at the estimated rocket center of mass. The first-flight configuration tracks the rocket throughout flight; payload separation is disabled.

## Run offline

Development was tested with Python 3.9. Create an environment and install the pinned Mac dependencies:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python launch.py --mode simulate --run-dir runs/demo
```

For a simulated dual-camera display:

```sh
.venv/bin/python launch.py --mode simulate --serve --start-s 110
```

Open the local URL printed by the launcher. `Start Simulation.command` is also available on macOS.

Run the complete saved trajectory with the included bounded regression fixture:

```sh
.venv/bin/python offline_bounded_flight.py \
  --saved-run tests/fixtures/bounded-flight \
  --output runs/bounded-demo
```

This is entirely offline. The fixture contains configuration and benchmark geometry, not recordings or current device state. The simulated plant retains the original 139.738 second trajectory, travel stops, 80 ms image delay, 70 ms command delay, 15 degrees/s speed ceiling and 75 degrees/s squared free-motion acceleration ceiling.

For a separately recorded hardware session with image sidecars:

```sh
.venv/bin/python offline_hil_replay.py --input /path/to/saved-run --output runs/image-replay
```

Image replay is read-only and uses the recorded camera path. Lossy recordings can change detections relative to the original uncompressed inputs.

## Architecture

| Component | Source |
|---|---|
| Image detection, association and phase heuristics | `perception.py` |
| Angular filter and saved OpenRocket prior | `prediction.py` |
| Measured-pose and narrow-image control | `execution.py` |
| Shared runtime and display | `runtime.py`, `launch.py`, `dashboard.py` |
| Offline image and closed-loop evaluation | `offline_hil_replay.py`, `offline_bounded_flight.py` |
| Pi capture and encoder transport | `pi_capture_gateway.py`, `encoder_reader.py` |
| Pi automatic motion gateway | `pi_tracking_gateway.py` |
| Manual dual viewfinder, wall calibration, jog and limits | `manual_dashboard/` |
| Explicit manual/automatic ownership handoff | `control_handoff.py` |
| SKR Pico leased velocity and DSTAT firmware | `firmware/gimbal_skr_pico/` |
| Optional camera tunnel reconnect helper | `tools/reconnect_pi.py` |

The older native capture gateway used by the manual viewfinder is retained in `manual_dashboard/csi_camera_gateway.py`. The tracker capture gateway adds capture-generation provenance and JPEG timing metadata. Do not run two camera gateways against the same camera hardware.

## Tests

These use simulated devices, temporary local HTTP servers, generated imagery and host firmware stubs. They do not require a connected gimbal:

```sh
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m unittest discover -s manual_dashboard -p 'test_*.py' -v
node manual_dashboard/test_geometry.js
node manual_dashboard/test_viewfinder_hold.js
```

The firmware tests compile native C++ fixtures, so a C++ compiler must be available. Pi runtime additionally needs its Picamera2/libcamera, Pillow, smbus and pyserial environment. `simplejpeg` is optional for the faster JPEG path. Those Pi system packages are not needed for the Mac offline tests.

## Field configuration and hardware operation

`launch_config.json` selects camera models, encoder registration, control settings and the trajectory. `field_config.json` contains editable site geometry, nominally 500 m pad distance and zero relative elevation. `field_setup.html` can edit and export that configuration. Recheck the actual lens, capture dimensions, orientation and boresight; the 16 mm wide lens remains a placeholder in the development configuration.

The committed camera and encoder calibration flags are intentionally nominal. `launch.py --check` reports live prerequisites. A brightness/body centroid is labeled uncertain COM until verified body orientation supports a center-of-mass correction. Simulation does not establish optical calibration, encoder axis registration, real pipeline latency or physical stopping accuracy.

Use the existing manual viewfinder with already configured local forwards:

```sh
.venv/bin/python manual_dashboard/viewfinder.py \
  --source http://127.0.0.1:18082 --port 18182 \
  --motion-source http://127.0.0.1:18083
```

The manual service runs on the Pi. Its movement limits are entered through the dashboard and explicitly applied. A reset coordinate frame requires limits confirmation. Manual jogging does not require zeroing or final camera calibration. Exactly one Pi gateway may own the motor serial device. Set `DET_MOTOR_DEVICE` in the Pi gateway's service environment to the intended controller, preferably its stable `/dev/serial/by-id/...` path. The Mac handoff does not forward this variable.

Deployment hosts and paths are environment settings, documented in [deployment.md](docs/DEPLOYMENT.md). No credentials or known-host entries are bundled. Live automatic tracking and the full-flight hardware runner are separate explicit actions; obtain operator approval before starting a hardware test. Repository publication does not run, deploy, flash or arm the hardware.

## Validation status

The November 4 source fixes unsupported distractor acquisition after long loss, tiny-canopy identity confusion, premature main inference at pixel scale, and retained integral effort at main control handover. In two ideal bounded simulations, the target remained in the narrow view for 98.84% and 98.61% of rendered main descent. Both selected the correct target throughout main, with no wrong-target selections or simulated travel violations. The boresight intersected the rendered body for 93.98% and 94.68%.

These are simulation results. The recorded earlier physical replay completed but failed tracking. A physical retest of the corrected code and final field calibration remain pending. See [VALIDATION.md](VALIDATION.md) for result definitions and limitations.

Recordings, telemetry, credential helpers, private notes, flash backups, vendor trees, environments and compiled artifacts are excluded from this repository.
