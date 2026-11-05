# Deployment configuration

The committed configuration is nominal and intended for offline development. Run `launch.py --check` before live operation. Final camera calibration, output-axis encoder registration, actual site geometry, measured latency and confirmed travel limits are separate prerequisites.

## Mac and Pi roles

The Mac runs the tracker and viewfinder. The Pi supplies camera/encoder HTTP data and one exclusive motor gateway. The SKR Pico uses the firmware under `firmware/gimbal_skr_pico/`. Pi runtime needs its working Picamera2/libcamera, Pillow, smbus and pyserial environment. The faster JPEG path optionally uses simplejpeg. Do not install Pi camera packages into the Mac environment.

Use authenticated SSH forwarding to expose only loopback endpoints on the Mac. Enroll and verify the intended Pi's SSH host key separately; the code retains strict host checking. No SSH credentials, known-host entries or USB console password helper are published.

## Host and path settings

The handoff code reads these environment variables, also available as CLI options when starting `control_handoff.py`:

| Variable | Default or meaning |
|---|---|
| `DET_SSH_HOST` | `pi@raspberrypi.local`; use the actual trusted user and host |
| `DET_SSH_HOST_KEY_ALIAS` | Optional existing known-host alias |
| `DET_PI_ROOT` | `/home/pi/gimbal`; absolute path on the Pi |
| `DET_MANUAL_GATEWAY` | Optional absolute path, otherwise `<root>/manual_dashboard/motion_gateway.py` |
| `DET_AUTO_GATEWAY` | Optional absolute path, otherwise `<root>/pi_tracking_gateway.py` |
| `DET_MOTOR_DEVICE` | Pi gateway process environment only; `/dev/ttyACM0` by default. Prefer the intended board's stable by-id device |
| `DET_USB_HOST_MAC` | Explicit Mac-side USB network interface MAC for the optional reconnect helper |
| `DET_PI_LINK_LOCAL` | Explicit Pi IPv6 link-local address without an interface suffix |

`.env.example` contains nonsecret examples. Python does not load it automatically. Export the selected variables in the shell used to start the tools. Keep local settings in ignored `.env` files if desired, and keep credentials in the existing SSH agent or credential store.

`DET_MOTOR_DEVICE` must be set in the Pi gateway's own service environment. The Mac handoff does not forward that variable over SSH. Configure the Pi user service environment before using a nondefault device; exporting it only on the Mac does not select the remote board.

The reconnect helper finds the Mac interface by the configured USB MAC and scopes the configured Pi IPv6 address to it. With either USB setting absent, it reports `not_configured` before probing. `tools/reconnect_pi.py --once` prints a proposed command without connecting. The helper restores only camera forwarding; it does not start a motion service or claim motors.

## Expected loopback ports

| Mac | Pi | Purpose |
|---|---|---|
| 18082 | 8082 | Camera and encoder capture |
| 18086 | 8083 | Existing manual gateway behind the handoff relay |
| 18084 | 8084 | Automatic gateway |
| 18085 | 8085 | Optional diagnostics endpoint |
| 18083 | Mac relay | Manual/automatic ownership handoff |
| 18182 | Mac viewfinder | Joint manual camera dashboard |

Port 8085 is optional and is not the current full-flight runner's raw AS5600 source. Its configured source is the capture gateway's `/encoders` route on port 18082.

The manual and automatic gateways must never own the motor serial device simultaneously. Starting the handoff relay alone never claims motors. A hardware run requires the explicit execution path, healthy current limits and operator approval. Manual takeover and Stop revoke automatic ownership. Do not call bootstrap or execute paths merely to run offline tests.

## Limits, calibration and firmware

The Pi gateways default to `~/gimbal/motion_limits.json` under their runtime user. Enter travel limits through the dashboard and explicitly Apply them in the current controller coordinate frame. A reset requires confirmation; the software does not silently reuse old coordinates, home, zero or synchronize the mount. Manual jogging does not require camera calibration or zeroing.

The committed camera model uses a nominal 400 mm narrow lens and a 16 mm wide placeholder. Match final intrinsics to actual capture size, focus, boresight and roll. Confirm AS5600 bus-to-axis mapping, sign, reference and magnet validity before absolute automatic tracking. A detected magnet alone does not establish output-angle calibration.

Build firmware separately with the documented board core and library. Preserve and verify the actual installed flash backup before an approved firmware change. Repository setup, tests and publication do not flash a board or establish physical launch readiness.
