# Validation status

Snapshot date: November 4, 2026. Intended launch: November 7, 2026.

The three-module tracker is implemented and tested offline. Physical tracking qualification and final field calibration remain pending. The repository contains source, tests and sanitized regression configuration; it does not contain the private recordings or live hardware state.

## Publication verification

The standalone checkout passed 222 core Python tests, 83 manual-dashboard Python tests, 28 JavaScript viewfinder-control tests and the JavaScript geometry checks. These tests used simulated device boundaries and temporary local servers. No connected hardware was contacted. Core tracking source hashes match the tested simulation snapshot; see `docs/PUBLICATION_VALIDATION.json`.

## Optional manual limits update

Step and continuous manual jogs now work without custom limits. Confirmed user bounds, existing controller travel stops, Stop/lease behavior and fault handling remain. Before unconfirmed initialization clears the encoder reset marker, the profile acknowledgement is atomically invalidated and both file and directory are flushed. Restart cannot promote the saved draft. Automatic handoff and gateway reject unconfirmed bounds. The updated checkout passed 224 core Python tests, 91 manual-dashboard Python tests and 31 JavaScript viewfinder-control tests. No physical jog test was requested.

## Corrected failure mechanisms

- Expired control predictions previously removed the visual acquisition prior. Runtime now restores an association-only hint without extending prior-only motion lifetime.
- An unanchored singleton could become rocket identity when the other camera was ambiguous. Acquisition now needs spatial support, expected edge entry or independent camera agreement. Expired track rates cannot manufacture agreement between asynchronous frames.
- Small resolved recovery canopies could inherit rocket identity. Main inference now requires enough unclipped image detail to distinguish scale, rather than relying on pixel-scale drogue features.
- Integral effort accumulated against a downstream travel stop could carry into the larger main gain. Main/nonmain gain-mode changes clear integral memory while retaining measured rates, velocity ramps and existing ceilings.

A single camera without a valid hint or fresh track requires manual acquisition. Independently agreeing cameras can acquire without a prior. Flight-stage estimates remain explicitly labeled heuristics or saved-simulation priors, not a trained flight classifier.

## Full-flight ideal bounded simulation

| Measure | Noise seed 7 | Noise seed 17 |
|---|---:|---:|
| Original trajectory duration | 139.738 s | 139.738 s |
| Rendered main frames with correct target selected | 432/432 | 432/432 |
| Narrow COM visible | 427/432 (98.84%) | 426/432 (98.61%) |
| Narrow boresight intersects rendered body | 406/432 (93.98%) | 409/432 (94.68%) |
| Wrong target selections, full flight | 0 | 0 |
| Simulated travel violations | 0 | 0 |
| Premature visual main classifications | 0 | 0 |
| Main capture pointing error, p95 | 0.04114 deg | 0.04169 deg |

The plant uses ideal controller coordinates, hard travel stops, bounded speed/acceleration, 80 ms image delay and 70 ms command delay. It does not model measured backlash, slip, torque limits, structural compliance, operating temperature or real exposure/transport jitter. No trajectory angle or clock was rescaled to fit travel. Target truth is isolated from perception and prediction and attached only after decisions for evaluation.

The default body centroid is an uncertain COM estimate. The approximately 0.035 degree median model pointing error is not measured physical accuracy or a confirmed mass-center calibration. The 0.2 degree detection metric is an evaluation threshold, not a user acceptance requirement.

## Saved image replay and hardware boundary

The corrected saved-image evaluation selected the true wide target in 39 of 41 target-visible drogue frames with no drogue false target selection. Recorded poses reflect the original wrong turn and therefore cannot demonstrate a corrected main descent. MJPEG compression changes detections, so original telemetry and decoded-image replay metrics must remain separate.

The earlier hardware replay completed the original flight sequence with 2093 acknowledged commands, but tracking failed. Controller step counts did not establish physical output-angle truth. The corrected code has not yet completed a new physical replay. Hardware tests require separate start approval, preserved current limits, calibrated camera/encoder registration and measured timing.

The saved OpenRocket case includes a high-speed main deployment warning at 37.398 m/s. Its outer body model length is 2.4516 m versus an earlier approximate 2.75 m. Confirm the as-flown model and recovery arrangement before flight use.

## Reproduce software checks

Run the commands in README.md from a clean checkout. The portable bounded fixture supports the complete flight regression without recordings, credentials or a device connection. Core and manual-dashboard suites use fakes for their hardware boundaries. Native C++ tests compile the actual firmware handlers against stubs, not a real motor controller.
