# DET Launch Stack Implementation Plan

> For agentic workers: implement independent components against the shared contract, then review and test the complete stack. Existing user architecture and latest request authorize execution.

**Goal:** Deliver a runnable Mac stack, simulation and development tests for the November 7 launch, preserving a clear boundary around physical validation.

**Architecture:** Perception, prediction and execution are separate Python modules connected by timestamped shared data types. Synthetic/replay/live sources feed the same runtime. HTTP I/O is isolated and defaults to dry run.

**Tech Stack:** Python 3.9+, NumPy, OpenCV, standard-library HTTP and unittest.

**Spec:** docs/DESIGN.md

## Global constraints

- All algorithms on Mac; first flight rocket only, main rocket COM, future payload canopy disabled.
- 500 m editable nominal geometry; existing manual jog unaffected.
- No hardware commands during development. Final calibration required for autonomous live mode.
- No simulator truth in algorithm observations. Never equate detection failure with proven target outside frame.

## Review focus

- Cross-host clocks, stale/duplicate data and failed network calls must not create old commands.
- A bright distractor or canopy cannot silently replace the body under main.
- Main uncertainty must remain visible and support explicit manual phase override.
- Motor saturation, link loss and user stop must not leave a persistent velocity.
- Changed geometry/calibration must be validated and actually consumed by the runtime.

## Tasks

- [x] Define shared pose, frame, detection, phase, prediction and command interfaces and geometry transformations. Test known rays and wrap boundaries.
- [x] Build fast dual-camera detection, temporal association, body COM estimate and slower phase estimate. Test distractors, loss, wide reacquisition and phase/source handling.
- [x] Build prior-assisted angular predictor and encoder feedback executor. Test latency, missing detections, wrapped bearings, stale encoders, saturation and stop behavior.
- [x] Build live capture/encoder adapter and exclusive leased Pi velocity gateway. Test real local HTTP, metadata age, malformed frames, duplicate commands and expiry using fake serial hardware.
- [x] Build same-path synthetic closed-loop simulation, video replay, local dashboard and CLI. Test end-to-end integration with no algorithm access to evaluation truth.
- [x] Run component tests, complete synthetic flights, HTTP/dashboard smoke checks and independent review. Package instructions, launch preparation and current limitations. Update Obsidian.

Software tasks are complete and verified offline. Firmware deployment, final calibration and physical launch qualification are outside the completed development results and remain pending. Browser visual inspection was blocked by the origin approval gate; HTTP behavior and inline JavaScript were checked.
