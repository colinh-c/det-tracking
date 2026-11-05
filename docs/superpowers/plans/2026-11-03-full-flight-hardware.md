# Full Flight Hardware Replay Implementation Plan

> **For agentic workers:** Use subagent-driven-development for the two independent modules below. Keep checks focused on hardware command ownership and actual trajectory timing.

**Goal:** Prepare a real-time OpenRocket synthetic-image replay through the existing three tracking modules to the actual gimbal, while preserving dashboard Stop and manual takeover.

**Architecture:** A Mac handoff proxy owns manual/automatic service transitions. The replay uses the existing SyntheticFlight renderer and TrackingEngine, with fresh Pico step counts as explicitly unqualified bench feedback and raw AS5600 diagnostics logged separately. Preparation never claims motors or starts a sweep.

**Tech Stack:** Existing Python 3.9 environment, OpenCV/NumPy, local HTTP, authenticated existing SSH, existing Pi manual and leased automatic gateways.

**Spec:** Colin's November 3 requests in this chat: enable motor movement; prepare actual-flight-rate synthetic end-to-end testing; keep manual access available after testing; use real gimbal rather than a software plant.

## Global Constraints

- No ZERO, SYNC, homing, firmware writes or new travel limits.
- Existing 15 degrees/s and 75 degrees/s2 ceilings remain. Preserve 1x elapsed wall time and original trajectory; report saturation rather than slowing time.
- Encoder magnetic status and output-angle registration remain unqualified. Explicit step-counter feedback is a bench mode and cannot finalize flight calibration.
- Only an explicit execute action claims motors. Manual jog or global Stop revokes automatic control. Cleanup restores manual availability and cannot cancel a newer manual owner.
- Keep dashboard 18182 and motion endpoint 18083. No alternate UI origins.
- No full-flight physical sweep during preparation.

## Review Focus

1. Late automatic requests after manual takeover must fail the prior generation.
2. Stop must remain reachable while a tracking request is in flight.
3. Failed/cancelled playback must restore manual controls without stopping subsequent user motion.
4. Counter samples must be fresh and tied to the claimed automatic session; they are not physical output-angle truth.
5. Processing overruns must skip real-time samples or fail safely, never silently slow the trajectory.

## Task 1: Control handoff and fresh pose endpoint

Files: create `control_handoff.py`; narrowly extend `pi_tracking_gateway.py` with read-only fresh pose sampling; focused tests for takeover and owner-fenced cleanup.

Interface: `HandoffClient(base_url).claim() -> {session, motor_base_url, position}`; `pose(session) -> {position, age_s, generation}`; `release(session)`.

HTTP proxy 18083 forwards manual traffic to 18086 (Pi 8083). An explicit `/flight/claim` starts automatic Pi 8084 only after the manual service releases serial. During automatic playback, global Stop or a fresh manual jog takes over, restores manual service, and forwards the user's command. Scoped stale hold messages must not cancel a newer automatic session. `/flight/release` is session fenced. Forward automatic motor traffic at 18084 and encoder diagnostics at 18085.

- [ ] Pin essential takeover/cleanup behavior with narrow tests.
- [ ] Implement the proxy, client and fresh pose endpoint.
- [ ] Verify ready manual mode on the connected device without initiating a move.

## Task 2: Full flight hardware runner and prepared profile

Files: create `full_flight_hil.py`, `full_flight_hil_config.json`, and `Start Full Flight Hardware Test.command`.

Use the existing OpenRocket renderer's stage/size changes and TrackingEngine's prior, phase estimation, reacquisition and executor. Convert fresh controller counts relative to claim origin into an explicitly assumed launch-pad world reference; retain raw encoders as diagnostics. Prime capture/position history before starting the launch clock. Use the original 139.738 second trajectory at 1x wall time. Final input frames, commands, counter feedback, raw encoder health and quality labels must be recorded. A configurable synthetic frame delay may use the measured camera publication-age estimate; distinguish it from a real optical exposure.

- [ ] Pin 1x timing, no simulated motor plant, no truth-to-detector leakage and cleanup on cancellation with narrow tests.
- [ ] Implement prepare-only default and explicit real-motor execution.
- [ ] Produce a preparation report using live manual-link availability and source demand; leave manual controls enabled.

## Completion

Read-only live preparation must show manual controls available, expected full-flight span/rate demand, and the exact executable test entry point. Do not claim the complete physical flight sweep has run.
