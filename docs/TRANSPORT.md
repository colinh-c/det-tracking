# Mac and Pi transport boundary

The Mac uses capture HTTP port 8082 and automatic motor HTTP port 8084, forwarded over the existing authenticated SSH link. Both Pi services bind loopback only. Mac defaults use matching loopback ports; change launch_config.json if the tunnel uses other ports.

PiSource fetches both camera frames and cached encoders concurrently. Camera time is estimated conservatively as Mac receive time minus reported Pi cache age minus request round trip. The Pi timestamp itself is never subtracted from a Mac clock. The camera cache age begins at array receipt before JPEG encoding, not optical exposure. Encoder wall-time sample IDs are opaque duplicate identifiers, not shared clocks.

Each camera frame retains its capture-time estimate and exposure-aligned encoder pose. Angle interpolation handles wrap; default maximum sample gap is .25 seconds and maximum extrapolation .05 seconds. Frames without a reliable matching history have invalid pose and are rejected for tracking. Geometry must exactly match calibrated image dimensions. Camera or encoder errors, weak/strong fields and stale data cannot silently become valid measurements.

The capture service waits .02 seconds after its two isolated bus reads and marks data stale after .25 seconds. That is not a promise of 50 Hz: process startup and I2C time add to the cycle. Measure actual cadence before flight. The read-only live view remains available with nominal calibration; automatic arming requires final registration and measured geometry.

HttpMotor binds one generation from GET /status when intentionally constructed. It requires the firmware watchdog flag. Each update sends a strictly increasing command sequence and a relative lifetime that conservatively subtracts the configured HTTP timeout. The default .05 second timeout is a provisional transport allowance paired with .25 second command validity; physical latency must support it. Commands are never retried. An old client cannot acquire a new token after Stop. Exceptions disarm the runtime, and a Pi fault requires deliberate recovery/service restart.

The Pi validates finite velocities, command age, lifetime and ownership. A watchdog thread stops if Mac updates expire, independent of new requests. Stop revokes the generation before waiting on serial and verifies acknowledgements, stable motor counters and a cleared firmware lease latch. Stable counters are not independent evidence of physical rest.

The launch firmware separately enforces a microsecond-clock deadline in its 1 kHz timer ISR. LSTAT supplies an MCU clock reference; LVEL carries that reference, remaining lifetime, sequence and both velocities atomically. Delayed or duplicate packets do not refresh a lease. Expiry brakes at configured acceleration and latches until STOP at rest. No legacy VEL fallback exists in the automatic gateway. See ../firmware/README.md for protocol, compiled build, source backup and remaining physical boundaries.

Exactly one service owns the SKR Pico serial port. Stop the automatic service before handing control back to the existing manual gateway. The existing manual jog does not require zeroing or camera calibration. Automatic absolute tracking requires final encoder and optical registration.

Recording uses schema version 2 and preserves each frame's own timestamp, sequence and capture pose. Each raw video has one slot per sidecar row; a missing view has a black slot and no observation. Replay skips that observation and retains alignment. Corrupt or legacy recordings fail explicitly. No replay path constructs a motor client.
