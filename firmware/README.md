# SKR Pico leased velocity firmware

Current DSTAT and VEL_LEASE_V1 source for the RP2040 SKR Pico. Only source and host test stubs are published. Generated UF2 files, vendor dependencies and installed-flash backups are excluded. Prior local builds used Arduino RP2040 core 6.0.0 and TMCStepper 0.7.3. Rebuild and verify the exact source on the intended board before any separate approved installation.

## Protocol

| Request | Reply | Meaning |
|---|---|---|
| `CAP` | `OK CAP VEL_LEASE_V1` | Required capability; automatic gateway refuses legacy firmware. |
| `LSTAT` | `OK LSTAT active tripped tick_us` | Atomic lease flags and the Pico's own unsigned microsecond clock. |
| `LVEL seq ttl_ms ref_us az el` | `OK LVEL seq ttl_ms ref_us` | Atomically apply both signed velocities with an existing MCU-clock deadline. |
| `STOP` | Existing `OK STOP` | Brake, then clear the lease only after both planned velocities are zero. |
| `DSTAT` | `AZ drvst=0xHEX en_pin=0`, `EL drvst=0xHEX en_pin=0`, `OK DSTAT` | One cached driver status read per axis and current enable pin. |

`seq` is an increasing unsigned 32-bit integer within a STOP session. Duplicates and older sequences do not refresh the deadline. `ttl_ms` is 1 through 500. `ref_us` is the previously returned LSTAT tick. Velocities must be finite and within the unchanged 15 degrees/second firmware ceiling. Unknown, malformed, expired, future-reference, or out-of-order commands are rejected. The gateway sends no unleased `VEL` commands.

The gateway reads LSTAT, then calculates the Pi command lifetime remaining after that response. The firmware uses `ref_us + ttl_ms*1000`, rather than giving a delayed packet a new lifetime when received. No host clock is subtracted from a Pico clock. Unsigned age checks and signed deadline comparisons handle the hardware timer wrap for these short leases.

The existing timer ISR checks expiry at the 1 kHz planner rate, independently of the USB command loop. Expiry latches the session, ignores stale tracking reference error, ramps velocity toward zero at the configured acceleration, and holds the resulting counted position. At the gateway's 75 degrees/second squared setting, the nominal ramp from the 15 degrees/second ceiling takes about 0.2 seconds after expiry. This is a planner calculation, not a measured physical stopping distance.

Only CAP, LSTAT, PING, STAT, DIAG, DSTAT, STOP and EST are available while a lease is active or tripped. Legacy manual motion remains unchanged outside a leased session. STOP preserves an expired latch if its existing 300 ms wait is insufficient at a previously selected low acceleration. The gateway requires both stable counts and cleared LSTAT flags before reporting a confirmed Stop. A remaining latch or another controller fault blocks automatic operation and is never cleared through CLR.

The gateway keeps full DIAG checks during initialization, then requires a valid DSTAT response before becoming ready. While tracking it polls DSTAT every 0.5 seconds, replacing the full DIAG whose repeated UART reads were measured at about 108 ms in the first HIL trial. DSTAT reads DRV_STATUS exactly once per axis; status helpers would each reread the same register. The gateway rejects zero/all-ones reads (TMCStepper 0.7.3 connection-failure values), disabled output pins, and any of the original `0x3F` mask bits: overtemperature warning/shutdown and four short-circuit flags. Open-load flags remain allowed. The complete three-line response is parsed strictly. Runtime configuration registers such as microsteps/toff are checked at initialization, while temperature and short-circuit monitoring continues at the original cadence. The actual DSTAT UART latency requires a hardware measurement.

## Reproduce the build

From the DET launch project directory, with Arduino RP2040 core 6.0.0 and TMCStepper 0.7.3 installed:

```sh
arduino-cli compile --fqbn rp2040:rp2040:rpipico --build-path /private/tmp/det-launch-dstat-build firmware/gimbal_skr_pico
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'
```

The native C++ test exercises the same lease helper included in the firmware: expiry, duplicate/stale sequences, an expired prior lease, timer wrap, future/expired references, delayed serial delivery, explicit Stop reset, and braking in both directions. Another native test compiles the actual sketch against hardware stubs and verifies DSTAT makes one read per axis without altering lease/motion state, including an active or expired lease. The full RP2040 sketch also compiles. These checks do not establish USB timing, ISR scheduling under physical load, motor torque, encoder registration, or physical rest.

## Gateway startup after separate firmware installation and bench verification

Stop the manual motion service before opening the serial device. Both gateways request exclusive serial ownership, and the automatic gateway fails if the port is already owned. On the Pi, run:

```sh
python3 pi_tracking_gateway.py --port 8084
```

It binds only `127.0.0.1:8084`. Startup performs CAP, PING, STAT, DIAG, STOP, ENC 0, MODE SLEW, ACC 75, DIAG, DSTAT, STOP and validation. It sends no motion, zero, homing, current-setting or fault-clear commands. Forward a Mac loopback port to this Pi loopback port over the existing authenticated SSH connection. Use `HttpMotor(base_url, timeout_s=.05)` with the launch executor's 0.25 second command lifetime only when bench measurements support that transport budget. The client binds a session generation on construction; Stop or any fault requires deliberate construction of a new client, and faults on the Pi require recovery and service restart.

Pi software expiry and MCU expiry are separate checks. The MCU check covers lost host updates while its firmware and motor electronics continue operating. It does not claim physical closed-loop control or protection against a failed MCU, disabled interrupts, loss of motor holding power, or unmodeled mechanics. Independent AS5600 validity and final calibration remain prerequisites in the Mac runtime.
