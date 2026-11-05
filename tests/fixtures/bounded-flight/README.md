# Bounded flight regression fixture

This is a deterministic software regression fixture. It is not current hardware state, measured physical tracking evidence, or a configuration for a live device.

The two JSON files contain only the timing, nominal camera models, algorithm settings, initial simulated controller coordinates and fixed travel bounds needed by the offline evaluator. Their numerical values reproduce the bounded flight test case. No recorded telemetry, videos, encoder readings, device identifiers, session tokens, network endpoints or private filesystem paths are included.

The trajectory and field geometry are the repository files referenced by paths relative to this fixture directory. The simulation keeps the complete 139.738 second trajectory at its original speed, with a 15 deg/s velocity ceiling, a 75 deg/s^2 free motion acceleration ceiling, an 80 ms capture delay and a 70 ms command delay. The target can leave mount travel during ascent and return during descent.

Tests require correct reacquisition after the long loss, no fixed distractor lock, more than 95% main target visibility in the narrow image, and more than 90% main boresight intersection with the rendered body. These are regression criteria for this ideal software fixture, not physical flight acceptance requirements.

From the repository root, run:

```sh
python -m unittest tests.test_offline_bounded_flight -v
```

To save an offline simulation in a new directory:

```sh
python offline_bounded_flight.py --saved-run tests/fixtures/bounded-flight --output runs/bounded-flight
```

The evaluator refuses to overwrite an existing output directory. Its CLI prohibits network access.
