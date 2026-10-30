# Tests

Layered test plan:

1. **Unit tests** — pure functions, message parsing, config validation.
2. **Bag replay** — replay a short bag and run the SLAM/driver stack offline;
   compare topic rates, TF tree and SLAM output.
3. **Live bench** — dev machine on the vehicle LAN, real sensors, RViz.
4. **Vehicle integration** — `sensors` + `slam` containers on `.10`.
5. **Long run** — at least one hour, monitor CPU/RAM/disk/temperature.

Useful commands:

```bash
# inside the dev container
colcon test --merge-install
colcon test-result --verbose

# record a short bag on the dev machine
./scripts/record.sh

# replay it
./scripts/replay.sh 20261025_120000

# validate the wheel-IMU offset estimator with a synthetic +50 ms offset
python3 /ws/tests/make_sync_test_bag.py /bags/sync_test_bag
python3 /ws/tools/estimate_offsets.py /bags/sync_test_bag --wheel-base 1.2
# expected: best wheel-IMU lag: +50.0 ms
```

No hardware is required for CI-style unit tests. Keep hardware tests as manual
runbooks until the driver layer is stable.
