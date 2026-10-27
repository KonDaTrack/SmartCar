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
```

No hardware is required for CI-style unit tests. Keep hardware tests as manual
runbooks until the driver layer is stable.
