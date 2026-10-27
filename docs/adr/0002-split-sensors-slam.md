# ADR 0002: Start with separate `sensors` and `slam` containers

- Status: Accepted
- Date: 2026-10-25

## Context

Production SLAM runs on the vehicle. SLAM parameters and code will be iterated
frequently, while sensor drivers are comparatively stable. A single container
is simpler, but restarting SLAM would also restart the sensors.

## Decision

Start with two containers on the vehicle:

- `sensors` — LSLIDAR C16, LakiBeam left/right, VX1000; chassis initially here.
- `slam` — SLAM/localization/map.

Both use the **same runtime image** and modular launch files. `chassis` will be
split into a third container when the chassis node is stable. A combined
`vehicle` compose profile remains for debugging.

## Consequences

- SLAM can be restarted without interrupting the sensor drivers.
- Clear topic/launch/config boundaries from day one.
- Docker overhead is negligible; the cost is a slightly larger compose file.
- Later splitting `chassis` is a compose/launch change, not an image change.
