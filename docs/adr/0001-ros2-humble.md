# ADR 0001: Use ROS 2 Humble

- Status: Accepted
- Date: 2026-10-25

## Context

The vehicle stack is moving from ROS 1 Melodic to ROS 2. The available sensors
have ROS 2 drivers for Humble (LSLIDAR CX, LakiBeam, RealSense). The vehicle IPC
runs Ubuntu 18.04, but Docker containers carry their own userland, so the host
OS does not determine the ROS 2 distribution.

## Decision

Use **ROS 2 Humble** in the dev and runtime images.

## Consequences

- Best compatibility with the selected LiDAR drivers.
- LTS support through 2027.
- The dev host runs Ubuntu 24.04, but Humble runs inside the container; no
  native Humble installation is required.
- If a future dependency requires Jazzy, it must be evaluated separately; do not
  mix distributions between dev and vehicle.
