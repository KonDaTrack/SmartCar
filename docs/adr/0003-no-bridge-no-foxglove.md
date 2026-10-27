# ADR 0003: Full ROS 2, no ROS 1 bridge, no Foxglove

- Status: Accepted
- Date: 2026-10-25

## Context

The project is moving entirely to ROS 2. The legacy ROS 1 autox stack will not
be used by the new runtime. The camera is not needed for SLAM and will not be
recorded. Visualization is required on the development machine only.

## Decision

- Do **not** build or run a `ros1_bridge` image/container.
- Do **not** install Foxglove.
- Use RViz on the development machine in a Docker container (X11 + RTX 5060).
- Do **not** start the RealSense driver in the runtime stack.
- Ensure the ROS 1 sensor drivers are stopped so they do not compete for the
  LiDAR/chassis UDP ports.

## Consequences

- Simpler image and compose setup.
- The ROS 2 stack must cover localization/planning/control that the ROS 1 stack
  previously provided; this is a separate implementation effort.
- Visualization is a DDS subscriber, so the same bandwidth rules apply as for
  recording.
