# Roadmap

## M0 - Skeleton and workflow (current)

- [x] Repository layout, Dockerfile, compose files, release/deploy scripts
- [x] Network/sensor/workflow documentation
- [x] Placeholder launch files and keepalive node
- [ ] Initial commit and GitHub private remote
- [ ] First `make build-dev` / `make dev` on the dev machine

## M1 - Sensor drivers

- [ ] LSLIDAR C16 ROS 2 driver in `sensors` container
- [ ] 2x LakiBeam1 drivers (`/scan_left`, `/scan_right`)
- [ ] VX1000 serial driver (`/VX1000_imu`, `/VX1000_gnss`)
- [ ] Verify rates/bandwidth on the vehicle LAN
- [ ] Record a short LiDAR + IMU bag on the dev machine

## M2 - Chassis

- [ ] `smartcar_chassis` ROS 2 node for `.110` UDP CAN frames
- [ ] Publish `/vehicle_status`; port wheel-speed odometry if needed
- [ ] Decide whether to split `chassis` into its own container

## M3 - SLAM offline

- [ ] Choose SLAM algorithm (FAST-LIO2 / Point-LIO / other)
- [ ] Run SLAM on recorded bags on the dev machine
- [ ] Benchmark CPU/RAM on the vehicle with a short bag
- [ ] Calibrate LiDAR-IMU extrinsics and time offset

## M4 - Vehicle live SLAM

- [ ] Deploy runtime image to `.10`
- [ ] `sensors` + `slam` containers live
- [ ] RViz on dev machine over DDS
- [ ] Verify 100M/1G bandwidth and CPU headroom

## M5 - Operations

- [ ] systemd autostart
- [ ] healthcheck and log rotation
- [ ] release/rollback drill
- [ ] long-run stability test
