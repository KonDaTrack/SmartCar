# Troubleshooting

## DDS: machines cannot see each other

- Both sides use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`.
- Bind DDS to the wired interface, not WiFi:
  - dev: `enp5s0`
  - vehicle: `enp1s0`
- With Fast DDS, use a profile with an interface whitelist, or set
  `ROS_STATIC_PEERS` to the peer IP.
- With CycloneDDS, set `<NetworkInterface name="enp5s0"/>` in `CYCLONEDDS_URI`.
- Check with `ros2 topic list`, `ros2 topic hz`, `ros2 doctor`.

## Sensor ports are already in use

```bash
ss -lunp | grep -E '2368|2369|2468|2568|8002|4001|4002'
```

ROS 1 and ROS 2 drivers must not run simultaneously on the same sensor.
Stop the ROS 1 drivers before starting the ROS 2 stack.

## VX1000 serial device

- Prefer the stable path:
  `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0029FI4-if00-port0`
- Map it into the container as `/dev/ttyUSB0`.
- The user/container needs access to the device (usually `dialout`).
- Verify data with a 460800 baud read; VX1000 frames start with `AA 55`.

## RViz on the dev machine

- X11: `DISPLAY=:1`, mount `/tmp/.X11-unix`, run `xhost +si:localuser:root`.
- GPU: use `runtime: nvidia` / `--gpus all`; if GLX fails, add
  `--device /dev/dri` (Mesa/AMD) or `LIBGL_ALWAYS_SOFTWARE=1`.
- On a 100M link, visualize a downsampled cloud; the full C16 cloud is
  30–50 Mbps per subscriber.

## Bandwidth

```bash
ros2 topic bw /lslidar_point_cloud
ros2 topic hz /lslidar_point_cloud
ifstat -i enp1s0 1
```

Full-resolution visualization + full-resolution bag recording over 100M may
drop packets. Use a downsampled visualization topic or record without live viz.

## Docker / Compose on the vehicle

- Ubuntu 18.04 `docker.io` is 20.10; it can load images built by Docker 29.
- Install the Compose v2 plugin (see `docs/deploy.md`); `docker-compose` v1 is
  too old for the compose files.
- `docker load` needs disk space; check `df -h`.
- The release bundle requires `rsync` and `zstd`.

## Bags

- Install `ros-humble-rosbag2-storage-mcap` and
  `ros-humble-rosbag2-compression-zstd` (already in the runtime image).
- Replay with `ros2 bag play /bags/<bag> --clock` and set `use_sim_time`.
- Sensor topics often use best-effort QoS; add a QoS override file if the
  recorder misses data.

## Camera

The D435i is intentionally not used or recorded. Do not start the RealSense
driver unless something actually needs it.

## Another Linux host `.101`

SSH is closed; only NoMachine on port 4000 is open. The username is not
discoverable remotely — check the physical screen or an existing NoMachine
session.

## Pgyer gateway

- Local admin: `http://192.168.100.1` (login password on the device/admin).
- WiFi: `robot1`, 2.4G channel 8.
- No confirmed public IPv4; remote access is expected through Oray cloud
  networking or upstream port mapping.
