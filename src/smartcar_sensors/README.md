# smartcar_sensors

Placeholder package for the sensor driver integration.

Planned content:

- LSLIDAR C16 wrapper/config (`192.168.100.201`, UDP 2368/2369).
- LakiBeam left/right wrappers (`192.168.100.200`/`.210`, UDP 2468/2568).
- VX1000 GNSS/IMU serial wrapper (`/dev/ttyUSB0`, 460800).
- Launch snippets used by `smartcar_bringup/sensors.launch.py`.

Driver source should be pinned in `deps.repos`; runtime dependencies belong in
`docker/packages-runtime.txt`, build dependencies in `docker/packages-dev.txt`.
