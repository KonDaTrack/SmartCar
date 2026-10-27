# RViz

Put development RViz configs here, e.g. `dev.rviz`, and run:

```bash
rviz2 -d /config/rviz/dev.rviz
```

Recommended displays:

- TF
- PointCloud2 on a downsampled `/points_viz` topic (100M links)
- LaserScan `/scan_left`, `/scan_right`
- Odometry / Path

RViz runs only on the development machine, never in the vehicle runtime.
