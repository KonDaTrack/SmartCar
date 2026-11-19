# Camera intrinsic calibration

Intrinsics are the first calibration step: camera-LiDAR, camera-IMU and VINS
all need a camera model.

The GUI runs on the development host in the `calib` Compose service. The
vehicle only publishes camera topics over DDS; it does not run X11 or any
calibration package.

## 1. Build the dev image

The dev image includes `camera_calibration`, `image_view`, `image_proc`,
`cv_bridge` and `rqt_image_view`:

```bash
./scripts/build-dev.sh
```

## 2. Prerequisites

- D435i connected to USB 3; `lsusb -t` must show `5000M`.
- A rigid checkerboard; measure the **inner corners** and **square size**.
- X11 on the development host (`DISPLAY` should point at it).

### Option A: camera on the vehicle

- Vehicle `sensors` container running with the camera enabled.
- Development host on the same DDS domain as the vehicle.

Check the stream on the development host:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
```

### Option B: camera plugged into the development host

Start the local D435i driver:

```bash
./scripts/calib-camera.sh
```

Stop it with:

```bash
./scripts/calib-camera-stop.sh
```

The `calib-camera` service is headless and exposes the camera topics over
DDS. It uses `privileged: true` and `/dev:/dev`; this is intended for the
development host only. Do not run it at the same time as the vehicle camera,
or both will publish the same topics.

If the camera serial differs, set `D435I_SERIAL` in `.env` or the environment.

Check:

```bash
lsusb -t
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
```

## 3. Run

Default: colour camera, 8x6 inner corners, 25 mm squares.

```bash
./scripts/calib-intrinsics.sh --size 8x6 --square 0.025
```

Use the actual measured values. For example, a 9x6 board with 30 mm squares:

```bash
./scripts/calib-intrinsics.sh --size 9x6 --square 0.030
```

The GUI shows the live image and X/Y/Size/Skew coverage bars. Move the board:

- different image positions;
- different distances;
- different yaw/pitch/roll;
- until all bars are green;
- avoid motion blur and glare.

Then click **CALIBRATE**, inspect the reported reprojection error, and click
**SAVE**. The script copies `/tmp/calibrationdata.tar.gz` to:

```text
config/calibration/intrinsics/calibrationdata_<timestamp>.tar.gz
```

## 4. Colour vs IR

### Colour raw

```bash
./scripts/calib-intrinsics.sh \
  --topic /camera/camera/color/image_raw \
  --camera /camera/camera/color
```

This estimates the colour lens distortion.

### IR rectified

```bash
./scripts/calib-intrinsics.sh \
  --topic /camera/camera/infra1/image_rect_raw \
  --camera /camera/camera/infra1
```

RealSense IR images are already rectified; this produces intrinsics for the
rectified stream and cannot recover the raw lens distortion. Use it when the
consumers are VIO/camera-LiDAR on the rectified IR stream.

### Stereo IR

```bash
./scripts/calib-intrinsics.sh --stereo
```

Defaults are `infra1` and `infra2`. Override with `--left/--right` if needed.

## 5. Calibrate from a bag

If the live DDS link is too slow, record a short bag and replay it in a second
terminal:

```bash
# terminal 1
./scripts/replay.sh <bag>
# terminal 2
./scripts/calib-intrinsics.sh --size 8x6 --square 0.025
```

The `calib` service mounts the repository at `/ws` and bags at `/bags`.

## 6. Inspect the result

```bash
tar -tzf config/calibration/intrinsics/calibrationdata_*.tar.gz
tar -xzf config/calibration/intrinsics/calibrationdata_*.tar.gz -C /tmp
```

Typical contents:

```text
ost.yaml
ost.txt
```

Check:

- `image_width`, `image_height`;
- `camera_matrix` (`fx`, `fy`, `cx`, `cy`);
- `distortion_coefficients`;
- reprojection RMS in the GUI/log.

Record the target size, camera stream, date and bag/session in the calibration
notes.

## 7. What comes next

After intrinsics:

1. camera-LiDAR extrinsic (target board visible in both);
2. LiDAR-IMU extrinsic/time offset (dynamic bag);
3. camera-IMU extrinsic by chaining;
4. camera-IMU td with VINS.
