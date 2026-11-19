# SmartCar

[English](README.md) | [简体中文](README.zh-CN.md)

面向 SmartCar 车辆的 ROS 2 Humble + Docker 工作区。

## 当前状态

车辆端驱动、标定流程和 recorder 已可用。SLAM 集成、UDP 驱动持久化镜像，
以及一次性完整全传感器大地图仍待完成。

### 已完成

- **驱动**
  - LSLIDAR C16 3D 雷达（`/lslidar_point_cloud`，10 Hz）。
  - VX1000 GNSS/RTK + IMU：支持串口和车载以太网 UDP。
    发布 `/VX1000_ins|imu|gnss|stat|cov|odom`、`/imu/data`、`/gnss/fix`。
  - JD01 底盘 CAN 转以太网网关和 Ackermann `/chassis/odom`。
  - Intel D435i 彩色/深度/双目红外驱动；相机 IMU 关闭，只使用 VX1000 IMU。
- **VX1000 UDP**
  - 和串口相同的 68 字节帧，通过 UDP 从
    `192.168.100.100:55325` 发到车端 `192.168.100.10:9000`。
  - 实测 100 包/s、330 帧/s，校验错误为 0。
  - 车端 compose 默认 `SMARTCAR_VX1000_TRANSPORT=udp`；设置成 `serial`
    可回退到 `/dev/ttyUSB0`。
- **Recorder 可靠性**
  - `tools/record.sh` 使用 128 MiB 的 per-topic 缓存和
    `config/record_qos.yaml`（VX1000 订阅 QoS depth 1000）。
  - 默认关闭压缩；已记录 20 GB 分卷时约 2.6 s 的停顿。
  - `RECORD_TOPICS` 支持单行和多行话题列表。
- **标定**
  - 相机-雷达棋盘格标定和相机-IMU VINS 时间偏移。
  - 通过相机-IMU + 相机-雷达链推导雷达-IMU 变换。
  - 静态地面/重力 roll-pitch 检查。
  - 结果和注意事项：`config/calibration/results_20261117.yaml`、
    `docs/calibration_20261117.md`。
- **录制实验**
  - LiDAR + RGB + VX1000 和 RGBD + 双目 + VX1000 数据包。
  - 分开同时录制：一个包 LiDAR + INS，另一个包对齐 RGBD；避免相机负载
    影响 VX1000 recorder。
- **外部实验**
  - ORB-SLAM2 源码部署在本仓库之外，用于学习：
    `/home/k/orbslam2_ws`，镜像 `smartcar-orbslam2:noetic`。

### TODO

1. **持久化车辆镜像**：构建并部署包含 UDP VX1000 驱动、recorder 缓存/QoS
   改动和新 compose 的运行时镜像。当前车上仍是旧版本镜像。
2. **VX1000 时间戳**：把 VX1000 的 `gps_week_second` 通过主机偏移映射到
   ROS 时间，处理 GPS 无效回退和时间戳单调性。`header.stamp = now()` 在
   调度停顿后仍会把样本挤成一簇。
3. **干净的大地图录制**：长时间全传感器地图使用
   `MAX_BAG_SIZE_GB=0`，或保持双 recorder 方案；验证全程 IMU/INS/ODOM
   都是 100 Hz。
4. **SLAM 集成**：选择和接入 LIO/VINS 前端，补齐 launch 和配置；用 RTK
   GNSS 真值比较不同外参候选。
5. **GNSS 真值**：确认 RTK Fixed 历元、天线杆臂和时间插值，再作为真值。
6. **相机-雷达精化**：当前棋盘格结果偏粗，需要用更好的采集或成熟的
   targetless 工具改进。
7. **底盘转向标定**：在小车上确认 `steer_scale` / 轮距。
8. **健康检查**：实现真实的话题频率、TF 和 SLAM 输出检查。
9. **ORB-SLAM2 数据路径**：把 ROS 2 MCAP 转成 ROS 1，先跑单目，再跑
   对齐 RGB-D 或 infra1/infra2 双目。

## 架构

- **2 个镜像**
  - `smartcar-ros2:dev` — ROS 2 Humble desktop、编译工具、RViz、
    rosbag2 MCAP/zstd。源码 bind mount，迭代快。
  - `smartcar-ros2:<version>` — 运行时镜像，源码编译进镜像，无编译工具、
    无 GUI。
- **车端容器**
  - `sensors` — C16、左右 LakiBeam、VX1000（串口或 UDP）、D435i。
  - `chassis` — CAN 网关接收和 Ackermann 里程计。
  - `slam` — SLAM / 定位 / 地图。
  - `vehicle` 合并 profile 仅保留用于调试。
- **开发机服务**
  - `dev` — 挂载 `src/` 和 `config/` 的交互容器。
  - `rviz` — X11 + NVIDIA GPU 可视化。
  - `record` — 无界面 rosbag2 recorder。
  - `replay` 为一次性容器。
- **车端运行栈不包含 ROS 1 bridge。** ORB-SLAM2 ROS 1 工作区是外部
  学习环境，不属于车端运行时。
- **非 root 容器**：镜像创建 `smartcar` 用户（UID/GID 1000）；VX1000
  串口访问使用 `dialout` 组。

## 传感器

| IP / 设备 | 传感器 | 说明 |
|---|---|---|
| 192.168.100.201 | LSLIDAR C16 3D 雷达 | UDP 2368/2369，`/lslidar_point_cloud`，10 Hz |
| 192.168.100.200 | LakiBeam1 2D 雷达，左 | UDP 2468，`/scan_left` |
| 192.168.100.210 | LakiBeam1 2D 雷达，右 | UDP 2568，`/scan_right` |
| 192.168.100.110 | 底盘 CAN 转以太网 | UDP 4001/4002 → `.10:8002` |
| `/dev/ttyUSB0`（by-id） | VX1000 GNSS/RTK + IMU，串口 | 460800 baud，`/dev/gps` |
| 192.168.100.100:55325 | VX1000 车载以太网输出 | UDP → `.10:9000`，同样 68 字节帧 |
| USB | Intel RealSense D435i | 彩色/深度/红外；相机 IMU 关闭 |

D435i 的双目是 `infra1` + `infra2`（Y8 灰度）；彩色是独立的 RGB8 传感器。
深度由红外双目计算，不是由 RGB 计算。

参见 [docs/sensors.md](docs/sensors.md) 和 [docs/network.md](docs/network.md)。

## 录制

默认 recorder：

```bash
./scripts/record.sh
```

recorder 把 MCAP 写到 `SMARTCAR_BAGS_HOST_DIR`（默认
`/home/yunle/bags`），按 `MAX_BAG_SIZE_GB` 分卷，低于 `MIN_FREE_GB`
拒绝启动，Ctrl-C 时正常写 metadata。

可靠性配置：

```text
MAX_CACHE_SIZE_MB=128        rosbag2 per-topic 缓存
QOS_PROFILE=/config/record_qos.yaml
COMPRESSION_MODE=none
```

长时间大地图建议设置 `MAX_BAG_SIZE_GB=0`（或大于预计输出大小），避免
20 GB 分卷时约 2.6 s 的 recorder 停顿。如果全四路相机和 INS 都要录，
建议同时跑两个 recorder：

```text
A: LiDAR + VX1000 + /imu/data + GNSS + 底盘
B: RGB 或 RGBD 图像 + camera_info
```

两个必须同时启动；结束后按时间戳合并或对齐。

参见 [docs/time_sync.md](docs/time_sync.md)。

## 标定

当前结果：

```text
config/calibration/results_20261117.yaml
config/calibration/time_offsets.yaml
docs/calibration_20261117.md
```

摘要：

- 相机-雷达棋盘格：平面残差中位数 8.8 mm，板中心重投影中位数 21.2 px
  （偏粗）。
- 相机-IMU VINS 时间偏移：固定外参 `td_cam_imu = 39.986 ms`；在线外参
  `45.053 ms`。
- 雷达-IMU 变换由 `T_imu_lidar = T_imu_cam * T_cam_lidar` 推导，不是独立
  6-DoF 优化。车辆不能做 roll/pitch，LI-Init 无法完成完整激励初始化。
- 静态地面/重力检查只能约束 roll/pitch；可选修正矩阵见结果 YAML。
- 位置真值优先使用 RTK GNSS；INS 是 GNSS+IMU 融合结果，不是独立真值。

## 快速开始

```bash
cp .env.example .env

# 1. 构建开发镜像
./scripts/build-dev.sh

# 2. 进入开发容器
./scripts/dev.sh
# 容器内：
cd /ws
colcon build --symlink-install
source install/setup.bash
```

车辆在线调试：把开发机 `enp5s0` 接到车辆交换机，然后：

```bash
./scripts/up.sh      # sensors + slam 开发容器
./scripts/rviz.sh    # RViz 容器（X11 + RTX 5060）
./scripts/record.sh  # rosbag2 MCAP
```

## 发布 / 部署

```bash
git add . && git commit
./scripts/release.sh v0.1.5     # 构建运行时镜像 + image.tar.zst + manifest
./scripts/deploy.sh  v0.1.5     # 局域网同步 + docker load + compose up
./scripts/health.sh             # 在车上运行健康检查
./scripts/rollback.sh           # 回滚到上一版本
```

车辆只需要 Docker、`rsync`、`zstd` 和 Docker Compose v2。参见
[docs/deploy.md](docs/deploy.md)。

## 仓库结构

```text
docker/          Dockerfile、entrypoint、apt 包列表
scripts/         build/dev/up/rviz/calib/record/release/deploy 快捷脚本
src/             ROS 2 包（bringup、sensors、chassis、slam、msgs、ins）
config/          站点配置：network、dev、vehicle、calibration、rviz
deploy/          compose 文件、deploy/rollback/healthcheck、systemd
tools/           record、release、benchmark 脚本
docs/            network、sensors、workflow、calibration、deploy、troubleshooting、ADR
tests/           单元 / bag 回放测试说明
bags/            本地 bag（gitignored）
release-manifests/  提交到 Git 的小体积 release manifest
```

## 固定工作流规则

1. GitHub 私有仓库只备份 **源码/配置/脚本/文档**，不备份镜像和 bag。
2. 开发容器挂载 `src/`；运行时镜像通过多阶段构建把源码编译进去。
3. 车辆只运行运行时镜像并挂载 `config/`；不挂源码、不装编译器。
4. Launch 文件放在镜像里；站点参数放在 `config/`。
5. 车辆使用不可变镜像 tag；保留当前和上一版本用于回滚。
6. 工作区有未提交改动时不发布。
7. 同一时间只运行一套传感器栈：车端 ROS 1 驱动不能和 ROS 2 驱动同时跑。
8. RViz 在开发机运行；车辆不运行 GUI。
9. 100M 局域网：可视化降采样点云；不要同时做全分辨率可视化和全分辨率
   录包。
10. 容器以非 root `smartcar`（UID 1000）运行；不要在容器内用 root 跑栈。

## 驱动

- `smartcar_ins` — VX1000，串口 `/dev/ttyUSB0` @ 460800，或 UDP 9000。
  发布 `/VX1000_ins|imu|gnss|stat|cov|odom`、`/imu/data`、`/gnss/fix`。
- `smartcar_chassis` — 接收 `.110` 的 CAN-over-UDP 数据（端口 8002），
  发布 `/vehicle_status`，积分 `/chassis/odom`（JD01 Ackermann，
  `wheel_base=1.2`）。默认不发布 TF，因为 SLAM 前端负责
  `/odom` 和 `odom -> base_link`。
- `smartcar_sensors` / `smartcar_bringup` — C16、LakiBeam 和 D435i launch
  接线。
- 站点参数在 `config/chassis.yaml`、`config/lslidar_cx.yaml` 和
  `config/network.yaml`。

## 外部学习工作区

```text
/home/k/orbslam2_ws
  src/ORB_SLAM2        官方 ORB-SLAM2，已做 OpenCV4 适配
  thirdparty/Pangolin  Pangolin 0.5
  config/D435i_color.yaml
  README.md
```

Docker 镜像：`smartcar-orbslam2:noetic`。ORB-SLAM2 是 ROS 1，且不使用
VX1000 IMU；先用它学习 ORB-SLAM2 流程，再转向支持 IMU 的 VIO/LIO 框架。
