#!/usr/bin/env bash
# Install the three SLAM candidates inside a persistent smartcar-ros2:dev
# container. Run as root:
#
#   docker exec -u root smartcar-slam-bench bash /ws/tools/slam_bench/setup.sh
#
# Nothing here is baked into the dev image. The container's writable layer
# keeps the packages; if the container is removed, re-run this script.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "run as root (docker exec -u root ...)" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
ROS_DISTRO="${ROS_DISTRO:-humble}"

apt-get update -qq
apt-get install -y --no-install-recommends \
  curl gnupg ca-certificates lsb-release \
  build-essential cmake ninja-build ccache patch \
  python3-pip python3-dev python3-venv \
  libeigen3-dev libsuitesparse-dev libtbb-dev \
  ros-"${ROS_DISTRO}"-sophus \
  ros-"${ROS_DISTRO}"-laser-geometry \
  ros-"${ROS_DISTRO}"-rosbag2-cpp \
  ros-"${ROS_DISTRO}"-rosbag2-storage \
  ros-"${ROS_DISTRO}"-rclcpp-components

# ---------------------------------------------------------------------------
# GLIM (CPU build) from the koide3 apt repository.
# ---------------------------------------------------------------------------
curl -fsSL --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" \
  | gpg --dearmor > /etc/apt/trusted.gpg.d/koide3_ppa.gpg
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" \
  > /etc/apt/sources.list.d/koide3_ppa.list
apt-get update -qq
apt-get install -y --no-install-recommends \
  libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev \
  libgtsam-notbb-dev libgtsam-points-dev \
  ros-"${ROS_DISTRO}"-glim-ros

# ---------------------------------------------------------------------------
# Kinematic-ICP: build the ROS 2 wrapper. Prefer the local vendored copy used
# for this benchmark; otherwise clone upstream and apply the adapter patch.
# ---------------------------------------------------------------------------
KIN_WS=/opt/kinematic_icp_ws
KIN_SRC=/opt/kinematic_icp_src
rm -rf "${KIN_WS}"
mkdir -p "${KIN_WS}/src"
if [ -d /ws/src/vendor/kinematic_icp ]; then
  cp -a /ws/src/vendor/kinematic_icp "${KIN_WS}/src/kinematic_icp_repo"
else
  rm -rf "${KIN_SRC}"
  git clone --depth 1 https://github.com/PRBonn/kinematic-icp.git "${KIN_SRC}"
  patch -p1 -d "${KIN_SRC}" \
    < /ws/tools/slam_bench/patches/kinematic_icp_ros_bench.patch
  cp -a "${KIN_SRC}" "${KIN_WS}/src/kinematic_icp_repo"
fi
# ROS setup scripts reference optional variables; do not run them with nounset.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${KIN_WS}"
colcon build --packages-select kinematic_icp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# KISS-SLAM and analysis tools from pip.
# ---------------------------------------------------------------------------
python3 -m pip install --upgrade pip
python3 -m pip install kiss-slam rosbags evo matplotlib pandas pyyaml

echo
echo "Installed:"
dpkg-query -W -f='  glim: ${Version}\n' ros-"${ROS_DISTRO}"-glim-ros 2>/dev/null || true
"${KIN_WS}/install/kinematic_icp/lib/kinematic_icp/kinematic_icp_offline_node" --help 2>&1 | head -3 || true
python3 -c 'import kiss_slam; print("  kiss-slam:", kiss_slam.__file__)'
