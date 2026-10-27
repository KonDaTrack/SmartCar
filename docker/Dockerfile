ARG ROS_DISTRO=humble

# ---------------------------------------------------------------------------
# Runtime base: only runtime dependencies. No compilers, no GUI.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

COPY docker/packages-runtime.txt /tmp/packages-runtime.txt
RUN apt-get update \
 && xargs -r -a /tmp/packages-runtime.txt apt-get install -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/* /tmp/packages-runtime.txt

# TODO: add driver runtime libraries here (LSLIDAR, LakiBeam, VX1000)
# when the third-party packages are pinned in deps.repos.

COPY docker/create-user.sh /tmp/create-user.sh
RUN bash /tmp/create-user.sh && rm -f /tmp/create-user.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home/smartcar
WORKDIR /ws

# ---------------------------------------------------------------------------
# Development image: source is bind-mounted at /ws, not copied.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-desktop AS dev
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

COPY docker/packages-runtime.txt /tmp/packages-runtime.txt
COPY docker/packages-dev.txt /tmp/packages-dev.txt
RUN apt-get update \
 && xargs -r -a /tmp/packages-runtime.txt apt-get install -y --no-install-recommends \
 && xargs -r -a /tmp/packages-dev.txt apt-get install -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/* /tmp/packages-runtime.txt /tmp/packages-dev.txt

COPY docker/create-user.sh /tmp/create-user.sh
RUN bash /tmp/create-user.sh && rm -f /tmp/create-user.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home/smartcar
WORKDIR /ws
USER smartcar

# ---------------------------------------------------------------------------
# Builder: source is copied and compiled. Do NOT use --symlink-install here:
# the install tree must be self-contained when copied into the runtime image.
# ---------------------------------------------------------------------------
FROM dev AS builder
ARG ROS_DISTRO
SHELL ["/bin/bash", "-lc"]

# Build as root; the runtime image switches back to the non-root user.
USER root

COPY src /ws/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --merge-install --install-base /ws/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# Runtime image: install tree only. Launch/config defaults live in the image;
# site parameters are mounted at /config.
# ---------------------------------------------------------------------------
FROM base AS runtime
ARG ROS_DISTRO
SHELL ["/bin/bash", "-lc"]

COPY --from=builder /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home/smartcar
WORKDIR /ws
USER smartcar
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
