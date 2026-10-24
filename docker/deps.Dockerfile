# syntax=docker/dockerfile:1.7
#
# BrightDrive dependency image.
#
# Built once during #18 on the GCP VM, pushed to GHCR, and pinned by digest.
# The live CI job runs this image; it never builds the workspace on the VM.
#
# Build (from the repo root):
#   docker build -f docker/deps.Dockerfile \
#     --build-arg CARLA_IMAGE=carlasim/carla:0.9.15@sha256:<digest> \
#     -t ghcr.io/iaminfadel/brightdrive-deps:validation .

ARG CARLA_IMAGE=carlasim/carla:0.9.15

FROM ${CARLA_IMAGE} AS carla

FROM ros:humble-ros-base-jammy

ARG CARLA_VERSION=0.9.15
ARG BRIDGE_REPO=https://github.com/ttgamage/carla-ros-bridge.git
ARG BRIDGE_COMMIT=3ce0be8daf2ae405f7fe47e101028c16fbff1af1
ARG KISS_ICP_VERSION=v1.3.0
ARG SMALL_GICP_VERSION=master
ARG SCENARIO_RUNNER_VERSION=v0.9.15
ARG FOXGLOVE_VERSION=0.7.10

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS 2, bridge dependencies, point-cloud tooling, and headless-RViz support.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git curl wget ca-certificates jq unzip \
      python3-pip python3-dev python3-setuptools python3-wheel \
      python3-rosdep python3-colcon-common-extensions python3-vcstool \
      ros-humble-ackermann-msgs \
      ros-humble-cv-bridge \
      ros-humble-derived-object-msgs \
      ros-humble-pcl-conversions \
      ros-humble-pcl-ros \
      ros-humble-robot-localization \
      ros-humble-rosbag2 \
      ros-humble-rosbag2-storage-default-plugins \
      ros-humble-rqt-gui-py \
      ros-humble-rqt-image-view \
      ros-humble-rviz2 \
      ros-humble-tf2-geometry-msgs \
      ros-humble-tf2-ros \
      ros-humble-tf2-tools \
      ros-humble-vision-opencv \
      ros-humble-xacro \
      libeigen3-dev libomp-dev libtbb-dev robin-map-dev \
      libpcl-dev libboost-all-dev libyaml-cpp-dev \
      libgl1-mesa-glx libgl1-mesa-dri mesa-utils xvfb imagemagick \
      ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# foxglove_bridge is released in Humble; fall back to source if the package
# is unavailable in the configured ROS apt snapshot.
RUN apt-get update \
    && (apt-get install -y --no-install-recommends ros-humble-foxglove-bridge \
        || echo "foxglove_bridge not in apt; will build from source") \
    && rm -rf /var/lib/apt/lists/*

# CARLA 0.9.15 Python API for Python 3.10. The official wheel in the CARLA
# image targets Python 3.7; this is the pinned community cp310 build used by
# the Humble bridge fork. Vendor `agents/` from the official image as well so
# ScenarioRunner route imports work.
COPY --from=carla /home/carla/PythonAPI/carla /opt/carla/PythonAPI/carla

RUN curl -fsSL -o /tmp/carla-0.9.15-cp310-cp310-linux_x86_64.whl \
      https://github.com/gezp/carla_ros/releases/download/carla-0.9.15-ubuntu-22.04/carla-0.9.15-cp310-cp310-linux_x86_64.whl \
    && echo "d8766616df2511a58ac152ccc53e3307a84ccef9e7fb7108006667188d5766f8  /tmp/carla-0.9.15-cp310-cp310-linux_x86_64.whl" \
       | sha256sum -c - \
    && python3 -m pip install --no-cache-dir /tmp/carla-0.9.15-cp310-cp310-linux_x86_64.whl \
    && rm -f /tmp/carla-0.9.15-cp310-cp310-linux_x86_64.whl

# ScenarioRunner v0.9.15 is a Python tree, not a ROS package.
RUN git clone --depth 1 --branch "${SCENARIO_RUNNER_VERSION}" \
      https://github.com/carla-simulator/scenario_runner.git /opt/scenario_runner \
    && python3 -m pip install --no-cache-dir \
      "numpy<2" \
      "rosbags>=0.11" \
      "scipy>=1.10" \
      "pydantic-settings>=2.0" \
      "kiss-icp==${KISS_ICP_VERSION#v}" \
      "Shapely>=1.8" \
      "networkx>=2.8" \
      "opencv-python-headless" \
      "psutil" \
      "xmlschema" \
      "ephem" \
      "tabulate" \
      "matplotlib" \
      "six" \
      "simple-watchdog-timer" \
      "antlr4-python3-runtime==4.10" \
      "graphviz" \
      "py-trees==0.8.3" \
      "pygame" \
      "pexpect" \
      "transforms3d" \
      "simple-pid"

# ROS workspace: bridge fork, KISS-ICP, small_gicp, and (if needed) foxglove.
RUN mkdir -p /opt/ros_ws/src \
    && cd /opt/ros_ws/src \
    && git clone --recurse-submodules "${BRIDGE_REPO}" carla-ros-bridge \
    && cd carla-ros-bridge \
    && git checkout "${BRIDGE_COMMIT}" \
    && git submodule update --init --recursive \
    && cd /opt/ros_ws/src \
    && git clone --depth 1 --branch "${KISS_ICP_VERSION}" https://github.com/PRBonn/kiss-icp.git \
    && git clone https://github.com/koide3/small_gicp.git \
    && cd small_gicp \
    && git checkout "${SMALL_GICP_VERSION}" \
    && if ! (source /opt/ros/humble/setup.bash && ros2 pkg prefix foxglove_bridge >/dev/null 2>&1); then \
         cd /opt/ros_ws/src \
         && git clone --depth 1 --branch "${FOXGLOVE_VERSION}" \
              https://github.com/foxglove/ros-foxglove-bridge.git foxglove_bridge; \
       fi

RUN python3 -m pip install --no-cache-dir "cmake==3.28.3" \
    && cmake --version \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/ros_ws \
    && (rosdep init || true) \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
         --skip-keys "carla sophus python-transforms3d-pip" \
    && colcon build --merge-install \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/ros_ws/build /opt/ros_ws/log

COPY docker/setup.bash /opt/brightdrive/setup.bash

RUN chmod 0644 /opt/brightdrive/setup.bash

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
CMD ["/bin/bash"]
