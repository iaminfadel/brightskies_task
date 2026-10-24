# LeGO-LOAM research image for the mapping-methods comparison.
#
# fishros/LeGO-LOAM-ROS2 (Humble port, pinned) + the IMU-damping patch from
# docker/legoloam/patch_lego.py (the port dropped the original IMU
# roll/pitch damping in transformUpdate; see docs/research/mapping-methods.md).
# GTSAM 4.2 from apt -- the tested path per docs/research/legoloam-ros2.md.
#
# Build on the VM from the repo root:
#   docker build -f docker/legoloam.Dockerfile -t legoloam:mm .
FROM ghcr.io/iaminfadel/brightdrive-deps:0.9.15-r1

ARG LEGO_REPO=https://github.com/fishros/LeGO-LOAM-ROS2.git
ARG LEGO_COMMIT=088856332b71e4bc4cc0f08cd8282ac370ca444a
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-gtsam \
    && rm -rf /var/lib/apt/lists/*

COPY docker/legoloam/patch_lego.py /tmp/patch_lego.py
RUN git clone "${LEGO_REPO}" /opt/lego_ws/src/LeGO-LOAM-ROS2 \
    && cd /opt/lego_ws/src/LeGO-LOAM-ROS2 \
    && git checkout "${LEGO_COMMIT}" \
    && python3 /tmp/patch_lego.py /opt/lego_ws/src/LeGO-LOAM-ROS2 \
    && rm /tmp/patch_lego.py

RUN source /opt/ros/humble/setup.bash \
    && cd /opt/lego_ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/lego_ws/build /opt/lego_ws/log

COPY docker/legoloam/loam_carla.yaml /opt/lego/loam_carla.yaml
COPY docker/legoloam/setup.bash /opt/lego/setup.bash
RUN chmod 0644 /opt/lego/setup.bash /opt/lego/loam_carla.yaml

WORKDIR /ws
CMD ["/bin/bash"]
