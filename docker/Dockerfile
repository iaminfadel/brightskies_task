# syntax=docker/dockerfile:1.7
#
# BrightDrive pipeline image: the pinned public deps image plus a colcon
# install of this repo's workspace. Published by
# .github/workflows/publish-image.yml on every push to main, tagged by
# commit SHA. `stable` is promoted only after the KPI gate passes.
ARG DEPS_IMAGE=ghcr.io/iaminfadel/brightdrive-deps@sha256:263332dd8da9b062726d15953ccddce1a11cb9754779840289e059792cd60c64

FROM ${DEPS_IMAGE}

ARG COMMIT_SHA=unknown
LABEL org.opencontainers.image.title="brightdrive"
LABEL org.opencontainers.image.source="https://github.com/iaminfadel/brightskies_task"
LABEL org.opencontainers.image.revision="${COMMIT_SHA}"

WORKDIR /ws

# Bake the repo and build the ROS workspace into the shared /ws layout so the
# image is the runtime of the compose services. Maps, bags, and evidence are
# runtime data, not image contents (see .dockerignore).
COPY . /ws/brightdrive

RUN bash -lc 'source /opt/brightdrive/setup.bash && \
      colcon build --base-paths /ws/brightdrive/src \
        --build-base /ws/build --install-base /ws/install'

ENV WS_DIR=/ws
CMD ["/bin/bash"]