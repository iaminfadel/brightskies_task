# Python stack for the mapping-methods research steps.
#
# The registry deps image (0.9.15-r1) ships an empty /usr/local dist-packages
# -- the pip stack (rosbags, kiss-icp, small-gicp, ...) only exists inside the
# long-lived brightdrive-deps container because it was installed at runtime.
# This derived image makes the python steps self-contained under `docker run`.
FROM ghcr.io/iaminfadel/brightdrive-deps:0.9.15-r1

ENV DEBIAN_FRONTEND=noninteractive
RUN python3 -m pip install --no-cache-dir \
      "numpy<2" \
      "rosbags>=0.11" \
      "scipy>=1.10" \
      "kiss-icp==1.3.0" \
      "pydantic-settings>=2.0" \
      "matplotlib" \
      "small-gicp"
