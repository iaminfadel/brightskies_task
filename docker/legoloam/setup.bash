#!/bin/bash
# Environment for the LeGO research container: ROS + LeGO workspace on top of
# the brightdrive deps image (which already carries rosbags/scipy/kiss-icp).
source /opt/ros/humble/setup.bash
source /opt/brightdrive/setup.bash
source /opt/lego_ws/install/setup.bash
