#!/usr/bin/env python3
"""Patch the fishros LeGO-LOAM-ROS2 port for the CARLA rig.

The Humble port dropped the original LeGO-LOAM IMU roll/pitch damping in
``transformUpdate()`` (the port removed the IMU subscription entirely), which
is the documented failure mechanism of the 2026-10-22 spike: with nothing
constraining roll/pitch, the scan matcher lets forward motion leak into the
height axis and the map smears vertically.

This patch restores the original behaviour (imuHandler + 0.998/0.002 damping
blend, verbatim semantics from the RobustFieldAutonomyLab master) and adds an
optional soft flat-ground height hold (``ground_z_hold``, camera-frame y =
lidar height) for CARLA's planar roads.  Both are ROS parameters, defaulting
to damping on / z hold off.

Every edit is anchor-based and fails loudly if the port source changes.
"""
import re
import sys
from pathlib import Path

HEADER_MEMBERS = """
  // BEGIN research patch: restored IMU damping (see docker/legoloam/patch_lego.py)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
  bool _imu_damping_enabled = true;
  bool _ground_z_hold_enabled = false;
  bool _ground_z_hold_set = false;
  double _ground_z_hold_height = 0.0;
  double _imu_period = 0.1;
  static const int imuQueLength = 200;
  double imuTime[imuQueLength];
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];
  int imuPointerFront = 0;
  int imuPointerLast = -1;
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuIn);
  // END research patch
"""

IMU_INCLUDE = '#include "sensor_msgs/msg/imu.hpp"\n'

CPP_REPLACEMENT = '''void MapOptimization::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuIn) {
  const double qw = imuIn->orientation.w;
  const double qx = imuIn->orientation.x;
  const double qy = imuIn->orientation.y;
  const double qz = imuIn->orientation.z;
  // ROS ZYX Euler angles; CARLA roads keep roll and pitch near zero.
  const double roll =
      std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  const double pitch = std::asin(
      std::max(-1.0, std::min(1.0, 2.0 * (qw * qy - qz * qx))));
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  imuTime[imuPointerLast] = rclcpp::Time(imuIn->header.stamp).seconds();
  imuRoll[imuPointerLast] = static_cast<float>(roll);
  imuPitch[imuPointerLast] = static_cast<float>(pitch);
}

void MapOptimization::transformUpdate() {
  if (imuPointerLast >= 0) {
    float imuRollLast = 0, imuPitchLast = 0;
    while (imuPointerFront != imuPointerLast) {
      if (timeLaserOdometry.seconds() + _imu_period < imuTime[imuPointerFront]) break;
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeLaserOdometry.seconds() + _imu_period > imuTime[imuPointerFront]) {
      imuRollLast = imuRoll[imuPointerFront];
      imuPitchLast = imuPitch[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront - 1 + imuQueLength) % imuQueLength;
      double ratioFront =
          (timeLaserOdometry.seconds() + _imu_period - imuTime[imuPointerBack]) /
          (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      double ratioBack = 1.0 - ratioFront;
      imuRollLast = imuRoll[imuPointerFront] * ratioFront +
                    imuRoll[imuPointerBack] * ratioBack;
      imuPitchLast = imuPitch[imuPointerFront] * ratioFront +
                     imuPitch[imuPointerBack] * ratioBack;
    }

    // Original LeGO-LOAM damping (index convention kept verbatim from the
    // RobustFieldAutonomyLab master).
    if (_imu_damping_enabled) {
      transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
      transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
    }
    // Soft flat-ground height hold (research addition for planar CARLA roads;
    // camera-frame y is the lidar height).
    if (_ground_z_hold_enabled) {
      if (_ground_z_hold_set) {
        transformTobeMapped[4] =
            0.998 * transformTobeMapped[4] + 0.002 * _ground_z_hold_height;
      } else {
        _ground_z_hold_height = transformTobeMapped[4];
        _ground_z_hold_set = true;
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
}
'''

LAZY_SUBSCRIPTION = '''  if (!subImu) {
    _imu_damping_enabled = this->declare_parameter<bool>("imu_damping", true);
    _ground_z_hold_enabled = this->declare_parameter<bool>("ground_z_hold", false);
    _imu_period = this->declare_parameter<double>("imu_period", 0.1);
    const std::string imu_topic =
        this->declare_parameter<std::string>("imu_topic", "/carla/hero/imu");
    subImu = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 50,
        std::bind(&MapOptimization::imuHandler, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "research patch: imu_topic=%s imu_damping=%d ground_z_hold=%d",
                imu_topic.c_str(), static_cast<int>(_imu_damping_enabled),
                static_cast<int>(_ground_z_hold_enabled));
  }
'''

RUN_GUARD_ANCHOR = (
    "timeLaserOdometry = association.laser_odometry.header.stamp;"
)
HEADER_MEMBER_ANCHOR = "bool isDegenerate;"
HEADER_INCLUDE_ANCHOR = '#include "lego_loam/utility.h"'
TRANSFORM_UPDATE_ANCHOR = "void MapOptimization::transformUpdate"


def replace_transform_update(cpp: str) -> str:
    if CPP_REPLACEMENT.strip() in cpp:
        print("transformUpdate: already patched")
        return cpp
    pattern = re.compile(
        r"void MapOptimization::transformUpdate\s*\([^)]*\)\s*\{.*?\n\}\n",
        re.DOTALL,
    )
    matches = pattern.findall(cpp)
    if len(matches) != 1:
        raise SystemExit(
            f"transformUpdate anchor failed: {len(matches)} matches "
            "(expected exactly 1)"
        )
    print(f"transformUpdate: replacing {len(matches[0])} chars")
    return pattern.sub(CPP_REPLACEMENT, cpp, count=1)


def insert_lazy_subscription(cpp: str) -> str:
    if "research patch: imu_topic" in cpp:
        print("lazy subscription: already patched")
        return cpp
    lines = cpp.splitlines(keepends=True)
    target = None
    for index, line in enumerate(lines):
        if RUN_GUARD_ANCHOR in line:
            target = index
            break
    if target is None:
        raise SystemExit(f"run() anchor not found: {RUN_GUARD_ANCHOR!r}")
    indent = lines[target][: len(lines[target]) - len(lines[target].lstrip())]
    block = "".join(
        (indent + stripped + "\n") if stripped else "\n"
        for stripped in LAZY_SUBSCRIPTION.strip("\n").split("\n")
    )
    lines.insert(target + 1, block)
    print(f"lazy subscription: inserted after line {target + 1}")
    return "".join(lines)


def patch_header(header: str) -> str:
    if "BEGIN research patch" in header:
        print("header: already patched")
        return header
    if HEADER_INCLUDE_ANCHOR not in header:
        raise SystemExit(f"header include anchor not found: {HEADER_INCLUDE_ANCHOR!r}")
    header = header.replace(
        HEADER_INCLUDE_ANCHOR, HEADER_INCLUDE_ANCHOR + "\n" + IMU_INCLUDE.rstrip("\n"), 1
    )
    count = header.count(HEADER_MEMBER_ANCHOR)
    if count != 1:
        raise SystemExit(
            f"header member anchor count={count}: {HEADER_MEMBER_ANCHOR!r}"
        )
    header = header.replace(
        HEADER_MEMBER_ANCHOR, HEADER_MEMBERS.strip("\n") + "\n" + HEADER_MEMBER_ANCHOR, 1
    )
    print("header: members + include inserted")
    return header


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: patch_lego.py <LeGO-LOAM-ROS2 checkout>")
    root = Path(sys.argv[1])
    cpp_path = root / "LeGO-LOAM" / "src" / "mapOptmization.cpp"
    header_path = root / "LeGO-LOAM" / "src" / "mapOptimization.h"
    for path in (cpp_path, header_path):
        if not path.is_file():
            raise SystemExit(f"missing {path}")
    cpp = cpp_path.read_text(encoding="utf-8")
    header = header_path.read_text(encoding="utf-8")
    cpp = replace_transform_update(cpp)
    cpp = insert_lazy_subscription(cpp)
    header = patch_header(header)
    cpp_path.write_text(cpp, encoding="utf-8")
    header_path.write_text(header, encoding="utf-8")
    print("PATCH_OK")


if __name__ == "__main__":
    main()
