#include "brightdrive_localization/se3.hpp"

#include <cmath>

#include "brightdrive_localization/geodesy.hpp"

namespace brightdrive {

Eigen::Matrix<double, 6, 6> se3_adjoint(const Eigen::Isometry3d& transform) {
  Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix3d rotation = transform.linear();
  adjoint.topLeftCorner<3, 3>() = rotation;
  adjoint.bottomRightCorner<3, 3>() = rotation;
  Eigen::Matrix3d skew;
  skew << 0.0, -transform.translation().z(), transform.translation().y(),
      transform.translation().z(), 0.0, -transform.translation().x(), -transform.translation().y(),
      transform.translation().x(), 0.0;
  adjoint.topRightCorner<3, 3>() = skew * rotation;
  return adjoint;
}

Eigen::Isometry3d from_xyzyaw(double x, double y, double z, double yaw) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(x, y, z);
  return transform;
}

double yaw_of(const Eigen::Isometry3d& transform) {
  const Eigen::Matrix3d rotation = transform.linear();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

}  // namespace brightdrive
