// Small SE(3) helpers used by the innovation gate and covariance transfer.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace brightdrive {

// Adjoint of T in the [translation, rotation] ordering used by small_gicp.
Eigen::Matrix<double, 6, 6> se3_adjoint(const Eigen::Isometry3d& transform);

Eigen::Isometry3d from_xyzyaw(double x, double y, double z, double yaw);

double yaw_of(const Eigen::Isometry3d& transform);

}  // namespace brightdrive
