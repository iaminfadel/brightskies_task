// Small SE(3) helpers used by the matcher, innovation gate, and covariance
// transfer. Right-perturbation convention: T * exp(xi).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace brightdrive {

Eigen::Matrix<double, 6, 1> se3_log(const Eigen::Isometry3d& transform);

Eigen::Isometry3d se3_exp(const Eigen::Matrix<double, 6, 1>& xi);

// Adjoint of T in the [translation, rotation] ordering used by small_gicp.
Eigen::Matrix<double, 6, 6> se3_adjoint(const Eigen::Isometry3d& transform);

Eigen::Isometry3d from_xyzyaw(double x, double y, double z, double yaw);

double yaw_of(const Eigen::Isometry3d& transform);

}  // namespace brightdrive
