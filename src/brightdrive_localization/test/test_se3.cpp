#include <gtest/gtest.h>

#include <cmath>

#include "brightdrive_localization/se3.hpp"

namespace brightdrive {
namespace {

TEST(Se3, ExpLogRoundTrip) {
  Eigen::Matrix<double, 6, 1> xi;
  xi << 1.2, -0.4, 0.7, 0.1, -0.2, 0.3;
  const Eigen::Matrix<double, 6, 1> recovered = se3_log(se3_exp(xi));
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(recovered(i), xi(i), 1e-9);
  }
}

TEST(Se3, YawAndPose) {
  const Eigen::Isometry3d pose = from_xyzyaw(3.0, -2.0, 1.0, 0.75);
  EXPECT_NEAR(yaw_of(pose), 0.75, 1e-12);
  EXPECT_NEAR(pose.translation().x(), 3.0, 1e-12);
  EXPECT_NEAR(pose.translation().y(), -2.0, 1e-12);
  EXPECT_NEAR(pose.translation().z(), 1.0, 1e-12);
}

TEST(Se3, BaseLinkHeroLidarComposition) {
  // base_link -> hero (rear axle to actor origin) and hero -> hero/lidar.
  const Eigen::Isometry3d T_base_hero = from_xyzyaw(1.39, 0.0, 0.30, 0.0);
  const Eigen::Isometry3d T_hero_lidar = from_xyzyaw(0.0, 0.0, 2.4, 0.0);
  const Eigen::Isometry3d T_base_lidar = T_base_hero * T_hero_lidar;

  const Eigen::Vector3d lidar_in_base = T_base_lidar.translation();
  EXPECT_NEAR(lidar_in_base.x(), 1.39, 1e-12);
  EXPECT_NEAR(lidar_in_base.y(), 0.0, 1e-12);
  EXPECT_NEAR(lidar_in_base.z(), 2.70, 1e-12);

  // A point measured 5 m in front of the LiDAR sits 6.39 m in front of the
  // rear axle.
  const Eigen::Vector3d point_lidar(5.0, 0.0, 0.0);
  const Eigen::Vector3d point_base = T_base_lidar * point_lidar;
  EXPECT_NEAR(point_base.x(), 6.39, 1e-12);
}

TEST(Se3, AdjointTranslatesRotationIntoTranslation) {
  const Eigen::Isometry3d T = from_xyzyaw(1.0, 2.0, 3.0, 0.4);
  Eigen::Matrix<double, 6, 1> xi = Eigen::Matrix<double, 6, 1>::Zero();
  xi.tail<3>() << 0.0, 0.0, 1.0;
  const Eigen::Matrix<double, 6, 1> transformed = se3_adjoint(T) * xi;
  const Eigen::Vector3d expected = T.translation().cross(T.rotation() * Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(transformed.head<3>().x(), expected.x(), 1e-12);
  EXPECT_NEAR(transformed.head<3>().y(), expected.y(), 1e-12);
  EXPECT_NEAR(transformed.head<3>().z(), expected.z(), 1e-12);
  EXPECT_NEAR(transformed.tail<3>().z(), 1.0, 1e-12);
}

TEST(Se3, CovarianceTransferAddsLeverArmUncertainty) {
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  covariance.diagonal() << 0.01, 0.01, 0.02, 1e-4, 1e-4, 2e-4;
  const Eigen::Isometry3d T_base_lidar = from_xyzyaw(0.0, 0.0, 2.4, 0.0);
  const Eigen::Matrix<double, 6, 6> adjoint = se3_adjoint(T_base_lidar);
  const Eigen::Matrix<double, 6, 6> transferred = adjoint * covariance * adjoint.transpose();

  // A 2.4 m vertical lever arm turns 1e-4 rad of roll uncertainty into
  // (2.4 * 1e-4)^2 = 5.76e-8 m^2 of lateral position uncertainty.
  EXPECT_NEAR(transferred(1, 1), 0.01 + 2.4 * 2.4 * 1e-4, 1e-12);
  for (int i = 0; i < 6; ++i) {
    EXPECT_GE(transferred(i, i), covariance(i, i) - 1e-12);
  }
}

}  // namespace
}  // namespace brightdrive
