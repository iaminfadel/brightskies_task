// GICP scan-to-map matcher built on small_gicp. The map cloud and its KdTree
// are built once; each live scan is downsampled, aligned, and scored against
// the contract's quality gates.
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace brightdrive {

struct GicpSettings {
  double voxel_size_m = 0.25;
  double max_correspondence_init_m = 1.0;
  double max_correspondence_final_m = 0.5;
  int max_iterations = 30;
  double convergence_eps = 1e-4;
  int num_threads = 2;
  int num_neighbors = 10;
  double crop_radius_m = 80.0;
  double rmse_gate_m = 0.20;
  double inlier_gate = 0.60;
  double condition_gate = 1e6;
};

struct MatchResult {
  bool converged = false;
  bool accepted = false;
  std::string reject_reason;
  Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();
  double rmse_m = 0.0;
  double inlier_ratio = 0.0;
  std::size_t inliers = 0;
  std::size_t source_points = 0;
  double hessian_condition = 0.0;
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
};

class GicpMatcher {
 public:
  GicpMatcher(const GicpSettings& settings,
              small_gicp::PointCloud::ConstPtr map_cloud,
              std::shared_ptr<const small_gicp::KdTree<small_gicp::PointCloud>> map_tree);

  // raw_scan is XYZ in the LiDAR frame; init_T_map_lidar is the prediction.
  // crop_center_map supports the local 80 m crop from the contract.
  MatchResult align(const std::vector<Eigen::Vector3f>& raw_scan,
                    const Eigen::Isometry3d& init_T_map_lidar,
                    const Eigen::Vector3d& crop_center_map) const;

  bool healthy() const { return static_cast<bool>(map_cloud_) && static_cast<bool>(map_tree_); }

  const GicpSettings& settings() const { return settings_; }

 private:
  GicpSettings settings_;
  small_gicp::PointCloud::ConstPtr map_cloud_;
  std::shared_ptr<const small_gicp::KdTree<small_gicp::PointCloud>> map_tree_;
};

}  // namespace brightdrive
