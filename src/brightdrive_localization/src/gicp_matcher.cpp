#include "brightdrive_localization/gicp_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Eigenvalues>

#include <small_gicp/ann/traits.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/rejector.hpp>
#include <small_gicp/registration/termination_criteria.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace brightdrive {
namespace {

using Registration = small_gicp::Registration<
    small_gicp::GICPFactor,
    small_gicp::SerialReduction,
    small_gicp::NullFactor,
    small_gicp::DistanceRejector>;

double degrees_to_radians(double degrees) { return degrees * M_PI / 180.0; }

}  // namespace

GicpMatcher::GicpMatcher(
    const GicpSettings& settings,
    small_gicp::PointCloud::ConstPtr map_cloud,
    std::shared_ptr<const small_gicp::KdTree<small_gicp::PointCloud>> map_tree)
    : settings_(settings), map_cloud_(std::move(map_cloud)), map_tree_(std::move(map_tree)) {}

MatchResult GicpMatcher::align(const std::vector<Eigen::Vector3f>& raw_scan,
                               const Eigen::Isometry3d& init_T_map_lidar,
                               const Eigen::Vector3d& crop_center_map) const {
  MatchResult result;
  result.source_points = raw_scan.size();
  if (!healthy() || raw_scan.empty()) {
    result.reject_reason = "map_or_scan_unavailable";
    return result;
  }

  auto source = std::make_shared<small_gicp::PointCloud>();
  source->resize(raw_scan.size());
  std::size_t kept = 0;
  const double crop_sq = settings_.crop_radius_m * settings_.crop_radius_m;
  for (const auto& point : raw_scan) {
    const Eigen::Vector4d homogeneous(point.x(), point.y(), point.z(), 1.0);
    const Eigen::Vector3d in_map = init_T_map_lidar * homogeneous.head<3>();
    if ((in_map - crop_center_map).squaredNorm() > crop_sq) {
      continue;
    }
    source->point(kept++) << point.x(), point.y(), point.z(), 1.0;
  }
  source->resize(kept);
  result.source_points = kept;
  if (kept < 20) {
    result.reject_reason = "too_few_points";
    return result;
  }

  auto downsampled =
      small_gicp::voxelgrid_sampling_omp(*source, settings_.voxel_size_m, settings_.num_threads);
  if (downsampled->size() < 20) {
    result.reject_reason = "too_few_points";
    return result;
  }
  small_gicp::estimate_covariances_omp(*downsampled, settings_.num_neighbors,
                                       settings_.num_threads);
  result.source_points = downsampled->size();

  Registration registration;
  // The Gauss-Newton update limit-cycles on the near-flat along-road direction
  // of this corridor map; LM damping converges without changing the factors.
  registration.optimizer = small_gicp::LevenbergMarquardtOptimizer();
  registration.optimizer.max_iterations = settings_.max_iterations;
  registration.criteria.translation_eps = settings_.convergence_eps;
  registration.criteria.rotation_eps = settings_.convergence_rotation_rad;

  registration.rejector.max_dist_sq =
      settings_.max_correspondence_init_m * settings_.max_correspondence_init_m;
  auto coarse = registration.align(*map_cloud_, *downsampled, *map_tree_, init_T_map_lidar);

  registration.rejector.max_dist_sq =
      settings_.max_correspondence_final_m * settings_.max_correspondence_final_m;
  auto fine = registration.align(*map_cloud_, *downsampled, *map_tree_, coarse.T_target_source);

  result.converged = fine.converged;
  result.iterations = fine.iterations;
  result.T_target_source = fine.T_target_source;

  // Geometric quality: nearest-neighbour residual of every downsampled source
  // point after alignment. This is the RMSE the KPI gate refers to, not the
  // Mahalanobis solver cost.
  const double max_dist_sq =
      settings_.max_correspondence_final_m * settings_.max_correspondence_final_m;
  double squared_sum = 0.0;
  std::size_t inliers = 0;
  for (std::size_t i = 0; i < downsampled->size(); ++i) {
    const Eigen::Vector4d point = fine.T_target_source * downsampled->point(i);
    std::size_t index = 0;
    double sq_dist = std::numeric_limits<double>::max();
    small_gicp::traits::knn_search(*map_tree_, point, 1, &index, &sq_dist);
    if (sq_dist <= max_dist_sq) {
      ++inliers;
      squared_sum += sq_dist;
    }
  }
  result.inliers = inliers;
  result.inlier_ratio =
      downsampled->size() > 0 ? static_cast<double>(inliers) / downsampled->size() : 0.0;
  result.rmse_m = inliers > 0 ? std::sqrt(squared_sum / inliers) : 0.0;

  // Hessian condition and GICP covariance per contract #9.
  const Eigen::Matrix<double, 6, 6>& hessian = fine.H;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(hessian);
  const auto& eigenvalues = solver.eigenvalues();
  const double lambda_max = eigenvalues.maxCoeff();
  const double lambda_min = eigenvalues.minCoeff();
  result.hessian_condition =
      lambda_min > 0.0 ? lambda_max / lambda_min : std::numeric_limits<double>::infinity();

  const bool hessian_usable =
      std::isfinite(result.hessian_condition) && result.hessian_condition < settings_.condition_gate &&
      lambda_min > 1e-12;
  if (hessian_usable && inliers > 6 && result.rmse_m > 0.0) {
    const double sigma_sq =
        result.rmse_m * result.rmse_m * static_cast<double>(inliers) /
        static_cast<double>(inliers - 6);
    result.covariance = sigma_sq * hessian.inverse();
  } else {
    result.covariance = Eigen::Matrix<double, 6, 6>::Zero();
    result.covariance.diagonal() << 0.3 * 0.3, 0.3 * 0.3, 0.5 * 0.5,
        degrees_to_radians(2.0) * degrees_to_radians(2.0),
        degrees_to_radians(2.0) * degrees_to_radians(2.0),
        degrees_to_radians(3.0) * degrees_to_radians(3.0);
  }

  // Diagonal floor from the contract.
  const Eigen::Matrix<double, 6, 1> floor =
      (Eigen::Matrix<double, 6, 1>() << 0.05 * 0.05, 0.05 * 0.05, 0.1 * 0.1,
       degrees_to_radians(0.5) * degrees_to_radians(0.5),
       degrees_to_radians(0.5) * degrees_to_radians(0.5),
       degrees_to_radians(1.0) * degrees_to_radians(1.0))
          .finished();
  for (int i = 0; i < 6; ++i) {
    result.covariance(i, i) = std::max(result.covariance(i, i), floor(i));
  }

  if (!result.converged) {
    result.reject_reason = "not_converged";
  } else if (result.rmse_m > settings_.rmse_gate_m) {
    result.reject_reason = "rmse_gate";
  } else if (result.inlier_ratio < settings_.inlier_gate) {
    result.reject_reason = "inlier_gate";
  } else if (result.hessian_condition >= settings_.condition_gate) {
    result.reject_reason = "condition_gate";
  } else {
    result.accepted = true;
  }
  return result;
}

}  // namespace brightdrive
