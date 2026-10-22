// Saved-map bundle loader. Reads only the runtime install (map.pcd + map.yaml)
// and rejects a hash mismatch before any point is used for matching.
#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include "brightdrive_localization/geodesy.hpp"

namespace brightdrive {

struct MapMetadata {
  int schema_version = 0;
  std::string map_id;
  std::string map_frame;
  GeoAnchor anchor;
  double anchor_stamp = 0.0;
  Eigen::Matrix4d T_map_slam = Eigen::Matrix4d::Identity();
  std::string pcd_sha256;
  double voxel_size_m = 0.25;
  std::size_t point_count = 0;
  double coverage_radius_m = 20.0;
  std::string source_run_id;
};

struct MapLoadResult {
  MapMetadata metadata;
  small_gicp::PointCloud::Ptr cloud;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree;
  double load_ms = 0.0;
  std::size_t input_points = 0;
};

// Loads <map_dir>/map.yaml and <map_dir>/map.pcd, verifies the SHA-256, and
// preprocesses the cloud for GICP. Throws std::runtime_error on any contract
// violation (missing file, hash mismatch, malformed metadata).
MapLoadResult load_map_bundle(const std::string& map_dir, int num_threads);

}  // namespace brightdrive
