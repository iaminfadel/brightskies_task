#include "brightdrive_localization/map_loader.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include "brightdrive_localization/hash.hpp"
#include "brightdrive_localization/pcd_io.hpp"

namespace brightdrive {
namespace {

Eigen::Matrix4d read_matrix4(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("map.yaml missing " + key);
  }
  const auto values = node[key].as<std::vector<double>>();
  if (values.size() != 16) {
    throw std::runtime_error("map.yaml " + key + " must contain 16 floats");
  }
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      matrix(row, column) = values[static_cast<std::size_t>(row * 4 + column)];
    }
  }
  return matrix;
}

}  // namespace

MapLoadResult load_map_bundle(const std::string& map_dir, int num_threads) {
  const auto start = std::chrono::steady_clock::now();

  MapLoadResult result;
  const std::string yaml_path = map_dir + "/map.yaml";
  const std::string pcd_path = map_dir + "/map.pcd";
  if (!file_exists(yaml_path)) {
    throw std::runtime_error("missing saved-map metadata: " + yaml_path);
  }
  if (!file_exists(pcd_path)) {
    throw std::runtime_error("missing saved map: " + pcd_path);
  }

  const YAML::Node root = YAML::LoadFile(yaml_path);
  MapMetadata& metadata = result.metadata;
  metadata.schema_version = root["schema_version"] ? root["schema_version"].as<int>() : 0;
  metadata.map_id = root["map_id"] ? root["map_id"].as<std::string>() : "";
  metadata.map_frame = root["map_frame"] ? root["map_frame"].as<std::string>() : "";
  metadata.pcd_sha256 = root["pcd_sha256"] ? root["pcd_sha256"].as<std::string>() : "";
  metadata.voxel_size_m = root["voxel_size_m"] ? root["voxel_size_m"].as<double>() : 0.25;
  metadata.point_count = root["point_count"] ? root["point_count"].as<std::size_t>() : 0;
  metadata.coverage_radius_m =
      root["coverage_radius_m"] ? root["coverage_radius_m"].as<double>() : 20.0;
  metadata.source_run_id = root["source"] && root["source"]["run_id"]
                               ? root["source"]["run_id"].as<std::string>()
                               : "";
  metadata.T_map_slam = read_matrix4(root, "T_map_slam");

  if (metadata.schema_version != 1) {
    throw std::runtime_error("unsupported map schema_version");
  }
  if (metadata.map_frame != "map") {
    throw std::runtime_error("saved map must use the map frame");
  }
  if (metadata.pcd_sha256.size() != 64) {
    throw std::runtime_error("map.yaml pcd_sha256 is missing or malformed");
  }
  if (!root["anchor"]) {
    throw std::runtime_error("map.yaml missing anchor");
  }
  metadata.anchor.lat_deg = root["anchor"]["lat_deg"].as<double>();
  metadata.anchor.lon_deg = root["anchor"]["lon_deg"].as<double>();
  metadata.anchor.alt_m = root["anchor"]["alt_m"].as<double>();
  metadata.anchor_stamp = root["anchor"]["stamp"] ? root["anchor"]["stamp"].as<double>() : 0.0;

  const std::string actual_hash = sha256_file_hex(pcd_path);
  if (actual_hash != metadata.pcd_sha256) {
    throw std::runtime_error("saved-map hash mismatch: metadata=" + metadata.pcd_sha256 +
                             " actual=" + actual_hash);
  }

  const PcdCloud pcd = read_pcd_xyz(pcd_path);
  result.input_points = pcd.points.size();
  if (pcd.points.empty()) {
    throw std::runtime_error("saved map contains no points");
  }

  small_gicp::PointCloud::Ptr raw(new small_gicp::PointCloud);
  raw->resize(pcd.points.size());
  for (std::size_t i = 0; i < pcd.points.size(); ++i) {
    raw->point(i) << static_cast<double>(pcd.points[i].x()),
        static_cast<double>(pcd.points[i].y()),
        static_cast<double>(pcd.points[i].z()), 1.0;
  }

  auto preprocessed = small_gicp::preprocess_points(
      *raw, metadata.voxel_size_m, 10, num_threads > 0 ? num_threads : 4);
  result.cloud = preprocessed.first;
  result.tree = preprocessed.second;

  const auto end = std::chrono::steady_clock::now();
  result.load_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  return result;
}

}  // namespace brightdrive
