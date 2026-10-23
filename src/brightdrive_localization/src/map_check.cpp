// Held-out map consistency check using the production GICP matcher.
//
// Reads <map_dir>/validation/heldout/poses.csv and the held-out scans, matches
// each scan to the saved map starting from identity with the #9 settings, and
// writes validation/map_consistency.json. This runs the same C++ matcher as
// localization_core, so the Hessian condition and convergence semantics match
// the live pipeline.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "brightdrive_localization/gicp_matcher.hpp"
#include "brightdrive_localization/map_loader.hpp"
#include "brightdrive_localization/pcd_io.hpp"
#include "brightdrive_localization/se3.hpp"

namespace {

struct HeldoutPose {
  int index = 0;
  double stamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

std::vector<HeldoutPose> read_poses(const std::string& path) {
  std::ifstream handle(path);
  if (!handle) {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<HeldoutPose> poses;
  std::string line;
  std::getline(handle, line);
  while (std::getline(handle, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream stream(line);
    std::string field;
    std::vector<std::string> fields;
    while (std::getline(stream, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() < 9) {
      throw std::runtime_error("malformed pose row: " + line);
    }
    HeldoutPose pose;
    pose.index = std::stoi(fields[0]);
    pose.stamp = std::stod(fields[1]);
    pose.position << std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4]);
    poses.push_back(pose);
  }
  return poses;
}

double innovation_d2(const Eigen::Isometry3d& estimate) {
  const double sigma_xy = 1.0;
  const double sigma_z = 2.0;
  const double sigma_yaw = 10.0 * M_PI / 180.0;
  const Eigen::Vector3d delta = estimate.translation();
  const double yaw_delta = brightdrive::yaw_of(estimate);
  return std::pow(delta.x() / sigma_xy, 2) + std::pow(delta.y() / sigma_xy, 2) +
         std::pow(delta.z() / sigma_z, 2) + std::pow(yaw_delta / sigma_yaw, 2);
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double index = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double weight = index - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

std::string number(double value) {
  std::ostringstream stream;
  stream.precision(9);
  stream << value;
  return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2) {
      std::cerr << "usage: map_check <map_dir> [json_out]\n";
      return 2;
    }
    const std::string map_dir = argv[1];
    const std::string json_out =
        argc > 2 ? argv[2] : map_dir + "/validation/map_consistency.json";

    auto map = brightdrive::load_map_bundle(map_dir, 2);
    brightdrive::GicpSettings settings;
    brightdrive::GicpMatcher matcher(settings, map.cloud, map.tree);

    const std::string heldout_yaml_path = map_dir + "/validation/heldout.yaml";
    if (!brightdrive::file_exists(heldout_yaml_path)) {
      throw std::runtime_error("missing " + heldout_yaml_path);
    }
    const auto poses = read_poses(map_dir + "/validation/heldout/poses.csv");
    if (poses.empty()) {
      throw std::runtime_error("no held-out poses");
    }

    struct Row {
      int index;
      bool converged;
      bool accepted;
      std::string reason;
      double rmse;
      double inlier_ratio;
      double condition;
      double d2;
      std::size_t iterations;
      std::size_t source_points;
    };
    std::vector<Row> rows;
    std::vector<double> rmses;
    std::size_t accepted = 0;
    for (const auto& pose : poses) {
      char name[64];
      std::snprintf(name, sizeof(name), "/validation/heldout/scans/scan_%04d.pcd",
                    pose.index);
      const auto cloud = brightdrive::read_pcd_xyz(map_dir + name);
      const auto result = matcher.align(cloud.points, Eigen::Isometry3d::Identity(),
                                        pose.position);
      const double d2 = innovation_d2(result.T_target_source);
      const bool ok = result.accepted && d2 < 18.47;
      if (ok) {
        ++accepted;
      }
      rmses.push_back(result.rmse_m);
      rows.push_back(Row{pose.index, result.converged, ok, result.reject_reason,
                         result.rmse_m, result.inlier_ratio, result.hessian_condition,
                         d2, result.iterations, result.source_points});
      std::cout << "MAP_CHECK index=" << pose.index
                << " converged=" << (result.converged ? "true" : "false")
                << " accepted=" << (ok ? "true" : "false")
                << " reason=" << (result.reject_reason.empty() ? "none" : result.reject_reason)
                << " rmse_m=" << number(result.rmse_m)
                << " inlier_ratio=" << number(result.inlier_ratio)
                << " condition=" << number(result.hessian_condition)
                << " d2=" << number(d2) << " iterations=" << result.iterations
                << " source_points=" << result.source_points << std::endl;
    }

    const double median_rmse = percentile(rmses, 0.5);
    const double mean_rmse =
        std::accumulate(rmses.begin(), rmses.end(), 0.0) / static_cast<double>(rmses.size());
    const double p95_rmse = percentile(rmses, 0.95);
    const double max_rmse = *std::max_element(rmses.begin(), rmses.end());
    const double fraction = static_cast<double>(accepted) / static_cast<double>(poses.size());
    const bool passed = fraction >= 0.80 && median_rmse <= 0.20;

    std::ostringstream json;
    json.precision(9);
    json << "{\n";
    json << "  \"schema_version\": 1,\n";
    json << "  \"map_id\": \"" << map.metadata.map_id << "\",\n";
    json << "  \"scans\": " << poses.size() << ",\n";
    json << "  \"accepted\": " << accepted << ",\n";
    json << "  \"accepted_fraction\": " << number(fraction) << ",\n";
    json << "  \"median_rmse_m\": " << number(median_rmse) << ",\n";
    json << "  \"mean_rmse_m\": " << number(mean_rmse) << ",\n";
    json << "  \"p95_rmse_m\": " << number(p95_rmse) << ",\n";
    json << "  \"max_rmse_m\": " << number(max_rmse) << ",\n";
    json << "  \"gates\": {\"accepted_fraction_min\": 0.8, \"median_rmse_max_m\": 0.2,"
            " \"rmse_m\": 0.2, \"inlier_ratio\": 0.6, \"condition\": 1000000.0,"
            " \"innovation_d2\": 18.47},\n";
    json << "  \"matcher\": \"brightdrive::GicpMatcher\",\n";
    json << "  \"passed\": " << (passed ? "true" : "false") << ",\n";
    json << "  \"scans_detail\": [\n";
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const auto& row = rows[i];
      json << "    {\"index\": " << row.index
           << ", \"converged\": " << (row.converged ? "true" : "false")
           << ", \"accepted\": " << (row.accepted ? "true" : "false")
           << ", \"reject_reason\": \"" << row.reason << "\""
           << ", \"rmse_m\": " << number(row.rmse)
           << ", \"inlier_ratio\": " << number(row.inlier_ratio)
           << ", \"condition\": " << number(row.condition)
           << ", \"innovation_d2\": " << number(row.d2)
           << ", \"iterations\": " << row.iterations
           << ", \"source_points\": " << row.source_points << "}"
           << (i + 1 == rows.size() ? "\n" : ",\n");
    }
    json << "  ]\n}\n";

    std::ofstream handle(json_out);
    if (!handle) {
      throw std::runtime_error("cannot write " + json_out);
    }
    handle << json.str();

    std::cout << "MAP_CONSISTENCY scans=" << poses.size() << " accepted=" << accepted
              << " fraction=" << number(fraction) << " median_rmse_m=" << number(median_rmse)
              << " mean_rmse_m=" << number(mean_rmse) << " p95_rmse_m=" << number(p95_rmse)
              << " max_rmse_m=" << number(max_rmse) << " passed=" << (passed ? "true" : "false")
              << std::endl;
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "map_check: " << error.what() << "\n";
    return 1;
  }
}
