// Minimal reader/writer for the binary XYZ PCD files produced by the offline
// map builder. The map bundle is ours, so only the formats we generate are
// supported; anything else is rejected rather than silently misread.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

namespace brightdrive {

struct PcdCloud {
  std::vector<Eigen::Vector3f> points;
  bool binary = true;
};

// Throws std::runtime_error on malformed input.
PcdCloud read_pcd_xyz(const std::string& path);

void write_pcd_xyz(const std::string& path, const std::vector<Eigen::Vector3f>& points);

bool file_exists(const std::string& path);

}  // namespace brightdrive
