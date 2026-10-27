// Windowed-serving set arithmetic (ADR 0003): pure functions over the tile
// index so the hysteresis behaviour is unit-testable without a ROS runtime.
#pragma once

#include <Eigen/Core>
#include <set>
#include <string>
#include <vector>

#include "brightdrive_map_server/tile_index.hpp"

namespace brightdrive::map_server {

struct WindowParams {
  double load_radius_m = 0.0;   // R_load = lidar_range_m + margin_m
  double evict_radius_m = 0.0;  // R_evict = R_load + hysteresis_m
};

// The serving set for a pose: every tile whose centre is within R_load of
// (x, y) must be served; a tile already in `current` survives until its
// centre passes R_evict, so boundary noise cannot thrash loads.
std::set<std::string> serving_set(const TileIndex& index, double x, double y,
                                  const std::set<std::string>& current, const WindowParams& params);

// Keys the tile cache and the serving set ("ix_iy", matching the bundle's
// tile file naming).
std::string cell_key(std::int64_t ix, std::int64_t iy);

// First-point-wins hash-grid voxel downsample; voxel <= 0 returns the input
// unchanged. The window merge applies it only when the parameter is set --
// tiles are already downsampled by the builder.
std::vector<Eigen::Vector3f> voxel_downsample(const std::vector<Eigen::Vector3f>& points,
                                              double voxel);

}  // namespace brightdrive::map_server
