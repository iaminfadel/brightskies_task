// Reader for the tiled map bundle's map_index.yaml, the contract emitted by
// scripts/build_map.py (#32) and fixed by ADR 0003: a regular 2D X/Y grid
// (full Z range) addressed by O(1) floor division against the index origin.
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace brightdrive::map_server {

struct TileEntry {
  std::string path;  // relative to the bundle directory
  std::int64_t ix = 0;
  std::int64_t iy = 0;
  std::int64_t point_count = 0;
};

struct TileIndex {
  std::string map_id;
  std::string map_frame;
  double tile_size_m = 0.0;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  double z_min_m = 0.0;
  double z_max_m = 0.0;
  std::vector<TileEntry> tiles;
};

// Centre of a grid cell in map coordinates (the point tile distance is
// measured against for windowing).
void tile_center(const TileIndex& index, std::int64_t ix, std::int64_t iy, double& cx, double& cy);

// Parses <bundle>/map_index.yaml. The builder writes it as JSON text, which
// yaml-cpp reads as YAML 1.2. Throws std::runtime_error on any contract
// violation (missing file, negative/zero tile size, duplicate cells).
TileIndex load_tile_index(const std::string& path);

}  // namespace brightdrive::map_server
