#include "brightdrive_map_server/tile_index.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace brightdrive::map_server {

std::pair<std::int64_t, std::int64_t> cell_of(const TileIndex& index, double x, double y) {
  const double size = index.tile_size_m;
  return {
      static_cast<std::int64_t>(std::floor((x - index.origin_x_m) / size)),
      static_cast<std::int64_t>(std::floor((y - index.origin_y_m) / size)),
  };
}

void tile_center(const TileIndex& index, std::int64_t ix, std::int64_t iy, double& cx, double& cy) {
  cx = index.origin_x_m + (static_cast<double>(ix) + 0.5) * index.tile_size_m;
  cy = index.origin_y_m + (static_cast<double>(iy) + 0.5) * index.tile_size_m;
}

namespace {

double require_number(const YAML::Node& node, const char* what) {
  if (!node || !node.IsScalar()) {
    throw std::runtime_error(std::string("map_index.yaml: missing or malformed ") + what);
  }
  return node.as<double>();
}

}  // namespace

TileIndex load_tile_index(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("map_index.yaml: cannot parse " + path + ": " + error.what());
  }
  if (!root.IsMap()) {
    throw std::runtime_error("map_index.yaml: " + path + " is not a mapping");
  }

  const int schema_version = root["schema_version"] ? root["schema_version"].as<int>() : 0;
  if (schema_version != 1) {
    throw std::runtime_error("map_index.yaml: unsupported schema_version " +
                             std::to_string(schema_version));
  }

  TileIndex index;
  index.map_id = root["map_id"] ? root["map_id"].as<std::string>() : "";
  index.map_frame = root["map_frame"] ? root["map_frame"].as<std::string>() : "map";
  index.tile_size_m = require_number(root["tile_size_m"], "tile_size_m");
  if (!(index.tile_size_m > 0.0)) {
    throw std::runtime_error("map_index.yaml: tile_size_m must be positive");
  }
  index.origin_x_m = require_number(root["origin"]["x_m"], "origin.x_m");
  index.origin_y_m = require_number(root["origin"]["y_m"], "origin.y_m");
  index.z_min_m = require_number(root["z_range_m"][0], "z_range_m[0]");
  index.z_max_m = require_number(root["z_range_m"][1], "z_range_m[1]");

  const YAML::Node files = root["files"];
  if (!files || !files.IsSequence()) {
    throw std::runtime_error("map_index.yaml: files must be a list");
  }
  std::unordered_map<std::string, bool> seen;
  for (const auto& entry : files) {
    TileEntry tile;
    tile.path = entry["path"] ? entry["path"].as<std::string>() : "";
    if (tile.path.empty()) {
      throw std::runtime_error("map_index.yaml: tile entry without path");
    }
    tile.ix = entry["ix"].as<std::int64_t>();
    tile.iy = entry["iy"].as<std::int64_t>();
    tile.point_count = entry["point_count"] ? entry["point_count"].as<std::int64_t>() : 0;
    const std::string key = std::to_string(tile.ix) + "_" + std::to_string(tile.iy);
    if (!seen.emplace(key, true).second) {
      throw std::runtime_error("map_index.yaml: duplicate tile cell " + key);
    }
    index.tiles.push_back(std::move(tile));
  }
  return index;
}

}  // namespace brightdrive::map_server
