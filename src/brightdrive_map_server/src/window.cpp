#include "brightdrive_map_server/window.hpp"

#include <cmath>
#include <unordered_set>

namespace brightdrive::map_server {

std::string cell_key(std::int64_t ix, std::int64_t iy) {
  return std::to_string(ix) + "_" + std::to_string(iy);
}

std::set<std::string> serving_set(const TileIndex& index, double x, double y,
                                  const std::set<std::string>& current,
                                  const WindowParams& params) {
  std::set<std::string> next;
  for (const auto& tile : index.tiles) {
    double cx = 0.0;
    double cy = 0.0;
    tile_center(index, tile.ix, tile.iy, cx, cy);
    const double distance = std::hypot(cx - x, cy - y);
    const bool loaded = current.count(cell_key(tile.ix, tile.iy)) > 0;
    if (distance <= params.load_radius_m || (loaded && distance <= params.evict_radius_m)) {
      next.insert(cell_key(tile.ix, tile.iy));
    }
  }
  return next;
}

std::vector<Eigen::Vector3f> voxel_downsample(const std::vector<Eigen::Vector3f>& points,
                                              double voxel) {
  if (voxel <= 0.0 || points.empty()) {
    return points;
  }
  std::unordered_set<std::string> seen;
  seen.reserve(points.size());
  std::vector<Eigen::Vector3f> kept;
  kept.reserve(points.size());
  for (const auto& point : points) {
    const long long kx = static_cast<long long>(std::floor(point.x() / voxel));
    const long long ky = static_cast<long long>(std::floor(point.y() / voxel));
    const long long kz = static_cast<long long>(std::floor(point.z() / voxel));
    if (seen.insert(std::to_string(kx) + "_" + std::to_string(ky) + "_" + std::to_string(kz))
            .second) {
      kept.push_back(point);
    }
  }
  return kept;
}

}  // namespace brightdrive::map_server
