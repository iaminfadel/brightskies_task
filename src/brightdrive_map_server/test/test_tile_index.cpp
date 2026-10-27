// Tile index contract: the map_index.yaml the builder emits (#32) parses,
// cell lookup is O(1) floor division (negative coordinates included), and
// malformed indexes are rejected rather than silently misread.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "brightdrive_map_server/tile_index.hpp"

namespace {

const char* kIndexJson = R"({
  "schema_version": 1,
  "map_id": "unit-map",
  "map_frame": "map",
  "tile_size_m": 10.0,
  "origin": {"x_m": -5.0, "y_m": 2.0},
  "z_range_m": [-1.5, 4.0],
  "tile_count": 2,
  "point_count": 42,
  "files": [
    {"path": "tiles/tile_0_0.pcd", "ix": 0, "iy": 0, "point_count": 30},
    {"path": "tiles/tile_-1_2.pcd", "ix": -1, "iy": 2, "point_count": 12}
  ]
})";

std::string write_index(const std::string& directory, const std::string& content) {
  const std::string path = directory + "/map_index.yaml";
  std::ofstream out(path, std::ios::binary);
  out << content;
  return path;
}

}  // namespace

TEST(TileIndex, ParsesTheBuilderBundle) {
  const std::string path = write_index("/tmp", kIndexJson);
  const brightdrive::map_server::TileIndex index = brightdrive::map_server::load_tile_index(path);
  EXPECT_EQ(index.map_id, "unit-map");
  EXPECT_EQ(index.map_frame, "map");
  EXPECT_DOUBLE_EQ(index.tile_size_m, 10.0);
  EXPECT_DOUBLE_EQ(index.origin_x_m, -5.0);
  EXPECT_DOUBLE_EQ(index.origin_y_m, 2.0);
  EXPECT_DOUBLE_EQ(index.z_min_m, -1.5);
  EXPECT_DOUBLE_EQ(index.z_max_m, 4.0);
  ASSERT_EQ(index.tiles.size(), 2u);
  EXPECT_EQ(index.tiles[0].path, "tiles/tile_0_0.pcd");
  EXPECT_EQ(index.tiles[0].ix, 0);
  EXPECT_EQ(index.tiles[0].point_count, 30);
  EXPECT_EQ(index.tiles[1].ix, -1);
  std::remove(path.c_str());
}

TEST(TileIndex, TileCenter) {
  brightdrive::map_server::TileIndex index;
  index.tile_size_m = 25.0;
  index.origin_x_m = -100.0;
  index.origin_y_m = -50.0;
  double cx = 0.0;
  double cy = 0.0;
  brightdrive::map_server::tile_center(index, 3, -2, cx, cy);
  EXPECT_DOUBLE_EQ(cx, -100.0 + 3.5 * 25.0);
  EXPECT_DOUBLE_EQ(cy, -50.0 - 1.5 * 25.0);
}

TEST(TileIndex, RejectsDuplicateCells) {
  const std::string path = write_index("/tmp", R"({
    "schema_version": 1,
    "tile_size_m": 10.0,
    "origin": {"x_m": 0.0, "y_m": 0.0},
    "z_range_m": [0.0, 0.0],
    "files": [
      {"path": "tiles/tile_1_1.pcd", "ix": 1, "iy": 1},
      {"path": "tiles/tile_1_1b.pcd", "ix": 1, "iy": 1}
    ]
  })");
  EXPECT_THROW(brightdrive::map_server::load_tile_index(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(TileIndex, RejectsBadSchemaAndBadTileSize) {
  const std::string bad_schema = write_index("/tmp", R"({
    "schema_version": 2, "tile_size_m": 10.0,
    "origin": {"x_m": 0.0, "y_m": 0.0}, "z_range_m": [0.0, 0.0], "files": []
  })");
  EXPECT_THROW(brightdrive::map_server::load_tile_index(bad_schema), std::runtime_error);
  std::remove(bad_schema.c_str());

  const std::string bad_size = write_index("/tmp", R"({
    "schema_version": 1, "tile_size_m": 0.0,
    "origin": {"x_m": 0.0, "y_m": 0.0}, "z_range_m": [0.0, 0.0], "files": []
  })");
  EXPECT_THROW(brightdrive::map_server::load_tile_index(bad_size), std::runtime_error);
  std::remove(bad_size.c_str());
}

TEST(TileIndex, ThrowsOnMissingFile) {
  EXPECT_THROW(brightdrive::map_server::load_tile_index("/tmp/definitely-not-here.yaml"),
               std::runtime_error);
}
