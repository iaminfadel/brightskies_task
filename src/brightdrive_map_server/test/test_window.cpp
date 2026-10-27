// Windowed-serving arithmetic: R_load membership, R_evict hysteresis (no
// thrash when the pose straddles a tile boundary), and the voxel merge.
#include <gtest/gtest.h>

#include "brightdrive_map_server/window.hpp"

namespace {

brightdrive::map_server::TileIndex grid(int cells_per_side, double tile_size) {
  brightdrive::map_server::TileIndex index;
  index.map_frame = "map";
  index.tile_size_m = tile_size;
  index.origin_x_m = 0.0;
  index.origin_y_m = 0.0;
  for (int iy = 0; iy < cells_per_side; ++iy) {
    for (int ix = 0; ix < cells_per_side; ++ix) {
      brightdrive::map_server::TileEntry tile;
      tile.path = "tiles/tile_" + std::to_string(ix) + "_" + std::to_string(iy) + ".pcd";
      tile.ix = ix;
      tile.iy = iy;
      index.tiles.push_back(tile);
    }
  }
  return index;
}

}  // namespace

TEST(ServingSet, LoadsTilesWithinRLoad) {
  const brightdrive::map_server::TileIndex index = grid(4, 10.0);
  const brightdrive::map_server::WindowParams params{15.0, 30.0};
  const std::set<std::string> next =
      brightdrive::map_server::serving_set(index, 5.0, 5.0, {}, params);
  // Centre distances from (5, 5): (0,0)=0, (1,0)=10, (0,1)=10, (1,1)=14.1;
  // (2,0)=20 and beyond are outside R_load.
  const std::set<std::string> expected{"0_0", "1_0", "0_1", "1_1"};
  EXPECT_EQ(next, expected);
}

TEST(ServingSet, HysteresisKeepsTilesWithinREvict) {
  const brightdrive::map_server::TileIndex index = grid(4, 10.0);
  const brightdrive::map_server::WindowParams params{15.0, 30.0};
  const std::set<std::string> current =
      brightdrive::map_server::serving_set(index, 5.0, 5.0, {}, params);

  // Pose moves to (25, 5): tile (0,0) centre is 20 m away -- outside
  // R_load but inside R_evict and already loaded, so it survives (no
  // thrash). (2,0) and (3,0) are now inside R_load and join.
  const std::set<std::string> next =
      brightdrive::map_server::serving_set(index, 25.0, 5.0, current, params);
  EXPECT_EQ(next.count("0_0"), 1u);
  EXPECT_EQ(next.count("2_0"), 1u);
  EXPECT_EQ(next.count("3_0"), 1u);

  // A tile NOT in current and outside R_load is not preloaded just because
  // it sits inside R_evict.
  const std::set<std::string> fresh =
      brightdrive::map_server::serving_set(index, 25.0, 5.0, {}, params);
  EXPECT_EQ(fresh.count("0_0"), 0u);
}

TEST(ServingSet, EvictsBeyondREvict) {
  const brightdrive::map_server::TileIndex index = grid(6, 10.0);
  const brightdrive::map_server::WindowParams params{15.0, 30.0};
  const std::set<std::string> current =
      brightdrive::map_server::serving_set(index, 5.0, 5.0, {}, params);

  // Pose jumps to (55, 5): tile (0,0) centre is 55 m away, beyond R_evict.
  const std::set<std::string> next =
      brightdrive::map_server::serving_set(index, 55.0, 5.0, current, params);
  EXPECT_EQ(next.count("0_0"), 0u);
  EXPECT_EQ(next.count("4_0"), 1u);  // centre (45, 5), 10 m away
}

TEST(ServingSet, EmptyIndexServesNothing) {
  brightdrive::map_server::TileIndex index;
  index.tile_size_m = 10.0;
  const brightdrive::map_server::WindowParams params{15.0, 30.0};
  EXPECT_TRUE(brightdrive::map_server::serving_set(index, 0.0, 0.0, {}, params).empty());
}

TEST(VoxelDownsample, KeepsFirstPointPerVoxel) {
  std::vector<Eigen::Vector3f> points = {
      {0.1f, 0.1f, 0.1f},
      {0.9f, 0.2f, 0.3f},  // same voxel (0,0,0)
      {1.1f, 0.1f, 0.1f},  // voxel (1,0,0)
  };
  const std::vector<Eigen::Vector3f> kept = brightdrive::map_server::voxel_downsample(points, 1.0);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_FLOAT_EQ(kept[0].x(), 0.1f);  // first wins
  EXPECT_FLOAT_EQ(kept[1].x(), 1.1f);
}

TEST(VoxelDownsample, DisabledOrEmpty) {
  std::vector<Eigen::Vector3f> points = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
  EXPECT_EQ(brightdrive::map_server::voxel_downsample(points, 0.0).size(), 2u);
  EXPECT_EQ(brightdrive::map_server::voxel_downsample(points, -1.0).size(), 2u);
  EXPECT_TRUE(brightdrive::map_server::voxel_downsample({}, 1.0).empty());
}
