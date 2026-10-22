#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "brightdrive_localization/hash.hpp"
#include "brightdrive_localization/map_loader.hpp"
#include "brightdrive_localization/pcd_io.hpp"

namespace brightdrive {
namespace {

std::string write_bundle(const std::filesystem::path& dir, int point_count = 200,
                         bool corrupt_hash = false) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(static_cast<std::size_t>(point_count));
  for (int i = 0; i < point_count; ++i) {
    points.emplace_back(static_cast<float>(i % 10) * 0.5f,
                        static_cast<float>(i / 10) * 0.5f,
                        static_cast<float>(i % 3) * 0.1f);
  }
  const std::string pcd_path = (dir / "map.pcd").string();
  write_pcd_xyz(pcd_path, points);
  const std::string hash = sha256_file_hex(pcd_path);

  std::ofstream yaml(dir / "map.yaml");
  yaml << "schema_version: 1\n"
       << "map_id: test-map\n"
       << "map_frame: map\n"
       << "anchor:\n"
       << "  lat_deg: 0.0\n"
       << "  lon_deg: 0.0\n"
       << "  alt_m: 0.0\n"
       << "  stamp: 1.5\n"
       << "T_map_slam: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
       << "pcd_sha256: "
       << (corrupt_hash ? std::string(64, 'a') : hash) << "\n"
       << "voxel_size_m: 0.25\n"
       << "point_count: " << point_count << "\n"
       << "coverage_radius_m: 20.0\n"
       << "source:\n"
       << "  run_id: test-run\n";
  return hash;
}

class MapLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("brightdrive_map_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }
  std::filesystem::path dir_;
};

TEST_F(MapLoaderTest, LoadsAndVerifiesTheBundle) {
  write_bundle(dir_);
  const MapLoadResult result = load_map_bundle(dir_.string(), 1);
  EXPECT_EQ(result.metadata.map_id, "test-map");
  EXPECT_EQ(result.metadata.schema_version, 1);
  EXPECT_EQ(result.metadata.point_count, 200u);
  EXPECT_NEAR(result.metadata.anchor.lat_deg, 0.0, 1e-12);
  EXPECT_EQ(result.input_points, 200u);
  EXPECT_GE(result.cloud->size(), 1u);
  EXPECT_TRUE(result.tree);
  EXPECT_GE(result.load_ms, 0.0);
}

TEST_F(MapLoaderTest, RejectsHashMismatch) {
  write_bundle(dir_, 200, true);
  EXPECT_THROW(load_map_bundle(dir_.string(), 1), std::runtime_error);
}

TEST_F(MapLoaderTest, RejectsMissingMetadata) {
  EXPECT_THROW(load_map_bundle(dir_.string(), 1), std::runtime_error);
}

TEST_F(MapLoaderTest, RejectsUnsupportedSchema) {
  write_bundle(dir_);
  std::ofstream yaml(dir_ / "map.yaml");
  yaml << "schema_version: 2\n";
  EXPECT_THROW(load_map_bundle(dir_.string(), 1), std::runtime_error);
}

TEST(PcdIo, RoundTripsBinaryXyz) {
  const auto path = std::filesystem::temp_directory_path() / "brightdrive_pcd_test.pcd";
  std::vector<Eigen::Vector3f> points{{1.0f, 2.0f, 3.0f}, {-1.5f, 0.25f, 9.0f}};
  write_pcd_xyz(path.string(), points);
  const PcdCloud cloud = read_pcd_xyz(path.string());
  ASSERT_EQ(cloud.points.size(), 2u);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(cloud.points[i], points[i]);
  }
  std::filesystem::remove(path);
}

}  // namespace
}  // namespace brightdrive
