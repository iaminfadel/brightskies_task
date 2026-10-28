// Bundle PCD I/O: the map server is the only reader of map.pcd and its tiles,
// so the reader lives here. The round trip guards the format the builder
// writes (binary xyz float32) against the reader that consumes it.
#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "brightdrive_map_server/pcd_io.hpp"

namespace {

TEST(PcdIo, RoundTripsBinaryXyz) {
  const auto path = std::filesystem::temp_directory_path() / "brightdrive_pcd_test.pcd";
  std::vector<Eigen::Vector3f> points{{1.0f, 2.0f, 3.0f}, {-1.5f, 0.25f, 9.0f}};
  brightdrive::map_server::write_pcd_xyz(path.string(), points);
  const brightdrive::map_server::PcdCloud cloud =
      brightdrive::map_server::read_pcd_xyz(path.string());
  ASSERT_EQ(cloud.points.size(), 2u);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(cloud.points[i], points[i]);
  }
  std::filesystem::remove(path);
}

TEST(PcdIo, RejectsAWrongFieldLayout) {
  const auto path = std::filesystem::temp_directory_path() / "brightdrive_pcd_bad.pcd";
  std::FILE* handle = std::fopen(path.string().c_str(), "wb");
  ASSERT_NE(handle, nullptr);
  std::fputs(
      "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
      "WIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA binary\n",
      handle);
  std::fclose(handle);
  EXPECT_THROW(brightdrive::map_server::read_pcd_xyz(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

}  // namespace
