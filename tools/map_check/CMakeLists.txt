cmake_minimum_required(VERSION 3.16)
project(brightdrive_map_check CXX)

# Standalone development tool: no ROS, no colcon package, nothing installed.
# The production sources are compiled in directly so the matcher and the PCD
# reader are the same code the estimator and the map server run.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(small_gicp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(BRIGHTDRIVE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_executable(map_check
  map_check.cpp
  "${BRIGHTDRIVE_ROOT}/src/brightdrive_localization/src/gicp_matcher.cpp"
  "${BRIGHTDRIVE_ROOT}/src/brightdrive_localization/src/se3.cpp"
  "${BRIGHTDRIVE_ROOT}/src/brightdrive_map_server/src/pcd_io.cpp"
)
target_include_directories(map_check PRIVATE
  "${BRIGHTDRIVE_ROOT}/src/brightdrive_localization/include"
  "${BRIGHTDRIVE_ROOT}/src/brightdrive_map_server/include"
)
target_link_libraries(map_check PRIVATE
  Eigen3::Eigen
  small_gicp::small_gicp
  yaml-cpp
)
target_compile_options(map_check PRIVATE -Wall -Wextra)
