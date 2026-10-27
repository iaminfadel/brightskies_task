cmake_minimum_required(VERSION 3.10)
project(brightdrive_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(brightdrive_localization REQUIRED)

add_library(brightdrive_map_server_core
  src/tile_index.cpp
  src/window.cpp
)
target_include_directories(brightdrive_map_server_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(brightdrive_map_server_core PUBLIC
  Eigen3::Eigen
  yaml-cpp
)
target_compile_options(brightdrive_map_server_core PRIVATE -Wall -Wextra)

add_executable(map_server src/map_server.cpp)
target_link_libraries(map_server
  brightdrive_map_server_core
  brightdrive_localization::brightdrive_localization_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

install(TARGETS map_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_tile_index test/test_tile_index.cpp)
    target_link_libraries(test_tile_index brightdrive_map_server_core)

    ament_add_gtest(test_window test/test_window.cpp)
    target_link_libraries(test_window brightdrive_map_server_core)
  else()
    message(WARNING "ament_cmake_gtest not found; unit tests skipped")
  endif()

  find_package(launch_testing_ament_cmake QUIET)
  if(launch_testing_ament_cmake_FOUND)
    add_launch_test(test/test_map_server_launch.py TIMEOUT 120)
  else()
    message(WARNING "launch_testing_ament_cmake not found; interface test skipped")
  endif()
endif()

ament_package()
