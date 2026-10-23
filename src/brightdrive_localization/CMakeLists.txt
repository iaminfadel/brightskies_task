cmake_minimum_required(VERSION 3.10)
project(brightdrive_localization)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(brightdrive_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(small_gicp REQUIRED)

add_library(brightdrive_localization_core
  src/geodesy.cpp
  src/pcd_io.cpp
  src/map_loader.cpp
  src/gicp_matcher.cpp
  src/health.cpp
  src/se3.cpp
  src/hash.cpp
)
target_include_directories(brightdrive_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(brightdrive_localization_core PUBLIC
  Eigen3::Eigen
  yaml-cpp
  small_gicp::small_gicp
)
target_compile_options(brightdrive_localization_core PRIVATE -Wall -Wextra)

add_executable(localization_core src/localization_core.cpp)
target_link_libraries(localization_core
  brightdrive_localization_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_srvs_TARGETS}
  brightdrive_interfaces::brightdrive_interfaces__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
)

add_executable(map_check src/map_check.cpp)
target_link_libraries(map_check brightdrive_localization_core)

install(TARGETS localization_core map_check
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_geodesy test/test_geodesy.cpp)
    target_link_libraries(test_geodesy brightdrive_localization_core)

    ament_add_gtest(test_se3 test/test_se3.cpp)
    target_link_libraries(test_se3 brightdrive_localization_core)

    ament_add_gtest(test_health test/test_health.cpp)
    target_link_libraries(test_health brightdrive_localization_core)

    ament_add_gtest(test_map_loader test/test_map_loader.cpp)
    target_link_libraries(test_map_loader brightdrive_localization_core)
  else()
    message(WARNING "ament_cmake_gtest not found; unit tests skipped")
  endif()

  find_package(launch_testing_ament_cmake QUIET)
  if(launch_testing_ament_cmake_FOUND)
    add_launch_test(test/test_interface_launch.py TIMEOUT 120)
  else()
    message(WARNING "launch_testing_ament_cmake not found; interface test skipped")
  endif()
endif()

ament_package()
