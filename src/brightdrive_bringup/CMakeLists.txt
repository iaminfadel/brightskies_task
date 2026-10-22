cmake_minimum_required(VERSION 3.10)
project(brightdrive_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/imu_contract_covariance.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
