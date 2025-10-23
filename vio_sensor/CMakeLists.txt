cmake_minimum_required(VERSION 3.8)
project(vio_sensor)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find libcamera
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Include directories
include_directories(
  include
  ${LIBCAMERA_INCLUDE_DIRS}
)

# Serial reader library
add_library(serial_reader
  src/serial_reader.cpp
)
ament_target_dependencies(serial_reader
  rclcpp
  sensor_msgs
)

# Camera capture library
add_library(camera_capture
  src/camera_capture.cpp
)
target_link_libraries(camera_capture
  ${LIBCAMERA_LIBRARIES}
)
ament_target_dependencies(camera_capture
  rclcpp
  sensor_msgs
)

# Main VIO sensor node executable (DEPRECATED - use 2-node architecture instead)
add_executable(vio_sensor_node
  src/vio_sensor_node.cpp
)
target_link_libraries(vio_sensor_node
  serial_reader
  camera_capture
)
ament_target_dependencies(vio_sensor_node
  rclcpp
  sensor_msgs
)

# NEW: Camera-only node (lightweight, no IMU)
add_executable(camera_only_node
  src/camera_only_node.cpp
)
target_link_libraries(camera_only_node
  ${LIBCAMERA_LIBRARIES}
)
ament_target_dependencies(camera_only_node
  rclcpp
  sensor_msgs
)

# NEW: IMU sync node (subscribes to camera)
add_executable(imu_sync_node
  src/imu_sync_node_v2.cpp
)
target_link_libraries(imu_sync_node
  serial_reader
)
ament_target_dependencies(imu_sync_node
  rclcpp
  sensor_msgs
)

# NEWEST: Standalone VIO node (single process, based on working standalone_camera_test.cpp)
add_executable(standalone_vio_node
  src/standalone_vio_node.cpp
)
target_link_libraries(standalone_vio_node
  serial_reader
  ${LIBCAMERA_LIBRARIES}
)
ament_target_dependencies(standalone_vio_node
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  vio_sensor_node
  camera_only_node
  imu_sync_node
  standalone_vio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
