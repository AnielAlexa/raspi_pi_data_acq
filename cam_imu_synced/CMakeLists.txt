cmake_minimum_required(VERSION 3.8)
project(cam_imu_synced)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find libcamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Include directories
include_directories(include)

# IMU Trigger Component
add_library(imu_trigger_component SHARED
  src/imu_trigger_component.cpp
)

ament_target_dependencies(imu_trigger_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

rclcpp_components_register_node(imu_trigger_component
  PLUGIN "cam_imu_synced::ImuTriggerComponent"
  EXECUTABLE imu_trigger_node
)

# Camera Component
add_library(camera_component SHARED
  src/camera_component.cpp
)

ament_target_dependencies(camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

target_link_libraries(camera_component
  PkgConfig::LIBCAMERA
)

rclcpp_components_register_node(camera_component
  PLUGIN "cam_imu_synced::CameraComponent"
  EXECUTABLE camera_node
)

# Install targets
install(TARGETS
  imu_trigger_component
  camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
