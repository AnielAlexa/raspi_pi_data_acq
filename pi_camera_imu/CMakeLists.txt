cmake_minimum_required(VERSION 3.8)
project(pi_camera_imu)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Camera Node
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)
target_link_libraries(camera_node ${OpenCV_LIBS})

# IMU Camera Sync Node
add_executable(imu_camera_sync_node src/imu_camera_sync_node.cpp)
ament_target_dependencies(imu_camera_sync_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Standalone Camera Test (minimal, no ROS dependencies for testing)
add_executable(camera_test src/standalone_camera_test.cpp)
ament_target_dependencies(camera_test
  rclcpp
  sensor_msgs
)
target_link_libraries(camera_test ${OpenCV_LIBS})

# Install executables
install(TARGETS
  camera_node
  imu_camera_sync_node
  camera_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
