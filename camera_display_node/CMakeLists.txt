cmake_minimum_required(VERSION 3.8)
project(camera_display_node)

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
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find libcamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Include directories
include_directories(include)

# Create camera display executable
add_executable(camera_display_node
  src/camera_display.cpp
  src/serial_sync.cpp
)

# Create FPS subscriber executable
add_executable(camera_fps_subscriber
  src/camera_fps_subscriber.cpp
)

# Link dependencies
ament_target_dependencies(camera_display_node
  rclcpp
  sensor_msgs
)

target_link_libraries(camera_display_node
  PkgConfig::LIBCAMERA
)

# Link dependencies for FPS subscriber
ament_target_dependencies(camera_fps_subscriber
  rclcpp
  sensor_msgs
)

target_link_libraries(camera_fps_subscriber
  ${OpenCV_LIBS}
)

# Install executables
install(TARGETS camera_display_node camera_fps_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
