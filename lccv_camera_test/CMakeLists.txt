cmake_minimum_required(VERSION 3.8)
project(lccv_camera_test)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include  # LCCV headers
  /usr/include/libcamera  # libcamera headers
)

# Link directories
link_directories(
  /usr/local/lib  # LCCV library
)

# Create executables
add_executable(lccv_test_node
  src/lccv_test_node.cpp
)

add_executable(lccv_publisher_node
  src/lccv_publisher_node.cpp
)

# Link libraries for lccv_test_node
target_link_libraries(lccv_test_node
  ${OpenCV_LIBS}
  lccv  # LCCV library
)

# Add dependencies for lccv_publisher_node
ament_target_dependencies(lccv_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
)

# Link libraries for lccv_publisher_node
target_link_libraries(lccv_publisher_node
  ${OpenCV_LIBS}
  lccv  # LCCV library
)

# Install targets
install(TARGETS
  lccv_test_node
  lccv_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
