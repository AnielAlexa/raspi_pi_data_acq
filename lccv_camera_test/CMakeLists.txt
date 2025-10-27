cmake_minimum_required(VERSION 3.8)
project(lccv_camera_test)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include  # LCCV headers
  /usr/include/libcamera  # libcamera headers
)

# Link directories
link_directories(
  /usr/local/lib  # LCCV library
)

# Create executable
add_executable(lccv_test_node
  src/lccv_test_node.cpp
)

# Link libraries
target_link_libraries(lccv_test_node
  ${OpenCV_LIBS}
  lccv  # LCCV library
)

# Install targets
install(TARGETS
  lccv_test_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
