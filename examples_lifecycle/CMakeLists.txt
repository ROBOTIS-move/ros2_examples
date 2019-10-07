################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(examples_lifecycle)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(LIFECYCLE_NODE_NAME robot)

set(dependencies
  "lifecycle_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
)

add_executable(${LIFECYCLE_NODE_NAME} src/main.cpp src/robot.cpp)
ament_target_dependencies(${LIFECYCLE_NODE_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS
  ${LIFECYCLE_NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
