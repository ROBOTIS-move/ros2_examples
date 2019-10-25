################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(examples_message_filter)

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
find_package(examples_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "examples_msgs"
  "message_filters"
  "rclcpp"
)

add_executable(examples_message_filter src/main.cpp)
ament_target_dependencies(examples_message_filter ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS
  examples_message_filter
  DESTINATION lib/${PROJECT_NAME}
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
