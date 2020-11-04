################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(examples_tf2)

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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(BROADCASTER_NODE_NAME broadcaster)
set(LISTENER_NODE_NAME listener)
set(STATIC_BROADCASTER_NODE_NAME static_broadcaster)

set(dependencies
  "geometry_msgs"
  "rclcpp"
  "std_srvs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

add_executable(${BROADCASTER_NODE_NAME} src/broadcaster.cpp)
ament_target_dependencies(${BROADCASTER_NODE_NAME} ${dependencies})

add_executable(${LISTENER_NODE_NAME} src/listener.cpp)
ament_target_dependencies(${LISTENER_NODE_NAME} ${dependencies})

add_executable(${STATIC_BROADCASTER_NODE_NAME} src/static_broadcaster.cpp)
ament_target_dependencies(${STATIC_BROADCASTER_NODE_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS
  ${BROADCASTER_NODE_NAME}
  ${LISTENER_NODE_NAME}
  ${STATIC_BROADCASTER_NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
