################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(examples_rqt)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

################################################################################
# Install
################################################################################
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/examples_rqt
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
ament_package()
