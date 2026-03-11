cmake_minimum_required(VERSION 3.16)
project(vtr_testing_common)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(lgmath REQUIRED)
find_package(OpenCV REQUIRED)

## This package is header-only — no compiled library needed.
## We still declare an interface target so downstream packages can
## resolve include paths and transitive dependencies via ament.
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  INTERFACE
    rclcpp
    vtr_common
    vtr_logging
)

## Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

## Install the interface target so downstream packages can find it
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include
)

## Install Python scripts so they can be imported from other packages
# install(
#   DIRECTORY script/
#   DESTINATION share/${PROJECT_NAME}/script
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Export targets for downstream find_package(vtr_testing_common)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp vtr_common vtr_logging OpenCV)

ament_package()