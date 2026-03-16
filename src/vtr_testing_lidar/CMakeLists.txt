cmake_minimum_required(VERSION 3.16)
project(vtr_testing_lidar)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lgmath REQUIRED)

find_package(vtr_testing_common REQUIRED)
find_package(vtr_lidar REQUIRED)
find_package(vtr_torch REQUIRED)

## Executables
add_executable(${PROJECT_NAME}_boreas_preprocessing src/boreas_preprocessing.cpp)
ament_target_dependencies(${PROJECT_NAME}_boreas_preprocessing rclcpp std_msgs vtr_testing_common vtr_lidar vtr_torch)

add_executable(${PROJECT_NAME}_boreas_odometry src/boreas_odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_boreas_odometry rclcpp std_msgs vtr_testing_common vtr_lidar vtr_torch lgmath)

add_executable(${PROJECT_NAME}_boreas_localization src/boreas_localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_boreas_localization rclcpp std_msgs vtr_testing_common vtr_lidar vtr_torch)

install(
  TARGETS
    ${PROJECT_NAME}_boreas_preprocessing
    ${PROJECT_NAME}_boreas_odometry
    ${PROJECT_NAME}_boreas_localization
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()