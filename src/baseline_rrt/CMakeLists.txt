cmake_minimum_required(VERSION 3.0.2)
project(baseline_rrt)

## C++14 standard
add_compile_options(-std=c++14)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  tf2_ros
)

find_package(Eigen3 REQUIRED)

## Catkin package configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES baseline_rrt_planner
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    tf2_ros
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# OMPL is included via moveit_core, but we need to add the specific version path
# This works across different ROS distributions
if(EXISTS "/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.6")
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.6)
elseif(EXISTS "/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.5")
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.5)
elseif(EXISTS "/opt/ros/$ENV{ROS_DISTRO}/include/ompl")
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ompl)
else()
  message(WARNING "OMPL include directory not found, KD-Tree may not compile")
endif()

## Baseline RRT library
add_library(baseline_rrt_planner
  src/basic_rrt.cpp
  src/basic_rrt_star.cpp
  src/baseline_rrt_planner_manager.cpp
)

target_link_libraries(baseline_rrt_planner
  ${catkin_LIBRARIES}
)

## Install
install(TARGETS baseline_rrt_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES baseline_rrt_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
