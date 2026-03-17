cmake_minimum_required(VERSION 3.0.2)
project(planner_benchmark)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    moveit_msgs
    moveit_core
    moveit_ros_planning_interface
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/paper_data_collection.py
  scripts/check_available_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
