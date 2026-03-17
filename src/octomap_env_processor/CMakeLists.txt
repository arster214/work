cmake_minimum_required(VERSION 3.0.2)
project(octomap_env_processor_deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  geometry_msgs
  std_msgs
)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_env_processor
#  CATKIN_DEPENDS roscpp rospy visualization_msgs geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

message(STATUS "This CMakeLists belongs to deprecated octomap_env_processor_deprecated; sources moved to map_process")
