cmake_minimum_required(VERSION 3.0.2)
project(map_process)

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
#  LIBRARIES map_process
#  CATKIN_DEPENDS roscpp rospy visualization_msgs geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_rebuild src/map_rebuild.cpp)
target_link_libraries(map_rebuild ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(octomap_bt_to_points src/octomap_bt_to_points.cpp)
target_link_libraries(octomap_bt_to_points ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(points_to_octomap_bt src/points_to_octomap_bt.cpp)
target_link_libraries(points_to_octomap_bt ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/process_octomap_markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
