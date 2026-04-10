cmake_minimum_required(VERSION 3.0.2)
project(dual_arm_rrt_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rospy
  pluginlib
  std_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  clearance_heatmap_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dual_arm_trrt_plugin
  CATKIN_DEPENDS
    geometry_msgs
    interactive_markers
    roscpp
    rospy
    pluginlib
    std_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    clearance_heatmap_msgs
    sensor_msgs
    tf2_ros
    tf2_geometry_msgs
    trajectory_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(dual_arm_trrt_plugin
  src/planner_config.cpp
  src/dual_arm_clearance_objective.cpp
  src/dual_arm_heatmap_objective.cpp
  src/dual_arm_distance_interface.cpp
  src/dual_arm_collision_interface.cpp
  src/dual_arm_tree_visualizer.cpp
  src/heatmap_field.cpp
  src/local_trrt.cpp
  src/dual_arm_trrt_planner_manager.cpp
)

add_dependencies(dual_arm_trrt_plugin ${catkin_EXPORTED_TARGETS})

target_link_libraries(dual_arm_trrt_plugin
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

install(TARGETS dual_arm_trrt_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/dual_arm_rrt_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES dual_arm_trrt_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/apf_force_visualizer.py
  scripts/simple_apf_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
