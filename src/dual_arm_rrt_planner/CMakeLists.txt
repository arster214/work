cmake_minimum_required(VERSION 3.0.2)
project(dual_arm_rrt_planner)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  pluginlib
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dual_arm_rrt_plugin
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface pluginlib tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# OMPL include locations (copied from original package to be robust across distros)
if(EXISTS "/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.6")
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.6)
elseif(EXISTS "/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.5")
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ompl-1.5)
elseif(EXISTS "/opt/ros/$ENV{ROS_DISTRO}/include/ompl")
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ompl)
else()
  message(WARNING "OMPL include directory not found, KD-Tree may not compile")
endif()

add_library(dual_arm_rrt_plugin
  src/dual_arm_rrt.cpp
  src/dual_arm_planner_manager.cpp
  src/modules/goal_bias_module.cpp
  src/modules/node_rejection_module.cpp
  src/modules/t_rrt_module.cpp
  src/modules/apf_module.cpp
  src/modules/variable_step_module.cpp
)

target_link_libraries(dual_arm_rrt_plugin
  ${catkin_LIBRARIES}
)

install(TARGETS dual_arm_rrt_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/dual_arm_rrt_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
