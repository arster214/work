cmake_minimum_required(VERSION 3.1.3)
project(improved_rrt_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    pluginlib
    roscpp
    tf2_ros
)

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

# Build the improved RRT algorithm library
# 使用简化的稳定版本
add_library(${PROJECT_NAME}
  src/improved_rrt.cpp
  src/improved_rrt_planner_manager.cpp
  src/modules/goal_bias_module.cpp
  src/modules/t_rrt_module.cpp
  src/modules/variable_step_module.cpp
  src/modules/apf_module.cpp
  src/modules/node_rejection_module.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES improved_rrt_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
