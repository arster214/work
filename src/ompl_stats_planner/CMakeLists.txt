cmake_minimum_required(VERSION 3.0.2)
project(ompl_stats_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_planners_ompl
)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_stats_planner
  CATKIN_DEPENDS
    roscpp
    std_msgs
    pluginlib
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_planners_ompl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(ompl_stats_planner
  src/ompl_stats_planner_manager.cpp
)

target_link_libraries(ompl_stats_planner
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

install(TARGETS ompl_stats_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ompl_stats_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
