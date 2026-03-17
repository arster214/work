cmake_minimum_required(VERSION 3.0.2)
project(map_build)


# 1. 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  octomap_ros
  visualization_msgs
)

# 2. 查找 PCL 和 OpenMP
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(octomap REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(point_process src/point_process.cpp)
target_link_libraries(point_process
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

# 添加 octomap_filter_node 可执行文件
add_executable(octomap_filter_node src/octomap_filter_node.cpp)
target_link_libraries(octomap_filter_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)


if(OpenMP_CXX_FOUND)
    target_compile_options(point_process PUBLIC ${OpenMP_CXX_FLAGS})
endif()
add_executable(tabletop_segmentation_node src/tabletop_segmentation_node.cpp)
target_link_libraries(tabletop_segmentation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

