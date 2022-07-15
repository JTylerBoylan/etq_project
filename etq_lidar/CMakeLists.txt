cmake_minimum_required(VERSION 3.0.2)
project(etq_lidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  grid_map_filters
  grid_map_cv
  filters
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES etq_lidar
  CATKIN_DEPENDS grid_map_ros roscpp grid_map_filters filters grid_map_cv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/lidar_mapper.cpp
)

add_executable(lidar_projection src/lidar_projection_node.cpp)
target_link_libraries(lidar_projection ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(pose_estimation src/pose_estimation_node.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(map_filters src/map_filters_node.cpp)
target_link_libraries(map_filters ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(world2lidar_transform src/world2lidar_tf_node.cpp)
target_link_libraries(world2lidar_transform ${catkin_LIBRARIES} ${PROJECT_NAME})
