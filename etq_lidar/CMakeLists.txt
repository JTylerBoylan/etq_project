cmake_minimum_required(VERSION 3.0.2)
project(etq_lidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  roscpp
  laser_geometry
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES etq_lidar
  CATKIN_DEPENDS grid_map_ros roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/lidar_mapper.cpp
)

add_executable(laser_projection src/laser_projection.cpp)
target_link_libraries(laser_projection ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(world2lidar_transform src/world2lidar_transform.cpp)
target_link_libraries(world2lidar_transform ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(lidar_conversion src/lidar_conversion.cpp)
target_link_libraries(lidar_conversion ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(scan2cloud_conversion src/scan2cloud_conversion.cpp)
target_link_libraries(scan2cloud_conversion ${catkin_LIBRARIES} ${PROJECT_NAME})