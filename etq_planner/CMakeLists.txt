cmake_minimum_required(VERSION 3.0.2)
project(etq_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp grid_map_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ETQLocalPlanner.cpp
)

add_executable(local_planner src/local_planner_node.cpp)
target_link_libraries(local_planner ${catkin_LIBRARIES} ${PROJECT_NAME})
