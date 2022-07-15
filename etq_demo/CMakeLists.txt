cmake_minimum_required(VERSION 3.0.2)
project(etq_demo)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_ros
)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp grid_map_ros
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${etq_global_planner_astar_INCLUDE_DIRS}
)

add_executable(demo_goal src/demo_goal.cpp)
target_link_libraries(demo_goal ${catkin_LIBRARIES})
