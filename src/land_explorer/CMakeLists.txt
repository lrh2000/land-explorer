cmake_minimum_required(VERSION 2.8.3)
project(land_explorer)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(land_explorer src/main.cpp src/node.cpp)
target_link_libraries(land_explorer ${catkin_LIBRARIES})
