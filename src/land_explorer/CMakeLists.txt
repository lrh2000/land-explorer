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

find_package(Open3D)
if (Open3D_FOUND)
  message(STATUS "Found Open3D ${Open3D_VERSION}")
else ()
  message(SEND_ERROR "Open3D not found")
endif ()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(land_explorer src/land_explorer.cpp)
target_link_libraries(land_explorer ${catkin_LIBRARIES})

add_executable(process_ply src/process_ply.cpp)

link_directories(${Open3D_LIBRARY_DIRS})
add_executable(mapping_3d src/mapping_3d.cpp)
target_include_directories(mapping_3d PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(mapping_3d ${catkin_LIBRARIES} ${Open3D_LIBRARIES})

add_executable(draw_ply src/draw_ply.cpp)
target_include_directories(draw_ply PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(draw_ply ${Open3D_LIBRARIES})
