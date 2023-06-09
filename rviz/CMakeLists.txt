cmake_minimum_required(VERSION 3.10.1)

project(points_and_lines)

include_directories("/usr/local/ubuntu_crossbuild_devkit/sysroot/opt/melodic/include")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

message(STATUS "catkin libraries is: ${catkin_LIBRARIES}")

add_executable(points_and_lines 
  src/points_and_lines.cpp
)

target_link_libraries(points_and_lines 
  ${catkin_LIBRARIES}
)

