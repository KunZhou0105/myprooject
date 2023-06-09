cmake_minimum_required(VERSION 3.10)

project(pengqiuzhi)

add_definitions("-g")

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE DEBUG)

find_package(absl REQUIRED)

find_package(PythonLibs 2.7 REQUIRED)
message(STATUS "PYTHON libraries is: ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON include libraries is: ${PYTHON_INCLUDE_DIRS}")

include_directories("/home/conan/matplotlib-cpp")
 
set(absl_LIBRARIES
absl::algorithm
absl::base
absl::debugging
absl::flat_hash_map
absl::memory
absl::meta
absl::numeric
absl::str_format
absl::strings
absl::synchronization
absl::time
absl::utility)

include_directories("/usr/local/ubuntu_crossbuild_devkit/sysroot/opt/melodic/include")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

add_subdirectory(modules/common)
add_subdirectory(modules/rviz)
add_subdirectory(modules/planning)


