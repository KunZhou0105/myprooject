cmake_minimum_required(VERSION 3.10.1)

include_directories("/usr/include/eigen3")

############## box2d ############################
add_library(box2d
  box2d.cpp
)
target_link_libraries(
  box2d
  common::line_segment2d
  common::vec2d
)
add_library(common::box2d ALIAS box2d)


############## cartesian_frenet_conversion ############################
add_library(cartesian_frenet_conversion
  cartesian_frenet_conversion.cpp
)
target_link_libraries(
  cartesian_frenet_conversion
  common::math_utils
)
add_library(common::cartesian_frenet_conversion ALIAS cartesian_frenet_conversion)


############## line_segment2d ############################
add_library(line_segment2d
  line_segment2d.cpp
)
target_link_libraries(
  line_segment2d
  common::vec2d
)
add_library(common::line_segment2d ALIAS line_segment2d)

############## math_util ############################
add_library(math_utils
  math_utils.cpp
)
target_link_libraries(
  math_utils
  common::vec2d
)
add_library(common::math_utils ALIAS math_utils)


############## polygon2d ############################
add_library(polygon2d
  polygon2d.cpp
)
target_link_libraries(
  polygon2d
  common::box2d
  common::line_segment2d
  common::math_utils
)
add_library(common::polygon2d ALIAS polygon2d)


############## vec2d ############################
add_library(vec2d
  vec2d.cpp
)
target_link_libraries(
  vec2d
)
add_library(common::vec2d ALIAS vec2d)