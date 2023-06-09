cmake_minimum_required(VERSION 3.10.1)

project(planning)


############## grid_search ############################
add_library(grid_search
  grid_search.cpp
)

target_link_libraries(grid_search
  common::line_segment2d
  common::box2d
)

add_library(apollo::planning::grid_search ALIAS grid_search)

############## hybrid_a_star ############################
add_library(hybrid_a_star
  hybrid_a_star.cpp
)

target_link_libraries(hybrid_a_star
  common::math_utils
  apollo::planning::grid_search
  apollo::planning::hybrid_a_star_rivz
)

add_library(apollo::planning::hybrid_a_star ALIAS hybrid_a_star)

############## node3d ############################
add_library(node3d
  node3d.cpp
)

target_link_libraries(node3d
  common::box2d
  ${absl_LIBRARIES}
)

add_library(apollo::planning::node3d ALIAS node3d)

############## reeds_shepp_path ############################
add_library(reeds_shepp_path
  reeds_shepp_path.cpp
)

target_link_libraries(reeds_shepp_path
  apollo::planning::node3d
)

add_library(apollo::planning::reeds_shepp_path ALIAS reeds_shepp_path)

############## hybrid a star bin ############################
add_executable(planning
  main.cpp
)


target_link_libraries(planning
  apollo::planning::grid_search
  apollo::planning::hybrid_a_star
  apollo::planning::node3d
  apollo::planning::reeds_shepp_path
)







