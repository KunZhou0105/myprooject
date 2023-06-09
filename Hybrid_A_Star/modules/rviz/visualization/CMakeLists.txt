cmake_minimum_required(VERSION 3.10.1)

############## hybrid_a_star_rivz ############################
add_library(hybrid_a_star_rivz
  hybrid_a_star_rivz.cpp
)

target_link_libraries(hybrid_a_star_rivz
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

target_include_directories(hybrid_a_star_rivz 
  PRIVATE 
  ${PYTHON_INCLUDE_DIRS})

add_library(apollo::planning::hybrid_a_star_rivz ALIAS hybrid_a_star_rivz)
