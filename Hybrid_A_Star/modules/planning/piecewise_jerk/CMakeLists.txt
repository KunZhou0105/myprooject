cmake_minimum_required(VERSION 3.10.1)


find_package(osqp REQUIRED)


############## piecewise_jerk_problem ############################
add_library(piecewise_jerk_problem
  piecewise_jerk_problem.cpp
)
target_link_libraries(
  piecewise_jerk_problem
  osqp::osqpstatic
  libdl.so
)
add_library(apollo::planning::piecewise_jerk_problem ALIAS piecewise_jerk_problem)


############## piecewise_jerk_path_problem ############################
add_library(piecewise_jerk_path_problem
  piecewise_jerk_path_problem.cpp
)
target_link_libraries(
  piecewise_jerk_path_problem
  apollo::planning::piecewise_jerk_problem
)
add_library(apollo::planning::piecewise_jerk_path_problem ALIAS piecewise_jerk_path_problem)


############## piecewise_jerk_speed_problem ############################
add_library(piecewise_jerk_speed_problem
  piecewise_jerk_speed_problem.cpp
)
target_link_libraries(
  piecewise_jerk_speed_problem
  apollo::planning::piecewise_jerk_problem
)
add_library(apollo::planning::piecewise_jerk_speed_problem ALIAS piecewise_jerk_speed_problem)
