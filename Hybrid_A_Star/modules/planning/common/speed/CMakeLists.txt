cmake_minimum_required(VERSION 3.10.1)


############## speed_data ############################
add_library(speed_data
speed_data.cpp
)
target_link_libraries(
  speed_data
)
add_library(apollo::planning::speed_data ALIAS speed_data)


