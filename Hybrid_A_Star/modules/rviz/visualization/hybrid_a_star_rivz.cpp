/******************************************************************************
 * Copyright (c) The Conch Global Authors. 2021 .All Rights Reserved.
 * Description:
 *
 * Author: liu meng
 * Create: 2021-11-29
 *****************************************************************************/

#include "../../../modules/rviz/visualization/hybrid_a_star_rivz.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace apollo {
namespace planning {

HybridAStarRviz::HybridAStarRviz() {
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "mapping");
    ros::NodeHandle o_nh_;
    o_occupancy_grid_rviz_ =  o_nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
    o_trajectory_rviz_     =  o_nh_.advertise<visualization_msgs::Marker>("/trajectory", 1);
}

HybridAStarRviz::~HybridAStarRviz() {
}

void HybridAStarRviz::vDisplayOccupancyGrid(tstOccupancyGrid st_occupancy_grid) {
    nav_msgs::OccupancyGrid st_pub_global_map;
    st_pub_global_map.info.resolution = st_occupancy_grid.f_resolution;
    st_pub_global_map.info.origin.position.x = 0.0;
    st_pub_global_map.info.origin.position.y = 0.0;
    st_pub_global_map.info.origin.position.z = 0.0;
    st_pub_global_map.info.origin.orientation.x = 0.0;
    st_pub_global_map.info.origin.orientation.y = 0.0;
    st_pub_global_map.info.origin.orientation.z = 0.0;
    st_pub_global_map.info.origin.orientation.w = 1.0;

    st_pub_global_map.info.origin.position.x = st_occupancy_grid.st_origin.f_x;
    st_pub_global_map.info.origin.position.y = st_occupancy_grid.st_origin.f_y;
    st_pub_global_map.info.width = st_occupancy_grid.f_width;
    st_pub_global_map.info.height = st_occupancy_grid.f_height;
    int all_size = st_occupancy_grid.f_width * st_occupancy_grid.f_height;

    st_pub_global_map.data.resize(all_size, 0);
    for (int i = 0; i < all_size; i++) {
        st_pub_global_map.data[i] = st_occupancy_grid.vt_data[i];
    }

    st_pub_global_map.header.stamp = ros::Time::now();
    st_pub_global_map.header.frame_id = "map";
    o_occupancy_grid_rviz_.publish(st_pub_global_map);
}

void HybridAStarRviz::vDisplayHybridAStartTrajectory(const HybridAStartResult& hybrid_a_start_result) {
  visualization_msgs::Marker marker;
  geometry_msgs::Point point;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hybrid_a_start_trajectory";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.id = 0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1;
  marker.color.g = 1;
  for (int i = 0; i < hybrid_a_start_result.x.size(); i++) {
    point.x = hybrid_a_start_result.x.at(i);
    point.y = hybrid_a_start_result.y.at(i);
    marker.points.push_back(point);
  }
  o_trajectory_rviz_.publish(marker);
}

void HybridAStarRviz::vSaveHybridAStartTrajectory(const HybridAStartResult& hybrid_a_start_result) {
  // Set the size of output image to 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot(hybrid_a_start_result.x, hybrid_a_start_result.y);
  // Set x-axis to interval [-200, 200]
  plt::xlim(-200, 200);
  // Set y-axis to interval [-200, 200]
  plt::ylim(-200, 200);
  // Add graph title
  plt::title("Hybrid A Start");
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./hybrid.png");
}

}  // namespace planning
}  // namespace apollo
