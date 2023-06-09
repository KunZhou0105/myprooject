/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <vector>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "../../../modules/planning/common/visualization.h"
#include "../../../modules/planning/common/common_data.h"


namespace apollo {
namespace planning {

using tstOccupancyGrid = apollo::common::tstOccupancyGrid;

class HybridAStarRviz {
 public:
  HybridAStarRviz();

  ~HybridAStarRviz();

  void vDisplayOccupancyGrid(tstOccupancyGrid st_occupancy_grid);

  void vDisplayHybridAStartTrajectory(const HybridAStartResult& hybrid_a_start_result);

  void vSaveHybridAStartTrajectory(const HybridAStartResult& hybrid_a_start_result);

 private:
  ros::Publisher o_occupancy_grid_rviz_;
  ros::Publisher o_trajectory_rviz_;
};

}  // namespace planning
}  // namespace apollo


