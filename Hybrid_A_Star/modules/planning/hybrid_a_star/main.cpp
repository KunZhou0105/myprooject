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
#include <iostream>
#include "../../../modules/planning/hybrid_a_star/hybrid_a_star.h"
#include "../../../modules/rviz/visualization/hybrid_a_star_rivz.h"

int main(int argc, char **argv) {
  apollo::planning::PlannerOpenSpaceConfig open_space_conf;
  apollo::planning::HybridAStar hybrid_a_star(open_space_conf);

  double start_x = 0;
  double start_y = 0;
  double start_phi = 90 * M_PI / 180;
  double end_x = 0;
  double end_y = 75;
  double end_phi = -90 * M_PI / 180;
  std::vector<double> XYbounds;
  double a = 0;
  XYbounds.push_back(a);
  double b = 100;
  XYbounds.push_back(b);
  double c = 0;
  XYbounds.push_back(c);
  double d = 100;
  XYbounds.push_back(d);
  std::vector<std::vector<apollo::common::math::Vec2d>> obstacles_vertices_vec;
  apollo::planning::HybridAStartResult result;
  hybrid_a_star.Plan(start_x, start_y, start_phi, end_x, end_y, end_phi, XYbounds, obstacles_vertices_vec, &result);
  // std::cout << GREEN << " result.x.size(): " << result.x.size() << std::endl;
  // std::cout << GREEN << " result.y.size(): " << result.y.size() << std::endl;
  // std::cout << GREEN << " result.phi.size(): " << result.phi.size() << std::endl;
  // std::cout << GREEN << " result.v.size(): " << result.v.size() << std::endl;
  // std::cout << GREEN << " result.a.size(): " << result.a.size() << std::endl;
  // std::cout << GREEN << " result.accumulated_s.size(): " << result.accumulated_s.size() << std::endl;
  // std::cout << GREEN << " result.steer.size(): " << result.steer.size() << std::endl;
  for (int i = 0; i < result.x.size(); i++) {
    std::cout << GREEN << " x  y : " << result.x.at(i) <<
                                "  " << result.y.at(i) <<
                                "  " << result.phi.at(i) <<
                                COLOUR_END << std::endl;
  }
  apollo::planning::HybridAStarRviz hybird_a_star_display_;
  // std::cout << " start display: " << std::endl;
  // hybird_a_star_display_.vSaveHybridAStartTrajectory(result);
  while (ros::ok()) {
    hybird_a_star_display_.vDisplayHybridAStartTrajectory(result);
  }

  return 0;
}
