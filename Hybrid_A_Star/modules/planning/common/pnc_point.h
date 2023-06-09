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

#include <memory>
#include <string>
#include <vector>

namespace apollo {
namespace common {

struct SLPoint {
  double s = 0.0;
  double l = 0.0;
};

struct FrenetFramePoint {
  double s = 0.0;
  double l = 0.0;
  double dl = 0.0;
  double ddl = 0.0;
};

struct SpeedPoint {
  double s = 0.0;
  double t = 0.0;
  // speed (m/s)
  double v = 0.0;
  // acceleration (m/s^2)
  double a = 0.0;
  // jerk (m/s^3)
  double da = 0.0;
};

struct GaussianInfo {
  // Information of gaussian distribution
  double sigma_x = 0.0;
  double sigma_y = 0.0;
  double correlation = 0.0;
  // Information of representative uncertainty area
  double area_probability = 0.0;
  double ellipse_a = 0.0;
  double ellipse_b = 0.0;
  double theta_a = 0.0;
};

struct PathPoint {
  // coordinates
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  // direction on the x-y plane
  double theta = 0.0;
  // curvature on the x-y planning
  double kappa = 0.0;
  // accumulated distance from beginning of the path
  double s = 0.0;

  // derivative of kappa w.r.t s.
  double dkappa = 0.0;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa = 0.0;
  // The lane ID where the path point is on
  std::string lane_id = "";

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative = 0.0;
  double y_derivative = 0.0;
};

struct Path {
  std::string name = "";
  PathPoint path_point;
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;
  // linear velocity
  double v = 0.0;  // in [m/s]
  // linear acceleration
  double a = 0.0;
  // relative time from beginning of the trajectory
  double relative_time = 0.0;
  // longitudinal jerk
  double da = 0.0;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer = 0.0;

  // Gaussian probability information
  GaussianInfo gaussian_info;
};

struct Trajectory {
  std::string name = "";
  TrajectoryPoint trajectory_point;
};

struct VehicleMotionPoint {
  // trajectory point
  TrajectoryPoint trajectory_point;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer = 0.0;
};

struct VehicleMotion {
  std::string name = "";
  VehicleMotionPoint vehicle_motion_point;
};

}  // namespace common
}  // namespace apollo
