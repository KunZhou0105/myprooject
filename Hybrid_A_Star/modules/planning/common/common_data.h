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
#include <cmath>

namespace apollo {
namespace planning {

enum DualWarmUpMode {
  IPOPT = 0,
  IPOPTQP = 1,
  OSQP = 2,
  DEBUG = 3,
  SLACKQP = 4,
};

enum DistanceApproachMode {
  DISTANCE_APPROACH_IPOPT = 0,
  DISTANCE_APPROACH_IPOPT_CUDA = 1,
  DISTANCE_APPROACH_IPOPT_FIXED_TS = 2,
  DISTANCE_APPROACH_IPOPT_FIXED_DUAL = 3,
  DISTANCE_APPROACH_IPOPT_RELAX_END = 4,
  DISTANCE_APPROACH_IPOPT_RELAX_END_SLACK = 5,
};

struct ROIConfig {
  // longitudinal range of parking roi backward
  double roi_longitudinal_range_start = 10.0;
  // longitudinal range of parking roi forward
  double roi_longitudinal_range_end = 10.0;
  // parking spot range detection threshold
  double parking_start_range = 7.0;
  // Parking orientation for reverse parking
  bool parking_inwards = false;
};

struct WarmStartConfig {
  // Hybrid a star for warm start
  double xy_grid_resolution = 1;
  double phi_grid_resolution =  7 * M_PI / 180;
  uint64_t next_node_num = 10;
  double step_size = 0.5;
  double traj_forward_penalty = 0.0;
  double traj_back_penalty = 0.0;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty = 100.0;
  double traj_steer_change_penalty = 10.0;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution = 1;
  double node_radius = 0.5;
  // PiecewiseJerkSpeedOptimizerConfig s_curve_config = 17;
};

struct IpoptConfig {
  // Ipopt configs
  uint32_t ipopt_print_level = 0;
  uint32_t mumps_mem_percent = 0;
  double mumps_pivtol = 0.0;
  uint32_t ipopt_max_iter = 0;
  double ipopt_tol = 0.0;
  double ipopt_acceptable_constr_viol_tol = 0.0;
  double ipopt_min_hessian_perturbation = 0.0;
  double ipopt_jacobian_regularization_value = 0.0;
  std::string ipopt_print_timing_statistics = "";
  std::string ipopt_alpha_for_y = "";
  std::string ipopt_recalc_y = "";
  double ipopt_mu_init = 0.1;
  // ipopt barrier parameter, default 0.1
};

// Dual variable configs for OSQP
struct OSQPConfig {
  double alpha = 1.0;
  double eps_abs = 1.0e-3;
  double eps_rel = 1.0e-3;
  uint32_t max_iter = 10000;
  bool polish = true;
  bool osqp_debug_log = false;
};

struct IterativeAnchoringConfig {
  // Ipopt configs
  double interpolated_delta_s = 0.1;
  uint32_t reanchoring_trails_num = 50;
  double reanchoring_pos_stddev = 0.25;
  double reanchoring_length_stddev = 1.0;
  bool estimate_bound = false;
  double default_bound = 2.0;
  double vehicle_shortest_dimension = 1.04;
  // FemPosDeviationSmootherConfig fem_pos_deviation_smoother_config = 8;
  double collision_decrease_ratio = 0.9;
  // TODO(QiL, Jinyun): Merge following with overall config for open space
  double max_forward_v = 2.0;
  double max_reverse_v = 2.0;
  double max_forward_acc = 3.0;
  double max_reverse_acc = 2.0;
  double max_acc_jerk = 4.0;
  double delta_t = 0.2;
  // PiecewiseJerkSpeedOptimizerConfig s_curve_config = 16;
};

struct TrajectoryPartitionConfig {
  uint64_t interpolated_pieces_num = 50;
  uint64_t initial_gear_check_horizon = 3;
  double heading_searching_range = 0.3;
  double gear_shift_period_duration = 2.0;
  double gear_shift_max_t = 3.0;
  double gear_shift_unit_t = 0.02;
};

struct DualVariableWarmStartConfig {
  // Dual variable Warm Start
  double weight_d = 1.0;
  IpoptConfig ipopt_config;
  DualWarmUpMode qp_format;
  double min_safety_distance = 0.0;
  bool debug_osqp = false;
  double beta = 1.0;
  OSQPConfig osqp_config;
};

struct DistanceApproachConfig {
  // Distance approach weight configs
  double weight_steer = 0.0;
  double weight_a = 0.0;
  double weight_steer_rate = 0.0;
  double weight_a_rate = 0.0;
  double weight_x = 0.0;
  double weight_y = 0.0;
  double weight_phi = 0.0;
  double weight_v = 0.0;
  double weight_steer_stitching = 0.0;
  double weight_a_stitching = 0.0;
  double weight_first_order_time = 0.0;
  double weight_second_order_time = 0.0;
  double min_safety_distance = 0.0;
  double max_speed_forward = 3.0;
  double max_speed_reverse = 2.0;
  double max_acceleration_forward = 2.0;
  double max_acceleration_reverse = 2.0;
  double min_time_sample_scaling = 0.1;
  double max_time_sample_scaling = 10.0;
  bool use_fix_time = false;
  IpoptConfig ipopt_config;
  bool enable_constraint_check = false;
  bool enable_hand_derivative = false;
  // True to enable hand derived derivative inside open space planner
  bool enable_derivative_check = false;
  // True to enable derivative check inside open space planner
  bool enable_initial_final_check = false;
  DistanceApproachMode distance_approach_mode;
  bool enable_jacobian_ad = false;
  bool enable_check_initial_state = false;
  double weight_end_state = 0.0;
  double weight_slack = 0.0;
};

struct PlannerOpenSpaceConfig {
  // Open Space ROIConfig
  ROIConfig roi_config;
  // Hybrid A Star Warm Start
  WarmStartConfig warm_start_config;
  // Dual Variable Warm Start
  DualVariableWarmStartConfig dual_variable_warm_start_config;
  // Distance Approach Configs
  DistanceApproachConfig distance_approach_config;
  // Iterative Anchoring Configs
  IterativeAnchoringConfig iterative_anchoring_smoother_config;
  // Trajectory PartitionConfig Configs
  TrajectoryPartitionConfig trajectory_partition_config;
  float delta_t = 1.0;
  double is_near_destination_threshold = 0.001;
  bool enable_check_parallel_trajectory = false;
  bool enable_linear_interpolation = false;
  double is_near_destination_theta_threshold = 0.05;
};

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

// cout colour define
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define COLOUR_END   "\033[0m"  /* COLOUR_END */



}  // namespace planning
}  // namespace apollo
