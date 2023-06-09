/******************************************************************************
 * Copyright (c) The apollo Global Authors. 2021 .All Rights Reserved.
 * Description: main function
 *
 * Author: liu meng
 * Create: 2021-04-22
 *****************************************************************************/
#include "math_utils.h"

#include <cmath>

namespace apollo {
namespace common {
namespace math {


double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

Eigen::Vector2d RotateVector2d(const Eigen::Vector2d& v_in,
                               const double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  auto x = cos_theta * v_in.x() - sin_theta * v_in.y();
  auto y = sin_theta * v_in.x() + cos_theta * v_in.y();

  return {x, y};
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
