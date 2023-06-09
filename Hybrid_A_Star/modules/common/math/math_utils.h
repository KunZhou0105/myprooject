/******************************************************************************
 * Copyright (c) The apollo Global Authors. 2021 .All Rights Reserved.
 * Description: main function
 *
 * Author: liu meng
 * Create: 2021-04-22
 *****************************************************************************/

#ifndef MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_MATH_UTILS_H_
#define MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_MATH_UTILS_H_

#include <utility>

#include "/usr/include/eigen3/Eigen/Core"

#include "vec2d.h"

namespace apollo {
namespace common {
namespace math {


/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Rotate a 2d vector counter-clockwise by theta
 */ 
Eigen::Vector2d RotateVector2d(const Eigen::Vector2d& v_in,
                               const double theta);

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);


}  // namespace math
}  // namespace common
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_MATH_UTILS_H_
