/******************************************************************************
 * Copyright (c) The apollo Global Authors. 2021 .All Rights Reserved.
 * Description:
 *
 * Author: liu meng
 * Create: 2021-04-22
 *****************************************************************************/

#ifndef MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_LINE_SEGMENT2D_H_
#define MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_LINE_SEGMENT2D_H_

#include <string>

#include "vec2d.h"

namespace apollo {
namespace common {
namespace math {

/**
 * @class LineSegment2d
 * @brief Line segment in 2-D.
 */
class LineSegment2d {
 public:
  /**
   * @brief Empty constructor.
   */
  LineSegment2d();

  /**
   * @brief Constructor with start point and end point.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  LineSegment2d(const Vec2d &start, const Vec2d &end);

  /**
   * @brief Get the start point.
   * @return The start point of the line segment.
   */
  const Vec2d &start() const { return start_; }

  /**
   * @brief Get the end point.
   * @return The end point of the line segment.
   */
  const Vec2d &end() const { return end_; }

  /**
   * @brief Get the unit direction from the start point to the end point.
   * @return The start point of the line segment.
   */
  const Vec2d &unit_direction() const { return unit_direction_; }

  /**
   * @brief Get the center of the line segment.
   * @return The center of the line segment.
   */
  Vec2d center() const { return (start_ + end_) / 2.0; }

  /** @brief Get a new line-segment with the same start point, but rotated
   * counterclock-wise by the given amount.
   * @return The rotated line-segment's end-point.
   */
  Vec2d rotate(const double angle);

  /**
   * @brief Get the heading of the line segment.
   * @return The heading, which is the angle between unit direction and x-axis.
   */
  double heading() const { return heading_; }

  /**
   * @brief Get the cosine of the heading.
   * @return The cosine of the heading.
   */
  double cos_heading() const { return unit_direction_.x(); }

  /**
   * @brief Get the sine of the heading.
   * @return The sine of the heading.
   */
  double sin_heading() const { return unit_direction_.y(); }

  /**
   * @brief Get the length of the line segment.
   * @return The length of the line segment.
   */
  double length() const;

  /**
   * @brief Get the square of length of the line segment.
   * @return The square of length of the line segment.
   */
  double length_sqr() const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D.
   * @param point The point to compute the distance to.
   * @return The shortest distance from points on the line segment to point.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D.
   * @param point The point to compute the squared of the distance to.
   * @return The square of the shortest distance from points
   *         on the line segment to the input point.
   */
  double DistanceSquareTo(const Vec2d &point) const;

 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_LINE_SEGMENT2D_H_
