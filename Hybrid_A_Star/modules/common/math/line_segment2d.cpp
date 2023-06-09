/******************************************************************************
 * Copyright (c) The apollo Global Authors. 2021 .All Rights Reserved.
 * Description:
 *
 * Author: liu meng
 * Create: 2021-04-22
 *****************************************************************************/
#include "line_segment2d.h"

#include "math_utils.h"

namespace apollo {
namespace common {
namespace math {

namespace {
  double kMathEpsilon = 1e-10;
}  // namespace

LineSegment2d::LineSegment2d() { unit_direction_ = Vec2d(1, 0); }

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();
}

Vec2d LineSegment2d::rotate(const double angle) {
  Vec2d diff_vec = end_ - start_;
  diff_vec.SelfRotate(angle);
  return start_ + diff_vec;
}

double LineSegment2d::length() const { return length_; }

double LineSegment2d::length_sqr() const { return length_ * length_; }

double LineSegment2d::DistanceTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

}  // namespace math
}  // namespace common
}  // namespace apollo
