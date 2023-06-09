/******************************************************************************
 * Copyright (c) The apollo Global Authors. 2021 .All Rights Reserved.
 * Description:
 *
 * Author: liu meng
 * Create: 2021-04-22
 *****************************************************************************/
#include "polygon2d.h"

#include <algorithm>
#include <utility>

#include "math_utils.h"

namespace apollo {
namespace common {
namespace math {

namespace {
  double kMathEpsilon = 1e-10;
}  // namespace

Polygon2d::Polygon2d(const Box2d &box) {
  box.GetAllCorners(&points_);
  BuildFromPoints();
}

Polygon2d::Polygon2d(std::vector<Vec2d> points)
  : points_(std::move(points)) {
  BuildFromPoints();
}

int Polygon2d::Next(int at) const { return at >= num_points_ - 1 ? 0 : at + 1; }

int Polygon2d::Prev(int at) const { return at == 0 ? num_points_ - 1 : at - 1; }

void Polygon2d::BuildFromPoints() {
  num_points_ = static_cast<int>(points_.size());
  // CHECK_GE(num_points_, 3);

  // Make sure the points are in ccw order.
  area_ = 0.0;
  for (int i = 1; i < num_points_; ++i) {
    area_ += CrossProd(points_[0], points_[i - 1], points_[i]);
  }
  if (area_ < 0) {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;
  // CHECK_GT(area_, kMathEpsilon);

  // Construct line_segments.
  line_segments_.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i) {
    line_segments_.emplace_back(points_[i], points_[Next(i)]);
  }

  // Check convexity.
  is_convex_ = true;
  for (int i = 0; i < num_points_; ++i) {
    if (CrossProd(points_[Prev(i)], points_[i], points_[Next(i)]) <=
        -kMathEpsilon) {
      is_convex_ = false;
      break;
    }
  }

  // Compute aabox.
  min_x_ = points_[0].x();
  max_x_ = points_[0].x();
  min_y_ = points_[0].y();
  max_y_ = points_[0].y();
  for (const auto &point : points_) {
    min_x_ = std::min(min_x_, point.x());
    max_x_ = std::max(max_x_, point.x());
    min_y_ = std::min(min_y_, point.y());
    max_y_ = std::max(max_y_, point.y());
  }
}

bool Polygon2d::ComputeConvexHull(const std::vector<Vec2d> &points,
                                  Polygon2d *const polygon) {
  // CHECK_NOTNULL(polygon);
  const int n = static_cast<int>(points.size());
  if (n < 3) {
    return false;
  }
  std::vector<int> sorted_indices(n);
  for (int i = 0; i < n; ++i) {
    sorted_indices[i] = i;
  }
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const int idx1, const int idx2) {
              const Vec2d &pt1 = points[idx1];
              const Vec2d &pt2 = points[idx2];
              const double dx = pt1.x() - pt2.x();
              if (std::abs(dx) > kMathEpsilon) {
                return dx < 0.0;
              }
              return pt1.y() < pt2.y();
            });
  int count = 0;
  std::vector<int> results;
  results.reserve(n);
  int last_count = 1;
  for (int i = 0; i < n + n; ++i) {
    if (i == n) {
      last_count = count;
    }
    const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
    const Vec2d &pt = points[idx];
    while (count > last_count &&
           CrossProd(points[results[count - 2]], points[results[count - 1]],
                     pt) <= kMathEpsilon) {
      results.pop_back();
      --count;
    }
    results.push_back(idx);
    ++count;
  }
  --count;
  if (count < 3) {
    return false;
  }
  std::vector<Vec2d> result_points;
  result_points.reserve(count);
  for (int i = 0; i < count; ++i) {
    result_points.push_back(points[results[i]]);
  }
  *polygon = Polygon2d(result_points);
  return true;
}

void Polygon2d::GetAllVertices(std::vector<Vec2d> *const vertices) const {
  if (vertices == nullptr) {
    return;
  }
  *vertices = points_;
}

std::vector<Vec2d> Polygon2d::GetAllVertices() const { return points_; }


}  // namespace math
}  // namespace common
}  // namespace apollo



