/******************************************************************************
 * Copyright (c) The apollo Global Authors. 2021 .All Rights Reserved.
 * Description:
 *
 * Author: liu meng
 * Create: 2021-04-22
 *****************************************************************************/

#ifndef MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_POLYGON2D_H_
#define MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_POLYGON2D_H_

#include <vector>

#include "box2d.h"
#include "line_segment2d.h"

namespace apollo {
namespace common {
namespace math {

/**
 * @class Polygon2d
 * @brief The class of polygon in 2-D.
 */
class Polygon2d {
 public:
  Polygon2d() = default;

  /**
   * @brief Constructor which takes a box.
   * @param box The box to construct the polygon.
   */
  explicit Polygon2d(const Box2d &box);

  /**
   * @brief Constructor which takes a vector of points as its vertices.
   * @param points The points to construct the polygon.
   */
  explicit Polygon2d(std::vector<Vec2d> points);

  /**
   * @brief Compute the convex hull of a group of points.
   * @param points The target points. To compute the convex hull of them.
   * @param polygon The convex hull of the points.
   * @return If successfully compute the convex hull.
   */
  static bool ComputeConvexHull(const std::vector<Vec2d> &points,
                                Polygon2d *const polygon);

  /**
   * @brief Get all vertices of the polygon
   * @param All vertices of the polygon
   */
  void GetAllVertices(std::vector<Vec2d> *const vertices) const;

  /**
   * @brief Get all vertices of the polygon
   */
  std::vector<Vec2d> GetAllVertices() const;

 protected:
  void BuildFromPoints();
  int Next(int at) const;
  int Prev(int at) const;

  std::vector<Vec2d> points_;
  int num_points_ = 0;
  std::vector<LineSegment2d> line_segments_;
  bool is_convex_ = false;
  double area_ = 0.0;
  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
};

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_PNC_COMMON_COMMON_MATH_POLYGON2D_H_
