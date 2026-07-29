#include <semaforr/core/FORRGeometry.h>
#include <semaforr/navigation/PathPlanner.h>

#include <cassert>
#include <cmath>
#include <limits>
#include <set>
#include <type_traits>
#include <vector>

int main() {
  static_assert(
    std::is_copy_assignable<CartesianPoint>::value,
    "geometry values must support normal value semantics");
  static_assert(
    std::is_copy_assignable<Line>::value,
    "lines must support normal value semantics");

  const CartesianPoint lower_x(1.0, 5.0);
  const CartesianPoint higher_x(2.0, 0.0);
  assert(lower_x < higher_x);
  assert(!(higher_x < lower_x));

  std::set<CartesianPoint> ordered_points = {
    CartesianPoint(2.0, 0.0),
    CartesianPoint(1.0, 5.0),
    CartesianPoint(1.0, 4.0)
  };
  assert(ordered_points.size() == 3);

  const Line default_line;
  assert(default_line.is_degenerate());
  assert(std::isinf(distance(CartesianPoint(1.0, 1.0), default_line)));
  assert(
    get_perpendicular(CartesianPoint(1.0, 1.0), default_line) ==
    CartesianPoint(1.0, 1.0));

  const Circle unit_circle(CartesianPoint(0.0, 0.0), 1.0);
  const LineSegment vertical(
    CartesianPoint(0.0, -2.0),
    CartesianPoint(0.0, 2.0));
  assert(do_intersect(unit_circle, vertical));
  const CartesianPoint vertical_intersection =
    intersection_point(unit_circle, vertical);
  assert(std::abs(vertical_intersection.get_x()) < kGeometryTolerance);
  assert(
    std::abs(vertical_intersection.get_y() + 1.0) <
    kGeometryTolerance);

  const LineSegment tangent(
    CartesianPoint(-2.0, 1.0),
    CartesianPoint(2.0, 1.0));
  assert(do_intersect(unit_circle, tangent));
  assert(!do_intersect(
    unit_circle,
    LineSegment(CartesianPoint(-2.0, 2.0), CartesianPoint(2.0, 2.0))));

  const Circle clamped_circle(CartesianPoint(0.0, 0.0), -2.0);
  assert(clamped_circle.get_radius() == 0.0);

  assert(!canAccessPoint(
    std::vector<CartesianPoint>{CartesianPoint(1.0, 0.0)},
    CartesianPoint(0.0, 0.0),
    CartesianPoint(0.5, 0.0),
    1.0));

  PathPlanner planner(Node(), Node(), "test");
  assert(planner.getPathCost() == 0.0);
  assert(planner.getOrigPathCost() == 0.0);
  assert(!planner.isObjectiveSet());
  assert(!planner.isPathCalculated());

  return 0;
}
