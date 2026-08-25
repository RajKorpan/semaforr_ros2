/**
 * @file geometry.hpp
 * @brief Geometry responsibilities.
 *
 * @details This file defines geometry behavior for ROS-independent domain state and
 * value types. It centers on `Distance`, `UncheckedTag`, `Angle`,
 * `Point2D`, `Pose2D`, `Segment2D`, `Circle`, `Polygon`. Its
 * package-relative location is `include/semaforr/domain/geometry.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_GEOMETRY_HPP
#define SEMAFORR_DOMAIN_GEOMETRY_HPP

#include <algorithm>
#include <cmath>
#include <compare>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace semaforr::domain {

inline constexpr double geometry_tolerance_m = 1.0e-9;
inline constexpr double angle_tolerance_rad = 1.0e-12;

/**
 * @brief Encapsulates distance state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class Distance {
 public:
  /**
   * @brief Performs the distance operation for this subsystem.
   *
   * Arguments:
   * - @p meters: Supplies meters input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit Distance(double meters) : meters_(meters) {
    if (!std::isfinite(meters_) || meters_ < 0.0) {
      throw std::invalid_argument("distance must be finite and non-negative");
    }
  }

  /**
   * @brief Performs the zero operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Distance` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static constexpr Distance zero() noexcept {
    return Distance(0.0, UncheckedTag{});
  }

  /**
   * @brief Performs the meters operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  constexpr double meters() const noexcept { return meters_; }
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Distance&) const = default;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::partial_ordering` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::partial_ordering operator<=>(const Distance&) const = default;

 private:
  /**
   * @brief Encapsulates unchecked tag state and behavior for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  struct UncheckedTag {};
  /**
   * @brief Performs the distance operation for this subsystem.
   *
   * Arguments:
   * - @p meters: Supplies meters input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  constexpr Distance(double meters, UncheckedTag) noexcept : meters_(meters) {}
  double meters_;
};

/**
 * @brief Encapsulates angle state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class Angle {
 public:
  /**
   * @brief Performs the angle operation for this subsystem.
   *
   * Arguments:
   * - @p radians: Supplies radians input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit Angle(double radians) : radians_(normalize(radians)) {
    if (!std::isfinite(radians)) {
      throw std::invalid_argument("angle must be finite");
    }
  }

  /**
   * @brief Performs the zero operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Angle` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static constexpr Angle zero() noexcept { return Angle(0.0, UncheckedTag{}); }

  /**
   * @brief Performs the radians operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  constexpr double radians() const noexcept { return radians_; }

  /**
   * @brief Performs the normalize operation for this subsystem.
   *
   * Arguments:
   * - @p radians: Supplies radians input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static double normalize(double radians) {
    if (!std::isfinite(radians)) {
      throw std::invalid_argument("angle must be finite");
    }
    double normalized = std::remainder(radians, 2.0 * std::numbers::pi);
    if (normalized <= -std::numbers::pi) {
      normalized += 2.0 * std::numbers::pi;
    }
    return normalized;
  }

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Angle&) const = default;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::partial_ordering` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::partial_ordering operator<=>(const Angle&) const = default;

 private:
  /**
   * @brief Encapsulates unchecked tag state and behavior for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  struct UncheckedTag {};
  /**
   * @brief Performs the angle operation for this subsystem.
   *
   * Arguments:
   * - @p radians: Supplies radians input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  constexpr Angle(double radians, UncheckedTag) noexcept : radians_(radians) {}
  double radians_;
};

/**
 * @brief Encapsulates point2 d state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct Point2D {
  double x_m = 0.0;
  double y_m = 0.0;

  /**
   * @brief Performs the finite operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool finite() const noexcept {
    return std::isfinite(x_m) && std::isfinite(y_m);
  }

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Point2D&) const = default;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::partial_ordering` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::partial_ordering operator<=>(const Point2D&) const = default;
};

/**
 * @brief Encapsulates pose2 d state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct Pose2D {
  Point2D position;
  Angle heading = Angle::zero();

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Pose2D&) const = default;
};

/**
 * @brief Encapsulates segment2 d state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct Segment2D {
  Point2D start;
  Point2D end;

  /**
   * @brief Performs the length operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Distance` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Distance length() const {
    return Distance(std::hypot(end.x_m - start.x_m, end.y_m - start.y_m));
  }

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Segment2D&) const = default;
};

/**
 * @brief Encapsulates circle state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct Circle {
  Point2D center;
  Distance radius = Distance::zero();

  /**
   * @brief Performs the contains operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   * - @p tolerance_m: Supplies tolerance m input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool contains(const Point2D& point,
                double tolerance_m = geometry_tolerance_m) const noexcept {
    return std::hypot(point.x_m - center.x_m, point.y_m - center.y_m) <=
           radius.meters() + std::max(0.0, tolerance_m);
  }

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Circle&) const = default;
};

/**
 * @brief Encapsulates polygon state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class Polygon {
 public:
  /**
   * @brief Performs the polygon operation for this subsystem.
   *
   * Arguments:
   * - @p vertices: Supplies vertices input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit Polygon(std::vector<Point2D> vertices)
      : vertices_(std::move(vertices)) {
    if (vertices_.size() < 3U) {
      throw std::invalid_argument("polygon requires at least three vertices");
    }
    if (!std::all_of(vertices_.begin(), vertices_.end(),
                     [](const Point2D& point) { return point.finite(); })) {
      throw std::invalid_argument("polygon vertices must be finite");
    }
  }

  /**
   * @brief Performs the vertices operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<Point2D>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<Point2D>& vertices() const noexcept { return vertices_; }

  /**
   * @brief Performs the contains operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool contains(const Point2D& point) const noexcept {
    bool inside = false;
    for (std::size_t i = 0, j = vertices_.size() - 1U; i < vertices_.size();
         j = i++) {
      const Point2D& a = vertices_[i];
      const Point2D& b = vertices_[j];
      const bool crosses =
          ((a.y_m > point.y_m) != (b.y_m > point.y_m)) &&
          (point.x_m <
           (b.x_m - a.x_m) * (point.y_m - a.y_m) / (b.y_m - a.y_m) + a.x_m);
      if (crosses) {
        inside = !inside;
      }
    }
    return inside;
  }

 private:
  std::vector<Point2D> vertices_;
};

/**
 * @brief Performs the distance operation for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 *
 * Returns:
 * - `Distance` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline Distance distance(const Point2D& first, const Point2D& second) {
  return Distance(std::hypot(second.x_m - first.x_m, second.y_m - first.y_m));
}

/**
 * @brief Performs the cross operation for this subsystem.
 *
 * Arguments:
 * - @p origin: Supplies origin input to the operation.
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline double cross(const Point2D& origin, const Point2D& first,
                    const Point2D& second) noexcept {
  return (first.x_m - origin.x_m) * (second.y_m - origin.y_m) -
         (first.y_m - origin.y_m) * (second.x_m - origin.x_m);
}

/**
 * @brief Performs the intersects operation for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 * - @p tolerance_m: Supplies tolerance m input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline bool intersects(const Segment2D& first, const Segment2D& second,
                       double tolerance_m = geometry_tolerance_m) noexcept {
  const double c1 = cross(first.start, first.end, second.start);
  const double c2 = cross(first.start, first.end, second.end);
  const double c3 = cross(second.start, second.end, first.start);
  const double c4 = cross(second.start, second.end, first.end);
  const double tolerance = std::max(0.0, tolerance_m);
  return ((c1 > tolerance && c2 < -tolerance) ||
          (c1 < -tolerance && c2 > tolerance)) &&
         ((c3 > tolerance && c4 < -tolerance) ||
          (c3 < -tolerance && c4 > tolerance));
}

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_GEOMETRY_HPP
