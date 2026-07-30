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

class Distance {
 public:
  explicit Distance(double meters) : meters_(meters) {
    if (!std::isfinite(meters_) || meters_ < 0.0) {
      throw std::invalid_argument("distance must be finite and non-negative");
    }
  }

  static constexpr Distance zero() noexcept {
    return Distance(0.0, UncheckedTag{});
  }

  constexpr double meters() const noexcept { return meters_; }
  bool operator==(const Distance&) const = default;
  std::partial_ordering operator<=>(const Distance&) const = default;

 private:
  struct UncheckedTag {};
  constexpr Distance(double meters, UncheckedTag) noexcept : meters_(meters) {}
  double meters_;
};

class Angle {
 public:
  explicit Angle(double radians) : radians_(normalize(radians)) {
    if (!std::isfinite(radians)) {
      throw std::invalid_argument("angle must be finite");
    }
  }

  static constexpr Angle zero() noexcept { return Angle(0.0, UncheckedTag{}); }

  constexpr double radians() const noexcept { return radians_; }

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

  bool operator==(const Angle&) const = default;
  std::partial_ordering operator<=>(const Angle&) const = default;

 private:
  struct UncheckedTag {};
  constexpr Angle(double radians, UncheckedTag) noexcept : radians_(radians) {}
  double radians_;
};

struct Point2D {
  double x_m = 0.0;
  double y_m = 0.0;

  bool finite() const noexcept {
    return std::isfinite(x_m) && std::isfinite(y_m);
  }

  bool operator==(const Point2D&) const = default;
  std::partial_ordering operator<=>(const Point2D&) const = default;
};

struct Pose2D {
  Point2D position;
  Angle heading = Angle::zero();

  bool operator==(const Pose2D&) const = default;
};

struct Segment2D {
  Point2D start;
  Point2D end;

  Distance length() const {
    return Distance(std::hypot(end.x_m - start.x_m, end.y_m - start.y_m));
  }

  bool operator==(const Segment2D&) const = default;
};

struct Circle {
  Point2D center;
  Distance radius = Distance::zero();

  bool contains(const Point2D& point,
                double tolerance_m = geometry_tolerance_m) const noexcept {
    return std::hypot(point.x_m - center.x_m, point.y_m - center.y_m) <=
           radius.meters() + std::max(0.0, tolerance_m);
  }

  bool operator==(const Circle&) const = default;
};

class Polygon {
 public:
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

  const std::vector<Point2D>& vertices() const noexcept { return vertices_; }

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

inline Distance distance(const Point2D& first, const Point2D& second) {
  return Distance(std::hypot(second.x_m - first.x_m, second.y_m - first.y_m));
}

inline double cross(const Point2D& origin, const Point2D& first,
                    const Point2D& second) noexcept {
  return (first.x_m - origin.x_m) * (second.y_m - origin.y_m) -
         (first.y_m - origin.y_m) * (second.x_m - origin.x_m);
}

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
