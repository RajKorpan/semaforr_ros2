#ifndef SEMAFORR_DOMAIN_HIGHWAY_HPP
#define SEMAFORR_DOMAIN_HIGHWAY_HPP

#include <cstddef>
#include <semaforr/domain/geometry.hpp>
#include <utility>
#include <vector>

namespace semaforr::domain {

using HighwayId = std::size_t;
using IntersectionId = std::size_t;
using ModelRevision = std::size_t;
using TrailId = std::size_t;

enum class Axis { Horizontal, Vertical };

struct GridCell {
  int row = 0;
  int column = 0;
  bool operator==(const GridCell&) const = default;
};

struct Intersection {
  IntersectionId id = 0U;
  GridCell cell;
  Point2D position;
  bool terminal_access = false;
};

struct HighwayEdge {
  IntersectionId from = 0U;
  IntersectionId to = 0U;
  HighwayId highway = 0U;
  double length_m = 0.0;
  std::vector<TrailId> trail_labels;
};

template <typename Vertex, typename Edge>
struct Graph {
  std::vector<Vertex> vertices;
  std::vector<Edge> edges;
};

struct Highway {
  HighwayId id = 0U;
  Axis axis = Axis::Horizontal;
  std::vector<GridCell> cells;
  std::vector<IntersectionId> endpoints;
};

struct HighwayIntersection {
  std::size_t node = 0U;
  std::size_t degree = 0U;
};

struct HighwayGraph {
  static constexpr std::size_t schema_version = 1U;
  Graph<Intersection, HighwayEdge> graph;
  std::vector<Highway> highways;
  ModelRevision revision = 0U;
  std::size_t serialized_schema_version = schema_version;
  std::vector<Point2D> nodes;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<HighwayIntersection> intersections;
};

}  // namespace semaforr::domain

#endif
