#ifndef SEMAFORR_SPATIAL_REPRESENTATIONS_MODELS_HPP
#define SEMAFORR_SPATIAL_REPRESENTATIONS_MODELS_HPP

#include <cstddef>
#include <cstdint>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/circumstance.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/highway.hpp>
#include <semaforr/domain/grid_layers.hpp>
#include <vector>

namespace semaforr::spatial {

struct TrailModel { std::vector<std::vector<domain::Point2D>> trails; };
struct ConveyorFlow {
  domain::Segment2D axis;
  std::size_t traversals = 1U;
};
struct ConveyorModel { std::vector<ConveyorFlow> flows; };
struct RegionModel { std::vector<domain::Circle> regions; };
struct DoorExitModel { std::vector<domain::Segment2D> openings; };
struct HallwayModel { std::vector<domain::Segment2D> centerlines; };
struct BarrierModel { std::vector<domain::Segment2D> barriers; };
struct SkeletonEdge {
  std::size_t from = 0U;
  std::size_t to = 0U;
};
struct PassageSkeletonModel {
  std::vector<domain::Point2D> nodes;
  std::vector<SkeletonEdge> edges;
  std::vector<std::size_t> component_by_node;
  std::size_t connectivity_revision = 0U;
};
struct GridGeometry {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  domain::Point2D origin;
};
struct SparseGridCell {
  std::size_t index = 0U;
  std::uint32_t value = 0U;
};
struct FamiliarityCellMetadata {
  std::size_t index = 0U;
  std::size_t last_observed_sequence = 0U;
  float confidence = 0.0F;
};
struct KnownGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> observations;
  std::vector<SparseGridCell> sparse_observations;
  std::vector<FamiliarityCellMetadata> sparse_metadata;
};
using SensedOccupancyModel = domain::SensedOccupancyGrid;
struct InclusionGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> included;
  std::vector<SparseGridCell> sparse_included;
};
using HighwayIntersection = domain::HighwayIntersection;
struct HighwayGridLabel {
  std::size_t row = 0U;
  std::size_t column = 0U;
  std::uint32_t label = 0U;
};
struct HighwayModel {
  static constexpr std::size_t schema_version = 1U;
  domain::Graph<domain::Intersection, domain::HighwayEdge> graph;
  std::vector<domain::Highway> highways;
  std::size_t serialized_schema_version = schema_version;
  std::vector<domain::Point2D> nodes;
  std::vector<SkeletonEdge> edges;
  std::vector<HighwayIntersection> intersections;
  std::vector<HighwayGridLabel> grid_labels;
  std::vector<std::size_t> touched_rows;
  std::vector<std::size_t> touched_columns;
};
using CircumstanceModel = domain::CircumstanceModel;

}  // namespace semaforr::spatial

#endif
