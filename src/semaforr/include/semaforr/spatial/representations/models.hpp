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
using GridGeometry = domain::GridGeometry;
using SparseGridCell = domain::SparseCountCell;
using FamiliarityCellMetadata = domain::SparseFamiliarityMetadata;
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
  int row = 0;
  int column = 0;
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
  std::vector<int> touched_rows;
  std::vector<int> touched_columns;
};
using CircumstanceModel = domain::CircumstanceModel;

}  // namespace semaforr::spatial

#endif
