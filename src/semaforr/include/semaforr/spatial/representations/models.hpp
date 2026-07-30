#ifndef SEMAFORR_SPATIAL_REPRESENTATIONS_MODELS_HPP
#define SEMAFORR_SPATIAL_REPRESENTATIONS_MODELS_HPP

#include <cstddef>
#include <cstdint>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
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
struct KnownGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> observations;
  std::vector<SparseGridCell> sparse_observations;
};
struct InclusionGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> included;
  std::vector<SparseGridCell> sparse_included;
};
struct HighwayIntersection {
  std::size_t node = 0U;
  std::size_t degree = 0U;
};
struct HighwayGridLabel {
  std::size_t row = 0U;
  std::size_t column = 0U;
  std::uint32_t label = 0U;
};
struct HighwayModel {
  std::vector<domain::Point2D> nodes;
  std::vector<SkeletonEdge> edges;
  std::vector<HighwayIntersection> intersections;
  std::vector<HighwayGridLabel> grid_labels;
  std::vector<std::size_t> touched_rows;
  std::vector<std::size_t> touched_columns;
};
struct CircumstanceObservation {
  domain::Action action = domain::Action::pause();
  std::size_t occurrences = 0U;
};
struct CircumstanceModel {
  std::vector<CircumstanceObservation> actions;
};

}  // namespace semaforr::spatial

#endif
