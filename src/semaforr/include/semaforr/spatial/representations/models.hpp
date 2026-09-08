/**
 * @file models.hpp
 * @brief Models responsibilities.
 *
 * @details This file defines models behavior for learned spatial
 * representations and their lifecycle. It centers on `TrailModel`,
 * `ConveyorFlow`, `ConveyorModel`, `RegionModel`, `DoorExitModel`,
 * `HallwayModel`, `BarrierModel`, `SkeletonEdge`. Its package-relative location
 * is `include/semaforr/spatial/representations/models.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_REPRESENTATIONS_MODELS_HPP
#define SEMAFORR_SPATIAL_REPRESENTATIONS_MODELS_HPP

#include <cstddef>
#include <cstdint>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/circumstance.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/grid_layers.hpp>
#include <semaforr/domain/highway.hpp>
#include <semaforr/domain/spatial_affordances.hpp>
#include <vector>

namespace semaforr::spatial {

/**
 * @brief Encapsulates trail model state and behavior for this subsystem.
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
struct TrailModel {
  std::vector<domain::LearnedTrail> learned_trails;
  std::vector<std::vector<domain::Point2D>> trails;
};
/**
 * @brief Encapsulates conveyor flow state and behavior for this subsystem.
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
struct ConveyorFlow {
  domain::Segment2D axis;
  std::size_t traversals = 1U;
};
/**
 * @brief Encapsulates conveyor model state and behavior for this subsystem.
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
struct ConveyorModel {
  domain::ConveyorGrid grid;
  std::vector<ConveyorFlow> flows;
};
/**
 * @brief Encapsulates region model state and behavior for this subsystem.
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
struct RegionModel {
  std::vector<domain::LearnedRegion> learned_regions;
  std::vector<domain::Circle> regions;
};
/**
 * @brief Encapsulates door exit model state and behavior for this
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
struct DoorExitModel {
  std::vector<domain::RegionExit> exits;
  std::vector<domain::LearnedDoor> doors;
  std::vector<domain::SensorOpening> sensor_openings;
  std::vector<domain::Segment2D> openings;
};
/**
 * @brief Encapsulates hallway model state and behavior for this subsystem.
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
struct HallwayModel {
  std::vector<domain::LearnedHallway> hallways;
  std::vector<domain::Segment2D> centerlines;
};
/**
 * @brief Encapsulates barrier model state and behavior for this subsystem.
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
struct BarrierModel {
  std::vector<domain::Segment2D> barriers;
};
/**
 * @brief Encapsulates skeleton edge state and behavior for this subsystem.
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
struct SkeletonEdge {
  std::size_t from = 0U;
  std::size_t to = 0U;
};
/**
 * @brief Encapsulates passage skeleton model state and behavior for this
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
struct PassageSkeletonModel {
  std::vector<domain::RegionSkeletonNode> region_nodes;
  std::vector<domain::RegionSkeletonEdge> region_edges;
  std::vector<domain::Point2D> nodes;
  std::vector<SkeletonEdge> edges;
  std::vector<domain::Point2D> sampled_path_nodes;
  std::vector<SkeletonEdge> sampled_path_edges;
  std::vector<std::size_t> component_by_node;
  std::size_t connectivity_revision = 0U;
};
using GridGeometry = domain::GridGeometry;
using SparseGridCell = domain::SparseCountCell;
using FamiliarityCellMetadata = domain::SparseFamiliarityMetadata;
/**
 * @brief Encapsulates known grid model state and behavior for this
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
struct KnownGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> observations;
  std::vector<SparseGridCell> sparse_observations;
  std::vector<FamiliarityCellMetadata> sparse_metadata;
};
using SensedOccupancyModel = domain::SensedOccupancyGrid;
/**
 * @brief Encapsulates inclusion grid model state and behavior for this
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
struct InclusionGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> included;
  std::vector<SparseGridCell> sparse_included;
};
using HighwayIntersection = domain::HighwayIntersection;
/**
 * @brief Encapsulates highway grid label state and behavior for this
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
struct HighwayGridLabel {
  int row = 0;
  int column = 0;
  std::uint32_t label = 0U;
};
/**
 * @brief Encapsulates highway model state and behavior for this subsystem.
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
struct HighwayModel {
  static constexpr std::size_t schema_version = 2U;
  GridGeometry geometry;
  domain::Graph<domain::Intersection, domain::HighwayEdge> graph;
  std::vector<domain::Highway> highways;
  std::size_t serialized_schema_version = schema_version;
  std::vector<domain::Point2D> nodes;
  std::vector<SkeletonEdge> edges;
  std::vector<HighwayIntersection> intersections;
  std::vector<HighwayGridLabel> grid_labels;
  std::vector<int> touched_rows;
  std::vector<int> touched_columns;
  std::string smoothing_policy{"von_neumann_three_of_four"};
  std::string component_selection_policy{"most_intersections"};
};
using CircumstanceModel = domain::CircumstanceModel;

}  // namespace semaforr::spatial

#endif
