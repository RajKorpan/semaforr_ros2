/**
 * @file spatial_affordances.hpp
 * @brief Spatial affordances responsibilities.
 *
 * @details This file defines spatial affordances behavior for ROS-independent
 * domain state and value types. It centers on `VisibilityEvidence`,
 * `TrailMarker`, `LearnedTrail`, `ConveyorCell`, `ConveyorGrid`,
 * `RegionVisibilityBin`, `LearnedRegion`, `RegionExit`. Its
 * package-relative location is
 * `include/semaforr/domain/spatial_affordances.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_SPATIAL_AFFORDANCES_HPP
#define SEMAFORR_DOMAIN_SPATIAL_AFFORDANCES_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/completed_path.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <utility>
#include <vector>

namespace semaforr::domain {

using TrailId = std::uint64_t;
using RegionId = std::uint64_t;
using ExitId = std::uint64_t;
using DoorId = std::uint64_t;
using HallwayId = std::uint64_t;

/**
 * @brief Encapsulates visibility evidence state and behavior for this
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
struct VisibilityEvidence {
  Point2D observer;
  Point2D endpoint;
  Distance visible_distance{0.0};
  std::size_t ray_index{0U};
  DecisionId decision_id{0U};
  bool historical_sensor_view{true};
};

/**
 * @brief Encapsulates trail marker state and behavior for this subsystem.
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
struct TrailMarker {
  std::size_t path_point_index{0U};
  Pose2D pose;
  LaserObservation view;
  std::optional<VisibilityEvidence> visibility_to_next;
};

/**
 * @brief Encapsulates learned trail state and behavior for this subsystem.
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
struct LearnedTrail {
  TrailId id{0U};
  PathId source_path{0U};
  std::optional<TaskId> task_id;
  std::optional<Point2D> target;
  std::vector<TrailMarker> markers;
  std::vector<std::vector<Point2D>> subtrail_geometry;
  double length_m{0.0};
};

/**
 * @brief Encapsulates conveyor cell state and behavior for this subsystem.
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
struct ConveyorCell {
  std::size_t index{0U};
  std::uint32_t traversal_frequency{0U};
  double direction_x{0.0};
  double direction_y{0.0};
  double normalized_strength{0.0};
};

/**
 * @brief Encapsulates conveyor grid state and behavior for this subsystem.
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
struct ConveyorGrid {
  GridGeometry geometry;
  std::vector<ConveyorCell> cells;
  double decay_factor{1.0};
  std::uint32_t maximum_frequency{0U};
  std::size_t revision{0U};

  /**
   * @brief Performs the at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `const ConveyorCell*` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const ConveyorCell* at(Point2D point) const noexcept;
};

/**
 * @brief Encapsulates region visibility bin state and behavior for this
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
struct RegionVisibilityBin {
  bool known{false};
  double maximum_distance_m{-1.0};
  Point2D ray_start;
  Point2D ray_end;
  DecisionId decision_id{0U};
};

/**
 * @brief Encapsulates learned region state and behavior for this subsystem.
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
struct LearnedRegion {
  RegionId id{0U};
  Circle boundary;
  RobotObservation supporting_observation;
  DecisionId supporting_decision{0U};
  std::vector<DecisionId> contributing_decisions;
  std::array<RegionVisibilityBin, 360U> visibility;
  std::size_t visibility_revision{0U};
};

/**
 * @brief Encapsulates region exit state and behavior for this subsystem.
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
struct RegionExit {
  ExitId id{0U};
  RegionId region{0U};
  Point2D point;
  /**
   * @brief Performs the zero operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Angle outward_heading{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Angle outward_heading{Angle::zero()};
  std::size_t traversal_count{0U};
  std::vector<PathId> supporting_paths;
  double confidence{0.0};
};

/**
 * @brief Encapsulates learned door state and behavior for this subsystem.
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
struct LearnedDoor {
  DoorId id{0U};
  RegionId region{0U};
  std::vector<ExitId> exits;
  double clockwise_start_rad{0.0};
  double clockwise_end_rad{0.0};
  double confidence{0.0};
  std::size_t supporting_traversals{0U};
};

/**
 * @brief Encapsulates sensor opening state and behavior for this subsystem.
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
struct SensorOpening {
  Segment2D opening;
  DecisionId decision_id{0U};
  double confidence{0.0};
};

/**
 * @brief Enumerates the supported hallway direction values used by this
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
enum class HallwayDirection {
  Horizontal,
  Vertical,
  MajorDiagonal,
  MinorDiagonal
};

/**
 * @brief Encapsulates hallway cell state and behavior for this subsystem.
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
struct HallwayCell {
  long long x{0};
  long long y{0};
  std::uint32_t heat{0U};
};

/**
 * @brief Encapsulates learned hallway state and behavior for this
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
struct LearnedHallway {
  HallwayId id{0U};
  HallwayDirection direction{HallwayDirection::Horizontal};
  Segment2D centerline;
  double width_m{0.0};
  double extent_m{0.0};
  std::vector<HallwayCell> connected_area;
  std::size_t supporting_segments{0U};
};

/**
 * @brief Encapsulates region skeleton node state and behavior for this
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
struct RegionSkeletonNode {
  std::size_t id{0U};
  RegionId region{0U};
  Point2D center;
  std::array<RegionVisibilityBin, 360U> visibility;
};

/**
 * @brief Encapsulates region skeleton trail evidence state and behavior for
 * this subsystem.
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
struct RegionSkeletonTrailEvidence {
  std::size_t from{0U};
  std::size_t to{0U};
  LearnedTrail trail;
};

/**
 * @brief Encapsulates region skeleton edge state and behavior for this
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
struct RegionSkeletonEdge {
  std::size_t from{0U};
  std::size_t to{0U};
  std::vector<Point2D> supporting_subtrail;
  double length_m{0.0};
  PathId source_path{0U};
  // Every entry is learned from one contiguous, execution-confirmed raw path
  // segment. `supporting_subtrail` is the shortest valid learned trail,
  // normalized from `from` to `to`, and is the operational edge label.
  std::vector<RegionSkeletonTrailEvidence> supporting_trails;
  std::optional<TrailId> operational_trail_id;

  /**
   * @brief Performs the region skeleton edge operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  RegionSkeletonEdge() = default;
  /**
   * @brief Performs the region skeleton edge operation for this subsystem.
   *
   * Arguments:
   * - @p edge_from: Supplies edge from input to the operation.
   * - @p edge_to: Supplies edge to input to the operation.
   * - @p subtrail: Supplies subtrail input to the operation.
   * - @p length: Supplies length input to the operation.
   * - @p path: Supplies path input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  RegionSkeletonEdge(std::size_t edge_from, std::size_t edge_to,
                     std::vector<Point2D> subtrail, double length,
                     PathId path)
      : from(edge_from),
        to(edge_to),
        supporting_subtrail(std::move(subtrail)),
        length_m(length),
        source_path(path) {}
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_SPATIAL_AFFORDANCES_HPP
