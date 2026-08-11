#ifndef SEMAFORR_DOMAIN_SPATIAL_AFFORDANCES_HPP
#define SEMAFORR_DOMAIN_SPATIAL_AFFORDANCES_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/completed_path.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <vector>

namespace semaforr::domain {

using TrailId = std::uint64_t;
using RegionId = std::uint64_t;
using ExitId = std::uint64_t;
using DoorId = std::uint64_t;
using HallwayId = std::uint64_t;

struct VisibilityEvidence {
  Point2D observer;
  Point2D endpoint;
  Distance visible_distance{0.0};
  std::size_t ray_index{0U};
  DecisionId decision_id{0U};
  bool historical_sensor_view{true};
};

struct TrailMarker {
  std::size_t path_point_index{0U};
  Pose2D pose;
  LaserObservation view;
  std::optional<VisibilityEvidence> visibility_to_next;
};

struct LearnedTrail {
  TrailId id{0U};
  PathId source_path{0U};
  std::optional<TaskId> task_id;
  std::optional<Point2D> target;
  std::vector<TrailMarker> markers;
  std::vector<std::vector<Point2D>> subtrail_geometry;
  double length_m{0.0};
};

struct ConveyorCell {
  std::size_t index{0U};
  std::uint32_t traversal_frequency{0U};
  double direction_x{0.0};
  double direction_y{0.0};
  double normalized_strength{0.0};
};

struct ConveyorGrid {
  GridGeometry geometry;
  std::vector<ConveyorCell> cells;
  double decay_factor{1.0};
  std::uint32_t maximum_frequency{0U};
  std::size_t revision{0U};

  const ConveyorCell* at(Point2D point) const noexcept;
};

struct RegionVisibilityBin {
  bool known{false};
  double maximum_distance_m{-1.0};
  Point2D ray_start;
  Point2D ray_end;
  DecisionId decision_id{0U};
};

struct LearnedRegion {
  RegionId id{0U};
  Circle boundary;
  RobotObservation supporting_observation;
  DecisionId supporting_decision{0U};
  std::vector<DecisionId> contributing_decisions;
  std::array<RegionVisibilityBin, 360U> visibility;
  std::size_t visibility_revision{0U};
};

struct RegionExit {
  ExitId id{0U};
  RegionId region{0U};
  Point2D point;
  Angle outward_heading{Angle::zero()};
  std::size_t traversal_count{0U};
  std::vector<PathId> supporting_paths;
  double confidence{0.0};
};

struct LearnedDoor {
  DoorId id{0U};
  RegionId region{0U};
  std::vector<ExitId> exits;
  double clockwise_start_rad{0.0};
  double clockwise_end_rad{0.0};
  double confidence{0.0};
  std::size_t supporting_traversals{0U};
};

struct SensorOpening {
  Segment2D opening;
  DecisionId decision_id{0U};
  double confidence{0.0};
};

enum class HallwayDirection {
  Horizontal,
  Vertical,
  MajorDiagonal,
  MinorDiagonal
};

struct HallwayCell {
  long long x{0};
  long long y{0};
  std::uint32_t heat{0U};
};

struct LearnedHallway {
  HallwayId id{0U};
  HallwayDirection direction{HallwayDirection::Horizontal};
  Segment2D centerline;
  double width_m{0.0};
  double extent_m{0.0};
  std::vector<HallwayCell> connected_area;
  std::size_t supporting_segments{0U};
};

struct RegionSkeletonNode {
  std::size_t id{0U};
  RegionId region{0U};
  Point2D center;
  std::array<RegionVisibilityBin, 360U> visibility;
};

struct RegionSkeletonEdge {
  std::size_t from{0U};
  std::size_t to{0U};
  std::vector<Point2D> supporting_subtrail;
  double length_m{0.0};
  PathId source_path{0U};
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_SPATIAL_AFFORDANCES_HPP
