#ifndef SEMAFORR_DOMAIN_WORLD_MODEL_HPP
#define SEMAFORR_DOMAIN_WORLD_MODEL_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/domain/circumstance.hpp>
#include <semaforr/domain/highway.hpp>
#include <semaforr/domain/mission.hpp>
#include <semaforr/domain/observation.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::domain {

class ActionSpace {
 public:
  ActionSpace(std::vector<double> move_distances_m,
              std::vector<double> rotation_angles_rad)
      : move_distances_m_(std::move(move_distances_m)),
        rotation_angles_rad_(std::move(rotation_angles_rad)) {
    validate(move_distances_m_, "move distances");
    validate(rotation_angles_rad_, "rotation angles");
    if (move_distances_m_.size() > Action::maximum_magnitude_index ||
        rotation_angles_rad_.size() > Action::maximum_magnitude_index) {
      throw std::invalid_argument(
          "action arrays may contain at most 299 values");
    }
  }

  const std::vector<double>& move_distances_m() const noexcept {
    return move_distances_m_;
  }
  const std::vector<double>& rotation_angles_rad() const noexcept {
    return rotation_angles_rad_;
  }

  bool contains(const Action& action) const noexcept {
    switch (action.type()) {
      case ActionType::Pause:
        return action.magnitude_index() == 0U;
      case ActionType::Forward:
        return action.magnitude_index() <= move_distances_m_.size();
      case ActionType::TurnRight:
      case ActionType::TurnLeft:
        return action.magnitude_index() <= rotation_angles_rad_.size();
    }
    return false;
  }

 private:
  static void validate(const std::vector<double>& values, const char* name) {
    if (values.empty()) {
      throw std::invalid_argument(std::string(name) + " must not be empty");
    }
    if (!std::all_of(values.begin(), values.end(), [](double value) {
          return std::isfinite(value) && value > 0.0;
        })) {
      throw std::invalid_argument(std::string(name) +
                                  " must contain finite positive values");
    }
    if (!std::is_sorted(values.begin(), values.end()) ||
        std::adjacent_find(values.begin(), values.end()) != values.end()) {
      throw std::invalid_argument(std::string(name) +
                                  " must be strictly increasing");
    }
  }

  std::vector<double> move_distances_m_;
  std::vector<double> rotation_angles_rad_;
};

struct RobotState {
  Pose2D pose;
  std::optional<LaserObservation> laser;
  std::chrono::steady_clock::time_point observed_at{};
};

struct NavigationHistoryEntry {
  Pose2D pose;
  LaserObservation laser;
  Action action = Action::pause();
  std::optional<TaskId> task_id;
};

class NavigationHistory {
 public:
  void record(NavigationHistoryEntry entry) {
    entries_.push_back(std::move(entry));
  }

  const std::vector<NavigationHistoryEntry>& entries() const noexcept {
    return entries_;
  }

 private:
  std::vector<NavigationHistoryEntry> entries_;
};

struct RecoveryState {
  bool confined = false;
  std::size_t get_out_attempts = 0U;
  std::size_t reposition_attempts = 0U;
};

struct FreespaceGrid {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  Point2D origin;
  std::vector<std::uint32_t> cells;
  std::size_t revision = 0U;
};

struct ExplorationCue {
  std::uint64_t id = 0U;
  Point2D start;
  Point2D target;
};

struct SpatialModel {
  std::vector<Polygon> obstacle_polygons;
  std::vector<std::vector<Point2D>> trails;
  std::vector<Segment2D> conveyor_flows;
  std::vector<std::size_t> conveyor_traversals;
  std::vector<Circle> learned_regions;
  std::vector<Segment2D> doorways;
  std::vector<Segment2D> hallways;
  std::vector<Segment2D> barriers;
  std::vector<Point2D> skeleton_nodes;
  std::vector<std::pair<std::size_t, std::size_t>> skeleton_edges;
  FreespaceGrid known_grid;
  FreespaceGrid inclusion_grid;
  std::vector<ExplorationCue> unfinished_hle_candidates;
  HighwayGraph highways;
  CircumstanceModel circumstances;
  std::size_t revision = 0U;
};

struct WorldModel {
  RobotState robot;
  Mission mission;
  NavigationHistory navigation_history;
  RecoveryState recovery;
  CrowdModel crowd;
  SpatialModel spatial;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_WORLD_MODEL_HPP
