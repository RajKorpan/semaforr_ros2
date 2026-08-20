#ifndef SEMAFORR_DOMAIN_ACTION_EXECUTION_HPP
#define SEMAFORR_DOMAIN_ACTION_EXECUTION_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/mission.hpp>
#include <string>
#include <string_view>

namespace semaforr::domain {

using DecisionId = std::uint64_t;
using ActionId = std::uint64_t;
using ExecutionTimestamp = std::chrono::steady_clock::time_point;

struct SelectedActionRecord {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  std::optional<TaskId> task_id;
  ExecutionTimestamp selected_at{};
  Pose2D expected_start;
  Action action{Action::pause()};
  std::string selected_tier;
  std::string provenance;
  double intended_distance_m{0.0};
  double intended_rotation_rad{0.0};
  double intended_duration_s{0.0};
};

enum class ExecutionCompletionStatus {
  Succeeded,
  PartialMovement,
  NoMovement,
  TimedOut,
  Cancelled,
  SafetyInterrupted,
  ControllerRejected,
  ControllerFailure,
  GoalPreempted,
  NavigationModeTransition,
  SensorLost,
  Shutdown,
  ClockReset,
  OdometryReset
};

struct ActionStartedEvent {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  ExecutionTimestamp started_at{};
  Pose2D start_pose;
};

struct ActionProgressEvent {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  ExecutionTimestamp observed_at{};
  Pose2D pose;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};
};

struct ActionExecutionResult {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  std::optional<TaskId> task_id;
  ExecutionTimestamp started_at{};
  ExecutionTimestamp finished_at{};
  ExecutionCompletionStatus status{ExecutionCompletionStatus::NoMovement};
  Pose2D start_pose;
  Pose2D final_pose;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};
  bool timed_out{false};
  std::string cancellation_reason;
  bool safety_interruption{false};
  bool controller_failure{false};
  bool collision{false};
  bool near_collision{false};

  bool terminal() const noexcept { return true; }
  bool successful() const noexcept {
    return status == ExecutionCompletionStatus::Succeeded;
  }
  bool moved() const noexcept {
    return distance_achieved_m > geometry_tolerance_m ||
           rotation_achieved_rad > geometry_tolerance_m;
  }
  bool translated() const noexcept {
    return distance_achieved_m > geometry_tolerance_m ||
           distance(start_pose.position, final_pose.position).meters() >
               geometry_tolerance_m;
  }
  bool rotated() const noexcept {
    return rotation_achieved_rad > geometry_tolerance_m ||
           std::abs(Angle::normalize(final_pose.heading.radians() -
                                     start_pose.heading.radians())) >
               geometry_tolerance_m;
  }
};

enum class FeedbackDisposition {
  Accepted,
  Duplicate,
  UnknownAction,
  StaleDecision,
  TaskMismatch,
  NotStarted,
  AlreadyStarted
};

std::string_view toString(ExecutionCompletionStatus status) noexcept;
std::string_view toString(FeedbackDisposition disposition) noexcept;

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_ACTION_EXECUTION_HPP
