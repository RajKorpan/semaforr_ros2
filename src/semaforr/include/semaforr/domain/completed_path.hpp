#ifndef SEMAFORR_DOMAIN_COMPLETED_PATH_HPP
#define SEMAFORR_DOMAIN_COMPLETED_PATH_HPP

#include <cstdint>
#include <optional>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/observation.hpp>
#include <vector>

namespace semaforr::domain {

using PathId = std::uint64_t;

struct PathDecisionPoint {
  SelectedActionRecord selection;
  ActionExecutionResult execution;
  RobotObservation decision_observation;
  std::optional<Action> executed_action;
  std::optional<Point2D> target;
  bool task_started{false};
  bool task_finished{false};
  bool interrupted{false};

  // Selection, command acceptance, and terminal execution are deliberately
  // separate. A terminal record without executed_action was selected but was
  // never accepted by the controller.
  bool selected() const noexcept {
    return selection.decision_id != 0U && selection.action_id != 0U;
  }
  bool started() const noexcept { return executed_action.has_value(); }
  bool completed() const noexcept {
    return execution.status == ExecutionCompletionStatus::Succeeded;
  }
  bool partiallyCompleted() const noexcept {
    return execution.status == ExecutionCompletionStatus::PartialMovement;
  }
  bool failed() const noexcept {
    switch (execution.status) {
      case ExecutionCompletionStatus::NoMovement:
      case ExecutionCompletionStatus::ControllerRejected:
      case ExecutionCompletionStatus::ControllerFailure:
      case ExecutionCompletionStatus::SensorLost:
      case ExecutionCompletionStatus::Shutdown:
      case ExecutionCompletionStatus::ClockReset:
      case ExecutionCompletionStatus::OdometryReset: return true;
      default: return false;
    }
  }
  bool cancelled() const noexcept {
    return execution.status == ExecutionCompletionStatus::Cancelled;
  }
  bool timedOut() const noexcept {
    return execution.status == ExecutionCompletionStatus::TimedOut ||
           execution.timed_out;
  }
  bool safetyInterrupted() const noexcept {
    return execution.status == ExecutionCompletionStatus::SafetyInterrupted ||
           execution.safety_interruption;
  }
  bool preempted() const noexcept {
    return execution.status == ExecutionCompletionStatus::GoalPreempted ||
           execution.status ==
               ExecutionCompletionStatus::NavigationModeTransition;
  }
  const Pose2D& actualReachedPose() const noexcept {
    return execution.final_pose;
  }
  bool successfulTraversal() const noexcept {
    return started() && execution.successful() && execution.translated();
  }
  bool partialTraversal() const noexcept {
    return started() && partiallyCompleted() && execution.translated();
  }
};

struct CompletedPath {
  PathId id{0U};
  std::optional<TaskId> task_id;
  std::optional<Point2D> target;
  std::vector<PathDecisionPoint> decision_points;
  ExecutionTimestamp started_at{};
  ExecutionTimestamp finished_at{};
  bool target_reached{false};
  bool task_skipped{false};

  bool empty() const noexcept { return decision_points.empty(); }
};

class PathHistory {
 public:
  void begin(PathId id, std::optional<TaskId> task_id,
             std::optional<Point2D> target);
  void record(PathDecisionPoint point);
  std::optional<CompletedPath> finish(bool target_reached, bool task_skipped,
                                      ExecutionTimestamp when);

  const std::optional<CompletedPath>& active() const noexcept {
    return active_;
  }
  const std::vector<CompletedPath>& completed() const noexcept {
    return completed_;
  }
  const std::vector<PathDecisionPoint>& terminalEvents() const noexcept {
    return terminal_events_;
  }

 private:
  std::optional<CompletedPath> active_;
  std::vector<CompletedPath> completed_;
  std::vector<PathDecisionPoint> terminal_events_;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_COMPLETED_PATH_HPP
