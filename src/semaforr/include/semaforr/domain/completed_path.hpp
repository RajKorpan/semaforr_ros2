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

  bool successfulTraversal() const noexcept {
    return execution.successful() && execution.moved();
  }
  bool partialTraversal() const noexcept {
    return execution.status == ExecutionCompletionStatus::PartialMovement &&
           execution.moved();
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
