#include <semaforr/domain/completed_path.hpp>
#include <stdexcept>

namespace semaforr::domain {

void PathHistory::begin(PathId id, std::optional<TaskId> task_id,
                        std::optional<Point2D> target) {
  if (active_ && !active_->decision_points.empty())
    throw std::logic_error("cannot replace an unfinished completed-path record");
  active_ = CompletedPath{};
  active_->id = id;
  active_->task_id = task_id;
  active_->target = target;
}

void PathHistory::record(PathDecisionPoint point) {
  terminal_events_.push_back(point);
  if (!active_)
    begin(point.selection.decision_id, point.selection.task_id, point.target);
  if (active_->decision_points.empty())
    active_->started_at = point.execution.started_at;
  if (active_->task_id != point.selection.task_id)
    throw std::logic_error("path decision point task does not match active path");
  active_->decision_points.push_back(std::move(point));
}

std::optional<CompletedPath> PathHistory::finish(
    bool target_reached, bool task_skipped, ExecutionTimestamp when) {
  if (!active_) return std::nullopt;
  active_->target_reached = target_reached;
  active_->task_skipped = task_skipped;
  active_->finished_at = when;
  if (!active_->decision_points.empty())
    active_->decision_points.back().task_finished = true;
  completed_.push_back(std::move(*active_));
  active_.reset();
  return completed_.back();
}

}  // namespace semaforr::domain
