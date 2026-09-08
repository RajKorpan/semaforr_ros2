/**
 * @file completed_path.cpp
 * @brief Completed path responsibilities.
 *
 * @details This file implements completed path behavior for ROS-independent
 * domain state and value types. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its package-relative
 * location is `src/domain/completed_path.cpp`.
 */
#include <semaforr/domain/completed_path.hpp>
#include <stdexcept>

namespace semaforr::domain {

/**
 * @brief Performs the begin operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p task_id: Supplies task id input to the operation.
 * - @p target: Supplies target input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void PathHistory::begin(PathId id, std::optional<TaskId> task_id,
                        std::optional<Point2D> target) {
  if (active_ && !active_->decision_points.empty())
    throw std::logic_error(
        "cannot replace an unfinished completed-path record");
  active_ = CompletedPath{};
  active_->id = id;
  active_->task_id = task_id;
  active_->target = target;
}

/**
 * @brief Records package content for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void PathHistory::record(PathDecisionPoint point) {
  terminal_events_.push_back(point);
  if (!active_)
    begin(point.selection.decision_id, point.selection.task_id, point.target);
  if (active_->decision_points.empty())
    active_->started_at = point.execution.started_at;
  if (active_->task_id != point.selection.task_id)
    throw std::logic_error(
        "path decision point task does not match active path");
  active_->decision_points.push_back(std::move(point));
}

/**
 * @brief Performs the finish operation for this subsystem.
 *
 * Arguments:
 * - @p target_reached: Supplies target reached input to the operation.
 * - @p task_skipped: Supplies task skipped input to the operation.
 * - @p when: Supplies when input to the operation.
 *
 * Returns:
 * - `std::optional<CompletedPath>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<CompletedPath> PathHistory::finish(bool target_reached,
                                                 bool task_skipped,
                                                 ExecutionTimestamp when) {
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
