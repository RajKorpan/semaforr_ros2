/**
 * @file mission.hpp
 * @brief Mission responsibilities.
 *
 * @details This file defines mission behavior for ROS-independent domain state and
 * value types. It centers on `NavigationTask`, `Mission`. Its
 * package-relative location is `include/semaforr/domain/mission.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_MISSION_HPP
#define SEMAFORR_DOMAIN_MISSION_HPP

#include <cstddef>
#include <deque>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace semaforr::domain {

using TaskId = std::size_t;

/**
 * @brief Encapsulates navigation task state and behavior for this
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
struct NavigationTask {
  TaskId id = 0U;
  Point2D target;
  std::vector<Point2D> plan;
  std::size_t waypoint_index = 0U;

  /**
   * @brief Performs the navigation task operation for this subsystem.
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
  NavigationTask() = default;
  /**
   * @brief Performs the navigation task operation for this subsystem.
   *
   * Arguments:
   * - @p task_id: Supplies task id input to the operation.
   * - @p task_target: Supplies task target input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationTask(TaskId task_id, Point2D task_target)
      : id(task_id), target(task_target) {}

  /**
   * @brief Performs the waypoint operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::optional<Point2D>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<Point2D> waypoint() const noexcept {
    if (waypoint_index >= plan.size()) {
      return std::nullopt;
    }
    return plan[waypoint_index];
  }
};

/**
 * @brief Encapsulates mission state and behavior for this subsystem.
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
class Mission {
 public:
  /**
   * @brief Performs the mission operation for this subsystem.
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
  Mission() = default;

  /**
   * @brief Performs the mission operation for this subsystem.
   *
   * Arguments:
   * - @p tasks: Supplies tasks input to the operation.
   * - @p decision_limit: Supplies decision limit input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit Mission(std::vector<NavigationTask> tasks,
                   std::size_t decision_limit = 1U)
      : decision_limit_(decision_limit) {
    if (decision_limit_ == 0U) {
      throw std::invalid_argument("mission decision limit must be positive");
    }
    for (NavigationTask& task : tasks) {
      pending_.push_back(std::move(task));
    }
  }

  /**
   * @brief Performs the activate next operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool activate_next() {
    if (active_ || pending_.empty()) {
      return false;
    }
    active_ = std::move(pending_.front());
    pending_.pop_front();
    decisions_for_active_ = 0U;
    return true;
  }

  /**
   * @brief Records decision for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool record_decision() {
    if (!active_) {
      return false;
    }
    ++decisions_for_active_;
    return decisions_for_active_ <= decision_limit_;
  }

  /**
   * @brief Performs the complete active operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool complete_active() {
    if (!active_) {
      return false;
    }
    completed_.push_back(std::move(*active_));
    active_.reset();
    decisions_for_active_ = 0U;
    return true;
  }

  /**
   * @brief Performs the skip active operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool skip_active() {
    if (!active_) {
      return false;
    }
    skipped_.push_back(std::move(*active_));
    active_.reset();
    decisions_for_active_ = 0U;
    return true;
  }

  /**
   * @brief Performs the install active plan operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void install_active_plan(std::vector<Point2D> plan) {
    if (!active_) {
      throw std::logic_error("cannot install a plan without an active task");
    }
    active_->plan = std::move(plan);
    active_->waypoint_index = 0U;
  }

  /**
   * @brief Performs the advance waypoint operation for this subsystem.
   *
   * Arguments:
   * - @p pose: Supplies pose input to the operation.
   * - @p tolerance: Supplies tolerance input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool advance_waypoint(const Pose2D& pose, Distance tolerance) {
    if (!active_) {
      return false;
    }
    bool advanced = false;
    while (const auto waypoint = active_->waypoint()) {
      if (distance(pose.position, *waypoint).meters() >
          tolerance.meters() + geometry_tolerance_m) {
        break;
      }
      ++active_->waypoint_index;
      advanced = true;
    }
    return advanced;
  }

  /**
   * @brief Performs the pending operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::deque<NavigationTask>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::deque<NavigationTask>& pending() const noexcept {
    return pending_;
  }
  /**
   * @brief Performs the active operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::optional<NavigationTask>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::optional<NavigationTask>& active() const noexcept {
    return active_;
  }
  /**
   * @brief Performs the completed operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<NavigationTask>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<NavigationTask>& completed() const noexcept {
    return completed_;
  }
  /**
   * @brief Performs the skipped operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<NavigationTask>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<NavigationTask>& skipped() const noexcept {
    return skipped_;
  }
  /**
   * @brief Performs the decision limit operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t decision_limit() const noexcept { return decision_limit_; }
  /**
   * @brief Performs the decisions for active operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t decisions_for_active() const noexcept {
    return decisions_for_active_;
  }
  /**
   * @brief Performs the finished operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool finished() const noexcept { return pending_.empty() && !active_; }

 private:
  std::deque<NavigationTask> pending_;
  std::optional<NavigationTask> active_;
  std::vector<NavigationTask> completed_;
  std::vector<NavigationTask> skipped_;
  std::size_t decision_limit_{1U};
  std::size_t decisions_for_active_ = 0U;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_MISSION_HPP
