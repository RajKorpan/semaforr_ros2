/**
 * @file context.hpp
 * @brief Context responsibilities.
 *
 * @details This file defines context behavior for tiered decision making and
 * action arbitration. It centers on `ActivePlanObjective`, `DecisionContext`.
 * Its package-relative location is `include/semaforr/decision/context.hpp`.
 */
#ifndef SEMAFORR_DECISION_CONTEXT_HPP
#define SEMAFORR_DECISION_CONTEXT_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/world_model.hpp>
#include <span>
#include <string>
#include <utility>

namespace semaforr::decision {

/**
 * @brief Encapsulates active plan objective state and behavior for this
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
struct ActivePlanObjective {
  domain::Point2D target;
  std::string step_type;
  std::optional<std::uint64_t> plan_id;
  std::optional<std::size_t> step_index;
};

/**
 * @brief Encapsulates decision context state and behavior for this
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
struct DecisionContext {
  const domain::WorldModel& world;
  const domain::ActionSpace* action_space = nullptr;
  std::span<const domain::Action> viable_actions{};
  std::optional<ActivePlanObjective> active_plan_objective;

  /**
   * @brief Performs the decision context operation for this subsystem.
   *
   * Arguments:
   * - @p model: Supplies model input to the operation.
   * - @p actions: Supplies actions input to the operation.
   * - @p viable: Supplies viable input to the operation.
   * - @p objective: Supplies objective input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionContext(const domain::WorldModel& model,
                  const domain::ActionSpace* actions = nullptr,
                  std::span<const domain::Action> viable = {},
                  std::optional<ActivePlanObjective> objective = std::nullopt)
      : world(model),
        action_space(actions),
        viable_actions(viable),
        active_plan_objective(std::move(objective)) {}
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_CONTEXT_HPP
