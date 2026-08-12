#ifndef SEMAFORR_DECISION_CONTEXT_HPP
#define SEMAFORR_DECISION_CONTEXT_HPP

#include <semaforr/domain/world_model.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace semaforr::decision {

struct ActivePlanObjective {
  domain::Point2D target;
  std::string step_type;
  std::optional<std::uint64_t> plan_id;
  std::optional<std::size_t> step_index;
};

struct DecisionContext {
  const domain::WorldModel& world;
  const domain::ActionSpace* action_space = nullptr;
  std::span<const domain::Action> viable_actions{};
  std::optional<ActivePlanObjective> active_plan_objective;

  DecisionContext(
      const domain::WorldModel& model,
      const domain::ActionSpace* actions = nullptr,
      std::span<const domain::Action> viable = {},
      std::optional<ActivePlanObjective> objective = std::nullopt)
      : world(model), action_space(actions), viable_actions(viable),
        active_plan_objective(std::move(objective)) {}
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_CONTEXT_HPP
