#ifndef SEMAFORR_DECISION_CONTEXT_HPP
#define SEMAFORR_DECISION_CONTEXT_HPP

#include <semaforr/domain/world_model.hpp>
#include <span>

namespace semaforr::decision {

struct DecisionContext {
  const domain::WorldModel& world;
  const domain::ActionSpace* action_space = nullptr;
  std::span<const domain::Action> viable_actions{};
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_CONTEXT_HPP
