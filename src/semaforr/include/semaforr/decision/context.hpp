#ifndef SEMAFORR_DECISION_CONTEXT_HPP
#define SEMAFORR_DECISION_CONTEXT_HPP

#include <semaforr/domain/world_model.hpp>

namespace semaforr::decision {

struct DecisionContext {
  const domain::WorldModel& world;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_CONTEXT_HPP
