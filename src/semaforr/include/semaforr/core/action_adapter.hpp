#ifndef SEMAFORR_CORE_ACTION_ADAPTER_HPP
#define SEMAFORR_CORE_ACTION_ADAPTER_HPP

#include <stdexcept>

#include <semaforr/core/FORRAction.hpp>
#include <semaforr/domain/action.hpp>

namespace semaforr::core {

inline domain::Action toDomainAction(const FORRAction& action)
{
  if (action.parameter < 0) {
    throw std::invalid_argument("legacy action magnitude cannot be negative");
  }

  const auto magnitude = static_cast<std::size_t>(action.parameter);
  if (magnitude == 0U) {
    return domain::Action::pause();
  }
  switch (action.type) {
    case FORWARD:
      return domain::Action(domain::ActionType::Forward, magnitude);
    case RIGHT_TURN:
      return domain::Action(domain::ActionType::TurnRight, magnitude);
    case LEFT_TURN:
      return domain::Action(domain::ActionType::TurnLeft, magnitude);
    case PAUSE:
      return domain::Action::pause();
  }
  throw std::invalid_argument("unknown legacy action type");
}

inline FORRAction toLegacyAction(const domain::Action& action)
{
  const auto magnitude = static_cast<int>(action.magnitude_index());
  switch (action.type()) {
    case domain::ActionType::Forward:
      return FORRAction(FORWARD, magnitude);
    case domain::ActionType::TurnRight:
      return FORRAction(RIGHT_TURN, magnitude);
    case domain::ActionType::TurnLeft:
      return FORRAction(LEFT_TURN, magnitude);
    case domain::ActionType::Pause:
      return FORRAction(PAUSE, 0);
  }
  throw std::invalid_argument("unknown domain action type");
}

}  // namespace semaforr::core

#endif  // SEMAFORR_CORE_ACTION_ADAPTER_HPP
