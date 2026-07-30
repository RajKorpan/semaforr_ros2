#ifndef SEMAFORR_DOMAIN_ACTION_HPP
#define SEMAFORR_DOMAIN_ACTION_HPP

#include <compare>
#include <cstddef>
#include <stdexcept>

namespace semaforr::domain {

enum class ActionType { Forward, TurnRight, TurnLeft, Pause };

class Action {
 public:
  static constexpr std::size_t maximum_magnitude_index = 299;

  Action(ActionType type, std::size_t magnitude_index)
      : type_(type), magnitude_index_(magnitude_index) {
    if (magnitude_index_ > maximum_magnitude_index) {
      throw std::out_of_range("action magnitude index exceeds 299");
    }
    if (type_ == ActionType::Pause && magnitude_index_ != 0U) {
      throw std::invalid_argument("pause action magnitude index must be zero");
    }
    if (type_ != ActionType::Pause && magnitude_index_ == 0U) {
      throw std::invalid_argument(
          "motion action magnitude index must be greater than zero");
    }
  }

  static constexpr Action pause() noexcept {
    return Action(ActionType::Pause, 0U, UncheckedTag{});
  }

  constexpr ActionType type() const noexcept { return type_; }
  constexpr std::size_t magnitude_index() const noexcept {
    return magnitude_index_;
  }

  bool operator==(const Action&) const = default;
  std::strong_ordering operator<=>(const Action&) const = default;

 private:
  struct UncheckedTag {};

  constexpr Action(ActionType type, std::size_t magnitude_index,
                   UncheckedTag) noexcept
      : type_(type), magnitude_index_(magnitude_index) {}

  ActionType type_;
  std::size_t magnitude_index_;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_ACTION_HPP
