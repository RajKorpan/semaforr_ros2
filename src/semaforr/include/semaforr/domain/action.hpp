/**
 * @file action.hpp
 * @brief Action responsibilities.
 *
 * @details This file defines action behavior for ROS-independent domain state
 * and value types. It centers on `ActionType`, `Action`, `UncheckedTag`. Its
 * package-relative location is `include/semaforr/domain/action.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_ACTION_HPP
#define SEMAFORR_DOMAIN_ACTION_HPP

#include <compare>
#include <cstddef>
#include <stdexcept>

namespace semaforr::domain {

/**
 * @brief Enumerates the supported action type values used by this
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
enum class ActionType { Forward, TurnRight, TurnLeft, Pause };

/**
 * @brief Encapsulates action state and behavior for this subsystem.
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
class Action {
 public:
  static constexpr std::size_t maximum_magnitude_index = 299;

  /**
   * @brief Performs the action operation for this subsystem.
   *
   * Arguments:
   * - @p type: Supplies type input to the operation.
   * - @p magnitude_index: Supplies magnitude index input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Action` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static constexpr Action pause() noexcept {
    return Action(ActionType::Pause, 0U, UncheckedTag{});
  }

  /**
   * @brief Performs the type operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `ActionType` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  constexpr ActionType type() const noexcept { return type_; }
  /**
   * @brief Performs the magnitude index operation for this subsystem.
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
  constexpr std::size_t magnitude_index() const noexcept {
    return magnitude_index_;
  }

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Action&) const = default;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::strong_ordering` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::strong_ordering operator<=>(const Action&) const = default;

 private:
  /**
   * @brief Encapsulates unchecked tag state and behavior for this
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
  struct UncheckedTag {};

  /**
   * @brief Performs the action operation for this subsystem.
   *
   * Arguments:
   * - @p type: Supplies type input to the operation.
   * - @p magnitude_index: Supplies magnitude index input to the operation.
   * - @p argument_3: Supplies argument 3 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  constexpr Action(ActionType type, std::size_t magnitude_index,
                   UncheckedTag) noexcept
      : type_(type), magnitude_index_(magnitude_index) {}

  ActionType type_;
  std::size_t magnitude_index_;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_ACTION_HPP
