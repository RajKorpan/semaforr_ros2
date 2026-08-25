/**
 * @file registry.hpp
 * @brief Registry responsibilities.
 *
 * @details This file defines registry behavior for tiered decision making and
 * action arbitration. It centers on `AdvisorRegistry`, `PlannerRegistry`.
 * Its package-relative location is
 * `include/semaforr/decision/registry.hpp`.
 */
#ifndef SEMAFORR_DECISION_REGISTRY_HPP
#define SEMAFORR_DECISION_REGISTRY_HPP

#include <functional>
#include <memory>
#include <semaforr/decision/advisor.hpp>
#include <semaforr/planning/planner.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

namespace semaforr::decision {

/**
 * @brief Encapsulates advisor registry state and behavior for this
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
class AdvisorRegistry {
 public:
  using Factory = std::function<std::unique_ptr<Advisor>()>;

  /**
   * @brief Registers factory for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p factory: Supplies factory input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerFactory(std::string name, Factory factory) {
    if (name.empty() || !factory) {
      throw std::invalid_argument(
          "advisor registration requires a name and factory");
    }
    if (!factories_.emplace(std::move(name), std::move(factory)).second) {
      throw std::invalid_argument("advisor name is already registered");
    }
  }

  /**
   * @brief Creates package content for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `std::unique_ptr<Advisor>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::unique_ptr<Advisor> create(std::string_view name) const {
    const auto found = factories_.find(std::string(name));
    if (found == factories_.end()) {
      throw std::invalid_argument("unknown advisor type '" + std::string(name) +
                                  "'");
    }
    return found->second();
  }

 private:
  std::unordered_map<std::string, Factory> factories_;
};

/**
 * @brief Encapsulates planner registry state and behavior for this
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
class PlannerRegistry {
 public:
  using Factory = std::function<std::unique_ptr<planning::Planner>()>;

  /**
   * @brief Registers factory for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p factory: Supplies factory input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerFactory(std::string name, Factory factory) {
    if (name.empty() || !factory) {
      throw std::invalid_argument(
          "planner registration requires a name and factory");
    }
    if (!factories_.emplace(std::move(name), std::move(factory)).second) {
      throw std::invalid_argument("planner name is already registered");
    }
  }

  /**
   * @brief Creates package content for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `std::unique_ptr<planning::Planner>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::unique_ptr<planning::Planner> create(std::string_view name) const {
    const auto found = factories_.find(std::string(name));
    if (found == factories_.end()) {
      throw std::invalid_argument("unknown planner type '" + std::string(name) +
                                  "'");
    }
    return found->second();
  }

 private:
  std::unordered_map<std::string, Factory> factories_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_REGISTRY_HPP
