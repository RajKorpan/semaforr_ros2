/**
 * @file highway_explorer.hpp
 * @brief Highway explorer responsibilities.
 *
 * @details This file defines highway explorer behavior for initial or reactive
 * exploration. It centers on `PassageCandidate`, `HleDecision`,
 * `HighwayExplorer`. Its package-relative location is
 * `include/semaforr/exploration/highway_explorer.hpp`.
 */
#ifndef SEMAFORR_EXPLORATION_HIGHWAY_EXPLORER_HPP
#define SEMAFORR_EXPLORATION_HIGHWAY_EXPLORER_HPP

#include <semaforr/exploration/high_level_explorer.hpp>

namespace semaforr::exploration {

// Compatibility view retained for existing callers. New code should use
// HighLevelExplorer and ExplorationResult.
/**
 * @brief Encapsulates passage candidate state and behavior for this
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
struct PassageCandidate {
  domain::Angle heading;
  domain::Distance clearance;
  PassageKind kind = PassageKind::Corridor;
  double confidence = 0.0;
  std::size_t first_beam = 0U;
  std::size_t last_beam = 0U;
};

/**
 * @brief Encapsulates hle decision state and behavior for this subsystem.
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
struct HleDecision {
  domain::Action action = domain::Action::pause();
  HleState state = HleState::Initialize;
  std::vector<PassageCandidate> candidates;
  std::string_view rationale;
};

/**
 * @brief Encapsulates highway explorer state and behavior for this
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
class HighwayExplorer {
 public:
  /**
   * @brief Performs the highway explorer operation for this subsystem.
   *
   * Arguments:
   * - @p minimum_clearance_m: Supplies minimum clearance m input to the
   * operation.
   * - @p heading_tolerance_rad: Supplies heading tolerance rad input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  HighwayExplorer(double minimum_clearance_m = 0.8,
                  double heading_tolerance_rad = 0.2);

  /**
   * @brief Selects package content for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   * - @p action_space: Supplies action space input to the operation.
   *
   * Returns:
   * - `HleDecision` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  HleDecision decide(const domain::RobotObservation& observation,
                     const domain::ActionSpace& action_space);
  /**
   * @brief Performs the finish operation for this subsystem.
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
  void finish() noexcept { explorer_.finish(); }
  /**
   * @brief Performs the state operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `HleState` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  HleState state() const noexcept { return explorer_.state(); }

  /**
   * @brief Performs the detect passages operation for this subsystem.
   *
   * Arguments:
   * - @p laser: Supplies laser input to the operation.
   * - @p minimum_clearance_m: Supplies minimum clearance m input to the
   * operation.
   *
   * Returns:
   * - `std::vector<PassageCandidate>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static std::vector<PassageCandidate> detectPassages(
      const domain::LaserObservation& laser, double minimum_clearance_m);

 private:
  HighLevelExplorationConfiguration configuration_;
  HighLevelExplorer explorer_;
};

}  // namespace semaforr::exploration

#endif
