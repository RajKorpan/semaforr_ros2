/**
 * @file exploration_candidate.hpp
 * @brief Exploration candidate responsibilities.
 *
 * @details This file defines exploration candidate behavior for initial or reactive
 * exploration. It centers on `PassageKind`, `PassageCueType`,
 * `HleBundleType`, `HleBundleMeasurement`, `ExplorationCandidateState`,
 * `ExplorationCandidate`, `CandidatePriority`, `CueValidation`. Its
 * package-relative location is
 * `include/semaforr/exploration/exploration_candidate.hpp`.
 */
#ifndef SEMAFORR_EXPLORATION_EXPLORATION_CANDIDATE_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_CANDIDATE_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <string>

namespace semaforr::exploration {

using ExplorationCandidateId = std::uint64_t;

/**
 * @brief Enumerates the supported passage kind values used by this
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
enum class PassageKind { Corridor, Doorway, IntersectionBranch, LargeRoom };
/**
 * @brief Enumerates the supported passage cue type values used by this
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
enum class PassageCueType { Generic, LeftFocus, RightFocus };
/**
 * @brief Enumerates the supported hle bundle type values used by this
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
enum class HleBundleType { LeftFocus, RightFocus, LeftOpen, RightOpen };

/**
 * @brief Encapsulates hle bundle measurement state and behavior for this
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
struct HleBundleMeasurement {
  HleBundleType type{HleBundleType::LeftFocus};
  std::size_t beam_count{0U};
  std::optional<std::size_t> first_beam;
  std::optional<std::size_t> last_beam;
  // Mean valid beam endpoint in the robot-relative Cartesian frame.
  domain::Point2D mean_endpoint;
  /**
   * @brief Performs the zero operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Distance representative_length{` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Distance representative_length{domain::Distance::zero()};
  /**
   * @brief Performs the zero operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Distance endpoint_span{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Distance endpoint_span{domain::Distance::zero()};
  bool valid{false};
};
/**
 * @brief Enumerates the supported exploration candidate state values used
 * by this subsystem.
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
enum class ExplorationCandidateState {
  Queued,
  Selected,
  Pursuing,
  Suspended,
  Completed,
  Abandoned
};

/**
 * @brief Encapsulates exploration candidate state and behavior for this
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
struct ExplorationCandidate {
  ExplorationCandidateId id = 0U;
  domain::Point2D start;
  domain::Point2D endpoint;
  // Global direction. `heading` is retained as the discovery-relative angle
  // for legacy consumers and is never used for pursuit control.
  domain::Angle direction = domain::Angle::zero();
  domain::Angle heading = domain::Angle::zero();
  domain::Distance clearance = domain::Distance::zero();
  domain::Distance length = domain::Distance::zero();
  domain::Distance width = domain::Distance::zero();
  domain::Distance openness_width = domain::Distance::zero();
  domain::Distance current_width = domain::Distance::zero();
  PassageKind kind = PassageKind::Corridor;
  PassageCueType cue_type = PassageCueType::Generic;
  double priority = 0.0;
  double confidence = 0.0;
  std::size_t first_beam = 0U;
  std::size_t last_beam = 0U;
  std::uint64_t discovery_observation_id = 0U;
  domain::Distance current_extension = domain::Distance::zero();
  std::optional<std::uint64_t> passage_id;
  ExplorationCandidateState state = ExplorationCandidateState::Queued;
  std::uint64_t revision = 1U;
};

/**
 * @brief Encapsulates candidate priority state and behavior for this
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
struct CandidatePriority {
  /**
   * @brief Performs the operator operation for this subsystem.
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
  bool operator()(const ExplorationCandidate& left,
                  const ExplorationCandidate& right) const noexcept {
    if (left.priority != right.priority)
      return left.priority < right.priority;
    return left.id > right.id;
  }
};

/**
 * @brief Encapsulates cue validation state and behavior for this subsystem.
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
struct CueValidation {
  bool start_clear = false;
  bool midpoint_clear = false;
  bool endpoint_clear = false;
  std::size_t passage_identities = 0U;
  bool geometrically_reachable = false;
  bool accepted = false;
  std::string reason;
};

}  // namespace semaforr::exploration

#endif
