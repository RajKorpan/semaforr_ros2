/**
 * @file crowd_model.hpp
 * @brief Crowd model responsibilities.
 *
 * @details This file defines crowd model behavior for ROS-independent domain
 * state and value types. It centers on `CrowdFlowDirection`, `CrowdFieldCell`,
 * `CrowdFieldSample`, `CrowdFieldSnapshot`, `CrowdModelStatus`,
 * `CrowdModel`. Its package-relative location is
 * `include/semaforr/domain/crowd_model.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_CROWD_MODEL_HPP
#define SEMAFORR_DOMAIN_CROWD_MODEL_HPP

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <semaforr/domain/model_revision.hpp>
#include <semaforr/domain/social.hpp>
#include <string>
#include <vector>

namespace semaforr::domain {

/**
 * @brief Enumerates the supported crowd flow direction values used by this
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
enum class CrowdFlowDirection : std::size_t {
  Right = 0U,
  UpRight = 1U,
  Up = 2U,
  UpLeft = 3U,
  Left = 4U,
  DownLeft = 5U,
  Down = 6U,
  DownRight = 7U
};

constexpr std::size_t kCrowdFlowDirectionCount = 8U;

/**
 * @brief Performs the crowd flow direction angle operation for this
 * subsystem.
 *
 * Arguments:
 * - @p direction: Supplies direction input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double crowdFlowDirectionAngle(CrowdFlowDirection direction) noexcept;

/**
 * @brief Encapsulates crowd field cell state and behavior for this
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
struct CrowdFieldCell {
  double density{0.0};
  double learned_encounter_risk{0.0};
  std::array<double, kCrowdFlowDirectionCount> directional_flow{};

  double visibility_exposures{0.0};
  double pedestrian_hits{0.0};
  double risk_encounters{0.0};
  double risk_experiences{0.0};

  SocialTimestamp last_updated{};
  double confidence{0.0};

  /**
   * @brief Reports whether evidence for this subsystem.
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
  bool hasEvidence() const noexcept;
  /**
   * @brief Performs the finite operation for this subsystem.
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
  bool finite() const noexcept;
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
  bool operator==(const CrowdFieldCell&) const = default;
};

/**
 * @brief Encapsulates crowd field sample state and behavior for this
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
struct CrowdFieldSample {
  Point2D center;
  CrowdFieldCell cell;
  bool stale{false};
};

/**
 * @brief Encapsulates crowd field snapshot state and behavior for this
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
struct CrowdFieldSnapshot {
  GridGeometry geometry;
  std::vector<CrowdFieldCell> cells;
  SocialTimestamp generated_at{};
  std::uint64_t version{0U};
  std::string estimator{"count_exposure"};

  /**
   * @brief Validates package content for this subsystem.
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
  void validate() const;
  /**
   * @brief Performs the available operation for this subsystem.
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
  bool available() const noexcept;
  /**
   * @brief Performs the sample operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   * - @p now: Supplies now input to the operation.
   * - @p maximum_age: Supplies maximum age input to the operation.
   *
   * Returns:
   * - `std::optional<CrowdFieldSample>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<CrowdFieldSample> sample(
      Point2D point, SocialTimestamp now = {},
      std::chrono::nanoseconds maximum_age =
          std::chrono::nanoseconds::zero()) const noexcept;

  /**
   * @brief Serializes package content for this subsystem.
   *
   * Arguments:
   * - @p output: Supplies output input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void save(std::ostream& output) const;
  /**
   * @brief Loads package content for this subsystem.
   *
   * Arguments:
   * - @p input: Supplies input input to the operation.
   *
   * Returns:
   * - `CrowdFieldSnapshot` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static CrowdFieldSnapshot load(std::istream& input);
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
  bool operator==(const CrowdFieldSnapshot&) const = default;
};

/**
 * @brief Enumerates the supported crowd model status values used by this
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
enum class CrowdModelStatus {
  Unavailable,
  LiveOnly,
  LearnedOnly,
  LiveAndLearned
};

/**
 * @brief Encapsulates crowd model state and behavior for this subsystem.
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
class CrowdModel {
 public:
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   * - @p history_limit: Supplies history limit input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void update(CrowdObservation observation, std::size_t history_limit = 100U);
  /**
   * @brief Performs the replace current operation for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void replaceCurrent(CrowdObservation observation);
  /**
   * @brief Clears current for this subsystem.
   *
   * Arguments:
   * - @p status: Supplies status input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void clearCurrent(std::string status = "unavailable");

  /**
   * @brief Performs the current operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::optional<CrowdObservation>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::optional<CrowdObservation>& current() const noexcept {
    return observations_.current();
  }

  /**
   * @brief Performs the history operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<CrowdObservation>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<CrowdObservation>& history() const noexcept {
    return observations_.history();
  }

  /**
   * @brief Performs the observations operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const CrowdState&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const CrowdState& observations() const noexcept { return observations_; }
  /**
   * @brief Performs the observations operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `CrowdState&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  CrowdState& observations() noexcept { return observations_; }
  /**
   * @brief Performs the input source operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::string&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::string& inputSource() const noexcept { return input_source_; }
  /**
   * @brief Performs the prediction source operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::string&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::string& predictionSource() const noexcept {
    return prediction_source_;
  }
  /**
   * @brief Performs the input status operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::string&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::string& inputStatus() const noexcept { return input_status_; }
  /**
   * @brief Performs the formation evidence available operation for this
   * subsystem.
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
  bool formationEvidenceAvailable() const noexcept {
    return formation_evidence_available_;
  }
  /**
   * @brief Performs the formation evidence participated operation for this
   * subsystem.
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
  bool formationEvidenceParticipated() const noexcept {
    return formation_evidence_participated_;
  }
  /**
   * @brief Sets formation evidence participated for this subsystem.
   *
   * Arguments:
   * - @p value: Supplies value input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void setFormationEvidenceParticipated(bool value) noexcept {
    formation_evidence_participated_ = value;
  }

  /**
   * @brief Reports whether valid data for this subsystem.
   *
   * Arguments:
   * - @p maximum_age: Supplies maximum age input to the operation.
   * - @p minimum_confidence: Supplies minimum confidence input to the
   * operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool hasValidData(std::chrono::nanoseconds maximum_age,
                    double minimum_confidence = 0.0) const noexcept {
    return observations_.hasValidData(maximum_age, minimum_confidence);
  }

  /**
   * @brief Sets learned for this subsystem.
   *
   * Arguments:
   * - @p snapshot: Supplies snapshot input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void setLearned(CrowdFieldSnapshot snapshot);
  /**
   * @brief Performs the learned operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const CrowdFieldSnapshot&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const CrowdFieldSnapshot& learned() const noexcept { return learned_; }
  /**
   * @brief Performs the learned available operation for this subsystem.
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
  bool learnedAvailable() const noexcept { return learned_.available(); }
  /**
   * @brief Performs the status operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `CrowdModelStatus` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  CrowdModelStatus status() const noexcept;
  /**
   * @brief Performs the revision of operation for this subsystem.
   *
   * Arguments:
   * - @p dependency: Supplies dependency input to the operation.
   *
   * Returns:
   * - `Revision` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Revision revisionOf(ModelDependency dependency) const noexcept {
    const auto found = revisions_.find(dependency);
    return found == revisions_.end() ? 0U : found->second;
  }
  /**
   * @brief Performs the mutation sequence operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Revision` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Revision mutationSequence() const noexcept { return mutation_sequence_; }
  /**
   * @brief Performs the mutation history operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<ModelMutation>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<ModelMutation>& mutationHistory() const noexcept {
    return mutation_history_;
  }

  /**
   * @brief Performs the learned at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   * - @p now: Supplies now input to the operation.
   * - @p maximum_age: Supplies maximum age input to the operation.
   *
   * Returns:
   * - `std::optional<CrowdFieldSample>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<CrowdFieldSample> learnedAt(
      Point2D point, SocialTimestamp now = {},
      std::chrono::nanoseconds maximum_age =
          std::chrono::nanoseconds::zero()) const noexcept;

  /**
   * @brief Performs the density at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double densityAt(Point2D point) const noexcept;
  /**
   * @brief Performs the learned encounter risk at operation for this
   * subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double learnedEncounterRiskAt(Point2D point) const noexcept;
  /**
   * @brief Performs the visibility exposures at operation for this
   * subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double visibilityExposuresAt(Point2D point) const noexcept;
  /**
   * @brief Performs the risk experiences at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double riskExperiencesAt(Point2D point) const noexcept;
  /**
   * @brief Performs the flow observation at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double flowObservationAt(Point2D point) const noexcept;
  /**
   * @brief Performs the flow alignment at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   * - @p travel_direction: Supplies travel direction input to the
   * operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double flowAlignmentAt(Point2D point, Angle travel_direction) const noexcept;
  /**
   * @brief Performs the predictive collision risk at operation for this
   * subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   * - @p gaussian_variance_m2: Supplies gaussian variance m2 input to the
   * operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double predictiveCollisionRiskAt(
      Point2D point, double gaussian_variance_m2 = 0.25) const noexcept;
  /**
   * @brief Performs the navigation risk at operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   * - @p gaussian_variance_m2: Supplies gaussian variance m2 input to the
   * operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double navigationRiskAt(Point2D point,
                          double gaussian_variance_m2 = 0.25) const noexcept;

 private:
  /**
   * @brief Records mutation for this subsystem.
   *
   * Arguments:
   * - @p dependency: Supplies dependency input to the operation.
   * - @p summary: Supplies summary input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordMutation(ModelDependency dependency, std::string summary);
  /**
   * @brief Updates input diagnostics for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void updateInputDiagnostics(const CrowdObservation& observation);

  CrowdState observations_;
  CrowdFieldSnapshot learned_;
  DependencyRevisions revisions_;
  Revision mutation_sequence_{0U};
  std::vector<ModelMutation> mutation_history_;
  std::string input_source_{"none"};
  std::string prediction_source_{"none"};
  std::string input_status_{"unavailable"};
  bool formation_evidence_available_{false};
  bool formation_evidence_participated_{false};
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_CROWD_MODEL_HPP
