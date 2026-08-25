/**
 * @file circumstance.hpp
 * @brief Circumstance responsibilities.
 *
 * @details This file defines circumstance behavior for ROS-independent domain state
 * and value types. It centers on `CircumstanceLearningMode`,
 * `CircumstanceCreationMethod`, `CaseOutcome`, `NormalizedSetting`,
 * `CircumstanceCluster`, `CircumstanceCaseKey`, `ActionPairEvidence`,
 * `ActionCaseEvidence`. Its package-relative location is
 * `include/semaforr/domain/circumstance.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_CIRCUMSTANCE_HPP
#define SEMAFORR_DOMAIN_CIRCUMSTANCE_HPP

#include <compare>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/observation.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::domain {

using CircumstanceId = std::uint64_t;

/**
 * @brief Enumerates the supported circumstance learning mode values used by
 * this subsystem.
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
enum class CircumstanceLearningMode {
  DissertationCompatible,
  AdaptedThreshold
};

/**
 * @brief Enumerates the supported circumstance creation method values used
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
enum class CircumstanceCreationMethod {
  OfflineSimilarityGraph,
  OnlineReclustering,
  LoadedModel
};

/**
 * @brief Enumerates the supported case outcome values used by this
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
enum class CaseOutcome {
  Successful,
  Partial,
  Failed,
  Cancelled,
  TimedOut,
  SafetyInterrupted,
  Preempted,
  Unknown
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CircumstanceLearningMode mode) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p method: Supplies method input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CircumstanceCreationMethod method) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p outcome: Supplies outcome input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CaseOutcome outcome) noexcept;

/**
 * @brief Encapsulates normalized setting state and behavior for this
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
struct NormalizedSetting {
  std::size_t side_cells = 0U;
  double resolution_m = 1.0;
  double radius_m = 0.0;
  std::vector<double> freespace;

  /**
   * @brief Performs the compatible with operation for this subsystem.
   *
   * Arguments:
   * - @p other: Supplies other input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool compatibleWith(const NormalizedSetting& other) const noexcept {
    return side_cells == other.side_cells &&
           resolution_m == other.resolution_m &&
           radius_m == other.radius_m &&
           freespace.size() == other.freespace.size();
  }
};

/**
 * @brief Encapsulates circumstance cluster state and behavior for this
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
struct CircumstanceCluster {
  CircumstanceId id = 0U;
  NormalizedSetting centroid;
  std::size_t evidence = 0U;
  double assignment_confidence = 0.0;
  CircumstanceCreationMethod creation_method{
      CircumstanceCreationMethod::OnlineReclustering};
  std::uint64_t model_version = 1U;
  std::uint64_t last_update_sequence = 0U;
  std::size_t revision = 0U;
  bool retired = false;
};

/**
 * @brief Encapsulates circumstance case key state and behavior for this
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
struct CircumstanceCaseKey {
  CircumstanceId circumstance_id = 0U;
  std::size_t distance_bin = 0U;
  std::size_t angle_bin = 0U;

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `auto` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  auto operator<=>(const CircumstanceCaseKey&) const = default;
};

/**
 * @brief Encapsulates action pair evidence state and behavior for this
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
struct ActionPairEvidence {
  Action actual = Action::pause();
  Action hypothetical = Action::pause();
  std::size_t occurrences = 0U;
};

/**
 * @brief Encapsulates action case evidence state and behavior for this
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
struct ActionCaseEvidence {
  Action action = Action::pause();
  std::size_t selected = 0U;
  std::size_t executed = 0U;
  std::size_t successful = 0U;
  std::size_t failed = 0U;
  std::size_t partial = 0U;
  std::size_t cancellations = 0U;
  std::size_t timeouts = 0U;
  std::size_t safety_interruptions = 0U;
  std::size_t preemptions = 0U;
  std::size_t unknown = 0U;
  double effective_evidence = 0.0;
  double success_credit = 0.0;
  double confidence = 0.0;
  double accuracy = 0.0;
  CaseOutcome last_outcome{CaseOutcome::Unknown};
  std::uint64_t last_update_sequence = 0U;
};

/**
 * @brief Encapsulates circumstance case evidence state and behavior for
 * this subsystem.
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
struct CircumstanceCaseEvidence {
  CircumstanceCaseKey key;
  std::vector<ActionPairEvidence> action_pairs;
  std::size_t evidence = 0U;
  double accuracy = 0.0;
  std::map<Action, double> confidence;
  std::vector<ActionCaseEvidence> actions;
  std::size_t revision = 0U;
};

/**
 * @brief Encapsulates circumstance migration state and behavior for this
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
struct CircumstanceMigration {
  CircumstanceId previous_id = 0U;
  CircumstanceId new_id = 0U;
  std::string operation;
  std::size_t evidence_moved = 0U;
  std::size_t model_revision = 0U;
};

/**
 * @brief Encapsulates circumstance metrics state and behavior for this
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
struct CircumstanceMetrics {
  std::size_t observations = 0U;
  std::size_t assignments = 0U;
  std::size_t unmatched = 0U;
  double assignment_confidence_sum = 0.0;
  std::size_t reclusterings = 0U;
  std::size_t precedent_evaluations = 0U;
  std::size_t precedent_vetoes = 0U;
  std::size_t tier_three_weighted_decisions = 0U;
  std::size_t tier_three_changed_winners = 0U;

  /**
   * @brief Performs the assignment rate operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double assignmentRate() const noexcept {
    return observations == 0U ? 0.0
                              : static_cast<double>(assignments) /
                                    static_cast<double>(observations);
  }
  /**
   * @brief Performs the average assignment confidence operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double averageAssignmentConfidence() const noexcept {
    return assignments == 0U
               ? 0.0
               : assignment_confidence_sum /
                     static_cast<double>(assignments);
  }
};

/**
 * @brief Encapsulates circumstance model state and behavior for this
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
struct CircumstanceModel {
  std::vector<CircumstanceCluster> clusters;
  std::vector<CircumstanceCaseEvidence> cases;
  std::vector<CircumstanceMigration> migrations;
  CircumstanceMetrics metrics;
  CircumstanceLearningMode learning_mode{
      CircumstanceLearningMode::AdaptedThreshold};
  std::string model_version{"circumstance_case_v2"};
  std::string classifier_version{"not_applicable"};
  std::string feature_version{"robot_centered_heading_normalized_freespace_v1"};
  std::string similarity_metric{"normalized_l1"};
  std::string reclustering_policy{"threshold_batch"};
  CircumstanceId next_circumstance_id = 1U;
  std::size_t unclustered_settings = 0U;
  std::size_t minimum_cluster_size = 50U;
  std::size_t minimum_case_evidence = 10U;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
  std::size_t revision = 0U;
};

/**
 * @brief Encapsulates setting normalization configuration state and
 * behavior for this subsystem.
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
struct SettingNormalizationConfiguration {
  double resolution_m = 1.0;
  double radius_m = 10.0;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
};

/**
 * @brief Encapsulates circumstance match state and behavior for this
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
struct CircumstanceMatch {
  CircumstanceId id = 0U;
  double l1_distance = 0.0;
  double confidence = 0.0;
  std::string confidence_semantics{"normalized_centroid_similarity"};
};

/**
 * @brief Performs the normalize setting operation for this subsystem.
 *
 * Arguments:
 * - @p laser: Supplies laser input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `NormalizedSetting` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
NormalizedSetting normalizeSetting(
    const LaserObservation& laser,
    const SettingNormalizationConfiguration& configuration);
/**
 * @brief Sets ting l1 distance for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double settingL1Distance(const NormalizedSetting& first,
                         const NormalizedSetting& second);
/**
 * @brief Performs the match circumstance operation for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 * - @p setting: Supplies setting input to the operation.
 *
 * Returns:
 * - `std::optional<CircumstanceMatch>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<CircumstanceMatch> matchCircumstance(
    const CircumstanceModel& model, const NormalizedSetting& setting);
/**
 * @brief Performs the circumstance case key operation for this subsystem.
 *
 * Arguments:
 * - @p circumstance_id: Supplies circumstance id input to the operation.
 * - @p pose: Supplies pose input to the operation.
 * - @p target: Supplies target input to the operation.
 * - @p model: Supplies model input to the operation.
 *
 * Returns:
 * - `CircumstanceCaseKey` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CircumstanceCaseKey circumstanceCaseKey(CircumstanceId circumstance_id,
                                        const Pose2D& pose, Point2D target,
                                        const CircumstanceModel& model);
/**
 * @brief Performs the find action evidence operation for this subsystem.
 *
 * Arguments:
 * - @p evidence: Supplies evidence input to the operation.
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `const ActionCaseEvidence*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const ActionCaseEvidence* findActionEvidence(
    const CircumstanceCaseEvidence& evidence, Action action) noexcept;
/**
 * @brief Serializes circumstance model for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 * - @p output: Supplies output input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void saveCircumstanceModel(const CircumstanceModel& model,
                           std::ostream& output);
/**
 * @brief Loads circumstance model for this subsystem.
 *
 * Arguments:
 * - @p input: Supplies input input to the operation.
 * - @p expected_model_version: Supplies expected model version input to the
 * operation.
 * - @p expected_feature_version: Supplies expected feature version input to
 * the operation.
 * - @p expected_classifier_version: Supplies expected classifier version
 * input to the operation.
 *
 * Returns:
 * - `CircumstanceModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CircumstanceModel loadCircumstanceModel(
    std::istream& input, std::string_view expected_model_version = {},
    std::string_view expected_feature_version = {},
    std::string_view expected_classifier_version = {});

}  // namespace semaforr::domain

#endif
