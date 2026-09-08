/**
 * @file replay.hpp
 * @brief Replay responsibilities.
 *
 * @details This file defines replay behavior for replay, experimental
 * validation, and performance measurement. It centers on `RandomSeeds`,
 * `RunMetadata`, `ReplayDecision`, `ReplayCycle`, `RunTrace`,
 * `ReplayDifference`, `ReplayReport`, `RunRecorder`. Its package-relative
 * location is `include/semaforr/validation/replay.hpp`.
 */
#ifndef SEMAFORR_VALIDATION_REPLAY_HPP
#define SEMAFORR_VALIDATION_REPLAY_HPP

#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/model_revision.hpp>
#include <semaforr/domain/observation.hpp>
#include <string>
#include <vector>

namespace semaforr::validation {

/**
 * @brief Encapsulates random seeds state and behavior for this subsystem.
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
struct RandomSeeds {
  std::uint32_t tier_three_ties{0U};
  std::uint32_t lle_fallback{0U};
  std::uint32_t planner_ties{0U};
  std::uint32_t clustering{0U};
  std::uint32_t simulation_noise{0U};

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
  bool operator==(const RandomSeeds&) const = default;
};

/**
 * @brief Encapsulates run metadata state and behavior for this subsystem.
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
struct RunMetadata {
  std::uint32_t schema_version{2U};
  std::string configuration_snapshot;
  std::string configuration_fingerprint;
  std::string behavior_mode;
  std::string profile;
  std::string map_checksum;
  std::string source_revision;
  std::string test_suite_revision;
  std::vector<std::string> model_versions;
  std::vector<std::string> component_manifest;
  std::vector<domain::Point2D> task_sequence;
  std::vector<std::string> compatibility_deviations;
  RandomSeeds seeds;
};

/**
 * @brief Encapsulates replay decision state and behavior for this
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
struct ReplayDecision {
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  std::string tier;
  std::string source;
  std::string selected_policy;
  std::optional<std::string> planner;
  domain::Revision plan_revision{0U};
  domain::DependencyRevisions spatial_revisions;
  std::string advisor_scores_digest;
  std::string plan_digest;
  std::string explanation_digest;
  std::string social_input_source{"none"};
  std::string social_prediction_source{"none"};
  std::string social_input_status{"unavailable"};
  bool formation_evidence_available{false};
  bool formation_evidence_participated{false};

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
  bool operator==(const ReplayDecision&) const = default;
};

/**
 * @brief Encapsulates replay cycle state and behavior for this subsystem.
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
struct ReplayCycle {
  std::uint64_t sensor_timestamp_ns{0U};
  domain::RobotObservation observation;
  ReplayDecision expected;
  std::optional<domain::ActionExecutionResult> controller_outcome;
};

/**
 * @brief Encapsulates run trace state and behavior for this subsystem.
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
struct RunTrace {
  RunMetadata metadata;
  std::vector<ReplayCycle> cycles;
};

/**
 * @brief Encapsulates replay difference state and behavior for this
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
struct ReplayDifference {
  std::size_t cycle{0U};
  std::string field;
  std::string expected;
  std::string actual;
};

/**
 * @brief Encapsulates replay report state and behavior for this subsystem.
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
struct ReplayReport {
  bool reproduced{false};
  std::vector<ReplayDifference> differences;
};

/**
 * @brief Performs the replay decision operation for this subsystem.
 *
 * Arguments:
 * - @p result: Supplies result input to the operation.
 * - @p revisions: Supplies revisions input to the operation.
 *
 * Returns:
 * - `ReplayDecision` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ReplayDecision replayDecision(const decision::DecisionResult& result,
                              const domain::DependencyRevisions& revisions);

/**
 * @brief Encapsulates run recorder state and behavior for this subsystem.
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
class RunRecorder {
 public:
  /**
   * @brief Performs the run recorder operation for this subsystem.
   *
   * Arguments:
   * - @p metadata: Supplies metadata input to the operation.
   * - @p trace_path: Supplies trace path input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  RunRecorder(RunMetadata metadata, std::filesystem::path trace_path = {});
  /**
   * @brief Records observation for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   * - @p sensor_timestamp_ns: Supplies sensor timestamp ns input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordObservation(
      const domain::RobotObservation& observation,
      std::optional<std::uint64_t> sensor_timestamp_ns = std::nullopt);
  /**
   * @brief Records decision for this subsystem.
   *
   * Arguments:
   * - @p decision: Supplies decision input to the operation.
   * - @p revisions: Supplies revisions input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordDecision(const decision::DecisionResult& decision,
                      const domain::DependencyRevisions& revisions);
  /**
   * @brief Records controller outcome for this subsystem.
   *
   * Arguments:
   * - @p outcome: Supplies outcome input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordControllerOutcome(const domain::ActionExecutionResult& outcome);
  /**
   * @brief Performs the trace operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const RunTrace&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const RunTrace& trace() const noexcept { return trace_; }
  /**
   * @brief Performs the flush operation for this subsystem.
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
  void flush() const;

  /**
   * @brief Serializes package content for this subsystem.
   *
   * Arguments:
   * - @p trace: Supplies trace input to the operation.
   * - @p path: Supplies path input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static void save(const RunTrace& trace, const std::filesystem::path& path);
  /**
   * @brief Loads package content for this subsystem.
   *
   * Arguments:
   * - @p path: Supplies path input to the operation.
   *
   * Returns:
   * - `RunTrace` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static RunTrace load(const std::filesystem::path& path);

 private:
  RunTrace trace_;
  std::filesystem::path trace_path_;
  std::optional<domain::RobotObservation> pending_observation_;
  std::optional<std::uint64_t> pending_sensor_timestamp_ns_;
};

/**
 * @brief Encapsulates offline replay state and behavior for this subsystem.
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
class OfflineReplay {
 public:
  using DecisionFunction = std::function<ReplayDecision(
      const domain::RobotObservation&,
      const std::optional<domain::ActionExecutionResult>&)>;

  /**
   * @brief Performs the run operation for this subsystem.
   *
   * Arguments:
   * - @p trace: Supplies trace input to the operation.
   * - @p active_metadata: Supplies active metadata input to the operation.
   * - @p decide: Supplies decide input to the operation.
   *
   * Returns:
   * - `ReplayReport` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static ReplayReport run(const RunTrace& trace,
                          const RunMetadata& active_metadata,
                          const DecisionFunction& decide);
};

}  // namespace semaforr::validation

#endif
