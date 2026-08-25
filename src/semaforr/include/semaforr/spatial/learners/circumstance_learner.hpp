/**
 * @file circumstance_learner.hpp
 * @brief Circumstance learner responsibilities.
 *
 * @details This file defines circumstance learner behavior for learned spatial
 * representations and their lifecycle. It centers on
 * `CircumstanceLearningConfiguration`, `CircumstanceLearner`,
 * `PendingExperience`. Its package-relative location is
 * `include/semaforr/spatial/learners/circumstance_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP
#define SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates circumstance learning configuration state and
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
struct CircumstanceLearningConfiguration {
  domain::CircumstanceLearningMode mode{
      domain::CircumstanceLearningMode::AdaptedThreshold};
  double setting_resolution_m = 1.0;
  double setting_radius_m = 10.0;
  std::size_t minimum_cluster_size = 50U;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  std::size_t reclustering_threshold = 100U;
  std::size_t minimum_case_evidence = 10U;
  std::size_t minimum_action_evidence = 5U;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
  double partial_success_credit = 0.5;
  bool safety_interruption_is_negative_evidence = true;
  std::string model_version{"circumstance_case_v2"};
  std::string classifier_version{"centroid_softmax_v1"};
  std::string feature_version{"robot_centered_heading_normalized_freespace_v1"};
  std::string persistence_policy{"session_only"};
  std::string model_path;

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
};

/**
 * @brief Encapsulates circumstance learner state and behavior for this
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
class CircumstanceLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the circumstance learner operation for this subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit CircumstanceLearner(
      CircumstanceLearningConfiguration configuration = {});

 private:
  /**
   * @brief Encapsulates pending experience state and behavior for this
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
  struct PendingExperience {
    domain::DecisionId decision_id = 0U;
    domain::ActionId action_id = 0U;
    std::optional<domain::TaskId> task_id;
    std::optional<domain::CircumstanceId> circumstance_id;
    std::size_t circumstance_revision = 0U;
    double assignment_confidence = 0.0;
    domain::NormalizedSetting setting;
    domain::Pose2D starting_pose;
    std::optional<domain::Point2D> target;
    domain::Action action = domain::Action::pause();
    bool terminal = false;
    std::optional<domain::ActionExecutionResult> result;
  };

  /**
   * @brief Performs the on observe operation for this subsystem.
   *
   * Arguments:
   * - @p episode: Supplies episode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void onObserve(const NavigationEpisode& episode) override;
  /**
   * @brief Performs the on rebuild operation for this subsystem.
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
  void onRebuild() override;
  /**
   * @brief Updates clusters for this subsystem.
   *
   * Arguments:
   * - @p setting: Supplies setting input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void updateClusters(const domain::NormalizedSetting& setting);
  /**
   * @brief Performs the recluster operation for this subsystem.
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
  void recluster();
  /**
   * @brief Records decision for this subsystem.
   *
   * Arguments:
   * - @p episode: Supplies episode input to the operation.
   * - @p setting: Supplies setting input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordDecision(const NavigationEpisode& episode,
                      const domain::NormalizedSetting& setting);
  /**
   * @brief Records terminal for this subsystem.
   *
   * Arguments:
   * - @p episode: Supplies episode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordTerminal(const NavigationEpisode& episode);
  /**
   * @brief Performs the classify operation for this subsystem.
   *
   * Arguments:
   * - @p setting: Supplies setting input to the operation.
   *
   * Returns:
   * - `std::optional<domain::CircumstanceMatch>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<domain::CircumstanceMatch> classify(
      const domain::NormalizedSetting& setting) const;
  /**
   * @brief Updates case for this subsystem.
   *
   * Arguments:
   * - @p pending: Supplies pending input to the operation.
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool updateCase(PendingExperience& pending,
                  const domain::ActionExecutionResult& result);

  CircumstanceLearningConfiguration configuration_;
  CircumstanceModel model_;
  std::vector<domain::NormalizedSetting> unclustered_;
  std::vector<PendingExperience> pending_experiences_;
  std::vector<std::pair<domain::ActionId, domain::CaseOutcome>>
      resolved_outcomes_;
};

}  // namespace semaforr::spatial
#endif
