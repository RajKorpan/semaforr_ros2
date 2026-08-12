#ifndef SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP
#define SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

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

  void validate() const;
};

class CircumstanceLearner final : public SpatialLearnerBase {
 public:
  explicit CircumstanceLearner(
      CircumstanceLearningConfiguration configuration = {});

 private:
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

  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  void updateClusters(const domain::NormalizedSetting& setting);
  void recluster();
  void recordDecision(const NavigationEpisode& episode,
                      const domain::NormalizedSetting& setting);
  void recordTerminal(const NavigationEpisode& episode);
  std::optional<domain::CircumstanceMatch> classify(
      const domain::NormalizedSetting& setting) const;
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
