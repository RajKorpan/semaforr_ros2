#ifndef SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP
#define SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

struct CircumstanceLearningConfiguration {
  double setting_resolution_m = 1.0;
  double setting_radius_m = 10.0;
  std::size_t minimum_cluster_size = 50U;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  std::size_t reclustering_threshold = 100U;
  std::size_t minimum_case_evidence = 10U;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;

  void validate() const;
};

class CircumstanceLearner final : public SpatialLearnerBase {
 public:
  explicit CircumstanceLearner(
      CircumstanceLearningConfiguration configuration = {});

 private:
  struct PendingExperience {
    domain::NormalizedSetting setting;
    NavigationEpisode episode;
    bool processed = false;
  };

  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  void updateClusters(const domain::NormalizedSetting& setting);
  void recluster();
  void processExperiences();

  CircumstanceLearningConfiguration configuration_;
  CircumstanceModel model_;
  std::vector<domain::NormalizedSetting> unclustered_;
  std::vector<PendingExperience> pending_experiences_;
};

}  // namespace semaforr::spatial
#endif
