#ifndef SEMAFORR_SPATIAL_PASSAGE_SKELETON_LEARNER_HPP
#define SEMAFORR_SPATIAL_PASSAGE_SKELETON_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class PassageSkeletonLearner final : public SpatialLearnerBase {
 public:
  explicit PassageSkeletonLearner(
      double minimum_node_spacing_m = 0.5,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      RegionLearningConfiguration region_configuration = {},
      TrailLearningConfiguration trail_configuration = {});

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_node_spacing_m_;
  SpatialLearningMode mode_;
  RegionLearningConfiguration region_configuration_;
  TrailLearningConfiguration trail_configuration_;
  PassageSkeletonModel model_;
  std::optional<domain::TaskId> last_task_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_PASSAGE_SKELETON_LEARNER_HPP
