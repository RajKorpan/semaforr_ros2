#ifndef SEMAFORR_SPATIAL_REGION_LEARNER_HPP
#define SEMAFORR_SPATIAL_REGION_LEARNER_HPP

#include <semaforr/spatial/spatial_learner_base.hpp>

namespace semaforr::spatial {

class RegionLearner final : public SpatialLearnerBase {
 public:
  RegionLearner(double cluster_radius_m = 1.0,
                std::size_t minimum_observations = 3U);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double cluster_radius_m_;
  std::size_t minimum_observations_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_REGION_LEARNER_HPP
