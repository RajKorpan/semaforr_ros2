#ifndef SEMAFORR_SPATIAL_REGION_LEARNER_HPP
#define SEMAFORR_SPATIAL_REGION_LEARNER_HPP

#include <cstdint>
#include <semaforr/spatial/learner_base.hpp>
#include <unordered_map>

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
  RegionModel model_;
  std::vector<std::size_t> observation_counts_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> spatial_index_;
  std::vector<std::uint64_t> region_buckets_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_REGION_LEARNER_HPP
