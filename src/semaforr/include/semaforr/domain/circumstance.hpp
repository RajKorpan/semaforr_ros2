#ifndef SEMAFORR_DOMAIN_CIRCUMSTANCE_HPP
#define SEMAFORR_DOMAIN_CIRCUMSTANCE_HPP

#include <compare>
#include <cstddef>
#include <map>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/observation.hpp>
#include <optional>
#include <vector>

namespace semaforr::domain {

struct NormalizedSetting {
  std::size_t side_cells = 0U;
  double resolution_m = 1.0;
  double radius_m = 0.0;
  std::vector<double> freespace;

  bool compatibleWith(const NormalizedSetting& other) const noexcept {
    return side_cells == other.side_cells &&
           resolution_m == other.resolution_m &&
           radius_m == other.radius_m &&
           freespace.size() == other.freespace.size();
  }
};

struct CircumstanceCluster {
  std::size_t id = 0U;
  NormalizedSetting centroid;
  std::size_t evidence = 0U;
  double assignment_confidence = 0.0;
};

struct CircumstanceCaseKey {
  std::size_t circumstance_id = 0U;
  std::size_t distance_bin = 0U;
  std::size_t angle_bin = 0U;

  auto operator<=>(const CircumstanceCaseKey&) const = default;
};

struct ActionPairEvidence {
  Action actual = Action::pause();
  Action hypothetical = Action::pause();
  std::size_t occurrences = 0U;
};

struct CircumstanceCaseEvidence {
  CircumstanceCaseKey key;
  std::vector<ActionPairEvidence> action_pairs;
  std::size_t evidence = 0U;
  double accuracy = 0.0;
  std::map<Action, double> confidence;
};

struct CircumstanceModel {
  std::vector<CircumstanceCluster> clusters;
  std::vector<CircumstanceCaseEvidence> cases;
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

struct SettingNormalizationConfiguration {
  double resolution_m = 1.0;
  double radius_m = 10.0;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
};

struct CircumstanceMatch {
  std::size_t id = 0U;
  double l1_distance = 0.0;
  double confidence = 0.0;
};

NormalizedSetting normalizeSetting(
    const LaserObservation& laser,
    const SettingNormalizationConfiguration& configuration);
double settingL1Distance(const NormalizedSetting& first,
                         const NormalizedSetting& second);
std::optional<CircumstanceMatch> matchCircumstance(
    const CircumstanceModel& model, const NormalizedSetting& setting);
CircumstanceCaseKey circumstanceCaseKey(std::size_t circumstance_id,
                                        const Pose2D& pose, Point2D target,
                                        const CircumstanceModel& model);

}  // namespace semaforr::domain

#endif
