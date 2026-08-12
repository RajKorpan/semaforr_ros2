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

enum class CircumstanceLearningMode {
  DissertationCompatible,
  AdaptedThreshold
};

enum class CircumstanceCreationMethod {
  OfflineSimilarityGraph,
  OnlineReclustering,
  LoadedModel
};

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

std::string_view toString(CircumstanceLearningMode mode) noexcept;
std::string_view toString(CircumstanceCreationMethod method) noexcept;
std::string_view toString(CaseOutcome outcome) noexcept;

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

struct CircumstanceCaseKey {
  CircumstanceId circumstance_id = 0U;
  std::size_t distance_bin = 0U;
  std::size_t angle_bin = 0U;

  auto operator<=>(const CircumstanceCaseKey&) const = default;
};

struct ActionPairEvidence {
  Action actual = Action::pause();
  Action hypothetical = Action::pause();
  std::size_t occurrences = 0U;
};

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

struct CircumstanceCaseEvidence {
  CircumstanceCaseKey key;
  std::vector<ActionPairEvidence> action_pairs;
  std::size_t evidence = 0U;
  double accuracy = 0.0;
  std::map<Action, double> confidence;
  std::vector<ActionCaseEvidence> actions;
  std::size_t revision = 0U;
};

struct CircumstanceMigration {
  CircumstanceId previous_id = 0U;
  CircumstanceId new_id = 0U;
  std::string operation;
  std::size_t evidence_moved = 0U;
  std::size_t model_revision = 0U;
};

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

  double assignmentRate() const noexcept {
    return observations == 0U ? 0.0
                              : static_cast<double>(assignments) /
                                    static_cast<double>(observations);
  }
  double averageAssignmentConfidence() const noexcept {
    return assignments == 0U
               ? 0.0
               : assignment_confidence_sum /
                     static_cast<double>(assignments);
  }
};

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

struct SettingNormalizationConfiguration {
  double resolution_m = 1.0;
  double radius_m = 10.0;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
};

struct CircumstanceMatch {
  CircumstanceId id = 0U;
  double l1_distance = 0.0;
  double confidence = 0.0;
  std::string confidence_semantics{"normalized_centroid_similarity"};
};

NormalizedSetting normalizeSetting(
    const LaserObservation& laser,
    const SettingNormalizationConfiguration& configuration);
double settingL1Distance(const NormalizedSetting& first,
                         const NormalizedSetting& second);
std::optional<CircumstanceMatch> matchCircumstance(
    const CircumstanceModel& model, const NormalizedSetting& setting);
CircumstanceCaseKey circumstanceCaseKey(CircumstanceId circumstance_id,
                                        const Pose2D& pose, Point2D target,
                                        const CircumstanceModel& model);
const ActionCaseEvidence* findActionEvidence(
    const CircumstanceCaseEvidence& evidence, Action action) noexcept;
void saveCircumstanceModel(const CircumstanceModel& model,
                           std::ostream& output);
CircumstanceModel loadCircumstanceModel(
    std::istream& input, std::string_view expected_model_version = {},
    std::string_view expected_feature_version = {},
    std::string_view expected_classifier_version = {});

}  // namespace semaforr::domain

#endif
