#ifndef SEMAFORR_DOMAIN_CROWD_MODEL_HPP
#define SEMAFORR_DOMAIN_CROWD_MODEL_HPP

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <semaforr/domain/model_revision.hpp>
#include <semaforr/domain/social.hpp>
#include <string>
#include <vector>

namespace semaforr::domain {

enum class CrowdFlowDirection : std::size_t {
  Right = 0U,
  UpRight = 1U,
  Up = 2U,
  UpLeft = 3U,
  Left = 4U,
  DownLeft = 5U,
  Down = 6U,
  DownRight = 7U
};

constexpr std::size_t kCrowdFlowDirectionCount = 8U;

double crowdFlowDirectionAngle(CrowdFlowDirection direction) noexcept;

struct CrowdFieldCell {
  double density{0.0};
  double learned_encounter_risk{0.0};
  std::array<double, kCrowdFlowDirectionCount> directional_flow{};

  double visibility_exposures{0.0};
  double pedestrian_hits{0.0};
  double risk_encounters{0.0};
  double risk_experiences{0.0};

  SocialTimestamp last_updated{};
  double confidence{0.0};

  bool hasEvidence() const noexcept;
  bool finite() const noexcept;
  bool operator==(const CrowdFieldCell&) const = default;
};

struct CrowdFieldSample {
  Point2D center;
  CrowdFieldCell cell;
  bool stale{false};
};

struct CrowdFieldSnapshot {
  GridGeometry geometry;
  std::vector<CrowdFieldCell> cells;
  SocialTimestamp generated_at{};
  std::uint64_t version{0U};
  std::string estimator{"count_exposure"};

  void validate() const;
  bool available() const noexcept;
  std::optional<CrowdFieldSample> sample(
      Point2D point, SocialTimestamp now = {},
      std::chrono::nanoseconds maximum_age =
          std::chrono::nanoseconds::zero()) const noexcept;

  void save(std::ostream& output) const;
  static CrowdFieldSnapshot load(std::istream& input);
  bool operator==(const CrowdFieldSnapshot&) const = default;
};

enum class CrowdModelStatus {
  Unavailable,
  LiveOnly,
  LearnedOnly,
  LiveAndLearned
};

class CrowdModel {
 public:
  void update(CrowdObservation observation, std::size_t history_limit = 100U) {
    observations_.update(std::move(observation), history_limit);
  }

  void replaceCurrent(CrowdObservation observation) {
    observations_.replaceCurrent(std::move(observation));
  }

  void clearCurrent() noexcept { observations_.clearCurrent(); }

  const std::optional<CrowdObservation>& current() const noexcept {
    return observations_.current();
  }

  const std::vector<CrowdObservation>& history() const noexcept {
    return observations_.history();
  }

  const CrowdState& observations() const noexcept { return observations_; }
  CrowdState& observations() noexcept { return observations_; }

  bool hasValidData(std::chrono::nanoseconds maximum_age,
                    double minimum_confidence = 0.0) const noexcept {
    return observations_.hasValidData(maximum_age, minimum_confidence);
  }

  void setLearned(CrowdFieldSnapshot snapshot);
  const CrowdFieldSnapshot& learned() const noexcept { return learned_; }
  bool learnedAvailable() const noexcept { return learned_.available(); }
  CrowdModelStatus status() const noexcept;
  Revision revisionOf(ModelDependency dependency) const noexcept {
    const auto found = revisions_.find(dependency);
    return found == revisions_.end() ? 0U : found->second;
  }
  Revision mutationSequence() const noexcept { return mutation_sequence_; }
  const std::vector<ModelMutation>& mutationHistory() const noexcept {
    return mutation_history_;
  }

  std::optional<CrowdFieldSample> learnedAt(
      Point2D point, SocialTimestamp now = {},
      std::chrono::nanoseconds maximum_age =
          std::chrono::nanoseconds::zero()) const noexcept;

  double densityAt(Point2D point) const noexcept;
  double learnedEncounterRiskAt(Point2D point) const noexcept;
  double visibilityExposuresAt(Point2D point) const noexcept;
  double riskExperiencesAt(Point2D point) const noexcept;
  double flowObservationAt(Point2D point) const noexcept;
  double flowAlignmentAt(Point2D point, Angle travel_direction) const noexcept;
  double predictiveCollisionRiskAt(
      Point2D point, double gaussian_variance_m2 = 0.25) const noexcept;
  double navigationRiskAt(Point2D point,
                          double gaussian_variance_m2 = 0.25) const noexcept;

 private:
  CrowdState observations_;
  CrowdFieldSnapshot learned_;
  DependencyRevisions revisions_;
  Revision mutation_sequence_{0U};
  std::vector<ModelMutation> mutation_history_;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_CROWD_MODEL_HPP
