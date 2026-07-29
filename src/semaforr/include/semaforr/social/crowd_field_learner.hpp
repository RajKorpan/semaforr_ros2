#ifndef SEMAFORR_SOCIAL_CROWD_FIELD_LEARNER_HPP
#define SEMAFORR_SOCIAL_CROWD_FIELD_LEARNER_HPP

#include <cstdint>
#include <optional>
#include <random>
#include <string_view>
#include <vector>

#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/domain/observation.hpp>

namespace semaforr::social {

enum class CrowdEstimatorStrategy {
  CountExposure,
  DiscountedCount,
  Cusum,
  Thompson
};

std::string_view toString(CrowdEstimatorStrategy strategy) noexcept;
CrowdEstimatorStrategy crowdEstimatorStrategyFromString(
  std::string_view value);

struct CrowdFieldLearnerConfiguration {
  domain::GridGeometry geometry;
  CrowdEstimatorStrategy strategy{CrowdEstimatorStrategy::CountExposure};
  double discount_factor{0.7};
  double minimum_update_period_s{1.0};
  double encounter_radius_m{1.0};
  double minimum_flow_speed_mps{0.05};
  double confidence_exposures{10.0};
  double cusum_increase{4.0};
  double cusum_decrease{-3.0};
  double cusum_threshold{10.0};
  std::uint32_t random_seed{0U};

  void validate() const;
};

class CrowdFieldLearner {
public:
  explicit CrowdFieldLearner(CrowdFieldLearnerConfiguration configuration);

  bool observe(
    const domain::Pose2D& robot_pose,
    const domain::LaserObservation& laser,
    const domain::CrowdObservation& crowd);

  const domain::CrowdFieldSnapshot& snapshot() const noexcept
  {
    return snapshot_;
  }

  void restore(domain::CrowdFieldSnapshot snapshot);
  void reset();

private:
  struct CusumState {
    double log_likelihood{0.0};
    double minimum_log_likelihood{0.0};
    double sample_sum{0.0};
    std::size_t sample_count{0U};

    bool detect(double sample, double change, double threshold);
    void reset() noexcept;
  };

  std::vector<bool> visibleCells(
    const domain::Pose2D& robot_pose,
    const domain::LaserObservation& laser) const;
  std::optional<std::size_t> directionBin(
    const domain::Point2D& velocity) const noexcept;
  void rebuild(domain::SocialTimestamp generated_at);
  void resetCell(std::size_t index);

  CrowdFieldLearnerConfiguration configuration_;
  std::vector<domain::CrowdFieldCell> evidence_;
  std::vector<CusumState> cusum_increase_;
  std::vector<CusumState> cusum_decrease_;
  std::optional<domain::SocialTimestamp> last_observation_;
  std::mt19937 random_;
  domain::CrowdFieldSnapshot snapshot_;
  std::uint64_t version_{0U};
};

}  // namespace semaforr::social

#endif  // SEMAFORR_SOCIAL_CROWD_FIELD_LEARNER_HPP
