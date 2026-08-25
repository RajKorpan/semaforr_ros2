/**
 * @file crowd_field_learner.hpp
 * @brief Crowd field learner responsibilities.
 *
 * @details This file defines crowd field learner behavior for social observation
 * processing and crowd learning. It centers on `CrowdEstimatorStrategy`,
 * `CrowdFieldLearnerConfiguration`, `CrowdLearningUpdate`,
 * `CrowdFieldLearner`, `CusumState`. Its package-relative location is
 * `include/semaforr/social/crowd_field_learner.hpp`.
 */
#ifndef SEMAFORR_SOCIAL_CROWD_FIELD_LEARNER_HPP
#define SEMAFORR_SOCIAL_CROWD_FIELD_LEARNER_HPP

#include <cstdint>
#include <optional>
#include <random>
#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/domain/observation.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::social {

/**
 * @brief Enumerates the supported crowd estimator strategy values used by
 * this subsystem.
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
enum class CrowdEstimatorStrategy {
  CountExposure,
  DiscountedCount,
  Cusum,
  Thompson
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p strategy: Supplies strategy input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CrowdEstimatorStrategy strategy) noexcept;
/**
 * @brief Performs the crowd estimator strategy from string operation for
 * this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `CrowdEstimatorStrategy` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CrowdEstimatorStrategy crowdEstimatorStrategyFromString(std::string_view value);

/**
 * @brief Encapsulates crowd field learner configuration state and behavior
 * for this subsystem.
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
 * @brief Encapsulates crowd learning update state and behavior for this
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
struct CrowdLearningUpdate {
  bool accepted{false};
  bool published{false};
  std::size_t visible_cells{0U};
  std::size_t pedestrian_hits{0U};
  std::size_t encounter_hits{0U};
  std::size_t directional_flow_updates{0U};
  std::size_t formation_count{0U};
  std::uint64_t snapshot_version{0U};
  std::string input_source;
  std::string prediction_source;
  std::string status{"not_observed"};
};

/**
 * @brief Encapsulates crowd field learner state and behavior for this
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
class CrowdFieldLearner {
 public:
  /**
   * @brief Performs the crowd field learner operation for this subsystem.
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
  explicit CrowdFieldLearner(CrowdFieldLearnerConfiguration configuration);

  /**
   * @brief Processes package content for this subsystem.
   *
   * Arguments:
   * - @p robot_pose: Supplies robot pose input to the operation.
   * - @p laser: Supplies laser input to the operation.
   * - @p crowd: Supplies crowd input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool observe(const domain::Pose2D& robot_pose,
               const domain::LaserObservation& laser,
               const domain::CrowdObservation& crowd);

  /**
   * @brief Performs the snapshot operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const domain::CrowdFieldSnapshot&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const domain::CrowdFieldSnapshot& snapshot() const noexcept {
    return snapshot_;
  }
  /**
   * @brief Performs the last update operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const CrowdLearningUpdate&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const CrowdLearningUpdate& lastUpdate() const noexcept {
    return last_update_;
  }

  /**
   * @brief Performs the restore operation for this subsystem.
   *
   * Arguments:
   * - @p snapshot: Supplies snapshot input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void restore(domain::CrowdFieldSnapshot snapshot);
  /**
   * @brief Resets package content for this subsystem.
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
  void reset();

 private:
  /**
   * @brief Encapsulates cusum state state and behavior for this subsystem.
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
  struct CusumState {
    double log_likelihood{0.0};
    double minimum_log_likelihood{0.0};
    double sample_sum{0.0};
    std::size_t sample_count{0U};

    /**
     * @brief Performs the detect operation for this subsystem.
     *
     * Arguments:
     * - @p sample: Supplies sample input to the operation.
     * - @p change: Supplies change input to the operation.
     * - @p threshold: Supplies threshold input to the operation.
     *
     * Returns:
     * - `bool` containing the operation result.
     *
     * Exceptions:
     * - None documented; validation or dependency failures may propagate.
     */
    bool detect(double sample, double change, double threshold);
    /**
     * @brief Resets package content for this subsystem.
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
    void reset() noexcept;
  };

  /**
   * @brief Performs the visible cells operation for this subsystem.
   *
   * Arguments:
   * - @p robot_pose: Supplies robot pose input to the operation.
   * - @p laser: Supplies laser input to the operation.
   *
   * Returns:
   * - `std::vector<bool>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<bool> visibleCells(const domain::Pose2D& robot_pose,
                                 const domain::LaserObservation& laser) const;
  /**
   * @brief Performs the direction bin operation for this subsystem.
   *
   * Arguments:
   * - @p velocity: Supplies velocity input to the operation.
   *
   * Returns:
   * - `std::optional<std::size_t>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<std::size_t> directionBin(
      const domain::Point2D& velocity) const noexcept;
  /**
   * @brief Performs the rebuild operation for this subsystem.
   *
   * Arguments:
   * - @p generated_at: Supplies generated at input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void rebuild(domain::SocialTimestamp generated_at);
  /**
   * @brief Resets cell for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void resetCell(std::size_t index);

  CrowdFieldLearnerConfiguration configuration_;
  std::vector<domain::CrowdFieldCell> evidence_;
  std::vector<CusumState> cusum_increase_;
  std::vector<CusumState> cusum_decrease_;
  std::optional<domain::SocialTimestamp> last_observation_;
  std::mt19937 random_;
  domain::CrowdFieldSnapshot snapshot_;
  std::uint64_t version_{0U};
  CrowdLearningUpdate last_update_;
};

}  // namespace semaforr::social

#endif  // SEMAFORR_SOCIAL_CROWD_FIELD_LEARNER_HPP
