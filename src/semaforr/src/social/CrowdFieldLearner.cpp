#include <semaforr/social/crowd_field_learner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace semaforr::social {
namespace {

constexpr double kPi = 3.14159265358979323846;

double seconds(domain::SocialTimestamp duration) noexcept
{
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

std::string_view toString(CrowdEstimatorStrategy strategy) noexcept
{
  switch (strategy) {
    case CrowdEstimatorStrategy::CountExposure: return "count_exposure";
    case CrowdEstimatorStrategy::DiscountedCount: return "discounted_count";
    case CrowdEstimatorStrategy::Cusum: return "cusum";
    case CrowdEstimatorStrategy::Thompson: return "thompson";
  }
  return "count_exposure";
}

CrowdEstimatorStrategy crowdEstimatorStrategyFromString(
  std::string_view value)
{
  if (value == "count_exposure" || value == "count") {
    return CrowdEstimatorStrategy::CountExposure;
  }
  if (value == "discounted_count" || value == "discount") {
    return CrowdEstimatorStrategy::DiscountedCount;
  }
  if (value == "cusum" || value == "bayes_cusum") {
    return CrowdEstimatorStrategy::Cusum;
  }
  if (value == "thompson" || value == "count_thompson") {
    return CrowdEstimatorStrategy::Thompson;
  }
  throw std::invalid_argument(
    "unknown crowd estimator strategy '" + std::string(value) + "'");
}

void CrowdFieldLearnerConfiguration::validate() const
{
  geometry.validate();
  if (!std::isfinite(discount_factor) || discount_factor <= 0.0 ||
      discount_factor > 1.0) {
    throw std::invalid_argument(
      "crowd discount factor must be within (0, 1]");
  }
  if (!std::isfinite(minimum_update_period_s) ||
      minimum_update_period_s < 0.0 ||
      !std::isfinite(encounter_radius_m) || encounter_radius_m <= 0.0 ||
      !std::isfinite(minimum_flow_speed_mps) ||
      minimum_flow_speed_mps < 0.0 ||
      !std::isfinite(confidence_exposures) ||
      confidence_exposures <= 0.0 ||
      !std::isfinite(cusum_increase) || cusum_increase <= 0.0 ||
      !std::isfinite(cusum_decrease) || cusum_decrease >= 0.0 ||
      !std::isfinite(cusum_threshold) || cusum_threshold <= 0.0) {
    throw std::invalid_argument(
      "crowd learner thresholds must be finite and within documented ranges");
  }
}

bool CrowdFieldLearner::CusumState::detect(
  double sample,
  double change,
  double threshold)
{
  ++sample_count;
  sample_sum += sample;
  const double current_mean = sample_sum / static_cast<double>(sample_count);
  if (current_mean + change < 1.0) {
    return false;
  }
  const double before = std::max(current_mean, 1.0e-5);
  const double after = std::max(current_mean + change, 1.0e-5);
  log_likelihood +=
    (before - after) + sample * std::log(after / before);
  minimum_log_likelihood =
    std::min(minimum_log_likelihood, log_likelihood);
  return log_likelihood - minimum_log_likelihood > threshold;
}

void CrowdFieldLearner::CusumState::reset() noexcept
{
  *this = {};
}

CrowdFieldLearner::CrowdFieldLearner(
  CrowdFieldLearnerConfiguration configuration)
  : configuration_(std::move(configuration)),
    random_(configuration_.random_seed)
{
  configuration_.validate();
  const std::size_t count = configuration_.geometry.cellCount();
  evidence_.resize(count);
  cusum_increase_.resize(count);
  cusum_decrease_.resize(count);
  snapshot_.geometry = configuration_.geometry;
  snapshot_.cells.resize(count);
  snapshot_.estimator = std::string(toString(configuration_.strategy));
}

std::vector<bool> CrowdFieldLearner::visibleCells(
  const domain::Pose2D& robot_pose,
  const domain::LaserObservation& laser) const
{
  std::vector<bool> visible(configuration_.geometry.cellCount(), false);
  if (const auto robot = configuration_.geometry.index(robot_pose.position)) {
    visible[*robot] = true;
  }
  const double step = configuration_.geometry.resolution_m * 0.5;
  for (std::size_t beam = 0U; beam < laser.ranges_m.size(); ++beam) {
    double range = laser.ranges_m[beam];
    if (std::isnan(range) || range < laser.minimum_range.meters()) {
      continue;
    }
    if (std::isinf(range) || range > laser.maximum_range.meters()) {
      range = laser.maximum_range.meters();
    }
    const double angle = robot_pose.heading.radians() +
      laser.angle_min.radians() +
      static_cast<double>(beam) * laser.angle_increment.radians();
    for (double distance = 0.0; distance <= range; distance += step) {
      const domain::Point2D point{
        robot_pose.position.x_m + distance * std::cos(angle),
        robot_pose.position.y_m + distance * std::sin(angle)};
      const auto index = configuration_.geometry.index(point);
      if (!index) {
        break;
      }
      visible[*index] = true;
    }
  }
  return visible;
}

std::optional<std::size_t> CrowdFieldLearner::directionBin(
  const domain::Point2D& velocity) const noexcept
{
  const double speed = std::hypot(velocity.x_m, velocity.y_m);
  if (!std::isfinite(speed) ||
      speed < configuration_.minimum_flow_speed_mps) {
    return std::nullopt;
  }
  double angle = std::atan2(velocity.y_m, velocity.x_m);
  if (angle < 0.0) angle += 2.0 * kPi;
  return static_cast<std::size_t>(
    std::floor((angle + kPi / 8.0) / (kPi / 4.0))) %
    domain::kCrowdFlowDirectionCount;
}

bool CrowdFieldLearner::observe(
  const domain::Pose2D& robot_pose,
  const domain::LaserObservation& laser,
  const domain::CrowdObservation& crowd)
{
  crowd.validate();
  if (crowd.frame_id != configuration_.geometry.frame_id) {
    throw std::invalid_argument(
      "crowd observation frame does not match learned grid frame");
  }
  if (last_observation_) {
    if (crowd.observed_at <= *last_observation_) {
      return false;
    }
    if (seconds(crowd.observed_at - *last_observation_) <
        configuration_.minimum_update_period_s) {
      return false;
    }
  }

  const std::vector<bool> visible = visibleCells(robot_pose, laser);
  const double discount =
    configuration_.strategy == CrowdEstimatorStrategy::DiscountedCount
    ? configuration_.discount_factor : 1.0;
  std::vector<double> sample(evidence_.size(), 0.0);

  for (std::size_t index = 0U; index < evidence_.size(); ++index) {
    if (!visible[index]) continue;
    auto& cell = evidence_[index];
    if (discount < 1.0) {
      cell.visibility_exposures *= discount;
      cell.pedestrian_hits *= discount;
      for (double& flow : cell.directional_flow) flow *= discount;
    }
    cell.visibility_exposures += 1.0;
    cell.last_updated = crowd.observed_at;
  }

  for (const auto& pedestrian : crowd.pedestrians) {
    const auto index = configuration_.geometry.index(pedestrian.position);
    if (!index || !visible[*index]) continue;
    auto& cell = evidence_[*index];
    cell.pedestrian_hits += pedestrian.confidence;
    sample[*index] += pedestrian.confidence;
    if (const auto direction = directionBin(pedestrian.velocity_mps)) {
      cell.directional_flow[*direction] += pedestrian.confidence;
    }
  }

  if (const auto robot_index =
      configuration_.geometry.index(robot_pose.position)) {
    auto& cell = evidence_[*robot_index];
    if (discount < 1.0) {
      cell.risk_experiences *= discount;
      cell.risk_encounters *= discount;
    }
    cell.risk_experiences += 1.0;
    for (const auto& pedestrian : crowd.pedestrians) {
      const double dx =
        pedestrian.position.x_m - robot_pose.position.x_m;
      const double dy =
        pedestrian.position.y_m - robot_pose.position.y_m;
      if (std::hypot(dx, dy) < configuration_.encounter_radius_m) {
        cell.risk_encounters += pedestrian.confidence;
      }
    }
    cell.last_updated = crowd.observed_at;
  }

  if (configuration_.strategy == CrowdEstimatorStrategy::Cusum) {
    for (std::size_t index = 0U; index < evidence_.size(); ++index) {
      if (!visible[index]) continue;
      const bool increased = cusum_increase_[index].detect(
        sample[index], configuration_.cusum_increase,
        configuration_.cusum_threshold);
      const bool decreased = cusum_decrease_[index].detect(
        sample[index], configuration_.cusum_decrease,
        configuration_.cusum_threshold);
      if (increased || decreased) {
        resetCell(index);
      }
    }
  }

  last_observation_ = crowd.observed_at;
  rebuild(crowd.observed_at);
  return true;
}

void CrowdFieldLearner::rebuild(domain::SocialTimestamp generated_at)
{
  snapshot_.geometry = configuration_.geometry;
  snapshot_.generated_at = generated_at;
  snapshot_.version = ++version_;
  snapshot_.estimator = std::string(toString(configuration_.strategy));
  snapshot_.cells = evidence_;
  for (auto& cell : snapshot_.cells) {
    if (cell.visibility_exposures > 0.0) {
      if (configuration_.strategy == CrowdEstimatorStrategy::Thompson) {
        const double shape = std::max(cell.pedestrian_hits, 1.0e-6);
        const double scale = 1.0 / cell.visibility_exposures;
        std::gamma_distribution<double> distribution(shape, scale);
        cell.density = distribution(random_);
      } else {
        cell.density =
          cell.pedestrian_hits / cell.visibility_exposures;
      }
      for (double& flow : cell.directional_flow) {
        flow /= cell.visibility_exposures;
      }
      cell.confidence = cell.visibility_exposures /
        (cell.visibility_exposures + configuration_.confidence_exposures);
    }
    if (cell.risk_experiences > 0.0) {
      cell.learned_encounter_risk =
        cell.risk_encounters / cell.risk_experiences;
    }
  }
  snapshot_.validate();
}

void CrowdFieldLearner::resetCell(std::size_t index)
{
  evidence_.at(index) = {};
  cusum_increase_.at(index).reset();
  cusum_decrease_.at(index).reset();
}

void CrowdFieldLearner::restore(domain::CrowdFieldSnapshot snapshot)
{
  snapshot.validate();
  if (!(snapshot.geometry == configuration_.geometry)) {
    throw std::invalid_argument(
      "restored crowd field geometry does not match learner geometry");
  }
  evidence_ = snapshot.cells;
  for (auto& cell : evidence_) {
    cell.density = 0.0;
    cell.learned_encounter_risk = 0.0;
    if (cell.visibility_exposures > 0.0) {
      for (double& flow : cell.directional_flow) {
        flow *= cell.visibility_exposures;
      }
    }
  }
  snapshot_ = std::move(snapshot);
  version_ = snapshot_.version;
  last_observation_ = snapshot_.generated_at;
}

void CrowdFieldLearner::reset()
{
  std::fill(evidence_.begin(), evidence_.end(), domain::CrowdFieldCell{});
  std::fill(cusum_increase_.begin(), cusum_increase_.end(), CusumState{});
  std::fill(cusum_decrease_.begin(), cusum_decrease_.end(), CusumState{});
  last_observation_.reset();
  version_ = 0U;
  random_.seed(configuration_.random_seed);
  snapshot_ = {};
  snapshot_.geometry = configuration_.geometry;
  snapshot_.cells.resize(configuration_.geometry.cellCount());
  snapshot_.estimator = std::string(toString(configuration_.strategy));
}

}  // namespace semaforr::social
