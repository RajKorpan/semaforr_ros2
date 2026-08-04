#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <semaforr/domain/circumstance.hpp>
#include <stdexcept>

namespace semaforr::domain {
namespace {

double assignmentConfidence(double distance, std::size_t cells) {
  if (cells == 0U) return 0.0;
  return std::clamp(1.0 - distance / static_cast<double>(cells), 0.0, 1.0);
}

void validate(const SettingNormalizationConfiguration& configuration) {
  if (!std::isfinite(configuration.resolution_m) ||
      !std::isfinite(configuration.radius_m) ||
      !std::isfinite(configuration.assignment_confidence_threshold) ||
      !std::isfinite(configuration.similarity_l1_threshold) ||
      !std::isfinite(configuration.distance_bin_base_m) ||
      configuration.resolution_m <= 0.0 || configuration.radius_m <= 0.0 ||
      configuration.assignment_confidence_threshold < 0.0 ||
      configuration.assignment_confidence_threshold > 1.0 ||
      configuration.similarity_l1_threshold <= 0.0 ||
      configuration.distance_bin_base_m <= 0.0 ||
      configuration.angle_bin_count == 0U)
    throw std::invalid_argument("invalid setting normalization configuration");
}

}  // namespace

NormalizedSetting normalizeSetting(
    const LaserObservation& laser,
    const SettingNormalizationConfiguration& configuration) {
  validate(configuration);
  laser.validate();
  const std::size_t half = static_cast<std::size_t>(
      std::ceil(configuration.radius_m / configuration.resolution_m));
  NormalizedSetting result;
  result.side_cells = 2U * half + 1U;
  result.resolution_m = configuration.resolution_m;
  result.radius_m = configuration.radius_m;
  result.freespace.assign(result.side_cells * result.side_cells, 0.0);
  const long center = static_cast<long>(half);
  const double sample_step = configuration.resolution_m * 0.5;
  double angle = laser.angle_min.radians();
  for (const double measured_range : laser.ranges_m) {
    const double range = std::min(
        configuration.radius_m,
        std::isfinite(measured_range) ? measured_range
                                      : laser.maximum_range.meters());
    for (double distance = 0.0; distance <= range; distance += sample_step) {
      const long column = center + static_cast<long>(std::floor(
                                       distance * std::cos(angle) /
                                       result.resolution_m));
      const long row = center + static_cast<long>(std::floor(
                                    distance * std::sin(angle) /
                                    result.resolution_m));
      if (row >= 0 && column >= 0 &&
          row < static_cast<long>(result.side_cells) &&
          column < static_cast<long>(result.side_cells))
        result.freespace[static_cast<std::size_t>(row) * result.side_cells +
                         static_cast<std::size_t>(column)] = 1.0;
    }
    angle += laser.angle_increment.radians();
  }
  return result;
}

double settingL1Distance(const NormalizedSetting& first,
                         const NormalizedSetting& second) {
  if (!first.compatibleWith(second))
    return std::numeric_limits<double>::infinity();
  double result = 0.0;
  for (std::size_t index = 0U; index < first.freespace.size(); ++index)
    result += std::abs(first.freespace[index] - second.freespace[index]);
  return result;
}

std::optional<CircumstanceMatch> matchCircumstance(
    const CircumstanceModel& model, const NormalizedSetting& setting) {
  const CircumstanceCluster* best = nullptr;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto& cluster : model.clusters) {
    if (!setting.compatibleWith(cluster.centroid)) continue;
    const double distance = settingL1Distance(setting, cluster.centroid);
    if (distance < best_distance) {
      best = &cluster;
      best_distance = distance;
    }
  }
  const double confidence =
      assignmentConfidence(best_distance, setting.freespace.size());
  if (best == nullptr || best_distance >= model.similarity_l1_threshold ||
      confidence < model.assignment_confidence_threshold)
    return std::nullopt;
  return CircumstanceMatch{best->id, best_distance, confidence};
}

CircumstanceCaseKey circumstanceCaseKey(std::size_t circumstance_id,
                                        const Pose2D& pose, Point2D target,
                                        const CircumstanceModel& model) {
  const double target_distance = distance(pose.position, target).meters();
  const std::size_t distance_bin =
      target_distance <= model.distance_bin_base_m
          ? 0U
          : static_cast<std::size_t>(std::ceil(
                std::log2(target_distance / model.distance_bin_base_m)));
  const double relative = Angle::normalize(
      std::atan2(target.y_m - pose.position.y_m,
                 target.x_m - pose.position.x_m) -
      pose.heading.radians());
  const double width =
      2.0 * std::numbers::pi / static_cast<double>(model.angle_bin_count);
  const double shifted =
      std::fmod(relative + width * 0.5 + 2.0 * std::numbers::pi,
                2.0 * std::numbers::pi);
  const std::size_t angle_bin = std::min(
      model.angle_bin_count - 1U, static_cast<std::size_t>(shifted / width));
  return {circumstance_id, distance_bin, angle_bin};
}

}  // namespace semaforr::domain
