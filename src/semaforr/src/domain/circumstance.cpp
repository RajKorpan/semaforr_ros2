/**
 * @file circumstance.cpp
 * @brief Circumstance responsibilities.
 *
 * @details This file implements circumstance behavior for ROS-independent domain
 * state and value types. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/domain/circumstance.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <iomanip>
#include <istream>
#include <ostream>
#include <semaforr/domain/circumstance.hpp>
#include <stdexcept>

namespace semaforr::domain {
namespace {

/**
 * @brief Performs the assignment confidence operation for this subsystem.
 *
 * Arguments:
 * - @p distance: Supplies distance input to the operation.
 * - @p cells: Supplies cells input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double assignmentConfidence(double distance, std::size_t cells) {
  if (cells == 0U) return 0.0;
  return std::clamp(1.0 - distance / static_cast<double>(cells), 0.0, 1.0);
}

/**
 * @brief Validates package content for this subsystem.
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

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CircumstanceLearningMode mode) noexcept {
  switch (mode) {
    case CircumstanceLearningMode::DissertationCompatible:
      return "dissertation_compatible";
    case CircumstanceLearningMode::AdaptedThreshold:
      return "adapted_threshold";
  }
  return "unknown";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p method: Supplies method input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CircumstanceCreationMethod method) noexcept {
  switch (method) {
    case CircumstanceCreationMethod::OfflineSimilarityGraph:
      return "offline_similarity_graph";
    case CircumstanceCreationMethod::OnlineReclustering:
      return "online_reclustering";
    case CircumstanceCreationMethod::LoadedModel:
      return "loaded_model";
  }
  return "unknown";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p outcome: Supplies outcome input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CaseOutcome outcome) noexcept {
  switch (outcome) {
    case CaseOutcome::Successful: return "successful";
    case CaseOutcome::Partial: return "partial";
    case CaseOutcome::Failed: return "failed";
    case CaseOutcome::Cancelled: return "cancelled";
    case CaseOutcome::TimedOut: return "timed_out";
    case CaseOutcome::SafetyInterrupted: return "safety_interrupted";
    case CaseOutcome::Preempted: return "preempted";
    case CaseOutcome::Unknown: return "unknown";
  }
  return "unknown";
}

/**
 * @brief Performs the normalize setting operation for this subsystem.
 *
 * Arguments:
 * - @p laser: Supplies laser input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `NormalizedSetting` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Sets ting l1 distance for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double settingL1Distance(const NormalizedSetting& first,
                         const NormalizedSetting& second) {
  if (!first.compatibleWith(second))
    return std::numeric_limits<double>::infinity();
  double result = 0.0;
  for (std::size_t index = 0U; index < first.freespace.size(); ++index)
    result += std::abs(first.freespace[index] - second.freespace[index]);
  return result;
}

/**
 * @brief Performs the match circumstance operation for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 * - @p setting: Supplies setting input to the operation.
 *
 * Returns:
 * - `std::optional<CircumstanceMatch>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<CircumstanceMatch> matchCircumstance(
    const CircumstanceModel& model, const NormalizedSetting& setting) {
  const CircumstanceCluster* best = nullptr;
  double best_distance = std::numeric_limits<double>::infinity();
  std::vector<double> compatible_distances;
  for (const auto& cluster : model.clusters) {
    if (cluster.retired || !setting.compatibleWith(cluster.centroid)) continue;
    const double distance = settingL1Distance(setting, cluster.centroid);
    compatible_distances.push_back(distance);
    if (distance < best_distance) {
      best = &cluster;
      best_distance = distance;
    }
  }
  double confidence = assignmentConfidence(best_distance,
                                           setting.freespace.size());
  std::string semantics = "normalized_centroid_similarity";
  if (best && model.learning_mode ==
                  CircumstanceLearningMode::DissertationCompatible) {
    // The compatibility classifier uses a softmax over negative normalized
    // centroid distances. This preserves classifier-probability semantics
    // while keeping the trained prototypes serializable with the model.
    const double scale = std::max(1.0, model.similarity_l1_threshold);
    double denominator = 0.0;
    for (const double distance : compatible_distances)
      denominator += std::exp(-distance / scale);
    confidence = denominator <= 0.0
                     ? 0.0
                     : std::exp(-best_distance / scale) / denominator;
    semantics = "centroid_softmax_probability";
  }
  if (best == nullptr || best_distance >= model.similarity_l1_threshold ||
      confidence < model.assignment_confidence_threshold)
    return std::nullopt;
  return CircumstanceMatch{best->id, best_distance, confidence, semantics};
}

/**
 * @brief Performs the circumstance case key operation for this subsystem.
 *
 * Arguments:
 * - @p circumstance_id: Supplies circumstance id input to the operation.
 * - @p pose: Supplies pose input to the operation.
 * - @p target: Supplies target input to the operation.
 * - @p model: Supplies model input to the operation.
 *
 * Returns:
 * - `CircumstanceCaseKey` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CircumstanceCaseKey circumstanceCaseKey(CircumstanceId circumstance_id,
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

/**
 * @brief Performs the find action evidence operation for this subsystem.
 *
 * Arguments:
 * - @p evidence: Supplies evidence input to the operation.
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `const ActionCaseEvidence*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const ActionCaseEvidence* findActionEvidence(
    const CircumstanceCaseEvidence& evidence, Action action) noexcept {
  const auto found = std::find_if(
      evidence.actions.begin(), evidence.actions.end(),
      [&](const auto& item) { return item.action == action; });
  return found == evidence.actions.end() ? nullptr : &*found;
}

/**
 * @brief Serializes circumstance model for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 * - @p output: Supplies output input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void saveCircumstanceModel(const CircumstanceModel& model,
                           std::ostream& output) {
  output << "SEMAFORR_CIRCUMSTANCE_CASE 2\n"
         << std::quoted(std::string(toString(model.learning_mode))) << ' '
         << std::quoted(model.model_version) << ' '
         << std::quoted(model.classifier_version) << ' '
         << std::quoted(model.feature_version) << ' '
         << std::quoted(model.similarity_metric) << ' '
         << std::quoted(model.reclustering_policy) << '\n'
         << model.next_circumstance_id << ' ' << model.minimum_cluster_size
         << ' ' << model.minimum_case_evidence << ' '
         << model.assignment_confidence_threshold << ' '
         << model.similarity_l1_threshold << ' ' << model.accuracy_threshold
         << ' ' << model.action_confidence_threshold << ' '
         << model.distance_bin_base_m << ' ' << model.angle_bin_count << ' '
         << model.revision << '\n';
  output << model.clusters.size() << '\n';
  for (const auto& cluster : model.clusters) {
    output << cluster.id << ' ' << cluster.evidence << ' '
           << cluster.assignment_confidence << ' '
           << static_cast<int>(cluster.creation_method) << ' '
           << cluster.model_version << ' ' << cluster.last_update_sequence
           << ' ' << cluster.revision << ' ' << cluster.retired << ' '
           << cluster.centroid.side_cells << ' '
           << cluster.centroid.resolution_m << ' '
           << cluster.centroid.radius_m << ' '
           << cluster.centroid.freespace.size();
    for (const double cell : cluster.centroid.freespace) output << ' ' << cell;
    output << '\n';
  }
  output << model.cases.size() << '\n';
  for (const auto& item : model.cases) {
    output << item.key.circumstance_id << ' ' << item.key.distance_bin << ' '
           << item.key.angle_bin << ' ' << item.evidence << ' '
           << item.accuracy << ' ' << item.revision << ' '
           << item.actions.size() << '\n';
    for (const auto& action : item.actions)
      output << static_cast<int>(action.action.type()) << ' '
             << action.action.magnitude_index() << ' ' << action.selected << ' '
             << action.executed << ' ' << action.successful << ' '
             << action.failed << ' ' << action.partial << ' '
             << action.cancellations << ' ' << action.timeouts << ' '
             << action.safety_interruptions << ' ' << action.preemptions << ' '
             << action.unknown << ' ' << action.effective_evidence << ' '
             << action.success_credit << ' ' << action.confidence << ' '
             << action.accuracy << ' '
             << static_cast<int>(action.last_outcome) << ' '
             << action.last_update_sequence << '\n';
  }
  output << model.migrations.size() << '\n';
  for (const auto& migration : model.migrations)
    output << migration.previous_id << ' ' << migration.new_id << ' '
           << std::quoted(migration.operation) << ' '
           << migration.evidence_moved << ' ' << migration.model_revision
           << '\n';
  output << model.metrics.observations << ' ' << model.metrics.assignments
         << ' ' << model.metrics.unmatched << ' '
         << model.metrics.assignment_confidence_sum << ' '
         << model.metrics.reclusterings << ' '
         << model.metrics.precedent_evaluations << ' '
         << model.metrics.precedent_vetoes << ' '
         << model.metrics.tier_three_weighted_decisions << ' '
         << model.metrics.tier_three_changed_winners << '\n';
  if (!output) throw std::runtime_error("failed to save circumstance model");
}

/**
 * @brief Loads circumstance model for this subsystem.
 *
 * Arguments:
 * - @p input: Supplies input input to the operation.
 * - @p expected_model_version: Supplies expected model version input to the
 * operation.
 * - @p expected_feature_version: Supplies expected feature version input to
 * the operation.
 * - @p expected_classifier_version: Supplies expected classifier version
 * input to the operation.
 *
 * Returns:
 * - `CircumstanceModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CircumstanceModel loadCircumstanceModel(
    std::istream& input, std::string_view expected_model_version,
    std::string_view expected_feature_version,
    std::string_view expected_classifier_version) {
  std::string magic;
  int schema = 0;
  input >> magic >> schema;
  if (magic != "SEMAFORR_CIRCUMSTANCE_CASE" || schema != 2)
    throw std::runtime_error("unsupported circumstance model schema");
  CircumstanceModel model;
  std::string mode;
  input >> std::quoted(mode) >> std::quoted(model.model_version) >>
      std::quoted(model.classifier_version) >>
      std::quoted(model.feature_version) >>
      std::quoted(model.similarity_metric) >>
      std::quoted(model.reclustering_policy);
  if (mode == "dissertation_compatible")
    model.learning_mode = CircumstanceLearningMode::DissertationCompatible;
  else if (mode == "adapted_threshold")
    model.learning_mode = CircumstanceLearningMode::AdaptedThreshold;
  else
    throw std::runtime_error("unsupported circumstance learning mode");
  if ((!expected_model_version.empty() &&
       model.model_version != expected_model_version) ||
      (!expected_feature_version.empty() &&
       model.feature_version != expected_feature_version) ||
      (!expected_classifier_version.empty() &&
       model.learning_mode == CircumstanceLearningMode::DissertationCompatible &&
       model.classifier_version != expected_classifier_version))
    throw std::runtime_error(
        "circumstance model feature, model, or classifier version mismatch");
  input >> model.next_circumstance_id >> model.minimum_cluster_size >>
      model.minimum_case_evidence >> model.assignment_confidence_threshold >>
      model.similarity_l1_threshold >> model.accuracy_threshold >>
      model.action_confidence_threshold >> model.distance_bin_base_m >>
      model.angle_bin_count >> model.revision;
  std::size_t count = 0U;
  input >> count;
  model.clusters.resize(count);
  for (auto& cluster : model.clusters) {
    int creation = 0;
    std::size_t freespace_size = 0U;
    input >> cluster.id >> cluster.evidence >>
        cluster.assignment_confidence >> creation >> cluster.model_version >>
        cluster.last_update_sequence >> cluster.revision >> cluster.retired >>
        cluster.centroid.side_cells >> cluster.centroid.resolution_m >>
        cluster.centroid.radius_m >> freespace_size;
    if (creation < 0 || creation > 2)
      throw std::runtime_error("invalid circumstance creation method");
    cluster.creation_method =
        static_cast<CircumstanceCreationMethod>(creation);
    cluster.centroid.freespace.resize(freespace_size);
    for (double& cell : cluster.centroid.freespace) input >> cell;
  }
  input >> count;
  model.cases.resize(count);
  for (auto& item : model.cases) {
    std::size_t action_count = 0U;
    input >> item.key.circumstance_id >> item.key.distance_bin >>
        item.key.angle_bin >> item.evidence >> item.accuracy >> item.revision >>
        action_count;
    item.actions.resize(action_count);
    for (auto& action : item.actions) {
      int type = 0, outcome = 0;
      std::size_t magnitude = 0U;
      input >> type >> magnitude >> action.selected >> action.executed >>
          action.successful >> action.failed >> action.partial >>
          action.cancellations >> action.timeouts >>
          action.safety_interruptions >> action.preemptions >> action.unknown >>
          action.effective_evidence >> action.success_credit >>
          action.confidence >> action.accuracy >> outcome >>
          action.last_update_sequence;
      if (type < static_cast<int>(ActionType::Forward) ||
          type > static_cast<int>(ActionType::Pause) || outcome < 0 ||
          outcome > static_cast<int>(CaseOutcome::Unknown))
        throw std::runtime_error("invalid action evidence in circumstance model");
      action.action = type == static_cast<int>(ActionType::Pause)
                          ? Action::pause()
                          : Action(static_cast<ActionType>(type), magnitude);
      action.last_outcome = static_cast<CaseOutcome>(outcome);
      item.confidence[action.action] = action.confidence;
    }
  }
  input >> count;
  model.migrations.resize(count);
  for (auto& migration : model.migrations)
    input >> migration.previous_id >> migration.new_id >>
        std::quoted(migration.operation) >> migration.evidence_moved >>
        migration.model_revision;
  input >> model.metrics.observations >> model.metrics.assignments >>
      model.metrics.unmatched >> model.metrics.assignment_confidence_sum >>
      model.metrics.reclusterings >> model.metrics.precedent_evaluations >>
      model.metrics.precedent_vetoes >>
      model.metrics.tier_three_weighted_decisions >>
      model.metrics.tier_three_changed_winners;
  if (!input) throw std::runtime_error("malformed circumstance model");
  return model;
}

}  // namespace semaforr::domain
