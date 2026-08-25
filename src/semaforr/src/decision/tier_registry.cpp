/**
 * @file tier_registry.cpp
 * @brief Tier registry responsibilities.
 *
 * @details This file implements tier registry behavior for tiered decision making
 * and action arbitration. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/decision/tier_registry.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>

namespace semaforr::decision {
namespace {

/**
 * @brief Performs the waypoint operation for this subsystem.
 *
 * Arguments:
 * - @p world: Supplies world input to the operation.
 *
 * Returns:
 * - `std::optional<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::Point2D> waypoint(const domain::WorldModel& world) {
  if (!world.mission.active()) return std::nullopt;
  return world.mission.active()->waypoint().value_or(
      world.mission.active()->target);
}

/**
 * @brief Performs the sensed operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p laser: Supplies laser input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool sensed(const domain::Pose2D& pose,
            const domain::LaserObservation& laser,
            domain::Point2D point) {
  if (laser.ranges_m.empty() || laser.angle_increment.radians() <= 0.0)
    return false;
  const double distance = domain::distance(pose.position, point).meters();
  const double bearing = domain::Angle::normalize(
      std::atan2(point.y_m - pose.position.y_m,
                 point.x_m - pose.position.x_m) -
      pose.heading.radians());
  const double coordinate =
      (bearing - laser.angle_min.radians()) /
      laser.angle_increment.radians();
  if (coordinate < 0.0 ||
      coordinate > static_cast<double>(laser.ranges_m.size() - 1U))
    return false;
  const auto center = static_cast<std::ptrdiff_t>(std::llround(coordinate));
  std::size_t visible = 0U;
  std::size_t sampled = 0U;
  for (std::ptrdiff_t offset = -2; offset <= 2; ++offset) {
    const auto beam = center + offset;
    if (beam < 0 ||
        beam >= static_cast<std::ptrdiff_t>(laser.ranges_m.size()))
      continue;
    ++sampled;
    if (laser.ranges_m[static_cast<std::size_t>(beam)] +
            domain::geometry_tolerance_m >=
        distance)
      ++visible;
  }
  return sampled >= 3U && visible >= 3U;
}

/**
 * @brief Performs the nearest operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p values: Supplies values input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double nearest(domain::Point2D point,
               const std::vector<domain::Point2D>& values) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto& value : values)
    result = std::min(result, domain::distance(point, value).meters());
  return std::isfinite(result) ? result : 0.0;
}

}  // namespace

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `std::optional<Decision>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<Decision> VictoryRule::evaluate(
    const DecisionContext& context) const {
  if (!context.world.mission.active()) return std::nullopt;
  const auto target = context.world.mission.active()->target;
  const double distance =
      domain::distance(context.world.robot.pose.position, target).meters();
  if (distance <= tolerance_.meters())
    return Decision{domain::Action::pause(), std::string(name()),
                    "victory:target_within_tolerance"};
  if (!context.world.robot.laser ||
      context.world.robot.laser->ranges_m.empty())
    return std::nullopt;
  if (!sensed(context.world.robot.pose, *context.world.robot.laser, target))
    return std::nullopt;
  const double error = domain::Angle::normalize(
      std::atan2(target.y_m - context.world.robot.pose.position.y_m,
                 target.x_m - context.world.robot.pose.position.x_m) -
      context.world.robot.pose.heading.radians());
  if (std::abs(error) > 0.15) {
    const auto& turns = action_space_.rotation_angles_rad();
    if (turns.empty()) return std::nullopt;
    const auto found =
        std::lower_bound(turns.begin(), turns.end(), std::abs(error));
    const auto magnitude =
        found == turns.end()
            ? turns.size()
            : static_cast<std::size_t>(found - turns.begin()) + 1U;
    return Decision{
        domain::Action(error < 0.0 ? domain::ActionType::TurnRight
                                   : domain::ActionType::TurnLeft,
                       magnitude),
        std::string(name()), "victory:turn_toward_visible_target"};
  }
  const auto& moves = action_space_.move_distances_m();
  if (moves.empty()) return std::nullopt;
  const auto found = std::upper_bound(moves.begin(), moves.end(), distance);
  const auto magnitude =
      found == moves.begin()
          ? 1U
          : static_cast<std::size_t>(found - moves.begin());
  return Decision{domain::Action(domain::ActionType::Forward, magnitude),
                  std::string(name()),
                  "victory:move_toward_visible_target"};
}

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `std::vector<Veto>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<Veto> ForwardRule::evaluate(
    const DecisionContext& context) const {
  if (!waypoint(context.world)) return {};
  synchronizeVisitedGrid(context.world);
  if (visited_cells_.empty()) return {};
  const auto& pose = context.world.robot.pose;
  std::vector<Veto> vetoes;
  std::size_t rotations = 0U;
  const double lookahead_m = action_space_.move_distances_m().back();
  std::vector<domain::Action> rotations_to_consider;
  if (!context.viable_actions.empty()) {
    for (const auto action : context.viable_actions)
      if (action.type() == domain::ActionType::TurnLeft ||
          action.type() == domain::ActionType::TurnRight)
        rotations_to_consider.push_back(action);
  } else {
    for (std::size_t index = 1U;
         index <= action_space_.rotation_angles_rad().size(); ++index)
      for (const auto type :
           {domain::ActionType::TurnLeft, domain::ActionType::TurnRight})
        rotations_to_consider.emplace_back(type, index);
  }
  for (const auto action : rotations_to_consider) {
      const auto expected =
          domain::expectedPoseAfterAction(pose, action, action_space_);
      const domain::Point2D projected{
          expected.position.x_m + lookahead_m *
                                      std::cos(expected.heading.radians()),
          expected.position.y_m + lookahead_m *
                                      std::sin(expected.heading.radians())};
      ++rotations;
      if (visited_cells_.contains(visitedCell(projected)))
        vetoes.push_back(
            {action, std::string(name()),
             "forward:projected_footprint_already_visited",
             RejectionKind::Cognitive,
             VetoCategory::ReturnsToVisitedSpace});
  }
  if (rotations > 0U && vetoes.size() == rotations) {
    visited_cells_.clear();
    return {};
  }
  return vetoes;
}

/**
 * @brief Performs the synchronize visited grid operation for this
 * subsystem.
 *
 * Arguments:
 * - @p world: Supplies world input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void ForwardRule::synchronizeVisitedGrid(
    const domain::WorldModel& world) const {
  if (!world.mission.active()) {
    task_id_.reset();
    decision_cursor_ = 0U;
    visited_cells_.clear();
    return;
  }
  const auto current_task = world.mission.active()->id;
  if (!task_id_ || *task_id_ != current_task) {
    task_id_ = current_task;
    decision_cursor_ = 0U;
    visited_cells_.clear();
  }
  const auto& decisions = world.decision_history.entries();
  for (; decision_cursor_ < decisions.size(); ++decision_cursor_) {
    const auto& entry = decisions[decision_cursor_];
    if (entry.task_id != current_task ||
        entry.provenance.rfind("mandatory_rule:Enforcer", 0U) != 0U)
      continue;
    const auto center = visitedCell(entry.expected_start.position);
    for (std::int64_t row = -1; row <= 1; ++row)
      for (std::int64_t column = -1; column <= 1; ++column)
        visited_cells_.insert(
            {center.first + column, center.second + row});
  }
}

/**
 * @brief Performs the visited cell operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `ForwardRule::VisitedCell` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ForwardRule::VisitedCell ForwardRule::visitedCell(
    domain::Point2D point) const noexcept {
  return {static_cast<std::int64_t>(
              std::floor(point.x_m)),
          static_cast<std::int64_t>(
              std::floor(point.y_m))};
}

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `std::vector<Veto>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<Veto> NotOppositeRule::evaluate(
    const DecisionContext& context) const {
  const auto& history = context.world.navigation_history.entries();
  if (history.empty()) return {};
  std::vector<double> recent_orientations;
  const auto first = history.size() > 2U ? history.size() - 2U : 0U;
  for (std::size_t index = first; index < history.size(); ++index)
    recent_orientations.push_back(history[index].pose.heading.radians());
  std::vector<Veto> vetoes;
  for (std::size_t magnitude = 1U;
       magnitude <= action_space_.rotation_angles_rad().size(); ++magnitude) {
    for (const auto type :
         {domain::ActionType::TurnLeft, domain::ActionType::TurnRight}) {
      const domain::Action action(type, magnitude);
      const auto predicted = domain::expectedPoseAfterAction(
          context.world.robot.pose, action, action_space_);
      if (std::any_of(recent_orientations.begin(), recent_orientations.end(),
                      [&](double orientation) {
                        return std::abs(domain::Angle::normalize(
                                   predicted.heading.radians() - orientation)) <=
                               orientation_tolerance_rad_;
                      }))
        vetoes.push_back(
            {action, std::string(name()),
             "not_opposite:predicted_heading_recently_executed",
             RejectionKind::Cognitive,
             VetoCategory::OpposesRecentOrientation});
    }
  }
  return vetoes;
}

/**
 * @brief Performs the precedent rule operation for this subsystem.
 *
 * Arguments:
 * - @p action_space: Supplies action space input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PrecedentRule::PrecedentRule(domain::ActionSpace action_space,
                             PrecedentConfiguration configuration)
    : action_space_(std::move(action_space)), configuration_(configuration) {
  if (configuration_.minimum_case_evidence == 0U ||
      configuration_.minimum_action_evidence == 0U ||
      !std::isfinite(configuration_.minimum_assignment_confidence) ||
      configuration_.minimum_assignment_confidence < 0.0 ||
      configuration_.minimum_assignment_confidence > 1.0 ||
      !std::isfinite(configuration_.accuracy_threshold) ||
      configuration_.accuracy_threshold < 0.0 ||
      configuration_.accuracy_threshold > 1.0 ||
      !std::isfinite(configuration_.action_confidence_threshold) ||
      configuration_.action_confidence_threshold < 0.0 ||
      configuration_.action_confidence_threshold > 1.0)
    throw std::invalid_argument("invalid Precedent evidence thresholds");
}

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `std::vector<Veto>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<Veto> PrecedentRule::evaluate(
    const DecisionContext& context) const {
  last_reason_.clear();
  const auto abstain = [&](std::string reason) {
    last_reason_ = "precedent:abstained:" + std::move(reason);
    return std::vector<Veto>{};
  };
  const auto& world = context.world;
  const auto& model = world.spatial.circumstances;
  if (!world.robot.laser || !world.mission.active() || model.clusters.empty())
    return abstain("circumstance_input_unavailable");
  domain::SettingNormalizationConfiguration setting_configuration;
  setting_configuration.resolution_m =
      model.clusters.front().centroid.resolution_m;
  setting_configuration.radius_m =
      model.clusters.front().centroid.radius_m;
  setting_configuration.assignment_confidence_threshold =
      model.assignment_confidence_threshold;
  setting_configuration.similarity_l1_threshold =
      model.similarity_l1_threshold;
  setting_configuration.distance_bin_base_m = model.distance_bin_base_m;
  setting_configuration.angle_bin_count = model.angle_bin_count;
  const auto setting =
      domain::normalizeSetting(*world.robot.laser, setting_configuration);
  const auto match = domain::matchCircumstance(model, setting);
  if (!match) return abstain("no_confident_circumstance_match");
  const auto cluster = std::find_if(
      model.clusters.begin(), model.clusters.end(),
      [&](const auto& item) { return item.id == match->id; });
  if (cluster == model.clusters.end() ||
      cluster->evidence < model.minimum_cluster_size ||
      match->confidence < std::max(model.assignment_confidence_threshold,
                                   configuration_.minimum_assignment_confidence))
    return abstain("insufficient_circumstance_evidence_or_assignment_confidence");
  const auto key = domain::circumstanceCaseKey(
      match->id, world.robot.pose, world.mission.active()->target, model);
  const auto evidence =
      std::find_if(model.cases.begin(), model.cases.end(),
                   [&](const auto& item) { return item.key == key; });
  const std::size_t required_evidence =
      std::max(configuration_.minimum_case_evidence,
               model.minimum_case_evidence);
  const double required_accuracy =
      std::max(configuration_.accuracy_threshold, model.accuracy_threshold);
  const double confidence_threshold = std::max(
      configuration_.action_confidence_threshold,
      model.action_confidence_threshold);
  if (evidence == model.cases.end() ||
      evidence->evidence < required_evidence ||
      evidence->accuracy < required_accuracy)
    return abstain("insufficient_case_evidence_or_accuracy");
  std::vector<domain::Action> actions{domain::Action::pause()};
  for (std::size_t index = 1U;
       index <= action_space_.move_distances_m().size(); ++index)
    actions.emplace_back(domain::ActionType::Forward, index);
  for (std::size_t index = 1U;
       index <= action_space_.rotation_angles_rad().size(); ++index) {
    actions.emplace_back(domain::ActionType::TurnRight, index);
    actions.emplace_back(domain::ActionType::TurnLeft, index);
  }
  std::vector<Veto> vetoes;
  for (const auto& action : actions) {
    const auto* action_evidence = domain::findActionEvidence(*evidence, action);
    if (!action_evidence ||
        action_evidence->effective_evidence <
            static_cast<double>(configuration_.minimum_action_evidence))
      continue;
    const double confidence = action_evidence->confidence;
    if (confidence < confidence_threshold)
      vetoes.push_back(
          {action, std::string(name()),
           "precedent:previously_ineffective_in_similar_circumstance;" +
               std::string("circumstance_id=") + std::to_string(match->id) +
               " assignment_confidence=" +
               std::to_string(match->confidence) + " evidence=" +
               std::to_string(evidence->evidence) +
               " action_evidence=" +
               std::to_string(action_evidence->effective_evidence) +
               " accuracy=" + std::to_string(evidence->accuracy) +
               " action_confidence=" + std::to_string(confidence) +
               " minimum_case_evidence=" +
               std::to_string(required_evidence) +
               " minimum_action_evidence=" +
               std::to_string(configuration_.minimum_action_evidence) +
               " confidence_threshold=" +
               std::to_string(confidence_threshold),
           RejectionKind::Cognitive,
           VetoCategory::CaseBasedPrecedent});
  }
  last_reason_ = vetoes.empty()
                     ? "precedent:abstained:no_action_had_sufficient_reliable_negative_evidence"
                     : "precedent:learned_cognitive_vetoes_applied";
  return vetoes;
}

/**
 * @brief Performs the spatial advisor operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p objective: Supplies objective input to the operation.
 * - @p action_space: Supplies action space input to the operation.
 * - @p weight: Supplies weight input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SpatialAdvisor::SpatialAdvisor(std::string name,
                               SpatialAdvisorObjective objective,
                               domain::ActionSpace action_space, double weight)
    : name_(std::move(name)),
      objective_(objective),
      action_space_(std::move(action_space)),
      weight_(weight) {
  if (name_.empty() || !std::isfinite(weight_))
    throw std::invalid_argument("invalid spatial advisor configuration");
}

/**
 * @brief Performs the dependencies operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `std::vector<std::string_view>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string_view> SpatialAdvisor::dependencies() const {
  switch (objective_) {
    case SpatialAdvisorObjective::AvoidRevisit:
      return {"navigation_history"};
    case SpatialAdvisorObjective::PreferRegions:
      return {"regions"};
    case SpatialAdvisorObjective::PreferHighways:
      return {"highways"};
    case SpatialAdvisorObjective::PreferDoors:
      return {"doors"};
    case SpatialAdvisorObjective::FollowTrails:
      return {"trails"};
  }
  return {};
}

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 * - @p candidates: Supplies candidates input to the operation.
 *
 * Returns:
 * - `AdvisorEvaluation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
AdvisorEvaluation SpatialAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation result;
  result.weight = weight_;
  result.explanation = "commonsense/spatial preference";
  for (const auto action : candidates) {
    const auto expected =
        domain::expectedPoseAfterAction(context.world.robot.pose, action,
                                        action_space_);
    double score = 0.0;
    switch (objective_) {
      case SpatialAdvisorObjective::AvoidRevisit: {
        std::vector<domain::Point2D> history;
        for (const auto& entry : context.world.navigation_history.entries())
          history.push_back(entry.pose.position);
        score = nearest(expected.position, history);
        break;
      }
      case SpatialAdvisorObjective::PreferRegions:
        for (const auto& region : context.world.spatial.learned_regions)
          score = std::max(
              score, region.radius.meters() -
                         domain::distance(expected.position, region.center)
                             .meters());
        break;
      case SpatialAdvisorObjective::PreferHighways:
        score = -nearest(expected.position,
                         context.world.spatial.highways.nodes);
        break;
      case SpatialAdvisorObjective::PreferDoors: {
        std::vector<domain::Point2D> centers;
        for (const auto& door : context.world.spatial.doorways)
          centers.push_back({(door.start.x_m + door.end.x_m) * 0.5,
                             (door.start.y_m + door.end.y_m) * 0.5});
        score = -nearest(expected.position, centers);
        break;
      }
      case SpatialAdvisorObjective::FollowTrails: {
        std::vector<domain::Point2D> points;
        for (const auto& trail : context.world.spatial.trails)
          points.insert(points.end(), trail.begin(), trail.end());
        score = -nearest(expected.position, points);
        break;
      }
    }
    result.scores.push_back({action, score});
  }
  result.participated = !result.scores.empty();
  return result;
}

/**
 * @brief Registers mandatory for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p factory: Supplies factory input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void TierOneRegistry::registerMandatory(std::string name,
                                        MandatoryFactory factory) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate mandatory rule");
  kinds_.emplace(name, Kind::Mandatory);
  mandatory_.emplace(std::move(name), std::move(factory));
}

/**
 * @brief Registers veto for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p factory: Supplies factory input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void TierOneRegistry::registerVeto(std::string name, VetoFactory factory) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate veto rule");
  kinds_.emplace(name, Kind::Veto);
  veto_.emplace(std::move(name), std::move(factory));
}

/**
 * @brief Registers operationalizer for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p factory: Supplies factory input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void TierOneRegistry::registerOperationalizer(
    std::string name, OperationalizerFactory factory) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate plan operationalizer");
  kinds_.emplace(name, Kind::PlanOperationalizer);
  operationalizers_.emplace(std::move(name), std::move(factory));
}

/**
 * @brief Registers reactive for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p factory: Supplies factory input to the operation.
 * - @p replanning_trigger: Supplies replanning trigger input to the
 * operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void TierOneRegistry::registerReactive(std::string name,
                                       ReactiveFactory factory,
                                       bool replanning_trigger) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate reactive planner");
  kinds_.emplace(name, replanning_trigger ? Kind::ReplanningTrigger
                                         : Kind::ReactivePlanner);
  reactive_.emplace(std::move(name), std::move(factory));
}

/**
 * @brief Performs the kind operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `TierOneRegistry::Kind` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
TierOneRegistry::Kind TierOneRegistry::kind(std::string_view name) const {
  const auto found = kinds_.find(std::string(name));
  if (found == kinds_.end())
    throw std::invalid_argument("unknown Tier-1 component '" +
                                std::string(name) + "'");
  return found->second;
}

/**
 * @brief Creates mandatory for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `std::unique_ptr<MandatoryRule>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::unique_ptr<MandatoryRule> TierOneRegistry::createMandatory(
    std::string_view name) const {
  const auto found = mandatory_.find(std::string(name));
  if (found == mandatory_.end()) throw std::invalid_argument("unknown rule");
  return found->second();
}

std::unique_ptr<PlanOperationalizer>
/**
 * @brief Creates operationalizer for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
TierOneRegistry::createOperationalizer(std::string_view name) const {
  const auto found = operationalizers_.find(std::string(name));
  if (found == operationalizers_.end())
    throw std::invalid_argument("unknown plan operationalizer");
  return found->second();
}

std::unique_ptr<planning::ReactivePlanner>
/**
 * @brief Creates reactive for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
TierOneRegistry::createReactive(std::string_view name) const {
  const auto found = reactive_.find(std::string(name));
  if (found == reactive_.end())
    throw std::invalid_argument("unknown reactive planner");
  return found->second();
}

/**
 * @brief Creates veto for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `std::unique_ptr<VetoRule>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::unique_ptr<VetoRule> TierOneRegistry::createVeto(
    std::string_view name) const {
  const auto found = veto_.find(std::string(name));
  if (found == veto_.end()) throw std::invalid_argument("unknown rule");
  return found->second();
}

/**
 * @brief Registers tier factories for this subsystem.
 *
 * Arguments:
 * - @p tier_one: Supplies tier one input to the operation.
 * - @p tier_three: Supplies tier three input to the operation.
 * - @p action_space: Supplies action space input to the operation.
 * - @p robot_radius_m: Supplies robot radius m input to the operation.
 * - @p obstacle_buffer_m: Supplies obstacle buffer m input to the
 * operation.
 * - @p precedent: Supplies precedent input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void registerTierFactories(TierOneRegistry& tier_one,
                           AdvisorRegistry& tier_three,
                           const domain::ActionSpace& action_space,
                           double robot_radius_m, double obstacle_buffer_m,
                           PrecedentConfiguration precedent) {
  tier_one.registerMandatory(
      "victory", [action_space] {
        return std::make_unique<VictoryRule>(domain::Distance(0.5),
                                             action_space);
      });
  tier_one.registerVeto(
      "avoid_obstacles",
      [action_space, robot_radius_m, obstacle_buffer_m] {
        return std::make_unique<ObstacleVetoRule>(
            action_space.move_distances_m(), robot_radius_m,
            obstacle_buffer_m);
      });
  tier_one.registerVeto("not_opposite", [action_space] {
    return std::make_unique<NotOppositeRule>(action_space);
  });
  tier_one.registerOperationalizer(
      "enforcer", [] { return std::make_unique<Enforcer>(); });
  tier_one.registerReactive("thru",
                            [] { return std::make_unique<planning::Thru>(); });
  tier_one.registerReactive(
      "behind", [] { return std::make_unique<planning::Behind>(); });
  tier_one.registerReactive("out",
                            [] { return std::make_unique<planning::Out>(); });
  tier_one.registerReactive(
      "low_level_exploration",
      [] { return std::make_unique<planning::LowLevelExplorer>(); }, true);
  tier_one.registerVeto(
      "forward", [action_space] {
        return std::make_unique<ForwardRule>(action_space);
      });
  tier_one.registerVeto(
      "precedent", [action_space, precedent] {
        return std::make_unique<PrecedentRule>(action_space, precedent);
      });
  const auto add = [&](std::string name, SpatialAdvisorObjective objective) {
    tier_three.registerFactory(
        name, [name, objective, action_space] {
          return std::make_unique<SpatialAdvisor>(
              name, objective, action_space);
        });
  };
  add("avoid_revisit", SpatialAdvisorObjective::AvoidRevisit);
  add("prefer_regions", SpatialAdvisorObjective::PreferRegions);
  add("prefer_highways", SpatialAdvisorObjective::PreferHighways);
  add("prefer_doors", SpatialAdvisorObjective::PreferDoors);
  add("follow_trails", SpatialAdvisorObjective::FollowTrails);
}

}  // namespace semaforr::decision
