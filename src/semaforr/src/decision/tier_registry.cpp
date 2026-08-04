#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>

namespace semaforr::decision {
namespace {

std::optional<domain::Point2D> waypoint(const domain::WorldModel& world) {
  if (!world.mission.active()) return std::nullopt;
  return world.mission.active()->waypoint().value_or(
      world.mission.active()->target);
}

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

double nearest(domain::Point2D point,
               const std::vector<domain::Point2D>& values) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto& value : values)
    result = std::min(result, domain::distance(point, value).meters());
  return std::isfinite(result) ? result : 0.0;
}

}  // namespace

std::optional<Decision> VictoryRule::evaluate(
    const DecisionContext& context) const {
  if (!context.world.mission.active()) return std::nullopt;
  const auto target = context.world.mission.active()->target;
  const double distance =
      domain::distance(context.world.robot.pose.position, target).meters();
  if (distance <= tolerance_.meters())
    return Decision{domain::Action::pause(), std::string(name()),
                    "target reached"};
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
        std::string(name()), "turn directly toward visible target"};
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
                  "move directly toward visible unobstructed target"};
}

std::vector<Veto> ForwardRule::evaluate(
    const DecisionContext& context) const {
  if (!waypoint(context.world)) return {};
  synchronizeVisitedGrid(context.world);
  if (visited_plan_positions_.empty()) return {};
  const auto& pose = context.world.robot.pose;
  std::vector<Veto> vetoes;
  std::size_t rotations = 0U;
  const double lookahead_m = action_space_.move_distances_m().back();
  for (std::size_t index = 1U;
       index <= action_space_.rotation_angles_rad().size(); ++index) {
    for (const auto type :
         {domain::ActionType::TurnLeft, domain::ActionType::TurnRight}) {
      const domain::Action action(type, index);
      const auto expected =
          domain::expectedPoseAfterAction(pose, action, action_space_);
      const domain::Point2D projected{
          expected.position.x_m + lookahead_m *
                                      std::cos(expected.heading.radians()),
          expected.position.y_m + lookahead_m *
                                      std::sin(expected.heading.radians())};
      ++rotations;
      if (std::any_of(visited_plan_positions_.begin(),
                      visited_plan_positions_.end(), [&](const auto& point) {
                        return domain::distance(projected, point).meters() <=
                               std::sqrt(2.0) * visited_grid_resolution_m_ +
                                   domain::geometry_tolerance_m;
                      }))
        vetoes.push_back(
            {action, std::string(name()),
             "one-step lookahead returns to the executed plan"});
    }
  }
  if (vetoes.size() == rotations) {
    visited_plan_positions_.clear();
    return {};
  }
  return vetoes;
}

void ForwardRule::synchronizeVisitedGrid(
    const domain::WorldModel& world) const {
  if (!world.mission.active()) {
    task_id_.reset();
    history_cursor_ = 0U;
    visited_plan_positions_.clear();
    return;
  }
  const auto current_task = world.mission.active()->id;
  if (!task_id_ || *task_id_ != current_task) {
    task_id_ = current_task;
    history_cursor_ = world.navigation_history.entries().size();
    visited_plan_positions_.clear();
  }
  const auto& history = world.navigation_history.entries();
  if (!world.mission.active()->plan.empty()) {
    for (; history_cursor_ < history.size(); ++history_cursor_)
      visited_plan_positions_.push_back(history[history_cursor_].pose.position);
  } else {
    history_cursor_ = history.size();
  }
}

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
            {action, std::string(name()), "avoid a recently occupied orientation"});
    }
  }
  return vetoes;
}

PrecedentRule::PrecedentRule(domain::ActionSpace action_space,
                             PrecedentConfiguration configuration)
    : action_space_(std::move(action_space)), configuration_(configuration) {
  if (configuration_.minimum_case_evidence == 0U ||
      !std::isfinite(configuration_.accuracy_threshold) ||
      configuration_.accuracy_threshold < 0.0 ||
      configuration_.accuracy_threshold > 1.0 ||
      !std::isfinite(configuration_.action_confidence_threshold) ||
      configuration_.action_confidence_threshold < 0.0 ||
      configuration_.action_confidence_threshold > 1.0)
    throw std::invalid_argument("invalid Precedent evidence thresholds");
}

std::vector<Veto> PrecedentRule::evaluate(
    const DecisionContext& context) const {
  const auto& world = context.world;
  const auto& model = world.spatial.circumstances;
  if (!world.robot.laser || !world.mission.active() || model.clusters.empty())
    return {};
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
  if (!match) return {};
  const auto cluster = std::find_if(
      model.clusters.begin(), model.clusters.end(),
      [&](const auto& item) { return item.id == match->id; });
  if (cluster == model.clusters.end() ||
      cluster->evidence < model.minimum_cluster_size ||
      match->confidence < model.assignment_confidence_threshold)
    return {};
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
    return {};
  std::map<domain::Action, std::size_t> counts;
  std::size_t maximum = 0U;
  for (const auto& pair : evidence->action_pairs) {
    counts[pair.hypothetical] += pair.occurrences;
    maximum = std::max(maximum, counts[pair.hypothetical]);
  }
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
    const double confidence =
        (1.0 + static_cast<double>(counts[action])) /
        (1.0 + static_cast<double>(maximum));
    if (confidence < confidence_threshold)
      vetoes.push_back(
          {action, std::string(name()),
           "case evidence=" + std::to_string(evidence->evidence) +
               " accuracy=" + std::to_string(evidence->accuracy) +
               " action confidence=" + std::to_string(confidence)});
  }
  return vetoes;
}

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

void TierOneRegistry::registerMandatory(std::string name,
                                        MandatoryFactory factory) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate mandatory rule");
  kinds_.emplace(name, Kind::Mandatory);
  mandatory_.emplace(std::move(name), std::move(factory));
}

void TierOneRegistry::registerVeto(std::string name, VetoFactory factory) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate veto rule");
  kinds_.emplace(name, Kind::Veto);
  veto_.emplace(std::move(name), std::move(factory));
}

void TierOneRegistry::registerOperationalizer(
    std::string name, OperationalizerFactory factory) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate plan operationalizer");
  kinds_.emplace(name, Kind::PlanOperationalizer);
  operationalizers_.emplace(std::move(name), std::move(factory));
}

void TierOneRegistry::registerReactive(std::string name,
                                       ReactiveFactory factory,
                                       bool replanning_trigger) {
  if (name.empty() || !factory || kinds_.contains(name))
    throw std::invalid_argument("invalid or duplicate reactive planner");
  kinds_.emplace(name, replanning_trigger ? Kind::ReplanningTrigger
                                         : Kind::ReactivePlanner);
  reactive_.emplace(std::move(name), std::move(factory));
}

TierOneRegistry::Kind TierOneRegistry::kind(std::string_view name) const {
  const auto found = kinds_.find(std::string(name));
  if (found == kinds_.end())
    throw std::invalid_argument("unknown Tier-1 component '" +
                                std::string(name) + "'");
  return found->second;
}

std::unique_ptr<MandatoryRule> TierOneRegistry::createMandatory(
    std::string_view name) const {
  const auto found = mandatory_.find(std::string(name));
  if (found == mandatory_.end()) throw std::invalid_argument("unknown rule");
  return found->second();
}

std::unique_ptr<PlanOperationalizer>
TierOneRegistry::createOperationalizer(std::string_view name) const {
  const auto found = operationalizers_.find(std::string(name));
  if (found == operationalizers_.end())
    throw std::invalid_argument("unknown plan operationalizer");
  return found->second();
}

std::unique_ptr<planning::ReactivePlanner>
TierOneRegistry::createReactive(std::string_view name) const {
  const auto found = reactive_.find(std::string(name));
  if (found == reactive_.end())
    throw std::invalid_argument("unknown reactive planner");
  return found->second();
}

std::unique_ptr<VetoRule> TierOneRegistry::createVeto(
    std::string_view name) const {
  const auto found = veto_.find(std::string(name));
  if (found == veto_.end()) throw std::invalid_argument("unknown rule");
  return found->second();
}

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
