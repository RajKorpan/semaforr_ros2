#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/restored_tiers.hpp>
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
  const double error = domain::Angle::normalize(
      std::atan2(target.y_m - context.world.robot.pose.position.y_m,
                 target.x_m - context.world.robot.pose.position.x_m) -
      context.world.robot.pose.heading.radians());
  const auto& laser = *context.world.robot.laser;
  if (laser.angle_increment.radians() <= 0.0 ||
      error < laser.angle_min.radians())
    return std::nullopt;
  const auto beam = static_cast<std::size_t>(std::llround(
      (error - laser.angle_min.radians()) /
      laser.angle_increment.radians()));
  if (beam >= laser.ranges_m.size() ||
      laser.ranges_m[beam] + domain::geometry_tolerance_m < distance)
    return std::nullopt;
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
  const auto target = waypoint(context.world);
  if (!target) return {};
  const auto& pose = context.world.robot.pose;
  const double current_error = domain::Angle::normalize(
      std::atan2(target->y_m - pose.position.y_m,
                 target->x_m - pose.position.x_m) -
      pose.heading.radians());
  std::vector<Veto> vetoes;
  for (std::size_t index = 1U;
       index <= action_space_.rotation_angles_rad().size(); ++index) {
    for (const auto type :
         {domain::ActionType::TurnLeft, domain::ActionType::TurnRight}) {
      const domain::Action action(type, index);
      const auto expected =
          domain::expectedPoseAfterAction(pose, action, action_space_);
      const double next_error = domain::Angle::normalize(
          std::atan2(target->y_m - expected.position.y_m,
                     target->x_m - expected.position.x_m) -
          expected.heading.radians());
      if (std::abs(current_error) <= 1.5707963267948966 &&
          std::abs(next_error) > 1.5707963267948966 +
                                     heading_tolerance_rad_)
        vetoes.push_back(
            {action, std::string(name()),
             "prevent orientation regression along the executed plan"});
    }
  }
  return vetoes;
}

std::vector<Veto> NotOppositeRule::evaluate(
    const DecisionContext& context) const {
  const auto& history = context.world.navigation_history.entries();
  if (history.empty()) return {};
  const domain::Action previous = history.back().action;
  if (previous.type() == domain::ActionType::TurnLeft)
    return {{domain::Action(domain::ActionType::TurnRight,
                            previous.magnitude_index()),
             std::string(name()), "avoid immediately reversing a turn"}};
  if (previous.type() == domain::ActionType::TurnRight)
    return {{domain::Action(domain::ActionType::TurnLeft,
                            previous.magnitude_index()),
             std::string(name()), "avoid immediately reversing a turn"}};
  return {};
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
  result.explanation = "restored commonsense/spatial preference";
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

void registerRestoredTierFactories(TierOneRegistry& tier_one,
                                   AdvisorRegistry& tier_three,
                                   const domain::ActionSpace& action_space,
                                   double robot_radius_m,
                                   double obstacle_buffer_m) {
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
  tier_one.registerVeto(
      "not_opposite", [] { return std::make_unique<NotOppositeRule>(); });
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
      "precedent", [] { return std::make_unique<PrecedentRule>(); });
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
