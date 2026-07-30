#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/restored_tiers.hpp>
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
  if (context.world.mission.active() &&
      domain::goalReached(context.world.robot.pose,
                          context.world.mission.active()->target, tolerance_))
    return Decision{domain::Action::pause(), std::string(name()),
                    "target reached"};
  return std::nullopt;
}

std::optional<Decision> ForwardRule::evaluate(
    const DecisionContext& context) const {
  const auto target = waypoint(context.world);
  if (!target) return std::nullopt;
  const auto& pose = context.world.robot.pose;
  const double error = domain::Angle::normalize(
      std::atan2(target->y_m - pose.position.y_m,
                 target->x_m - pose.position.x_m) -
      pose.heading.radians());
  if (std::abs(error) <= heading_tolerance_rad_)
    return Decision{domain::Action(domain::ActionType::Forward, 1U),
                    std::string(name()), "waypoint is directly ahead"};
  return std::nullopt;
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
  if (name.empty() || !factory ||
      !mandatory_.emplace(std::move(name), std::move(factory)).second)
    throw std::invalid_argument("invalid or duplicate mandatory rule");
}

void TierOneRegistry::registerVeto(std::string name, VetoFactory factory) {
  if (name.empty() || !factory ||
      !veto_.emplace(std::move(name), std::move(factory)).second)
    throw std::invalid_argument("invalid or duplicate veto rule");
}

std::unique_ptr<MandatoryRule> TierOneRegistry::createMandatory(
    std::string_view name) const {
  const auto found = mandatory_.find(std::string(name));
  if (found == mandatory_.end()) throw std::invalid_argument("unknown rule");
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
                                   const domain::ActionSpace& action_space) {
  tier_one.registerMandatory("Victory",
                             [] { return std::make_unique<VictoryRule>(); });
  tier_one.registerMandatory(
      "Forward", [action_space] {
        return std::make_unique<ForwardRule>(action_space);
      });
  tier_one.registerVeto(
      "NotOpposite", [] { return std::make_unique<NotOppositeRule>(); });
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
