#include <semaforr/decision/advisor_catalog_registry.hpp>
#include <semaforr/decision/dissertation_advisor.hpp>
#include <semaforr/decision/learned_crowd_advisor.hpp>
#include <semaforr/decision/navigation_advisor.hpp>
#include <semaforr/decision/restored_tiers.hpp>
#include <semaforr/decision/social_navigation_advisor.hpp>
#include <unordered_map>

namespace semaforr::decision {

void registerAdvisorCatalog(
    AdvisorRegistry& registry, const domain::ActionSpace& action_space,
    const std::vector<config::AdvisorConfiguration>& configured) {
  std::unordered_map<std::string, double> weights;
  for (const auto& advisor : configured) weights[advisor.name] = advisor.weight;
  const auto weight = [&weights](const std::string& name) {
    const auto found = weights.find(name);
    return found == weights.end() ? 1.0 : found->second;
  };
  const auto navigation =
      [&](std::string name, NavigationAdvisorObjective objective,
          ActionSelection selection) {
        registry.registerFactory(
            name, [name, objective, selection, action_space,
                   value = weight(name)] {
              return std::make_unique<NavigationAdvisor>(
                  NavigationAdvisorConfiguration{name, objective, selection,
                                                 action_space, value});
            });
      };
  navigation("goal_progress", NavigationAdvisorObjective::GoalProgress,
             ActionSelection::All);
  navigation("goal_progress_linear", NavigationAdvisorObjective::GoalProgress,
             ActionSelection::Linear);
  navigation("clearance", NavigationAdvisorObjective::Clearance,
             ActionSelection::All);
  navigation("clearance_rotation", NavigationAdvisorObjective::Clearance,
             ActionSelection::Rotation);
  navigation("exploration", NavigationAdvisorObjective::Exploration,
             ActionSelection::All);

  const auto spatial =
      [&](std::string name, SpatialAdvisorObjective objective) {
        registry.registerFactory(
            name, [name, objective, action_space, value = weight(name)] {
              return std::make_unique<SpatialAdvisor>(
                  name, objective, action_space, value);
            });
      };
  spatial("avoid_revisit", SpatialAdvisorObjective::AvoidRevisit);
  spatial("prefer_regions", SpatialAdvisorObjective::PreferRegions);
  spatial("prefer_highways", SpatialAdvisorObjective::PreferHighways);
  spatial("prefer_doors", SpatialAdvisorObjective::PreferDoors);
  spatial("follow_trails", SpatialAdvisorObjective::FollowTrails);

  const std::vector<std::pair<std::string, DissertationAdvisorObjective>>
      dissertation{
          {"big_step", DissertationAdvisorObjective::BigStep},
          {"elbow_room", DissertationAdvisorObjective::ElbowRoom},
          {"novelty", DissertationAdvisorObjective::Novelty},
          {"go_around", DissertationAdvisorObjective::GoAround},
          {"greedy", DissertationAdvisorObjective::Greedy},
          {"curiosity", DissertationAdvisorObjective::Curiosity},
          {"enfilade", DissertationAdvisorObjective::Enfilade},
          {"visual_scan", DissertationAdvisorObjective::VisualScan},
          {"convey", DissertationAdvisorObjective::Convey},
          {"enter", DissertationAdvisorObjective::Enter},
          {"exit", DissertationAdvisorObjective::Exit},
          {"trailer", DissertationAdvisorObjective::Trailer},
          {"unlikely", DissertationAdvisorObjective::Unlikely},
          {"access", DissertationAdvisorObjective::Access},
          {"crossroads", DissertationAdvisorObjective::Crossroads},
          {"follow", DissertationAdvisorObjective::Follow},
          {"least_angle", DissertationAdvisorObjective::LeastAngle},
          {"spatial_learner", DissertationAdvisorObjective::SpatialLearner},
          {"stay", DissertationAdvisorObjective::Stay}};
  for (const auto& [name, objective] : dissertation)
    registry.registerFactory(
        name, [name, objective, action_space, value = weight(name)] {
          return std::make_unique<DissertationAdvisor>(
              DissertationAdvisorConfiguration{name, objective, action_space,
                                                value});
        });

  SocialAdvisorConfiguration live;
  live.move_distances_m = action_space.move_distances_m();
  live.rotation_angles_rad = action_space.rotation_angles_rad();
  live.weight = weight("social_navigation");
  live.advisor_name = "social_navigation";
  registry.registerFactory(
      "social_navigation",
      [live] { return std::make_unique<SocialNavigationAdvisor>(live); });
  const auto learned = [&](std::string name, LearnedCrowdObjective objective) {
    LearnedCrowdAdvisorConfiguration configuration;
    configuration.move_distances_m = action_space.move_distances_m();
    configuration.rotation_angles_rad = action_space.rotation_angles_rad();
    configuration.weight = weight(name);
    configuration.advisor_name = name;
    configuration.objective = objective;
    registry.registerFactory(
        name, [configuration] {
          return std::make_unique<LearnedCrowdAdvisor>(configuration);
        });
  };
  learned("crowd_avoid", LearnedCrowdObjective::AvoidDensity);
  learned("risk_avoid", LearnedCrowdObjective::AvoidEncounterRisk);
  learned("flow_follow", LearnedCrowdObjective::PreferFollowingFlow);
}

}  // namespace semaforr::decision
