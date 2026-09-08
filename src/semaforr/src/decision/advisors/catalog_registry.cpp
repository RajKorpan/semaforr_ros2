/**
 * @file catalog_registry.cpp
 * @brief Catalog registry responsibilities.
 *
 * @details This file implements catalog registry behavior for tiered decision
 * making and action arbitration. It centers on `RandomTieAdvisor`. Its
 * package-relative location is
 * `src/decision/advisors/catalog_registry.cpp`.
 */
#include <semaforr/decision/advisors/catalog_registry.hpp>
#include <semaforr/decision/advisors/heuristic_advisor.hpp>
#include <semaforr/decision/advisors/navigation_advisor.hpp>
#include <semaforr/decision/advisors/social/learned_crowd_advisor.hpp>
#include <semaforr/decision/advisors/social/social_navigation_advisor.hpp>
#include <semaforr/decision/tier_registry.hpp>
#include <unordered_map>

namespace semaforr::decision {
namespace {

/**
 * @brief Encapsulates random tie advisor state and behavior for this
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
class RandomTieAdvisor final : public Advisor {
 public:
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return "random"; }
  /**
   * @brief Performs the metadata operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `AdvisorMetadata` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  AdvisorMetadata metadata() const override {
    return {{},
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            true,
            ScoreNormalization::None,
            "leave all viable actions tied for seeded selection"};
  }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `AdvisorEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  AdvisorEvaluation evaluate(
      const DecisionContext&,
      std::span<const domain::Action> candidates) const override {
    AdvisorEvaluation result;
    result.participated = !candidates.empty();
    result.explanation = "seeded random baseline";
    for (const auto action : candidates) result.scores.push_back({action, 0.0});
    return result;
  }
};

}  // namespace

/**
 * @brief Registers advisor catalog for this subsystem.
 *
 * Arguments:
 * - @p registry: Supplies registry input to the operation.
 * - @p action_space: Supplies action space input to the operation.
 * - @p configured: Supplies configured input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void registerAdvisorCatalog(
    AdvisorRegistry& registry, const domain::ActionSpace& action_space,
    const std::vector<config::AdvisorConfiguration>& configured) {
  std::unordered_map<std::string, double> weights;
  for (const auto& advisor : configured) weights[advisor.name] = advisor.weight;
  registry.registerFactory("random",
                           [] { return std::make_unique<RandomTieAdvisor>(); });
  const auto weight = [&weights](const std::string& name) {
    const auto found = weights.find(name);
    return found == weights.end() ? 1.0 : found->second;
  };
  const auto navigation = [&](std::string name,
                              NavigationAdvisorObjective objective,
                              ActionSelection selection) {
    registry.registerFactory(name, [name, objective, selection, action_space,
                                    value = weight(name)] {
      return std::make_unique<NavigationAdvisor>(NavigationAdvisorConfiguration{
          name, objective, selection, action_space, value});
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

  const auto spatial = [&](std::string name,
                           SpatialAdvisorObjective objective) {
    registry.registerFactory(
        name, [name, objective, action_space, value = weight(name)] {
          return std::make_unique<SpatialAdvisor>(name, objective, action_space,
                                                  value);
        });
  };
  spatial("avoid_revisit", SpatialAdvisorObjective::AvoidRevisit);
  spatial("prefer_regions", SpatialAdvisorObjective::PreferRegions);
  spatial("prefer_highways", SpatialAdvisorObjective::PreferHighways);
  spatial("prefer_doors", SpatialAdvisorObjective::PreferDoors);
  spatial("follow_trails", SpatialAdvisorObjective::FollowTrails);

  const std::vector<std::pair<std::string, HeuristicObjective>> heuristics{
      {"big_step", HeuristicObjective::BigStep},
      {"elbow_room", HeuristicObjective::ElbowRoom},
      {"novelty", HeuristicObjective::Novelty},
      {"go_around", HeuristicObjective::GoAround},
      {"greedy", HeuristicObjective::Greedy},
      {"curiosity", HeuristicObjective::Curiosity},
      {"enfilade", HeuristicObjective::Enfilade},
      {"visual_scan", HeuristicObjective::VisualScan},
      {"convey", HeuristicObjective::Convey},
      {"enter", HeuristicObjective::Enter},
      {"exit", HeuristicObjective::Exit},
      {"trailer", HeuristicObjective::Trailer},
      {"unlikely", HeuristicObjective::Unlikely},
      {"access", HeuristicObjective::Access},
      {"crossroads", HeuristicObjective::Crossroads},
      {"follow", HeuristicObjective::Follow},
      {"least_angle", HeuristicObjective::LeastAngle},
      {"spatial_learner", HeuristicObjective::SpatialLearner},
      {"stay", HeuristicObjective::Stay}};
  for (const auto& [name, objective] : heuristics)
    registry.registerFactory(name, [name, objective, action_space,
                                    value = weight(name)] {
      return std::make_unique<HeuristicAdvisor>(
          HeuristicAdvisorConfiguration{name, objective, action_space, value});
    });

  SocialAdvisorConfiguration live;
  live.move_distances_m = action_space.move_distances_m();
  live.rotation_angles_rad = action_space.rotation_angles_rad();
  live.weight = weight("social_navigation");
  live.advisor_name = "social_navigation";
  registry.registerFactory("social_navigation", [live] {
    return std::make_unique<SocialNavigationAdvisor>(live);
  });
  const auto learned = [&](std::string name, LearnedCrowdObjective objective) {
    LearnedCrowdAdvisorConfiguration configuration;
    configuration.move_distances_m = action_space.move_distances_m();
    configuration.rotation_angles_rad = action_space.rotation_angles_rad();
    configuration.weight = weight(name);
    configuration.advisor_name = name;
    configuration.objective = objective;
    registry.registerFactory(name, [configuration] {
      return std::make_unique<LearnedCrowdAdvisor>(configuration);
    });
  };
  learned("crowd_avoid", LearnedCrowdObjective::AvoidDensity);
  learned("risk_avoid", LearnedCrowdObjective::AvoidEncounterRisk);
  learned("flow_follow", LearnedCrowdObjective::PreferFollowingFlow);
}

}  // namespace semaforr::decision
