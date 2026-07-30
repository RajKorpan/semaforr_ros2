#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <semaforr/decision/learned_crowd_advisor.hpp>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/decision/navigation_advisor.hpp>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <semaforr/decision/restored_tiers.hpp>
#include <semaforr/decision/social_navigation_advisor.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/ros/navigation_engine_adapter.hpp>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::ros {
namespace {

domain::WorldModel makeWorld(const config::Configuration& configuration) {
  std::vector<domain::NavigationTask> tasks;
  tasks.reserve(configuration.tasks.size());
  for (std::size_t index = 0U; index < configuration.tasks.size(); ++index) {
    tasks.push_back(
        {index, {configuration.tasks[index].x, configuration.tasks[index].y}});
  }
  domain::WorldModel world;
  world.mission = domain::Mission(
      std::move(tasks),
      static_cast<std::size_t>(configuration.navigation.task_decision_limit));
  return world;
}

social::CrowdFieldLearnerConfiguration crowdConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.crowd_learning;
  social::CrowdFieldLearnerConfiguration result;
  result.geometry = {source.frame,
                     static_cast<double>(configuration.map_dimensions.length),
                     static_cast<double>(configuration.map_dimensions.height),
                     source.resolution_m,
                     source.origin_x_m,
                     source.origin_y_m};
  result.strategy = social::crowdEstimatorStrategyFromString(source.estimator);
  result.discount_factor = source.discount_factor;
  result.minimum_update_period_s = source.minimum_update_period_s;
  result.encounter_radius_m = source.encounter_radius_m;
  result.minimum_flow_speed_mps = source.minimum_flow_speed_mps;
  result.confidence_exposures = source.confidence_exposures;
  result.cusum_increase = source.cusum_increase;
  result.cusum_decrease = source.cusum_decrease;
  result.cusum_threshold = source.cusum_threshold;
  result.random_seed = source.random_seed;
  return result;
}

decision::ActionSelection selectionFor(const std::string& name) {
  if (name == "clearance_rotation") {
    return decision::ActionSelection::Rotation;
  }
  if (name == "goal_progress_linear") {
    return decision::ActionSelection::Linear;
  }
  return decision::ActionSelection::All;
}

decision::NavigationAdvisorObjective objectiveFor(const std::string& name) {
  if (name == "exploration") {
    return decision::NavigationAdvisorObjective::Exploration;
  }
  if (name == "clearance" || name == "clearance_rotation") {
    return decision::NavigationAdvisorObjective::Clearance;
  }
  return decision::NavigationAdvisorObjective::GoalProgress;
}

decision::SpatialAdvisorObjective spatialObjectiveFor(
    const std::string& name) {
  if (name == "prefer_regions")
    return decision::SpatialAdvisorObjective::PreferRegions;
  if (name == "prefer_highways")
    return decision::SpatialAdvisorObjective::PreferHighways;
  if (name == "prefer_doors")
    return decision::SpatialAdvisorObjective::PreferDoors;
  if (name == "follow_trails")
    return decision::SpatialAdvisorObjective::FollowTrails;
  return decision::SpatialAdvisorObjective::AvoidRevisit;
}

void addPlanner(planning::PlanningCoordinator& coordinator,
                const std::string& name, planning::PlannerObjective objective) {
  coordinator.registerPlanner(
      std::make_unique<planning::DomainPlanner>(name, objective));
}

}  // namespace

class NavigationEngineAdapter::Impl {
 public:
  explicit Impl(config::Configuration configuration)
      : configuration_(std::move(configuration)),
        action_space_(configuration_.navigation.move_actions,
                      configuration_.navigation.rotate_actions),
        world_(makeWorld(configuration_)),
        decisions_({1.0e-9, decision::UnscoredActionPolicy::Exclude, 0.0,
                    domain::Action::pause(),
                    configuration_.experiment.random_seed}),
        mission_(world_.mission),
        learning_(spatial::SpatialLearningCoordinator::defaults()),
        hard_safety_(action_space_.move_distances_m(),
                     action_space_.rotation_angles_rad(),
                     configuration_.navigation.robot_footprint,
                     configuration_.navigation.robot_footprint_buffer,
                     configuration_.experiment.safety_envelope
                         .sensor_freshness_timeout_s),
        phases_({configuration_.experiment.initial_exploration.enabled,
                 configuration_.experiment.initial_exploration
                     .observation_budget}) {
    configureLearning();
    configurePlanning();
    configureDecisions();
    if (configuration_.navigation.crowd_learning.enabled) {
      crowd_learning_ = std::make_unique<social::CrowdFieldLearner>(
          crowdConfiguration(configuration_));
    }
    std::vector<std::string> enabled_reactive;
    for (const auto& planner :
         configuration_.experiment.tiers.reactive_planners) {
      if (!configuration_.experiment.tiers.tier_one) break;
      if (std::find(configuration_.experiment.tiers.tier_one_rules.begin(),
                    configuration_.experiment.tiers.tier_one_rules.end(),
                    planner) !=
          configuration_.experiment.tiers.tier_one_rules.end())
        enabled_reactive.push_back(planner);
    }
    const auto has_tier_one_rule = [this](const std::string& name) {
      const auto& rules = configuration_.experiment.tiers.tier_one_rules;
      return configuration_.experiment.tiers.tier_one &&
             std::find(rules.begin(), rules.end(), name) != rules.end();
    };
    engine_ = std::make_unique<decision::NavigationEngine>(
        world_, action_space_, decisions_, mission_, planning_, learning_,
        crowd_learning_.get(), domain::Distance(0.5),
        &hard_safety_,
        &phases_,
        config::configurationFingerprint(configuration_),
        config::componentManifest(configuration_),
        enabled_reactive,
        configuration_.experiment.reactive_exploration_enabled &&
            has_tier_one_rule("low_level_exploration"),
        has_tier_one_rule("enforcer"));
  }

  void configureLearning() {
    const auto& features = configuration_.navigation;
    learning_.setEnabled(spatial::SpatialRepresentation::Trails,
                         features.trails_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Conveyors,
                         features.conveyors_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Regions,
                         features.regions_on);
    learning_.setEnabled(spatial::SpatialRepresentation::DoorsAndExits,
                         features.doors_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Hallways,
                         features.hallways_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Barriers,
                         features.barriers_on);
    learning_.setEnabled(spatial::SpatialRepresentation::PassagesAndSkeleton,
                         features.a_star_on || features.planners.skeleton);
    learning_.setEnabled(spatial::SpatialRepresentation::KnownGrid,
                         features.known_grid_on);
    learning_.setEnabled(spatial::SpatialRepresentation::InclusionGrid,
                         features.inclusion_grid_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Highways,
                         features.highways_on);
  }

  void configurePlanning() {
    if (!configuration_.experiment.tiers.tier_two) return;
    const auto& planners = configuration_.navigation.planners;
    if (planners.distance)
      addPlanner(planning_, "distance", planning::PlannerObjective::Distance);
    if (planners.skeleton) {
      planning_.registerPlanner(std::make_unique<planning::SkeletonPlan>());
    }
    if (planners.highway) {
      planning_.registerPlanner(std::make_unique<planning::HighwayPlan>());
    }
    if (planners.density) {
      addPlanner(planning_, "density",
                 planning::PlannerObjective::CrowdDensity);
    }
    if (planners.risk) {
      addPlanner(planning_, "risk", planning::PlannerObjective::EncounterRisk);
    }
    if (planners.flow) {
      addPlanner(planning_, "flow", planning::PlannerObjective::FlowAlignment);
    }
  }

  void configureDecisions() {
    decision::TierOneRegistry tier_one_registry;
    decision::AdvisorRegistry tier_three_registry;
    decision::registerRestoredTierFactories(
        tier_one_registry, tier_three_registry, action_space_);
    if (configuration_.experiment.tiers.tier_one) {
      for (const auto& rule :
           configuration_.experiment.tiers.tier_one_rules) {
        if (rule == "victory")
          decisions_.addMandatoryRule(
              tier_one_registry.createMandatory("Victory"));
        else if (rule == "forward")
          decisions_.addMandatoryRule(
              tier_one_registry.createMandatory("Forward"));
        else if (rule == "not_opposite")
          decisions_.addVetoRule(
              tier_one_registry.createVeto("NotOpposite"));
        else if (rule == "avoid_obstacles")
          decisions_.addVetoRule(std::make_unique<decision::ObstacleVetoRule>(
              action_space_.move_distances_m(),
              configuration_.navigation.robot_footprint,
              configuration_.navigation.robot_footprint_buffer));
      }
    }
    if (!configuration_.experiment.tiers.tier_three) return;
    for (const auto& advisor : configuration_.advisors) {
      if (!advisor.active) {
        continue;
      }
      const bool social_advisor =
          advisor.name == "social_navigation" ||
          advisor.name == "crowd_avoid" || advisor.name == "risk_avoid" ||
          advisor.name == "flow_follow";
      if (social_advisor &&
          (!configuration_.experiment.social.enabled ||
           !configuration_.experiment.social.advisors))
        continue;
      if (advisor.name == "social_navigation") {
        decision::SocialAdvisorConfiguration social_configuration;
        social_configuration.move_distances_m =
            action_space_.move_distances_m();
        social_configuration.rotation_angles_rad =
            action_space_.rotation_angles_rad();
        social_configuration.weight = advisor.weight;
        social_configuration.advisor_name = advisor.name;
        decisions_.addAdvisor(
            std::make_unique<decision::SocialNavigationAdvisor>(
                std::move(social_configuration)));
        continue;
      }
      if (advisor.name == "crowd_avoid" || advisor.name == "risk_avoid" ||
          advisor.name == "flow_follow") {
        decision::LearnedCrowdAdvisorConfiguration learned;
        learned.move_distances_m = action_space_.move_distances_m();
        learned.rotation_angles_rad = action_space_.rotation_angles_rad();
        learned.weight = advisor.weight;
        learned.advisor_name = advisor.name;
        if (advisor.name == "risk_avoid") {
          learned.objective =
              decision::LearnedCrowdObjective::AvoidEncounterRisk;
        } else if (advisor.name == "flow_follow") {
          learned.objective =
              decision::LearnedCrowdObjective::PreferFollowingFlow;
        } else {
          learned.objective = decision::LearnedCrowdObjective::AvoidDensity;
        }
        decisions_.addAdvisor(std::make_unique<decision::LearnedCrowdAdvisor>(
            std::move(learned)));
        continue;
      }
      if (advisor.name == "avoid_revisit" ||
          advisor.name == "prefer_regions" ||
          advisor.name == "prefer_highways" ||
          advisor.name == "prefer_doors" ||
          advisor.name == "follow_trails") {
        decisions_.addAdvisor(std::make_unique<decision::SpatialAdvisor>(
            advisor.name, spatialObjectiveFor(advisor.name), action_space_,
            advisor.weight));
        continue;
      }
      decisions_.addAdvisor(std::make_unique<decision::NavigationAdvisor>(
          decision::NavigationAdvisorConfiguration{
              advisor.name, objectiveFor(advisor.name),
              selectionFor(advisor.name), action_space_, advisor.weight}));
    }
  }

  config::Configuration configuration_;
  domain::ActionSpace action_space_;
  domain::WorldModel world_;
  decision::DecisionCoordinator decisions_;
  decision::MissionManager mission_;
  planning::PlanningCoordinator planning_;
  spatial::SpatialLearningCoordinator learning_;
  decision::HardSafetyFilter hard_safety_;
  navigation::NavigationPhaseCoordinator phases_;
  std::unique_ptr<social::CrowdFieldLearner> crowd_learning_;
  std::unique_ptr<decision::NavigationEngine> engine_;
};

NavigationEngineAdapter::NavigationEngineAdapter(
    config::Configuration configuration)
    : impl_(std::make_unique<Impl>(std::move(configuration))) {}

NavigationEngineAdapter::~NavigationEngineAdapter() = default;
NavigationEngineAdapter::NavigationEngineAdapter(
    NavigationEngineAdapter&&) noexcept = default;
NavigationEngineAdapter& NavigationEngineAdapter::operator=(
    NavigationEngineAdapter&&) noexcept = default;

void NavigationEngineAdapter::observe(const SynchronizedSensors& sensors,
                                      const domain::CrowdState& crowd) {
  impl_->engine_->observe({sensors.pose, sensors.scan, crowd.current(),
                           std::chrono::steady_clock::now()});
}

bool NavigationEngineAdapter::missionComplete() {
  return impl_->engine_->missionComplete();
}

navigation::NavigationPhase NavigationEngineAdapter::phase() const noexcept {
  return impl_->engine_->phase();
}

decision::DecisionResult NavigationEngineAdapter::decide() {
  return impl_->engine_->decide();
}

ActionExecutionRequest NavigationEngineAdapter::executionRequest(
    const domain::Action& action) const {
  ActionExecutionRequest request{action, 0.0, 0.0};
  const std::size_t magnitude = action.magnitude_index();
  if (action.type() == domain::ActionType::Forward) {
    request.target_distance_m =
        impl_->action_space_.move_distances_m().at(magnitude - 1U);
  } else if (action.type() == domain::ActionType::TurnLeft ||
             action.type() == domain::ActionType::TurnRight) {
    request.target_angle_rad =
        impl_->action_space_.rotation_angles_rad().at(magnitude - 1U);
  }
  return request;
}

const domain::WorldModel& NavigationEngineAdapter::worldModel() const noexcept {
  return impl_->world_;
}

}  // namespace semaforr::ros
