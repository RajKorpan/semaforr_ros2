#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <semaforr/decision/advisors/catalog_registry.hpp>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <semaforr/planning/static_map_loader.hpp>
#include <stdexcept>
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
    const config::Configuration& configuration,
    const domain::StaticMap* static_map) {
  const auto& source = configuration.navigation.crowd_learning;
  social::CrowdFieldLearnerConfiguration result;
  result.geometry = static_map && static_map->occupancyAvailable()
                        ? static_map->occupancy.geometry
                        : domain::GridGeometry{
                              source.frame,
                              static_cast<double>(
                                  configuration.map_dimensions.length),
                              static_cast<double>(
                                  configuration.map_dimensions.height),
                              source.resolution_m, source.origin_x_m,
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

spatial::CircumstanceLearningConfiguration circumstanceConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.circumstances;
  return {source.setting_resolution_m,
          source.setting_radius_m,
          source.minimum_cluster_size,
          source.assignment_confidence_threshold,
          source.similarity_l1_threshold,
          source.reclustering_threshold,
          source.minimum_case_evidence,
          source.accuracy_threshold,
          source.action_confidence_threshold,
          source.distance_bin_base_m,
          source.angle_bin_count};
}

decision::PrecedentConfiguration precedentConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.circumstances;
  return {source.minimum_case_evidence, source.accuracy_threshold,
          source.action_confidence_threshold};
}

planning::MapSearchPaths mapSearchPaths() {
  planning::MapSearchPaths paths;
  paths.working_directory = std::filesystem::current_path();
  for (const std::string package : {"semaforr", "semaforr_examples"}) {
    try {
      paths.package_shares.emplace(
          package, ament_index_cpp::get_package_share_directory(package));
    } catch (const std::exception&) {
      // Source-only tests may not have every workspace package installed.
    }
  }
  const auto examples = paths.package_shares.find("semaforr_examples");
  if (examples != paths.package_shares.end()) {
    paths.example_core = examples->second / "core";
  } else {
    const auto source_examples = paths.working_directory / "src/examples/core";
    if (std::filesystem::exists(source_examples))
      paths.example_core = source_examples;
  }
  return paths;
}

domain::UnknownSpacePolicy unknownPolicy(const std::string& value) {
  if (value == "prohibited") return domain::UnknownSpacePolicy::Prohibited;
  if (value == "high_cost") return domain::UnknownSpacePolicy::HighCost;
  if (value == "within_sensor_range")
    return domain::UnknownSpacePolicy::WithinSensorRange;
  if (value == "exploration_only")
    return domain::UnknownSpacePolicy::ExplorationOnly;
  throw std::runtime_error("unknown grid unknown-space policy '" + value +
                           "'");
}

spatial::GridExtentPolicy gridExtentPolicy(const std::string& value) {
  if (value == "expand") return spatial::GridExtentPolicy::Expand;
  if (value == "fixed") return spatial::GridExtentPolicy::Fixed;
  throw std::runtime_error("unknown grid extent policy '" + value + "'");
}

spatial::LearnedGridConfiguration learnedGridConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.grids;
  spatial::LearnedGridConfiguration result;
  result.frame_id = source.frame_id;
  result.initial_width_m = source.mapless_initial_width_m;
  result.initial_height_m = source.mapless_initial_height_m;
  result.resolution_m = source.resolution_m;
  result.extent_policy = gridExtentPolicy(source.extent_policy);
  result.expansion = {source.expansion_margin_m,
                      source.expansion_increment_cells,
                      source.maximum_width_m,
                      source.maximum_height_m,
                      source.memory_limit_cells};
  result.initialize_around_first_pose = true;
  return result;
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
        learning_(spatial::SpatialLearningCoordinator::defaults(
            10U, circumstanceConfiguration(configuration_),
            {static_cast<std::uint16_t>(
                 configuration_.navigation.grids.free_observations_to_clear),
             configuration_.navigation.grids.dynamic_expiry_observations,
             true},
            learnedGridConfiguration(configuration_))),
        hard_safety_(action_space_.move_distances_m(),
                     action_space_.rotation_angles_rad(),
                     configuration_.navigation.robot_footprint,
                     configuration_.navigation.robot_footprint_buffer,
                     configuration_.experiment.safety_envelope
                         .sensor_freshness_timeout_s),
        phases_(
            {configuration_.experiment.initial_exploration.enabled,
             configuration_.experiment.initial_exploration.observation_budget,
             configuration_.experiment.initial_exploration.time_limit_s}) {
    configureStaticMap();
    configureLearning();
    configurePlanning();
    configureDecisions();
    if (configuration_.navigation.crowd_learning.enabled) {
      crowd_learning_ = std::make_unique<social::CrowdFieldLearner>(
          crowdConfiguration(configuration_, world_.static_map));
    }
    decision::TierOneRegistry tier_one_registry;
    decision::AdvisorRegistry unused_advisors;
    decision::registerTierFactories(
        tier_one_registry, unused_advisors, action_space_,
        configuration_.navigation.robot_footprint,
        configuration_.navigation.robot_footprint_buffer,
        precedentConfiguration(configuration_));
    std::vector<std::unique_ptr<planning::ReactivePlanner>> enabled_reactive;
    for (const auto& planner :
         configuration_.experiment.tiers.reactive_planners) {
      if (!configuration_.experiment.tiers.tier_one) break;
      if (std::find(configuration_.experiment.tiers.tier_one_rules.begin(),
                    configuration_.experiment.tiers.tier_one_rules.end(),
                    planner) !=
              configuration_.experiment.tiers.tier_one_rules.end() &&
          tier_one_registry.kind(planner) !=
              decision::TierOneRegistry::Kind::ReplanningTrigger)
        enabled_reactive.push_back(tier_one_registry.createReactive(planner));
    }
    const auto has_tier_one_rule = [this](const std::string& name) {
      const auto& rules = configuration_.experiment.tiers.tier_one_rules;
      return configuration_.experiment.tiers.tier_one &&
             std::find(rules.begin(), rules.end(), name) != rules.end();
    };
    auto lle_component =
        configuration_.experiment.reactive_exploration_enabled &&
                has_tier_one_rule("low_level_exploration")
            ? tier_one_registry.createReactive("low_level_exploration")
            : nullptr;
    auto enforcer_component =
        has_tier_one_rule("enforcer")
            ? tier_one_registry.createOperationalizer("enforcer")
            : nullptr;
    auto manifest = config::componentManifest(configuration_);
    for (const auto& diagnostic : map_diagnostics_) {
      constexpr std::string_view prefix = "planner_disabled_no_map:";
      if (!diagnostic.starts_with(prefix)) continue;
      const std::string configured =
          "planner:" + diagnostic.substr(prefix.size());
      std::erase(manifest, configured);
    }
    manifest.push_back(world_.map_capabilities.map_available
                           ? "map:loaded"
                           : "map:not_available");
    manifest.push_back(world_.map_capabilities.map_based_planning_available
                           ? "capability:map_based_planning"
                           : "capability:no_map_based_planning");
    manifest.insert(manifest.end(), map_diagnostics_.begin(),
                    map_diagnostics_.end());
    engine_ = std::make_unique<decision::NavigationEngine>(
        world_, action_space_, decisions_, mission_, planning_, learning_,
        crowd_learning_.get(), domain::Distance(0.5), &hard_safety_, &phases_,
        config::configurationFingerprint(configuration_),
        std::move(manifest), std::move(enabled_reactive),
        configuration_.experiment.reactive_exploration_enabled &&
            has_tier_one_rule("low_level_exploration"),
        has_tier_one_rule("enforcer"),
        exploration::HighLevelExplorationConfiguration{
            domain::Distance(configuration_.experiment.initial_exploration
                                 .minimum_clearance_m),
            domain::Angle(configuration_.experiment.initial_exploration
                              .heading_tolerance_rad),
            domain::Distance(configuration_.experiment.initial_exploration
                                 .candidate_completion_distance_m),
            domain::Distance(configuration_.experiment.initial_exploration
                                 .cue_similarity_radius_m),
            domain::Distance(configuration_.experiment.initial_exploration
                                 .passage_grid_resolution_m),
            configuration_.experiment.initial_exploration.minimum_bundle_beams,
            std::chrono::duration<double>(
                configuration_.experiment.initial_exploration.time_limit_s),
            configuration_.experiment.initial_exploration.decision_budget},
        std::move(lle_component), std::move(enforcer_component),
        planning::TraversabilityConfiguration{
            unknownPolicy(configuration_.navigation.grids.map_unknown_policy),
            unknownPolicy(
                configuration_.navigation.grids.sensor_unknown_policy),
            configuration_.navigation.robot_footprint,
            configuration_.navigation.robot_footprint_buffer,
            configuration_.navigation.grids.localization_uncertainty_m,
            configuration_.navigation.grids.turning_footprint_margin_m,
            configuration_.navigation.grids.dynamic_obstacle_margin_m,
            static_cast<float>(
                configuration_.navigation.grids.unknown_cost_multiplier)});
  }

  void configureStaticMap() {
    if (configuration_.static_map.mode == config::MapOperatingMode::Mapless) {
      map_diagnostics_.push_back("map_mode:mapless");
      map_diagnostics_.push_back("map_status:mapless_parser_not_invoked");
      map_diagnostics_.push_back("map_capabilities:geometry=false,occupancy=false,planning=false");
      return;
    }
    map_diagnostics_.push_back("map_mode:map_enabled");
    try {
      const auto resolved = planning::resolveMapPath(
          configuration_.static_map.path, mapSearchPaths());
      auto loaded = planning::loadStaticMap(
          resolved, configuration_.map_dimensions, configuration_.static_map);
      for (const auto& task : configuration_.tasks) {
        if (!loaded.bounds.contains({task.x, task.y}))
          throw std::runtime_error(
              "mission target lies outside resolved static-map bounds");
      }
      static_map_owner_ =
          std::make_unique<const domain::StaticMap>(std::move(loaded));
      world_.static_map = static_map_owner_.get();
      world_.map_capabilities = {
          true, true, true,
          configuration_.static_map.map_based_planning_enabled};
      map_diagnostics_.push_back("map_status:loaded");
      map_diagnostics_.push_back("map_source:" + world_.static_map->source);
      map_diagnostics_.push_back("map_checksum:" +
                                 world_.static_map->checksum);
      map_diagnostics_.push_back(
          std::string("map_grid_extent_source:") +
          domain::toString(
              world_.static_map->occupancy.geometry.extent_source));
      map_diagnostics_.push_back(
          "map_grid_geometry_revision:" +
          std::to_string(
              world_.static_map->occupancy.geometry.geometry_revision));
      map_diagnostics_.push_back(
          std::string("map_capabilities:geometry=true,occupancy=true,planning=") +
          (world_.map_capabilities.map_based_planning_available ? "true"
                                                                : "false"));
    } catch (const std::exception& error) {
      world_.static_map = nullptr;
      static_map_owner_.reset();
      world_.map_capabilities = {};
      if (configuration_.static_map.failure_policy ==
          config::MapLoadFailurePolicy::FailStartup)
        throw std::runtime_error(
            "failed to initialize requested SemaFORR map '" +
            configuration_.static_map.path + "': " + error.what());
      map_diagnostics_.push_back("map_status:load_failed_map_disabled");
      map_diagnostics_.push_back("map_capabilities:geometry=false,occupancy=false,planning=false");
      map_diagnostics_.push_back("map_error:" + std::string(error.what()));
    }
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
                         features.a_star_on || features.planners.skeleton ||
                             features.planners.region ||
                             features.planners.hallway ||
                             features.planners.trail ||
                             features.planners.conveyor);
    learning_.setEnabled(spatial::SpatialRepresentation::KnownGrid,
                         features.known_grid_on);
    learning_.setEnabled(spatial::SpatialRepresentation::SensedOccupancy,
                         features.sensed_occupancy_on);
    learning_.setEnabled(spatial::SpatialRepresentation::InclusionGrid,
                         features.inclusion_grid_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Highways,
                         features.highways_on);
    learning_.setEnabled(spatial::SpatialRepresentation::Circumstances,
                         features.circumstances_on);
  }

  void configurePlanning() {
    if (!configuration_.experiment.tiers.tier_two) return;
    const auto& planners = configuration_.navigation.planners;
    planning_.setSelectionPolicy(
        planning::planSelectionPolicyFromString(planners.selection_policy));
    const auto registry = planning::defaultPlannerRegistry();
    const std::vector<std::pair<std::string, bool>> enabled = {
        {"distance", planners.distance},
        {"sensor_distance", planners.sensor_distance},
        {"density", planners.density},
        {"risk", planners.risk},         {"flow", planners.flow},
        {"region", planners.region},     {"hallway", planners.hallway},
        {"trail", planners.trail},       {"conveyor", planners.conveyor},
        {"skeleton", planners.skeleton}, {"highway", planners.highway}};
    for (const auto& [name, on] : enabled) {
      if (!on) continue;
      if (registry.mapRequirement(name) ==
              planning::StaticMapRequirement::Required &&
          !world_.map_capabilities.map_based_planning_available) {
        map_diagnostics_.push_back("planner_disabled_no_map:" + name);
        continue;
      }
      planning_.registerPlanner(registry.create(name));
      map_diagnostics_.push_back(
          registry.occupancyRequirement(name) ==
                  planning::OccupancyRequirement::SensedPartial
              ? "planner_registered_dormant_until_sensed_occupancy:" + name
              : "planner_enabled:" + name);
    }
  }

  void configureDecisions() {
    decision::TierOneRegistry tier_one_registry;
    decision::AdvisorRegistry tier_three_registry;
    decision::AdvisorRegistry unused_advisors;
    decision::registerTierFactories(
        tier_one_registry, unused_advisors, action_space_,
        configuration_.navigation.robot_footprint,
        configuration_.navigation.robot_footprint_buffer,
        precedentConfiguration(configuration_));
    decision::registerAdvisorCatalog(tier_three_registry, action_space_,
                                     configuration_.advisors);
    if (configuration_.experiment.tiers.tier_one) {
      for (const auto& rule : configuration_.experiment.tiers.tier_one_rules) {
        switch (tier_one_registry.kind(rule)) {
          case decision::TierOneRegistry::Kind::Mandatory:
            decisions_.addMandatoryRule(
                tier_one_registry.createMandatory(rule));
            break;
          case decision::TierOneRegistry::Kind::Veto:
            decisions_.addVetoRule(tier_one_registry.createVeto(rule));
            break;
          case decision::TierOneRegistry::Kind::PlanOperationalizer:
          case decision::TierOneRegistry::Kind::ReactivePlanner:
          case decision::TierOneRegistry::Kind::ReplanningTrigger:
            break;
        }
      }
    }
    if (!configuration_.experiment.tiers.tier_three) return;
    for (const auto& advisor : configuration_.advisors) {
      if (advisor.active)
        decisions_.addAdvisor(tier_three_registry.create(advisor.name));
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
  std::unique_ptr<const domain::StaticMap> static_map_owner_;
  std::vector<std::string> map_diagnostics_;
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
    const decision::DecisionResult& decision) const {
  const domain::Action& action = decision.action;
  ActionExecutionRequest request{action, 0.0, 0.0, decision.decision_id,
                                 decision.action_id};
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

domain::FeedbackDisposition NavigationEngineAdapter::onActionStarted(
    const ActionExecutionUpdate& update) {
  return impl_->engine_->onActionStarted(
      {update.decision_id, update.action_id, std::chrono::steady_clock::now(),
       update.start_pose});
}

domain::FeedbackDisposition NavigationEngineAdapter::onActionProgress(
    const ActionExecutionUpdate& update) {
  return impl_->engine_->onActionProgress(
      {update.decision_id, update.action_id, std::chrono::steady_clock::now(),
       update.final_pose, update.distance_achieved_m,
       update.rotation_achieved_rad});
}

domain::FeedbackDisposition NavigationEngineAdapter::onActionTerminal(
    const ActionExecutionUpdate& update,
    domain::ExecutionCompletionStatus status, std::string detail) {
  domain::ActionExecutionResult result;
  result.decision_id = update.decision_id;
  result.action_id = update.action_id;
  if (const auto* pending = impl_->engine_->pendingAction())
    result.task_id = pending->task_id;
  result.finished_at = std::chrono::steady_clock::now();
  result.status = status;
  result.start_pose = update.start_pose;
  result.final_pose = update.final_pose;
  result.distance_achieved_m = update.distance_achieved_m;
  result.rotation_achieved_rad = update.rotation_achieved_rad;
  result.timed_out = status == domain::ExecutionCompletionStatus::TimedOut;
  result.cancellation_reason = std::move(detail);
  result.safety_interruption =
      status == domain::ExecutionCompletionStatus::SafetyInterrupted;
  result.controller_failure =
      status == domain::ExecutionCompletionStatus::ControllerFailure ||
      status == domain::ExecutionCompletionStatus::ControllerRejected;
  if (status == domain::ExecutionCompletionStatus::Succeeded)
    return impl_->engine_->onActionCompleted(std::move(result));
  if (status == domain::ExecutionCompletionStatus::Cancelled ||
      status == domain::ExecutionCompletionStatus::GoalPreempted ||
      status == domain::ExecutionCompletionStatus::NavigationModeTransition ||
      status == domain::ExecutionCompletionStatus::SensorLost ||
      status == domain::ExecutionCompletionStatus::Shutdown)
    return impl_->engine_->onActionCancelled(std::move(result));
  return impl_->engine_->onActionFailed(std::move(result));
}

domain::FeedbackDisposition NavigationEngineAdapter::onControllerRestart(
    const domain::Pose2D& pose) {
  return impl_->engine_->onControllerRestart(std::chrono::steady_clock::now(),
                                             pose);
}

const domain::WorldModel& NavigationEngineAdapter::worldModel() const noexcept {
  return impl_->world_;
}

const std::vector<std::string>& NavigationEngineAdapter::startupDiagnostics()
    const noexcept {
  return impl_->map_diagnostics_;
}

}  // namespace semaforr::ros
