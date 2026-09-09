/**
 * @file navigation_engine_adapter.cpp
 * @brief Navigation engine adapter responsibilities.
 *
 * @details This file implements navigation engine adapter behavior for the ROS
 * 2 composition and message-adaptation boundary. It centers on
 * `NavigationEngineAdapter`. Its package-relative location is
 * `src/ros/navigation_engine_adapter.cpp`.
 */
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
#include <semaforr/ros/navigation_engine_adapter.hpp>
#include <semaforr/validation/replay.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::ros {
namespace {

/**
 * @brief Creates world for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `domain::WorldModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the crowd configuration operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 * - @p static_map: Supplies static map input to the operation.
 *
 * Returns:
 * - `social::CrowdFieldLearnerConfiguration` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
social::CrowdFieldLearnerConfiguration crowdConfiguration(
    const config::Configuration& configuration,
    const domain::StaticMap* static_map) {
  const auto& source = configuration.navigation.crowd_learning;
  social::CrowdFieldLearnerConfiguration result;
  result.geometry =
      static_map && static_map->occupancyAvailable()
          ? static_map->occupancy.geometry
          : domain::GridGeometry{
                source.frame,
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

/**
 * @brief Performs the circumstance configuration operation for this
 * subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `spatial::CircumstanceLearningConfiguration` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
spatial::CircumstanceLearningConfiguration circumstanceConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.circumstances;
  spatial::CircumstanceLearningConfiguration result;
  if (source.learning_mode == "dissertation_compatible")
    result.mode = domain::CircumstanceLearningMode::DissertationCompatible;
  else if (source.learning_mode == "adapted_threshold")
    result.mode = domain::CircumstanceLearningMode::AdaptedThreshold;
  else
    throw std::runtime_error("unknown circumstance learning mode '" +
                             source.learning_mode + "'");
  result.setting_resolution_m = source.setting_resolution_m;
  result.setting_radius_m = source.setting_radius_m;
  result.minimum_cluster_size = source.minimum_cluster_size;
  result.assignment_confidence_threshold =
      source.assignment_confidence_threshold;
  result.similarity_l1_threshold = source.similarity_l1_threshold;
  result.reclustering_threshold = source.reclustering_threshold;
  result.minimum_case_evidence = source.minimum_case_evidence;
  result.minimum_action_evidence = source.minimum_action_evidence;
  result.accuracy_threshold = source.accuracy_threshold;
  result.action_confidence_threshold = source.action_confidence_threshold;
  result.distance_bin_base_m = source.distance_bin_base_m;
  result.angle_bin_count = source.angle_bin_count;
  result.partial_success_credit = source.partial_success_credit;
  result.safety_interruption_is_negative_evidence =
      source.safety_interruption_is_negative_evidence;
  result.model_version = source.model_version;
  result.classifier_version = source.classifier_version;
  result.feature_version = source.feature_version;
  result.persistence_policy = source.persistence_policy;
  result.model_path = source.model_path;
  return result;
}

/**
 * @brief Performs the precedent configuration operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `decision::PrecedentConfiguration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
decision::PrecedentConfiguration precedentConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.circumstances;
  return {source.minimum_case_evidence, source.minimum_action_evidence,
          source.assignment_confidence_threshold, source.accuracy_threshold,
          source.action_confidence_threshold};
}

/**
 * @brief Performs the map search paths operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `planning::MapSearchPaths` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the unknown policy operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `domain::UnknownSpacePolicy` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::UnknownSpacePolicy unknownPolicy(const std::string& value) {
  if (value == "prohibited") return domain::UnknownSpacePolicy::Prohibited;
  if (value == "high_cost") return domain::UnknownSpacePolicy::HighCost;
  if (value == "within_sensor_range")
    return domain::UnknownSpacePolicy::WithinSensorRange;
  if (value == "exploration_only")
    return domain::UnknownSpacePolicy::ExplorationOnly;
  throw std::runtime_error("unknown grid unknown-space policy '" + value + "'");
}

/**
 * @brief Performs the grid extent policy operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `spatial::GridExtentPolicy` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
spatial::GridExtentPolicy gridExtentPolicy(const std::string& value) {
  if (value == "expand") return spatial::GridExtentPolicy::Expand;
  if (value == "fixed") return spatial::GridExtentPolicy::Fixed;
  throw std::runtime_error("unknown grid extent policy '" + value + "'");
}

/**
 * @brief Performs the learned grid configuration operation for this
 * subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `spatial::LearnedGridConfiguration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
spatial::LearnedGridConfiguration learnedGridConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.navigation.grids;
  spatial::LearnedGridConfiguration result;
  result.frame_id = source.frame_id;
  result.initial_width_m = source.mapless_initial_width_m;
  result.initial_height_m = source.mapless_initial_height_m;
  result.resolution_m = source.resolution_m;
  result.highway_origin = {source.highway_origin_x_m,
                           source.highway_origin_y_m};
  result.highway_smoothing_policy = source.highway_smoothing_policy;
  result.highway_component_selection_policy =
      source.highway_component_selection_policy;
  result.extent_policy = gridExtentPolicy(source.extent_policy);
  result.expansion = {source.expansion_margin_m,
                      source.expansion_increment_cells, source.maximum_width_m,
                      source.maximum_height_m, source.memory_limit_cells};
  result.initialize_around_first_pose = true;
  result.learning_mode =
      configuration.experiment.behavior_mode ==
                  config::BehaviorMode::Compatibility ||
              configuration.navigation.spatial_learning_profile ==
                  config::SpatialLearningProfile::Chapter3Compatibility
          ? spatial::SpatialLearningMode::Compatibility
          : spatial::SpatialLearningMode::Modernized;
  return result;
}

/**
 * @brief Performs the hle configuration operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `exploration::HighLevelExplorationConfiguration` containing the
 * operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
exploration::HighLevelExplorationConfiguration hleConfiguration(
    const config::Configuration& configuration) {
  const auto& source = configuration.experiment.initial_exploration;
  exploration::HighLevelExplorationConfiguration result;
  if (source.behavior_policy == "compatibility" ||
      (source.behavior_policy == "profile" &&
       configuration.experiment.behavior_mode ==
           config::BehaviorMode::Compatibility))
    result.behavior_policy = exploration::HleBehaviorPolicy::Compatibility;
  else if (source.behavior_policy == "modernized" ||
           source.behavior_policy == "profile")
    result.behavior_policy = exploration::HleBehaviorPolicy::Modernized;
  else
    throw std::runtime_error("unknown HLE behavior policy '" +
                             source.behavior_policy + "'");
  result.minimum_clearance = domain::Distance(source.minimum_clearance_m);
  result.heading_tolerance = domain::Angle(source.heading_tolerance_rad);
  result.candidate_completion_distance =
      domain::Distance(source.candidate_completion_distance_m);
  result.cue_similarity_radius =
      domain::Distance(source.cue_similarity_radius_m);
  result.passage_grid_resolution =
      domain::Distance(source.passage_grid_resolution_m);
  result.minimum_bundle_beams = source.minimum_bundle_beams;
  result.left_focus = {domain::Angle(source.left_focus_min_rad),
                       domain::Angle(source.left_focus_max_rad)};
  result.right_focus = {domain::Angle(source.right_focus_min_rad),
                        domain::Angle(source.right_focus_max_rad)};
  result.left_open = {domain::Angle(source.left_open_min_rad),
                      domain::Angle(source.left_open_max_rad)};
  result.right_open = {domain::Angle(source.right_open_min_rad),
                       domain::Angle(source.right_open_max_rad)};
  result.minimum_length_to_width_ratio = source.minimum_length_to_width_ratio;
  result.minimum_passage_length =
      domain::Distance(source.minimum_passage_length_m);
  result.large_room_width = domain::Distance(source.large_room_width_m);
  result.large_room_length = domain::Distance(source.large_room_length_m);
  result.cue_clearance_margin = domain::Distance(source.cue_clearance_margin_m);
  result.maximum_width_change_ratio = source.maximum_width_change_ratio;
  result.hard_turn_threshold = domain::Angle(source.hard_turn_threshold_rad);
  result.end_of_passage_clearance =
      domain::Distance(source.end_of_passage_clearance_m);
  result.minimum_extension = domain::Distance(source.minimum_extension_m);
  result.time_budget = std::chrono::duration<double>(source.time_limit_s);
  result.decision_budget = source.decision_budget;
  return result;
}

/**
 * @brief Performs the lle configuration operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `planning::LowLevelExplorationConfiguration` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
planning::LowLevelExplorationConfiguration lleConfiguration(
    const config::Configuration& configuration) {
  planning::LowLevelExplorationConfiguration result;
  const auto& policy =
      configuration.experiment.reactive_exploration_behavior_policy;
  result.behavior_policy =
      policy == "compatibility" ||
              (policy == "profile" && configuration.experiment.behavior_mode ==
                                          config::BehaviorMode::Compatibility)
          ? planning::LLEBehaviorPolicy::Compatibility
          : planning::LLEBehaviorPolicy::Modernized;
  result.stalled_history_extension =
      result.behavior_policy == planning::LLEBehaviorPolicy::Modernized &&
      configuration.experiment.reactive_exploration_stalled_history_extension;
  result.closest_target_bin_m =
      configuration.experiment.reactive_exploration_closest_target_bin_m;
  result.random_seed = configuration.experiment.seeds.lle_fallback;
  return result;
}

/**
 * @brief Performs the arbitration configuration operation for this
 * subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `decision::ArbitrationConfiguration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
decision::ArbitrationConfiguration arbitrationConfiguration(
    const config::Configuration& configuration) {
  decision::ArbitrationConfiguration result;
  result.tie_tolerance = configuration.experiment.tier_three_tie_tolerance;
  result.unscored_policy = decision::UnscoredActionPolicy::Exclude;
  result.fallback = domain::Action::pause();
  result.random_seed = configuration.experiment.seeds.tier_three_ties;
  const bool compatibility_profile = configuration.experiment.behavior_mode ==
                                     config::BehaviorMode::Compatibility;
  const auto& scoring = configuration.experiment.tier_three_scoring_policy;
  if (scoring == "compatibility_comments" ||
      (scoring == "profile" && compatibility_profile))
    result.scoring_policy =
        decision::TierThreeScoringPolicy::CompatibilityComments;
  else if (scoring == "weighted_normalized" || scoring == "profile")
    result.scoring_policy =
        decision::TierThreeScoringPolicy::WeightedNormalized;
  else
    throw std::runtime_error("unknown Tier-3 scoring policy '" + scoring + "'");
  const auto& tie = configuration.experiment.tier_three_tie_policy;
  if (tie == "exact" || (tie == "profile" && compatibility_profile))
    result.tie_policy = decision::TierThreeTiePolicy::Exact;
  else if (tie == "tolerance" || tie == "profile")
    result.tie_policy = decision::TierThreeTiePolicy::Tolerance;
  else
    throw std::runtime_error("unknown Tier-3 tie policy '" + tie + "'");
  result.circumstance_weighting_enabled =
      configuration.navigation.circumstances.tier_three_weighting_enabled;
  result.circumstance_minimum_evidence =
      configuration.navigation.circumstances.minimum_case_evidence;
  result.circumstance_minimum_action_evidence =
      configuration.navigation.circumstances.minimum_action_evidence;
  result.circumstance_minimum_assignment_confidence =
      configuration.navigation.circumstances.assignment_confidence_threshold;
  result.circumstance_minimum_case_accuracy =
      configuration.navigation.circumstances.accuracy_threshold;
  result.circumstance_maximum_influence =
      configuration.navigation.circumstances.tier_three_maximum_influence;
  return result;
}

}  // namespace

/**
 * @brief Encapsulates navigation engine adapter state and behavior for this
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
class NavigationEngineAdapter::Impl {
 public:
  /**
   * @brief Performs the impl operation for this subsystem.
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
  explicit Impl(config::Configuration configuration)
      : configuration_(std::move(configuration)),
        action_space_(configuration_.navigation.move_actions,
                      configuration_.navigation.rotate_actions),
        world_(makeWorld(configuration_)),
        decisions_(arbitrationConfiguration(configuration_)),
        mission_(world_.mission),
        learning_(spatial::SpatialLearningCoordinator::defaults(
            10U, circumstanceConfiguration(configuration_),
            {static_cast<std::uint16_t>(
                 configuration_.navigation.grids.free_observations_to_clear),
             configuration_.navigation.grids.dynamic_expiry_observations, true},
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
    const auto& circumstances = configuration_.navigation.circumstances;
    map_diagnostics_.push_back("circumstance_learning_mode:" +
                               circumstances.learning_mode);
    map_diagnostics_.push_back("circumstance_model_version:" +
                               circumstances.model_version);
    map_diagnostics_.push_back("circumstance_classifier_version:" +
                               circumstances.classifier_version);
    map_diagnostics_.push_back(
        std::string("circumstance_tier3_weighting:") +
        (circumstances.tier_three_weighting_enabled ? "enabled" : "disabled"));
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
            ? std::unique_ptr<planning::ReactivePlanner>(
                  std::make_unique<planning::LowLevelExplorer>(
                      lleConfiguration(configuration_)))
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
    const auto recorded_manifest = manifest;
    engine_ = std::make_unique<decision::NavigationEngine>(
        world_, action_space_, decisions_, mission_, planning_, learning_,
        crowd_learning_.get(), domain::Distance(0.5), &hard_safety_, &phases_,
        config::configurationFingerprint(configuration_), std::move(manifest),
        std::move(enabled_reactive),
        configuration_.experiment.reactive_exploration_enabled &&
            has_tier_one_rule("low_level_exploration"),
        has_tier_one_rule("enforcer"), hleConfiguration(configuration_),
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
                configuration_.navigation.grids.unknown_cost_multiplier)},
        configuration_.experiment.tiers.maximum_planning_attempts_per_task);
    configureReproducibility(recorded_manifest);
    validateRuntimeActivation();
  }

  /**
   * @brief Performs the configure reproducibility operation for this
   * subsystem.
   *
   * Arguments:
   * - @p component_manifest: Supplies component manifest input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void configureReproducibility(
      const std::vector<std::string>& component_manifest) {
    const auto& reproducibility = configuration_.experiment.reproducibility;
    const auto& seeds = configuration_.experiment.seeds;
    map_diagnostics_.push_back(
        "random_seeds:tier3=" + std::to_string(seeds.tier_three_ties) +
        ",lle=" + std::to_string(seeds.lle_fallback) +
        ",planner=" + std::to_string(seeds.planner_ties) +
        ",clustering=" + std::to_string(seeds.clustering) +
        ",simulation=" + std::to_string(seeds.simulation_noise));
    map_diagnostics_.push_back(
        std::string("replay_recording:") +
        (reproducibility.recording_enabled ? "enabled" : "disabled"));
    if (!reproducibility.recording_enabled) return;
    validation::RunMetadata metadata;
    metadata.configuration_snapshot =
        config::configurationSnapshot(configuration_);
    metadata.configuration_fingerprint =
        config::configurationFingerprint(configuration_);
    metadata.behavior_mode =
        std::string(config::toString(configuration_.experiment.behavior_mode));
    metadata.profile =
        std::string(config::toString(configuration_.experiment.profile));
    metadata.map_checksum =
        world_.static_map ? world_.static_map->checksum : "mapless";
    metadata.source_revision = reproducibility.source_revision;
    metadata.test_suite_revision = reproducibility.test_suite_revision;
    metadata.model_versions = {
        "circumstance:" + configuration_.navigation.circumstances.model_version,
        "classifier:" +
            configuration_.navigation.circumstances.classifier_version,
        "features:" + configuration_.navigation.circumstances.feature_version};
    metadata.component_manifest = component_manifest;
    for (const auto& task : configuration_.tasks)
      metadata.task_sequence.push_back({task.x, task.y});
    if (configuration_.experiment.behavior_mode ==
        config::BehaviorMode::Modernized) {
      metadata.compatibility_deviations = {
          "behavior_mode:modernized",
          "hard_safety:non_ablatable_engineering_extension",
          "see:docs/compatibility-matrix.md"};
    }
    metadata.seeds = {seeds.tier_three_ties, seeds.lle_fallback,
                      seeds.planner_ties, seeds.clustering,
                      seeds.simulation_noise};
    recorder_ = std::make_unique<validation::RunRecorder>(
        std::move(metadata), reproducibility.trace_path);
  }

  /**
   * @brief Validates runtime activation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void validateRuntimeActivation() {
    const auto& planners = configuration_.navigation.planners;
    const bool planner_requested =
        configuration_.experiment.tiers.tier_two &&
        (planners.distance || planners.sensor_distance || planners.density ||
         planners.risk || planners.flow || planners.region ||
         planners.hallway || planners.trail || planners.conveyor ||
         planners.skeleton || planners.highway);
    if (planner_requested && planning_.plannerCount() == 0U) {
      const bool explicitly_disabled =
          std::any_of(map_diagnostics_.begin(), map_diagnostics_.end(),
                      [](const std::string& diagnostic) {
                        return diagnostic.starts_with("planner_disabled_");
                      });
      if (explicitly_disabled || configuration_.static_map.failure_policy ==
                                     config::MapLoadFailurePolicy::DisableMap) {
        map_diagnostics_.push_back(
            "runtime_validation:all_requested_planners_disabled");
      } else {
        throw std::runtime_error(
            "runtime validation: planners.enabled requested components but "
            "none registered against an available representation; startup "
            "will stop");
      }
    } else {
      map_diagnostics_.push_back("runtime_validation:active_planners=" +
                                 std::to_string(planning_.plannerCount()));
    }
    map_diagnostics_.push_back("runtime_validation:explanations=" +
                               configuration_.experiment.explanations.mode);
    map_diagnostics_.push_back("runtime_validation:passed");
  }

  /**
   * @brief Performs the configure static map operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void configureStaticMap() {
    if (configuration_.static_map.mode == config::MapOperatingMode::Mapless) {
      map_diagnostics_.push_back("map_mode:mapless");
      map_diagnostics_.push_back("map_status:mapless_parser_not_invoked");
      map_diagnostics_.push_back(
          "map_capabilities:geometry=false,occupancy=false,planning=false");
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
      map_diagnostics_.push_back("map_checksum:" + world_.static_map->checksum);
      map_diagnostics_.push_back(
          std::string("map_grid_extent_source:") +
          domain::toString(
              world_.static_map->occupancy.geometry.extent_source));
      map_diagnostics_.push_back(
          "map_grid_geometry_revision:" +
          std::to_string(
              world_.static_map->occupancy.geometry.geometry_revision));
      map_diagnostics_.push_back(
          std::string(
              "map_capabilities:geometry=true,occupancy=true,planning=") +
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
      map_diagnostics_.push_back(
          "map_capabilities:geometry=false,occupancy=false,planning=false");
      map_diagnostics_.push_back("map_error:" + std::string(error.what()));
    }
  }

  /**
   * @brief Performs the configure learning operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void configureLearning() {
    const auto& features = configuration_.navigation;
    const bool skeleton_advisor =
        configuration_.experiment.tiers.tier_three &&
        std::any_of(configuration_.advisors.begin(),
                    configuration_.advisors.end(), [](const auto& advisor) {
                      return advisor.active && (advisor.name == "unlikely" ||
                                                advisor.name == "least_angle");
                    });
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
    learning_.setEnabled(
        spatial::SpatialRepresentation::PassagesAndSkeleton,
        features.a_star_on || features.planners.skeleton ||
            features.planners.highway || skeleton_advisor ||
            features.planners.region || features.planners.hallway ||
            features.planners.trail || features.planners.conveyor);
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

  /**
   * @brief Performs the configure planning operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void configurePlanning() {
    if (!configuration_.experiment.tiers.tier_two) return;
    const auto& planners = configuration_.navigation.planners;
    planning_.setSelectionPolicy(
        planning::planSelectionPolicyFromString(planners.selection_policy));
    planning_.setTiePolicy(planners.tie_policy == "seeded_exact" ||
                               (planners.tie_policy == "profile" &&
                                configuration_.experiment.behavior_mode ==
                                    config::BehaviorMode::Compatibility),
                           configuration_.experiment.seeds.planner_ties);
    const auto registry = planning::defaultPlannerRegistry();
    const std::vector<std::pair<std::string, bool>> enabled = {
        {"distance", planners.distance},
        {"sensor_distance", planners.sensor_distance},
        {"density", planners.density},
        {"risk", planners.risk},
        {"flow", planners.flow},
        {"region", planners.region},
        {"hallway", planners.hallway},
        {"trail", planners.trail},
        {"conveyor", planners.conveyor},
        {"skeleton", planners.skeleton},
        {"highway", planners.highway}};
    for (const auto& [name, on] : enabled) {
      if (!on) continue;
      if (registry.mapRequirement(name) ==
              planning::StaticMapRequirement::Required &&
          !world_.map_capabilities.map_based_planning_available) {
        map_diagnostics_.push_back("planner_disabled_no_map:" + name);
        continue;
      }
      planning_.registerPlanner(registry.create(name));
      const auto occupancy = registry.occupancyRequirement(name);
      const bool waits_for_sensor =
          occupancy == planning::OccupancyRequirement::SensedPartial ||
          (occupancy == planning::OccupancyRequirement::StaticOrSensedPartial &&
           !world_.map_capabilities.map_occupancy_available);
      map_diagnostics_.push_back(
          waits_for_sensor
              ? "planner_registered_dormant_until_sensed_occupancy:" + name
              : "planner_enabled:" + name);
    }
  }

  /**
   * @brief Performs the configure decisions operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
    for (const auto& diagnostic : configuration_.dependency_diagnostics)
      map_diagnostics_.push_back(diagnostic);
    if (configuration_.experiment.tiers.tier_one) {
      for (const auto& rule : configuration_.experiment.tiers.tier_one_rules) {
        if (rule == "precedent" &&
            !configuration_.navigation.circumstances.precedent_veto_enabled)
          continue;
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
  std::unique_ptr<validation::RunRecorder> recorder_;
};

/**
 * @brief Performs the navigation engine adapter operation for this
 * subsystem.
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
NavigationEngineAdapter::NavigationEngineAdapter(
    config::Configuration configuration)
    : impl_(std::make_unique<Impl>(std::move(configuration))) {}

/**
 * @brief Performs the navigation engine adapter operation for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
NavigationEngineAdapter::~NavigationEngineAdapter() = default;
/**
 * @brief Performs the navigation engine adapter operation for this
 * subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
NavigationEngineAdapter::NavigationEngineAdapter(
    NavigationEngineAdapter&&) noexcept = default;
/**
 * @brief Performs the operator operation for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 *
 * Returns:
 * - `NavigationEngineAdapter& NavigationEngineAdapter::` containing the
 * operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
NavigationEngineAdapter& NavigationEngineAdapter::operator=(
    NavigationEngineAdapter&&) noexcept = default;

/**
 * @brief Processes package content for this subsystem.
 *
 * Arguments:
 * - @p sensors: Supplies sensors input to the operation.
 * - @p crowd: Supplies crowd input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void NavigationEngineAdapter::observe(const SynchronizedSensors& sensors,
                                      const domain::CrowdState& crowd) {
  domain::RobotObservation observation{sensors.pose, sensors.scan,
                                       crowd.current(),
                                       std::chrono::steady_clock::now()};
  if (impl_->recorder_) {
    const auto scan_timestamp = sensors.scan_stamp.nanoseconds();
    impl_->recorder_->recordObservation(
        observation,
        static_cast<std::uint64_t>(std::max<std::int64_t>(0, scan_timestamp)));
  }
  impl_->engine_->observe(observation);
}

/**
 * @brief Performs the mission complete operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool NavigationEngineAdapter::missionComplete() {
  return impl_->engine_->missionComplete();
}

/**
 * @brief Performs the phase operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `navigation::NavigationPhase` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
navigation::NavigationPhase NavigationEngineAdapter::phase() const noexcept {
  return impl_->engine_->phase();
}

/**
 * @brief Selects package content for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `decision::DecisionResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
decision::DecisionResult NavigationEngineAdapter::decide() {
  auto result = impl_->engine_->decide();
  if (impl_->recorder_) {
    auto revisions = impl_->world_.spatial.revisions;
    for (const auto dependency : {domain::ModelDependency::LiveCrowdObservation,
                                  domain::ModelDependency::CrowdDensity,
                                  domain::ModelDependency::CrowdRisk,
                                  domain::ModelDependency::CrowdFlow})
      revisions[dependency] = impl_->world_.crowd.revisionOf(dependency);
    impl_->recorder_->recordDecision(result, revisions);
  }
  return result;
}

/**
 * @brief Performs the execution request operation for this subsystem.
 *
 * Arguments:
 * - @p decision: Supplies decision input to the operation.
 *
 * Returns:
 * - `ActionExecutionRequest` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the on action started operation for this subsystem.
 *
 * Arguments:
 * - @p update: Supplies update input to the operation.
 *
 * Returns:
 * - `domain::FeedbackDisposition` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::FeedbackDisposition NavigationEngineAdapter::onActionStarted(
    const ActionExecutionUpdate& update) {
  return impl_->engine_->onActionStarted({update.decision_id, update.action_id,
                                          std::chrono::steady_clock::now(),
                                          update.start_pose});
}

/**
 * @brief Performs the on action progress operation for this subsystem.
 *
 * Arguments:
 * - @p update: Supplies update input to the operation.
 *
 * Returns:
 * - `domain::FeedbackDisposition` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::FeedbackDisposition NavigationEngineAdapter::onActionProgress(
    const ActionExecutionUpdate& update) {
  return impl_->engine_->onActionProgress(
      {update.decision_id, update.action_id, std::chrono::steady_clock::now(),
       update.final_pose, update.distance_achieved_m,
       update.rotation_achieved_rad});
}

/**
 * @brief Performs the on action terminal operation for this subsystem.
 *
 * Arguments:
 * - @p update: Supplies update input to the operation.
 * - @p status: Supplies status input to the operation.
 * - @p detail: Supplies detail input to the operation.
 *
 * Returns:
 * - `domain::FeedbackDisposition` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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
  if (impl_->recorder_) impl_->recorder_->recordControllerOutcome(result);
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

/**
 * @brief Performs the on controller restart operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 *
 * Returns:
 * - `domain::FeedbackDisposition` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::FeedbackDisposition NavigationEngineAdapter::onControllerRestart(
    const domain::Pose2D& pose) {
  return impl_->engine_->onControllerRestart(std::chrono::steady_clock::now(),
                                             pose);
}

/**
 * @brief Performs the world model operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `const domain::WorldModel&` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const domain::WorldModel& NavigationEngineAdapter::worldModel() const noexcept {
  return impl_->world_;
}

/**
 * @brief Performs the startup diagnostics operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `const std::vector<std::string>&` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const std::vector<std::string>& NavigationEngineAdapter::startupDiagnostics()
    const noexcept {
  return impl_->map_diagnostics_;
}

}  // namespace semaforr::ros
