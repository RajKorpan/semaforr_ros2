/**
 * @file navigation_configuration.hpp
 * @brief Navigation configuration responsibilities.
 *
 * @details This file defines navigation configuration behavior for runtime
 * configuration and reproducible experiment setup. It centers on
 * `BehaviorMode`, `SpatialLearningProfile`, `MapOperatingMode`,
 * `MapLoadFailurePolicy`, `StaticMapConfiguration`, `AblationProfile`,
 * `TierConfiguration`, `SafetyEnvelopeConfiguration`. Its package-relative
 * location is `include/semaforr/config/navigation_configuration.hpp`.
 */
#ifndef SEMAFORR_CONFIG_CONFIGURATION_HPP
#define SEMAFORR_CONFIG_CONFIGURATION_HPP

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr {
namespace config {

/**
 * @brief Enumerates the supported behavior mode values used by this
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
enum class BehaviorMode { Compatibility, Modernized };

// Component-scoped selection.  This does not claim whole-system behavioral
// compatibility; it permits validating the Chapter 3 learning pipeline while
// unrelated compatibility-mode blockers remain fail-closed.
/**
 * @brief Enumerates the supported spatial learning profile values used by
 * this subsystem.
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
enum class SpatialLearningProfile { Modernized, Chapter3Compatibility };

/**
 * @brief Enumerates the supported map operating mode values used by this
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
enum class MapOperatingMode { Mapless, MapEnabled };
/**
 * @brief Enumerates the supported map load failure policy values used by
 * this subsystem.
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
enum class MapLoadFailurePolicy { FailStartup, DisableMap };

/**
 * @brief Encapsulates static map configuration state and behavior for this
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
struct StaticMapConfiguration {
  MapOperatingMode mode = MapOperatingMode::Mapless;
  MapLoadFailurePolicy failure_policy = MapLoadFailurePolicy::FailStartup;
  std::string path;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  double occupancy_resolution_m = 0.3;
  double obstacle_inflation_m = 0.0;
  bool map_based_planning_enabled = true;
  bool visualizations_enabled = false;
  std::string bounds_policy = "require_declared";
  double inferred_bounds_padding_m = 1.0;
};

/**
 * @brief Enumerates the supported ablation profile values used by this
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
enum class AblationProfile {
  Full,
  TierOneOnly,
  TierOneTierThree,
  TierThreeOnly,
  TierOneTierTwoTierThree,
  NoInitialExploration,
  NoOpportunisticExploration,
  NoSpatialModel,
  NoSocial,
  PurelyReactive,
  Original,
  Doors,
  LeastAngle,
  Access,
  Tentative,
  Hallways,
  ShortestPath,
  CostGraph,
  Wander,
  Deliberator,
  ForwardOnly,
  GlobalExploration,
  LocalExploration,
  Highway,
  Circumstances,
  Naive,
  Custom
};

/**
 * @brief Encapsulates tier configuration state and behavior for this
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
struct TierConfiguration {
  bool tier_one = true;
  bool tier_two = true;
  bool tier_three = true;
  std::vector<std::string> tier_one_rules{
      "victory", "avoid_obstacles", "not_opposite", "enforcer",
      "thru",    "behind",          "out",          "low_level_exploration",
      "forward", "precedent"};
  std::vector<std::string> reactive_planners{"thru", "behind", "out",
                                             "low_level_exploration"};
  std::size_t maximum_planning_attempts_per_task = 3U;
};

/**
 * @brief Encapsulates safety envelope configuration state and behavior for
 * this subsystem.
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
struct SafetyEnvelopeConfiguration {
  bool enabled = true;
  double sensor_freshness_timeout_s = 0.5;
};

/**
 * @brief Encapsulates initial exploration configuration state and behavior
 * for this subsystem.
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
struct InitialExplorationConfiguration {
  bool enabled = false;
  std::size_t observation_budget = 0U;
  std::string strategy = "hle";
  std::string behavior_policy = "profile";
  double time_limit_s = 1200.0;
  std::size_t decision_budget = 10000U;
  double minimum_clearance_m = 0.8;
  double heading_tolerance_rad = 0.2;
  double candidate_completion_distance_m = 0.1;
  double cue_similarity_radius_m = 0.5;
  double passage_grid_resolution_m = 0.5;
  std::size_t minimum_bundle_beams = 1U;
  double left_focus_min_rad = 0.6544984694978736;
  double left_focus_max_rad = 0.9162978572970231;
  double right_focus_min_rad = -0.9162978572970231;
  double right_focus_max_rad = -0.6544984694978736;
  double left_open_min_rad = 0.0;
  double left_open_max_rad = 1.5707963267948966;
  double right_open_min_rad = -1.5707963267948966;
  double right_open_max_rad = 0.0;
  double minimum_length_to_width_ratio = 1.5;
  double minimum_passage_length_m = 1.0;
  double large_room_width_m = 3.0;
  double large_room_length_m = 3.0;
  double cue_clearance_margin_m = 0.05;
  double maximum_width_change_ratio = 0.35;
  double hard_turn_threshold_rad = 0.7853981633974483;
  double end_of_passage_clearance_m = 0.8;
  double minimum_extension_m = 0.25;
};

/**
 * @brief Encapsulates target navigation configuration state and behavior
 * for this subsystem.
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
struct TargetNavigationConfiguration {
  bool enabled = true;
};

/**
 * @brief Encapsulates social configuration state and behavior for this
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
struct SocialConfiguration {
  bool enabled = true;
  bool observations = true;
  bool learning = true;
  bool advisors = true;
  bool planners = true;
};

/**
 * @brief Encapsulates random seed configuration state and behavior for this
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
struct RandomSeedConfiguration {
  unsigned int tier_three_ties = 0U;
  unsigned int lle_fallback = 0U;
  unsigned int planner_ties = 0U;
  unsigned int clustering = 0U;
  unsigned int simulation_noise = 0U;
};

/**
 * @brief Encapsulates reproducibility configuration state and behavior for
 * this subsystem.
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
struct ReproducibilityConfiguration {
  bool recording_enabled = false;
  std::string trace_path;
  std::string source_revision{"unknown"};
  std::string test_suite_revision{"unknown"};
};

/**
 * @brief Encapsulates explanation configuration state and behavior for this
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
struct ExplanationConfiguration {
  std::string mode{"why"};
  bool retain_candidate_plans = true;
};

/**
 * @brief Encapsulates experiment configuration state and behavior for this
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
struct ExperimentConfiguration {
  BehaviorMode behavior_mode = BehaviorMode::Modernized;
  AblationProfile profile = AblationProfile::Custom;
  unsigned int random_seed = 0U;
  RandomSeedConfiguration seeds;
  ReproducibilityConfiguration reproducibility;
  ExplanationConfiguration explanations;
  std::string tier_three_scoring_policy = "profile";
  std::string tier_three_tie_policy = "profile";
  double tier_three_tie_tolerance = 1.0e-9;
  TierConfiguration tiers;
  InitialExplorationConfiguration initial_exploration;
  TargetNavigationConfiguration target_navigation;
  bool reactive_exploration_enabled = true;
  std::string reactive_exploration_strategy = "lle";
  std::string reactive_exploration_behavior_policy = "profile";
  bool reactive_exploration_stalled_history_extension = true;
  double reactive_exploration_closest_target_bin_m = 1.0;
  SocialConfiguration social;
  SafetyEnvelopeConfiguration safety_envelope;
  // Backward-compatible mirrors populated by parameter loading.
  bool opportunistic_exploration = false;
  bool social_enabled = true;
};

/**
 * @brief Encapsulates planner configuration state and behavior for this
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
struct PlannerConfiguration {
  bool distance = false;
  bool sensor_distance = false;
  bool density = false;
  bool risk = false;
  bool flow = false;
  bool region = false;
  bool hallway = false;
  bool trail = false;
  bool conveyor = false;
  bool skeleton = false;
  bool highway = false;
  std::string selection_policy = "range_vote";
  std::string tie_policy = "profile";
};

/**
 * @brief Encapsulates grid layer configuration state and behavior for this
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
struct GridLayerConfiguration {
  std::string extent_policy = "expand";
  std::string frame_id = "map";
  double mapless_initial_width_m = 20.0;
  double mapless_initial_height_m = 20.0;
  double resolution_m = 0.5;
  double highway_origin_x_m = 0.0;
  double highway_origin_y_m = 0.0;
  std::string highway_smoothing_policy = "profile";
  std::string highway_component_selection_policy = "profile";
  double expansion_margin_m = 2.0;
  std::size_t expansion_increment_cells = 32U;
  double maximum_width_m = 0.0;
  double maximum_height_m = 0.0;
  std::size_t memory_limit_cells = 10'000'000U;
  std::size_t free_observations_to_clear = 3U;
  std::size_t dynamic_expiry_observations = 30U;
  std::string map_unknown_policy = "prohibited";
  std::string sensor_unknown_policy = "prohibited";
  double localization_uncertainty_m = 0.05;
  double turning_footprint_margin_m = 0.0;
  double dynamic_obstacle_margin_m = 0.10;
  double unknown_cost_multiplier = 8.0;
};

/**
 * @brief Encapsulates crowd learning configuration state and behavior for
 * this subsystem.
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
struct CrowdLearningConfiguration {
  bool enabled = true;
  std::string estimator = "count_exposure";
  std::string frame = "map";
  double resolution_m = 1.0;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  double discount_factor = 0.7;
  double minimum_update_period_s = 1.0;
  double encounter_radius_m = 1.0;
  double minimum_flow_speed_mps = 0.05;
  double confidence_exposures = 10.0;
  double cusum_increase = 4.0;
  double cusum_decrease = -3.0;
  double cusum_threshold = 10.0;
  unsigned int random_seed = 0U;
};

/**
 * @brief Encapsulates circumstance configuration state and behavior for
 * this subsystem.
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
struct CircumstanceConfiguration {
  std::string learning_mode = "adapted_threshold";
  double setting_resolution_m = 1.0;
  double setting_radius_m = 10.0;
  std::size_t minimum_cluster_size = 50U;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  std::size_t reclustering_threshold = 100U;
  std::size_t minimum_case_evidence = 10U;
  std::size_t minimum_action_evidence = 5U;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
  double partial_success_credit = 0.5;
  bool safety_interruption_is_negative_evidence = true;
  bool precedent_veto_enabled = true;
  bool tier_three_weighting_enabled = false;
  double tier_three_maximum_influence = 0.5;
  std::string persistence_policy = "session_only";
  std::string model_path;
  std::string model_version = "circumstance_case_v2";
  std::string classifier_version = "centroid_softmax_v1";
  std::string feature_version =
      "robot_centered_heading_normalized_freespace_v1";
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
};

/**
 * @brief Encapsulates navigation configuration state and behavior for this
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
struct NavigationConfiguration {
  int task_decision_limit = 0;
  double can_see_point_epsilon = 0.0;
  double laser_scan_radian_increment = 0.0;
  double robot_footprint = 0.0;
  double robot_footprint_buffer = 0.0;
  double max_laser_range = 0.0;
  double max_forward_action_buffer = 0.0;
  double max_forward_action_sweep_angle = 0.0;

  std::vector<double> move_actions;
  std::vector<double> rotate_actions;

  bool trails_on = false;
  bool conveyors_on = false;
  bool regions_on = false;
  bool doors_on = false;
  bool hallways_on = false;
  bool barriers_on = false;
  bool a_star_on = false;
  bool known_grid_on = true;
  bool sensed_occupancy_on = true;
  bool inclusion_grid_on = true;
  bool highways_on = false;
  bool circumstances_on = true;
  std::string loaded_highway_model;
  SpatialLearningProfile spatial_learning_profile =
      SpatialLearningProfile::Modernized;

  PlannerConfiguration planners;
  GridLayerConfiguration grids;
  CrowdLearningConfiguration crowd_learning;
  CircumstanceConfiguration circumstances;
};

/**
 * @brief Encapsulates map dimensions state and behavior for this subsystem.
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
struct MapDimensions {
  int length = 0;
  int height = 0;
  double granularity = 0.0;
};

/**
 * @brief Encapsulates advisor configuration state and behavior for this
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
struct AdvisorConfiguration {
  std::string name;
  std::string description;
  bool active = false;
  double weight = 0.0;
  std::array<double, 4> parameters{{0.0, 0.0, 0.0, 0.0}};
};

/**
 * @brief Encapsulates task configuration state and behavior for this
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
struct TaskConfiguration {
  double x = 0.0;
  double y = 0.0;
};

/**
 * @brief Encapsulates configuration state and behavior for this subsystem.
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
struct Configuration {
  ExperimentConfiguration experiment;
  NavigationConfiguration navigation;
  MapDimensions map_dimensions;
  std::vector<AdvisorConfiguration> advisors;
  std::vector<TaskConfiguration> tasks;
  std::string map_file;
  StaticMapConfiguration static_map;
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p profile: Supplies profile input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(AblationProfile profile) noexcept;
/**
 * @brief Performs the ablation profile from string operation for this
 * subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `AblationProfile` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
AblationProfile ablationProfileFromString(const std::string& value);
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
std::string_view toString(BehaviorMode mode) noexcept;
/**
 * @brief Performs the behavior mode from string operation for this
 * subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `BehaviorMode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
BehaviorMode behaviorModeFromString(const std::string& value);
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p profile: Supplies profile input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SpatialLearningProfile profile) noexcept;
/**
 * @brief Performs the spatial learning profile from string operation for
 * this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `SpatialLearningProfile` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SpatialLearningProfile spatialLearningProfileFromString(
    const std::string& value);
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
std::string_view toString(MapOperatingMode mode) noexcept;
/**
 * @brief Performs the map operating mode from string operation for this
 * subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `MapOperatingMode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
MapOperatingMode mapOperatingModeFromString(const std::string& value);
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(MapLoadFailurePolicy policy) noexcept;
/**
 * @brief Performs the map load failure policy from string operation for
 * this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `MapLoadFailurePolicy` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
MapLoadFailurePolicy mapLoadFailurePolicyFromString(const std::string& value);
/**
 * @brief Applies ablation profile for this subsystem.
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
void applyAblationProfile(Configuration& configuration);
/**
 * @brief Performs the configuration fingerprint operation for this
 * subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string configurationFingerprint(const Configuration& configuration);
/**
 * @brief Performs the configuration snapshot operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string configurationSnapshot(const Configuration& configuration);
/**
 * @brief Performs the component manifest operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `std::vector<std::string>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string> componentManifest(const Configuration& configuration);

/**
 * @brief Loads structured configuration for this subsystem.
 *
 * Arguments:
 * - @p navigation: Supplies navigation input to the operation.
 * - @p map_dimensions: Supplies map dimensions input to the operation.
 * - @p advisors: Supplies advisors input to the operation.
 * - @p tasks_file: Supplies tasks file input to the operation.
 * - @p map_file: Supplies map file input to the operation.
 *
 * Returns:
 * - `Configuration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
Configuration loadStructuredConfiguration(
    NavigationConfiguration navigation, MapDimensions map_dimensions,
    std::vector<AdvisorConfiguration> advisors, const std::string& tasks_file,
    const std::string& map_file);
/**
 * @brief Validates configuration for this subsystem.
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
void validateConfiguration(const Configuration& configuration);

}  // namespace config
}  // namespace semaforr

#endif  // SEMAFORR_CONFIG_CONFIGURATION_HPP
