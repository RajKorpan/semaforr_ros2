#ifndef SEMAFORR_CONFIG_CONFIGURATION_HPP
#define SEMAFORR_CONFIG_CONFIGURATION_HPP

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr {
namespace config {

enum class BehaviorMode {
  Compatibility,
  Modernized
};

enum class MapOperatingMode { Mapless, MapEnabled };
enum class MapLoadFailurePolicy { FailStartup, DisableMap };

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
};

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
};

struct SafetyEnvelopeConfiguration {
  bool enabled = true;
  double sensor_freshness_timeout_s = 0.5;
};

struct InitialExplorationConfiguration {
  bool enabled = false;
  std::size_t observation_budget = 0U;
  std::string strategy = "hle";
  double time_limit_s = 1200.0;
  std::size_t decision_budget = 10000U;
  double minimum_clearance_m = 0.8;
  double heading_tolerance_rad = 0.2;
  double candidate_completion_distance_m = 0.1;
  double cue_similarity_radius_m = 0.5;
  double passage_grid_resolution_m = 0.5;
  std::size_t minimum_bundle_beams = 1U;
};

struct TargetNavigationConfiguration {
  bool enabled = true;
};

struct SocialConfiguration {
  bool enabled = true;
  bool observations = true;
  bool learning = true;
  bool advisors = true;
  bool planners = true;
};

struct ExperimentConfiguration {
  BehaviorMode behavior_mode = BehaviorMode::Modernized;
  AblationProfile profile = AblationProfile::Custom;
  unsigned int random_seed = 0U;
  TierConfiguration tiers;
  InitialExplorationConfiguration initial_exploration;
  TargetNavigationConfiguration target_navigation;
  bool reactive_exploration_enabled = true;
  std::string reactive_exploration_strategy = "lle";
  SocialConfiguration social;
  SafetyEnvelopeConfiguration safety_envelope;
  // Backward-compatible mirrors populated by parameter loading.
  bool opportunistic_exploration = false;
  bool social_enabled = true;
};

struct PlannerConfiguration {
  bool distance = false;
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
};

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

struct CircumstanceConfiguration {
  double setting_resolution_m = 1.0;
  double setting_radius_m = 10.0;
  std::size_t minimum_cluster_size = 50U;
  double assignment_confidence_threshold = 0.95;
  double similarity_l1_threshold = 125.0;
  std::size_t reclustering_threshold = 100U;
  std::size_t minimum_case_evidence = 10U;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
  double distance_bin_base_m = 2.0;
  std::size_t angle_bin_count = 8U;
};

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
  bool inclusion_grid_on = true;
  bool highways_on = false;
  bool circumstances_on = true;
  std::string loaded_highway_model;

  PlannerConfiguration planners;
  CrowdLearningConfiguration crowd_learning;
  CircumstanceConfiguration circumstances;
};

struct MapDimensions {
  int length = 0;
  int height = 0;
  double granularity = 0.0;
};

struct AdvisorConfiguration {
  std::string name;
  std::string description;
  bool active = false;
  double weight = 0.0;
  std::array<double, 4> parameters{{0.0, 0.0, 0.0, 0.0}};
};

struct TaskConfiguration {
  double x = 0.0;
  double y = 0.0;
};

struct Configuration {
  ExperimentConfiguration experiment;
  NavigationConfiguration navigation;
  MapDimensions map_dimensions;
  std::vector<AdvisorConfiguration> advisors;
  std::vector<TaskConfiguration> tasks;
  std::string map_file;
  StaticMapConfiguration static_map;
};

std::string_view toString(AblationProfile profile) noexcept;
AblationProfile ablationProfileFromString(const std::string& value);
std::string_view toString(BehaviorMode mode) noexcept;
BehaviorMode behaviorModeFromString(const std::string& value);
std::string_view toString(MapOperatingMode mode) noexcept;
MapOperatingMode mapOperatingModeFromString(const std::string& value);
std::string_view toString(MapLoadFailurePolicy policy) noexcept;
MapLoadFailurePolicy mapLoadFailurePolicyFromString(const std::string& value);
void applyAblationProfile(Configuration& configuration);
std::string configurationFingerprint(const Configuration& configuration);
std::vector<std::string> componentManifest(const Configuration& configuration);

Configuration loadStructuredConfiguration(
    NavigationConfiguration navigation, MapDimensions map_dimensions,
    std::vector<AdvisorConfiguration> advisors, const std::string& tasks_file,
    const std::string& map_file);
void validateConfiguration(const Configuration& configuration);

}  // namespace config
}  // namespace semaforr

#endif  // SEMAFORR_CONFIG_CONFIGURATION_HPP
