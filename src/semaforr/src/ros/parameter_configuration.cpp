#include <array>
#include <rclcpp/rclcpp.hpp>
#include <semaforr/ros/parameter_configuration.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace semaforr::ros {
namespace {

const std::vector<std::string>& defaultAdvisorNames() {
  static const std::vector<std::string> names = {
      "goal_progress",      "goal_progress_linear", "clearance",
      "clearance_rotation", "exploration",          "social_navigation",
      "crowd_avoid",        "risk_avoid",           "flow_follow"};
  return names;
}

const std::vector<double>& defaultAdvisorParameters() {
  static const std::vector<double> parameters(defaultAdvisorNames().size() * 4U,
                                              0.0);
  return parameters;
}

void applyPlanner(config::PlannerConfiguration& planners,
                  const std::string& name) {
  if (name == "distance")
    planners.distance = true;
  else if (name == "density")
    planners.density = true;
  else if (name == "risk")
    planners.risk = true;
  else if (name == "flow")
    planners.flow = true;
  else if (name == "region")
    planners.region = true;
  else if (name == "hallway")
    planners.hallway = true;
  else if (name == "trail")
    planners.trail = true;
  else if (name == "conveyor")
    planners.conveyor = true;
  else if (name == "skeleton")
    planners.skeleton = true;
  else if (name == "highway")
    planners.highway = true;
  else
    throw std::runtime_error("unknown planner name '" + name + "'");
}

}  // namespace

void declareConfigurationParameters(rclcpp::Node& node) {
  node.declare_parameter("experiment.profile", std::string{"custom"});
  node.declare_parameter("experiment.mode", std::string{"custom"});
  node.declare_parameter("experiment.random_seed", 0);
  node.declare_parameter("tiers.tier1.enabled", true);
  node.declare_parameter("tiers.tier2.enabled", true);
  node.declare_parameter("tiers.tier3.enabled", true);
  node.declare_parameter("tiers.tier1.rules",
                         config::TierConfiguration{}.tier_one_rules);
  node.declare_parameter("tiers.tier1.reactive_planners",
                         config::TierConfiguration{}.reactive_planners);
  node.declare_parameter("phases.initial_exploration.enabled", false);
  node.declare_parameter("phases.initial_exploration.observation_budget", 0);
  node.declare_parameter("phases.initial_exploration.strategy",
                         std::string{"hle"});
  node.declare_parameter("phases.initial_exploration.time_limit_s", 1200.0);
  node.declare_parameter("phases.initial_exploration.decision_budget", 10000);
  node.declare_parameter("phases.initial_exploration.minimum_clearance_m", 0.8);
  node.declare_parameter("phases.initial_exploration.heading_tolerance_rad",
                         0.2);
  node.declare_parameter(
      "phases.initial_exploration.candidate_completion_distance_m", 0.1);
  node.declare_parameter("phases.initial_exploration.cue_similarity_radius_m",
                         0.5);
  node.declare_parameter("phases.initial_exploration.passage_grid_resolution_m",
                         0.5);
  node.declare_parameter("phases.initial_exploration.minimum_bundle_beams", 1);
  node.declare_parameter("phases.target_navigation.enabled", true);
  node.declare_parameter("exploration.reactive.enabled", true);
  node.declare_parameter("exploration.reactive.strategy", std::string{"lle"});
  node.declare_parameter("exploration.opportunistic.enabled", false);
  node.declare_parameter("social.enabled", true);
  node.declare_parameter("social.observations.enabled", true);
  node.declare_parameter("social.advisors.enabled", true);
  node.declare_parameter("social.planners.enabled", true);
  node.declare_parameter("safety.command_envelope.enabled", true);
  node.declare_parameter("safety.sensor_freshness_timeout_s", 0.5);
  node.declare_parameter("map.path", std::string{});
  node.declare_parameter("mission.tasks_path", std::string{});
  node.declare_parameter("map.length_m", 200);
  node.declare_parameter("map.height_m", 200);
  node.declare_parameter("map.granularity_m", 0.3);

  node.declare_parameter("actions.move_distances_m",
                         std::vector<double>{0.1, 0.2, 0.4, 0.8, 1.6, 3.2});
  node.declare_parameter(
      "actions.rotation_angles_rad",
      std::vector<double>{0.0873, 0.2618, 0.5236, 0.7854, 1.0472, 1.5708});

  node.declare_parameter("mission.decision_limit", 20);
  node.declare_parameter("safety.visibility_epsilon_m", 0.005);
  node.declare_parameter("safety.laser_angle_increment_rad", 0.005817);
  node.declare_parameter("safety.robot_radius_m", 0.2794);
  node.declare_parameter("safety.obstacle_buffer_m", 0.05);
  node.declare_parameter("safety.max_laser_range_m", 5.0);
  node.declare_parameter("safety.max_forward_buffer_m", 0.1);
  node.declare_parameter("safety.max_forward_sweep_rad", 0.5236);
  for (const std::string feature :
       {"trails", "conveyors", "regions", "doors", "hallways", "barriers",
        "astar", "known_grid", "inclusion_grid", "highways", "circumstances"}) {
    const bool default_value = feature == "trails" || feature == "conveyors" ||
                               feature == "regions" || feature == "doors" ||
                               feature == "circumstances" ||
                               feature == "known_grid" ||
                               feature == "inclusion_grid";
    node.declare_parameter("features." + feature, default_value);
  }
  node.declare_parameter("features.loaded_highway_model", std::string{});
  node.declare_parameter("circumstances.setting_resolution_m", 1.0);
  node.declare_parameter("circumstances.setting_radius_m", 10.0);
  node.declare_parameter("circumstances.minimum_cluster_size", 50);
  node.declare_parameter("circumstances.assignment_confidence_threshold", 0.95);
  node.declare_parameter("circumstances.similarity_l1_threshold", 125.0);
  node.declare_parameter("circumstances.reclustering_threshold", 100);
  node.declare_parameter("circumstances.minimum_case_evidence", 10);
  node.declare_parameter("circumstances.accuracy_threshold", 0.75);
  node.declare_parameter("circumstances.action_confidence_threshold", 0.25);
  node.declare_parameter("circumstances.distance_bin_base_m", 2.0);
  node.declare_parameter("circumstances.angle_bin_count", 8);

  node.declare_parameter("planners.enabled", std::vector<std::string>{});
  node.declare_parameter("planners.selection_policy",
                         std::string{"range_vote"});
  node.declare_parameter("advisors.names", defaultAdvisorNames());
  node.declare_parameter("advisors.enabled",
                         std::vector<bool>(defaultAdvisorNames().size(), true));
  node.declare_parameter(
      "advisors.weights",
      std::vector<double>(defaultAdvisorNames().size(), 1.0));
  node.declare_parameter("advisors.parameters", defaultAdvisorParameters());
}

config::Configuration configurationFromParameters(rclcpp::Node& node) {
  std::string map_file = node.get_parameter("map.path").as_string();
  std::string tasks_file = node.get_parameter("mission.tasks_path").as_string();
  if (map_file.empty()) {
    throw std::runtime_error("map.path: required path is empty");
  }
  if (tasks_file.empty()) {
    throw std::runtime_error("mission.tasks_path: required path is empty");
  }

  config::NavigationConfiguration navigation;
  navigation.move_actions =
      node.get_parameter("actions.move_distances_m").as_double_array();
  navigation.rotate_actions =
      node.get_parameter("actions.rotation_angles_rad").as_double_array();
  navigation.task_decision_limit =
      static_cast<int>(node.get_parameter("mission.decision_limit").as_int());
  navigation.can_see_point_epsilon =
      node.get_parameter("safety.visibility_epsilon_m").as_double();
  navigation.laser_scan_radian_increment =
      node.get_parameter("safety.laser_angle_increment_rad").as_double();
  navigation.robot_footprint =
      node.get_parameter("safety.robot_radius_m").as_double();
  navigation.robot_footprint_buffer =
      node.get_parameter("safety.obstacle_buffer_m").as_double();
  navigation.max_laser_range =
      node.get_parameter("safety.max_laser_range_m").as_double();
  navigation.max_forward_action_buffer =
      node.get_parameter("safety.max_forward_buffer_m").as_double();
  navigation.max_forward_action_sweep_angle =
      node.get_parameter("safety.max_forward_sweep_rad").as_double();
  navigation.crowd_learning.enabled =
      node.get_parameter("social.learning.enabled").as_bool();
  navigation.crowd_learning.estimator =
      node.get_parameter("social.learning.estimator").as_string();
  navigation.crowd_learning.frame =
      node.get_parameter("frames.global").as_string();
  navigation.crowd_learning.resolution_m =
      node.get_parameter("social.learning.resolution_m").as_double();
  navigation.crowd_learning.origin_x_m =
      node.get_parameter("social.learning.origin_x_m").as_double();
  navigation.crowd_learning.origin_y_m =
      node.get_parameter("social.learning.origin_y_m").as_double();
  navigation.crowd_learning.discount_factor =
      node.get_parameter("social.learning.discount_factor").as_double();
  navigation.crowd_learning.minimum_update_period_s =
      node.get_parameter("social.learning.minimum_update_period_s").as_double();
  navigation.crowd_learning.encounter_radius_m =
      node.get_parameter("social.learning.encounter_radius_m").as_double();
  navigation.crowd_learning.minimum_flow_speed_mps =
      node.get_parameter("social.learning.minimum_flow_speed_mps").as_double();
  navigation.crowd_learning.confidence_exposures =
      node.get_parameter("social.learning.confidence_exposures").as_double();
  navigation.crowd_learning.cusum_increase =
      node.get_parameter("social.learning.cusum_increase").as_double();
  navigation.crowd_learning.cusum_decrease =
      node.get_parameter("social.learning.cusum_decrease").as_double();
  navigation.crowd_learning.cusum_threshold =
      node.get_parameter("social.learning.cusum_threshold").as_double();
  const auto crowd_seed =
      node.get_parameter("social.learning.random_seed").as_int();
  if (crowd_seed < 0) {
    throw std::runtime_error("social.learning.random_seed must be nonnegative");
  }
  navigation.crowd_learning.random_seed = static_cast<unsigned int>(crowd_seed);

  navigation.trails_on = node.get_parameter("features.trails").as_bool();
  navigation.conveyors_on = node.get_parameter("features.conveyors").as_bool();
  navigation.regions_on = node.get_parameter("features.regions").as_bool();
  navigation.doors_on = node.get_parameter("features.doors").as_bool();
  navigation.hallways_on = node.get_parameter("features.hallways").as_bool();
  navigation.barriers_on = node.get_parameter("features.barriers").as_bool();
  navigation.a_star_on = node.get_parameter("features.astar").as_bool();
  navigation.known_grid_on =
      node.get_parameter("features.known_grid").as_bool();
  navigation.inclusion_grid_on =
      node.get_parameter("features.inclusion_grid").as_bool();
  navigation.highways_on = node.get_parameter("features.highways").as_bool();
  navigation.circumstances_on =
      node.get_parameter("features.circumstances").as_bool();
  navigation.loaded_highway_model =
      node.get_parameter("features.loaded_highway_model").as_string();
  auto& circumstances = navigation.circumstances;
  circumstances.setting_resolution_m =
      node.get_parameter("circumstances.setting_resolution_m").as_double();
  circumstances.setting_radius_m =
      node.get_parameter("circumstances.setting_radius_m").as_double();
  circumstances.minimum_cluster_size = static_cast<std::size_t>(
      node.get_parameter("circumstances.minimum_cluster_size").as_int());
  circumstances.assignment_confidence_threshold = node.get_parameter(
      "circumstances.assignment_confidence_threshold").as_double();
  circumstances.similarity_l1_threshold = node.get_parameter(
      "circumstances.similarity_l1_threshold").as_double();
  circumstances.reclustering_threshold = static_cast<std::size_t>(
      node.get_parameter("circumstances.reclustering_threshold").as_int());
  circumstances.minimum_case_evidence = static_cast<std::size_t>(
      node.get_parameter("circumstances.minimum_case_evidence").as_int());
  circumstances.accuracy_threshold =
      node.get_parameter("circumstances.accuracy_threshold").as_double();
  circumstances.action_confidence_threshold = node.get_parameter(
      "circumstances.action_confidence_threshold").as_double();
  circumstances.distance_bin_base_m =
      node.get_parameter("circumstances.distance_bin_base_m").as_double();
  circumstances.angle_bin_count = static_cast<std::size_t>(
      node.get_parameter("circumstances.angle_bin_count").as_int());
  const auto enabled_planners =
      node.get_parameter("planners.enabled").as_string_array();
  for (const std::string& planner : enabled_planners) {
    applyPlanner(navigation.planners, planner);
  }
  navigation.planners.selection_policy =
      node.get_parameter("planners.selection_policy").as_string();

  const auto names = node.get_parameter("advisors.names").as_string_array();
  const auto enabled = node.get_parameter("advisors.enabled").as_bool_array();
  const auto weights = node.get_parameter("advisors.weights").as_double_array();
  const auto parameters =
      node.get_parameter("advisors.parameters").as_double_array();
  if (names.size() != enabled.size() || names.size() != weights.size() ||
      parameters.size() != names.size() * 4U) {
    throw std::runtime_error(
        "advisors arrays must have equal lengths and four parameters per "
        "advisor");
  }

  std::vector<config::AdvisorConfiguration> advisors;
  advisors.reserve(names.size());
  for (std::size_t index = 0; index < names.size(); ++index) {
    config::AdvisorConfiguration advisor;
    advisor.name = names[index];
    advisor.description = "ROS-parameter advisor";
    advisor.active = enabled[index];
    advisor.weight = weights[index];
    for (std::size_t parameter = 0; parameter < 4U; ++parameter) {
      advisor.parameters[parameter] = parameters[index * 4U + parameter];
    }
    advisors.push_back(std::move(advisor));
  }

  config::MapDimensions dimensions;
  dimensions.length =
      static_cast<int>(node.get_parameter("map.length_m").as_int());
  dimensions.height =
      static_cast<int>(node.get_parameter("map.height_m").as_int());
  dimensions.granularity = node.get_parameter("map.granularity_m").as_double();

  auto configuration = config::loadStructuredConfiguration(
      std::move(navigation), dimensions, std::move(advisors), tasks_file,
      map_file);
  const auto mode = node.get_parameter("experiment.mode").as_string();
  const auto legacy_profile =
      node.get_parameter("experiment.profile").as_string();
  if (mode != "custom" && legacy_profile != "custom" && mode != legacy_profile)
    throw std::runtime_error(
        "experiment.mode and deprecated experiment.profile conflict");
  configuration.experiment.profile = config::ablationProfileFromString(
      mode != "custom" ? mode : legacy_profile);
  configuration.experiment.tiers.tier_one =
      node.get_parameter("tiers.tier1.enabled").as_bool();
  configuration.experiment.tiers.tier_two =
      node.get_parameter("tiers.tier2.enabled").as_bool();
  configuration.experiment.tiers.tier_three =
      node.get_parameter("tiers.tier3.enabled").as_bool();
  configuration.experiment.tiers.tier_one_rules =
      node.get_parameter("tiers.tier1.rules").as_string_array();
  configuration.experiment.tiers.reactive_planners =
      node.get_parameter("tiers.tier1.reactive_planners").as_string_array();
  const auto experiment_seed =
      node.get_parameter("experiment.random_seed").as_int();
  if (experiment_seed < 0)
    throw std::runtime_error("experiment.random_seed must be nonnegative");
  configuration.experiment.random_seed =
      static_cast<unsigned int>(experiment_seed);
  configuration.experiment.initial_exploration.enabled =
      node.get_parameter("phases.initial_exploration.enabled").as_bool();
  const auto observation_budget =
      node.get_parameter("phases.initial_exploration.observation_budget")
          .as_int();
  if (observation_budget < 0) {
    throw std::runtime_error(
        "phases.initial_exploration.observation_budget must be nonnegative");
  }
  configuration.experiment.initial_exploration.observation_budget =
      static_cast<std::size_t>(observation_budget);
  configuration.experiment.initial_exploration.strategy =
      node.get_parameter("phases.initial_exploration.strategy").as_string();
  configuration.experiment.initial_exploration.time_limit_s =
      node.get_parameter("phases.initial_exploration.time_limit_s").as_double();
  const auto decision_budget =
      node.get_parameter("phases.initial_exploration.decision_budget").as_int();
  if (decision_budget < 0)
    throw std::runtime_error(
        "phases.initial_exploration.decision_budget must be nonnegative");
  configuration.experiment.initial_exploration.decision_budget =
      static_cast<std::size_t>(decision_budget);
  auto& hle = configuration.experiment.initial_exploration;
  hle.minimum_clearance_m =
      node.get_parameter("phases.initial_exploration.minimum_clearance_m")
          .as_double();
  hle.heading_tolerance_rad =
      node.get_parameter("phases.initial_exploration.heading_tolerance_rad")
          .as_double();
  hle.candidate_completion_distance_m =
      node.get_parameter(
              "phases.initial_exploration.candidate_completion_distance_m")
          .as_double();
  hle.cue_similarity_radius_m =
      node.get_parameter("phases.initial_exploration.cue_similarity_radius_m")
          .as_double();
  hle.passage_grid_resolution_m =
      node.get_parameter("phases.initial_exploration.passage_grid_resolution_m")
          .as_double();
  const auto minimum_bundle_beams =
      node.get_parameter("phases.initial_exploration.minimum_bundle_beams")
          .as_int();
  if (minimum_bundle_beams <= 0)
    throw std::runtime_error(
        "phases.initial_exploration.minimum_bundle_beams must be positive");
  hle.minimum_bundle_beams = static_cast<std::size_t>(minimum_bundle_beams);
  configuration.experiment.target_navigation.enabled =
      node.get_parameter("phases.target_navigation.enabled").as_bool();
  configuration.experiment.reactive_exploration_enabled =
      node.get_parameter("exploration.reactive.enabled").as_bool();
  configuration.experiment.reactive_exploration_strategy =
      node.get_parameter("exploration.reactive.strategy").as_string();
  configuration.experiment.opportunistic_exploration =
      node.get_parameter("exploration.opportunistic.enabled").as_bool();
  configuration.experiment.social_enabled =
      node.get_parameter("social.enabled").as_bool();
  configuration.experiment.social.enabled =
      configuration.experiment.social_enabled;
  configuration.experiment.social.observations =
      node.get_parameter("social.observations.enabled").as_bool();
  configuration.experiment.social.learning =
      node.get_parameter("social.learning.enabled").as_bool();
  configuration.experiment.social.advisors =
      node.get_parameter("social.advisors.enabled").as_bool();
  configuration.experiment.social.planners =
      node.get_parameter("social.planners.enabled").as_bool();
  configuration.experiment.safety_envelope.enabled =
      node.get_parameter("safety.command_envelope.enabled").as_bool();
  configuration.experiment.safety_envelope.sensor_freshness_timeout_s =
      node.get_parameter("safety.sensor_freshness_timeout_s").as_double();
  if (!configuration.experiment.social.enabled) {
    configuration.experiment.social.observations = false;
    configuration.experiment.social.learning = false;
    configuration.experiment.social.advisors = false;
    configuration.experiment.social.planners = false;
    configuration.navigation.crowd_learning.enabled = false;
  }
  config::applyAblationProfile(configuration);
  config::validateConfiguration(configuration);
  return configuration;
}

}  // namespace semaforr::ros
