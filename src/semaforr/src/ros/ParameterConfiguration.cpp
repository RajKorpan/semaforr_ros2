#include <array>
#include <rclcpp/rclcpp.hpp>
#include <semaforr/ros/ParameterConfiguration.hpp>
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
  else if (name == "skeleton")
    planners.skeleton = true;
  else
    throw std::runtime_error("unknown planner name '" + name + "'");
}

}  // namespace

void declareConfigurationParameters(rclcpp::Node& node) {
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
  for (const std::string feature : {"trails", "conveyors", "regions", "doors",
                                    "hallways", "barriers", "astar"}) {
    const bool default_value = feature == "trails" || feature == "conveyors" ||
                               feature == "regions" || feature == "doors";
    node.declare_parameter("features." + feature, default_value);
  }

  node.declare_parameter("planners.enabled", std::vector<std::string>{});
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
  const auto enabled_planners =
      node.get_parameter("planners.enabled").as_string_array();
  for (const std::string& planner : enabled_planners) {
    applyPlanner(navigation.planners, planner);
  }

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

  return config::loadStructuredConfiguration(std::move(navigation), dimensions,
                                             std::move(advisors), tasks_file,
                                             map_file);
}

}  // namespace semaforr::ros
