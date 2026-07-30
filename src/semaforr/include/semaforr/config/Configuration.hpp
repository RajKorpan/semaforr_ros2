#ifndef SEMAFORR_CONFIG_CONFIGURATION_HPP
#define SEMAFORR_CONFIG_CONFIGURATION_HPP

#include <array>
#include <string>
#include <vector>

namespace semaforr {
namespace config {

struct PlannerConfiguration {
  bool distance = false;
  bool density = false;
  bool risk = false;
  bool flow = false;
  bool skeleton = false;
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

  PlannerConfiguration planners;
  CrowdLearningConfiguration crowd_learning;
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
  NavigationConfiguration navigation;
  MapDimensions map_dimensions;
  std::vector<AdvisorConfiguration> advisors;
  std::vector<TaskConfiguration> tasks;
  std::string map_file;
};

Configuration loadStructuredConfiguration(
    NavigationConfiguration navigation, MapDimensions map_dimensions,
    std::vector<AdvisorConfiguration> advisors, const std::string& tasks_file,
    const std::string& map_file);
void validateConfiguration(const Configuration& configuration);

}  // namespace config
}  // namespace semaforr

#endif  // SEMAFORR_CONFIG_CONFIGURATION_HPP
