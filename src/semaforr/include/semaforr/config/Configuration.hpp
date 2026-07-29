#ifndef SEMAFORR_CONFIG_CONFIGURATION_H
#define SEMAFORR_CONFIG_CONFIGURATION_H

#include <array>
#include <istream>
#include <string>
#include <vector>

namespace semaforr {
namespace config {

struct PlannerConfiguration {
  bool distance = false;
  bool smooth = false;
  bool novel = false;
  bool density = false;
  bool risk = false;
  bool flow = false;
  bool combined = false;
  bool cusum = false;
  bool discount = false;
  bool explore = false;
  bool spatial = false;
  bool hallwayer = false;
  bool trailer = false;
  bool barrier = false;
  bool conveys = false;
  bool safe = false;
  bool skeleton = false;
  bool hallway_skeleton = false;
};

struct ControllerConfiguration {
  int task_decision_limit = 0;
  int plan_limit = 0;

  double can_see_point_epsilon = 0.0;
  double laser_scan_radian_increment = 0.0;
  double robot_footprint = 0.0;
  double robot_footprint_buffer = 0.0;
  double max_laser_range = 0.0;
  double max_forward_action_buffer = 0.0;
  double max_forward_action_sweep_angle = 0.0;
  double highway_distance_threshold = 0.0;
  double highway_time_threshold = 0.0;
  double highway_decision_threshold = 0.0;

  std::vector<double> move_actions;
  std::vector<double> rotate_actions;

  bool trails_on = false;
  bool conveyors_on = false;
  bool regions_on = false;
  bool doors_on = false;
  bool hallways_on = false;
  bool barriers_on = false;
  bool a_star_on = false;
  bool highways_on = false;
  bool frontiers_on = false;
  bool out_of_here_on = false;
  bool doorway_on = false;
  bool find_a_way_on = false;
  bool behind_on = false;
  bool dont_go_back_on = false;

  PlannerConfiguration planners;
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

struct ConfigurationFiles {
  std::string advisors;
  std::string parameters;
  std::string map;
  std::string tasks;
  std::string dimensions;
};

struct Configuration {
  ControllerConfiguration controller;
  MapDimensions map_dimensions;
  std::vector<AdvisorConfiguration> advisors;
  std::vector<TaskConfiguration> tasks;
  std::string map_file;
};

ControllerConfiguration parseControllerConfiguration(
    std::istream& input, const std::string& source_name);
MapDimensions parseMapDimensions(
    std::istream& input, const std::string& source_name);
std::vector<AdvisorConfiguration> parseAdvisorConfigurations(
    std::istream& input, const std::string& source_name);
std::vector<TaskConfiguration> parseTaskConfigurations(
    std::istream& input, const std::string& source_name);
Configuration loadConfiguration(const ConfigurationFiles& files);
Configuration loadStructuredConfiguration(
    ControllerConfiguration controller,
    MapDimensions map_dimensions,
    std::vector<AdvisorConfiguration> advisors,
    const std::string& tasks_file,
    const std::string& map_file);
void validateConfiguration(const Configuration& configuration);

}  // namespace config
}  // namespace semaforr

#endif  // SEMAFORR_CONFIG_CONFIGURATION_H
