/*
 * Controller construction and configuration.
 *
 */

#include <semaforr/decision/Controller.hpp>
#include <semaforr/core/FORRGeometry.hpp>
#include "DecisionTierFactory.h"
#include <unistd.h>

#include <algorithm>
#include <deque>
#include <iostream>
#include <math.h>
#include <stdexcept>
#include <time.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

#define CTRL_DEBUG true

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize advisors and weights and spatial learning modules based on the advisors
//
//
void Controller::initialize_advisors(
  const std::vector<semaforr::config::AdvisorConfiguration>& advisors) {
  for (const auto& advisor : advisors) {
    double parameters[4] = {
      advisor.parameters[0],
      advisor.parameters[1],
      advisor.parameters[2],
      advisor.parameters[3]
    };
    std::unique_ptr<Tier3Advisor> created = Tier3Advisor::makeAdvisor(
      getBeliefs(), advisor.name, advisor.description, advisor.weight,
      parameters, advisor.active);
    if (!created) {
      throw std::runtime_error(
        "unknown advisor type '" + advisor.name + "'");
    }
    tier3Advisors.push_back(std::move(created));
  }
}

void Controller::initialize_planner(
  const semaforr::config::MapDimensions& dimensions) {
  const int length = dimensions.length;
  const int height = dimensions.height;
  const double granularity = dimensions.granularity;
  Node node;

  if (skeleton) {
    std::unique_ptr<Graph> navigation_graph = std::make_unique<Graph>(
      static_cast<int>(granularity * 100.0), length * 100, height * 100);
    cout << "initialized nav skeleton graph" << endl;
    std::unique_ptr<PathPlanner> skeleton_planner =
      std::make_unique<PathPlanner>(
        std::move(navigation_graph), node, node, "skeleton");
    cout << "sk planner initialzied" << endl;
    if (planner == nullptr) {
      planner = skeleton_planner.get();
    }
    tier2Planners.push_back(std::move(skeleton_planner));
  }

  if (hallwayskel) {
    std::unique_ptr<Graph> navigation_graph = std::make_unique<Graph>(
      static_cast<int>(granularity * 100.0), length * 100, height * 100);
    cout << "initialized nav hallway skeleton graph" << endl;
    std::unique_ptr<PathPlanner> hallway_skeleton_planner =
      std::make_unique<PathPlanner>(
        std::move(navigation_graph), node, node, "hallwayskel");
    std::unique_ptr<Graph> original_navigation_graph =
      std::make_unique<Graph>(
        static_cast<int>(granularity * 100.0), length * 100, height * 100);
    hallway_skeleton_planner->setOriginalNavGraph(
      std::move(original_navigation_graph));
    if (planner == nullptr) {
      planner = hallway_skeleton_planner.get();
    }
    tier2Planners.push_back(std::move(hallway_skeleton_planner));
  }

  const std::vector<std::pair<std::string, bool>> crowd_cost_planners{
    {"density", plannerConfiguration.density},
    {"risk", plannerConfiguration.risk},
    {"flow", plannerConfiguration.flow},
    {"combined", plannerConfiguration.combined}};
  for (const auto& [name, enabled] : crowd_cost_planners) {
    if (!enabled) continue;
    auto navigation_graph = std::make_unique<Graph>(
      static_cast<int>(granularity * 100.0), length * 100, height * 100);
    auto cost_planner = std::make_unique<PathPlanner>(
      std::move(navigation_graph), node, node, name);
    if (planner == nullptr) {
      planner = cost_planner.get();
    }
    tier2Planners.push_back(std::move(cost_planner));
  }
  cout << "initialized planners" << endl;
}

void Controller::initialize_tasks(
  const std::vector<semaforr::config::TaskConfiguration>& tasks,
  int length,
  int height) {
  for (const auto& task : tasks) {
    beliefs->getAgentState()->addTask(task.x, task.y, length, height);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Initialize the controller and setup messaging to ROS
//
Controller::Controller(
  string advisor_config,
  string params_config,
  string map_config,
  string target_set,
  string map_dimensions)
  : Controller(semaforr::config::loadConfiguration({
      advisor_config,
      params_config,
      map_config,
      target_set,
      map_dimensions})) {}

Controller::Controller(semaforr::config::Configuration configuration) {
  const semaforr::config::ControllerConfiguration& params =
    configuration.controller;
  initialize_crowd_learning(configuration);

  taskDecisionLimit = params.task_decision_limit;
  planLimit = params.plan_limit;
  canSeePointEpsilon = params.can_see_point_epsilon;
  laserScanRadianIncrement = params.laser_scan_radian_increment;
  robotFootPrint = params.robot_footprint;
  robotFootPrintBuffer = params.robot_footprint_buffer;
  maxLaserRange = params.max_laser_range;
  maxForwardActionBuffer = params.max_forward_action_buffer;
  maxForwardActionSweepAngle = params.max_forward_action_sweep_angle;
  highwayDistanceThreshold = params.highway_distance_threshold;
  highwayTimeThreshold = params.highway_time_threshold;
  highwayDecisionThreshold = params.highway_decision_threshold;

  moveArrMax = static_cast<int>(params.move_actions.size()) + 1;
  rotateArrMax = static_cast<int>(params.rotate_actions.size()) + 1;
  arrMove[0] = 0.0;
  arrRotate[0] = 0.0;
  std::copy(
    params.move_actions.begin(), params.move_actions.end(), arrMove + 1);
  std::copy(
    params.rotate_actions.begin(), params.rotate_actions.end(), arrRotate + 1);

  trailsOn = params.trails_on;
  conveyorsOn = params.conveyors_on;
  regionsOn = params.regions_on;
  doorsOn = params.doors_on;
  hallwaysOn = params.hallways_on;
  barrsOn = params.barriers_on;
  aStarOn = params.a_star_on;
  highwaysOn = params.highways_on;
  frontiersOn = params.frontiers_on;
  outofhereOn = params.out_of_here_on;
  doorwayOn = params.doorway_on;
  findawayOn = params.find_a_way_on;
  behindOn = params.behind_on;
  dontgobackOn = params.dont_go_back_on;
  skeleton = params.planners.skeleton;
  hallwayskel = params.planners.hallway_skeleton;
  plannerConfiguration = params.planners;

  const int l = configuration.map_dimensions.length;
  const int h = configuration.map_dimensions.height;
  initialize_planner(configuration.map_dimensions);

  // Initialize the agent's 'beliefs' of the world state with the map and nav
  // graph and spatial models
  beliefs = std::make_unique<Beliefs>(
    l, h, 2, arrMove, arrRotate, moveArrMax, rotateArrMax); // Hunter Fourth

  // Initialize advisors and weights from the typed configuration.
  initialize_advisors(configuration.advisors);

  // Initialize tasks from the typed configuration.
  initialize_tasks(configuration.tasks, l, h);

  // Initialize parameters
  beliefs->getAgentState()->setAgentStateParameters(canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle);
  tier1 = std::make_unique<Tier1Advisor>(beliefs.get());
  firstTaskAssigned = false;
  decisionStats = FORRActionStats();

  // Initialize highways
  highwayFinished = 0;
  highwayExploration = std::make_unique<HighwayExplorer>(
    l, h, highwayDistanceThreshold, highwayTimeThreshold,
    highwayDecisionThreshold, arrMove, arrRotate, moveArrMax, rotateArrMax);

  // Initialize frontiers
  frontierFinished = 0;
  frontierExploration = std::make_unique<FrontierExplorer>(
    l, h, highwayTimeThreshold, highwayDecisionThreshold,
    arrMove, arrRotate, moveArrMax, rotateArrMax);

  tierOneDecision = semaforr::decision::makeTierOneDecision({
    *beliefs,
    *tier1,
    doorwayOn,
    behindOn,
    outofhereOn,
    findawayOn,
    dontgobackOn,
    highwayFinished,
    frontierFinished
  });
  tierTwoDecision = semaforr::decision::makeTierTwoDecision({
    *beliefs,
    *tier1,
    tier2Planners,
    aStarOn,
    highwayFinished,
    frontierFinished
  });
  tierThreeDecision = semaforr::decision::makeTierThreeDecision({
    tier3Advisors
  });

  // Initialize circumnavigator
  // PathPlanner *skeleton_planner;
  // for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
  //   if((*it)->getName() == "skeleton"){
  //     skeleton_planner = *it;
  //   }
  // }
  // circumnavigator = new Circumnavigate(l, h, arrMove, arrRotate, moveArrMax, rotateArrMax, beliefs, skeleton_planner);
}
