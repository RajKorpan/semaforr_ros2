/*
 * Controller.cpp
 *
 */
          
#include <semaforr/decision/Controller.h>
#include <semaforr/core/FORRGeometry.h>
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

  moveArrMax = static_cast<int>(params.move_actions.size());
  rotateArrMax = static_cast<int>(params.rotate_actions.size());
  std::copy(params.move_actions.begin(), params.move_actions.end(), arrMove);
  std::copy(
    params.rotate_actions.begin(), params.rotate_actions.end(), arrRotate);

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

  // Initialize circumnavigator
  // PathPlanner *skeleton_planner;
  // for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
  //   if((*it)->getName() == "skeleton"){
  //     skeleton_planner = *it;
  //   }
  // }
  // circumnavigator = new Circumnavigate(l, h, arrMove, arrRotate, moveArrMax, rotateArrMax, beliefs, skeleton_planner);
}


// Function which takes sensor inputs and updates it for semaforr to use for decision making, and updates task status
void Controller::updateState(
  Position current,
  const semaforr::domain::LaserScan& laser_scan,
  const semaforr::domain::PoseArray& crowdpose,
  const semaforr::domain::PoseArray& crowdposeall){
  cout << "In update state" << endl;
  beliefs->getAgentState()->setCurrentSensor(current, laser_scan);
  beliefs->getAgentState()->setCrowdPose(crowdpose);
  beliefs->getAgentState()->setCrowdPoseAll(crowdposeall);
  if(firstTaskAssigned == false){
      cout << "Set first task" << endl;
      // if(aStarOn and (!highwaysOn or (highwaysOn and highwayExploration->getHighwaysComplete())) and (!frontiersOn or (frontiersOn and frontierExploration->getFrontiersComplete()))){
      //   tierTwoDecision(current, true);
      // }
      // else{
      beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
      // }
      firstTaskAssigned = true;
  }
  if((highwayExploration->getHighwaysComplete() or !highwaysOn) and (frontierExploration->getFrontiersComplete() or !frontiersOn)){
    //bool waypointReached = beliefs->getAgentState()->getCurrentTask()->isWaypointComplete(current);
    bool waypointReached = beliefs->getAgentState()->getCurrentTask()->isAnyWaypointComplete(current, beliefs->getAgentState()->getCurrentLaserEndpoints());
    bool taskCompleted = beliefs->getAgentState()->getCurrentTask()->isTaskComplete(current);
    bool isPlanActive = beliefs->getAgentState()->getCurrentTask()->getIsPlanActive();
    // cout << "waypointReached " <<   waypointReached << " taskCompleted " << taskCompleted << " isPlanActive " << isPlanActive << endl;
    if(highwayFinished == 1 or frontierFinished == 1){
      if(highwaysOn or frontiersOn){
        learnSpatialModel(beliefs->getAgentState(), true, false);
        // RCLCPP_DEBUG(this->get_logger(), "Finished Learning Spatial Model!!");
        updateSkeletonGraph(beliefs->getAgentState());
        // RCLCPP_DEBUG(this->get_logger(), "Finished Updating Skeleton Graph!!");
        // beliefs->getAgentState()->setPassageGrid(highwayExploration->getHighwayGrid());
        if(highwaysOn){
          beliefs->getAgentState()->setRemainingCandidates(highwayExploration->getRemainingHighwayStack());
        }
        else if(frontiersOn){
          beliefs->getAgentState()->setRemainingCandidates(frontierExploration->getRemainingFrontierStack());
        }
      }
      beliefs->getAgentState()->finishTask(false);
      // RCLCPP_DEBUG(this->get_logger(), "Selecting Next Task");
      if(aStarOn){
        tierTwoDecision(current, true);
        // RCLCPP_DEBUG(this->get_logger(), "Next Plan Generated!!");
      }
      else{
        beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
        // RCLCPP_DEBUG(this->get_logger(), "Next Task Selected!!");
      }
      beliefs->getAgentState()->setGetOutTriggered(false);
      beliefs->getAgentState()->setRepositionTriggered(false);
      beliefs->getAgentState()->setRepositionCount(0);
      beliefs->getAgentState()->setFindAWayCount(0);
      beliefs->getAgentState()->setEnforcerCount(0);
      tier1->resetLocalExploration();
      // beliefs->getAgentState()->resetDirections();
      // circumnavigator->resetCircumnavigate();
      beliefs->getAgentState()->getCurrentTask()->resetPlanPositions();
    }
    //if task is complete
    if(taskCompleted == true){
      cout << "Target Achieved, moving on to next target!!" << endl;
      // RCLCPP_DEBUG(this->get_logger(), "Target Achieved, moving on to next target!!");
      //Learn spatial model only on tasks completed successfully
      if(beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size() <= 2000){
        learnSpatialModel(beliefs->getAgentState(), true, false);
        cout << "Learned spatial model" << endl;
        // RCLCPP_DEBUG(this->get_logger(), "Finished Learning Spatial Model!!");
        updateSkeletonGraph(beliefs->getAgentState());
        cout << "Updated skeleton graph" << endl;
        // RCLCPP_DEBUG(this->get_logger(), "Finished Updating Skeleton Graph!!");
      }
      beliefs->getAgentState()->setGetOutTriggered(false);
      beliefs->getAgentState()->setRepositionTriggered(false);
      beliefs->getAgentState()->setRepositionCount(0);
      beliefs->getAgentState()->setFindAWayCount(0);
      beliefs->getAgentState()->setEnforcerCount(0);
      tier1->resetLocalExploration();
      // beliefs->getAgentState()->resetDirections();
      // circumnavigator->resetCircumnavigate();
      beliefs->getAgentState()->getCurrentTask()->resetPlanPositions();
      //Clear existing task and associated plans
      beliefs->getAgentState()->finishTask(false);
      cout << "Cleared task" << endl;
      //// RCLCPP_DEBUG(this->get_logger(), "Task Cleared!!");
      //cout << "Agenda Size = " << beliefs->getAgentState()->getAgenda().size() << endl;
      if(beliefs->getAgentState()->getAgenda().size() > 0){
        //Tasks the next task , current position and a planner and generates a sequence of waypoints if astaron is true
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Controller.cpp taskCount > " << (beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) << " planLimit " << (planLimit - 1));
        if((beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) > (planLimit - 1)){
          aStarOn = false;
        }
        cout << "Selecting Next Task " << endl;
        // RCLCPP_DEBUG(this->get_logger(), "Selecting Next Task");
        if(aStarOn){
          tierTwoDecision(current, true);
          cout << "Next Plan Generated!!" << endl;
          // RCLCPP_DEBUG(this->get_logger(), "Next Plan Generated!!");
        }
        else{
          beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
          cout << "Next Task Selected!!" << endl;
          // RCLCPP_DEBUG(this->get_logger(), "Next Task Selected!!");
        }
      }
    }
    // else if subtask is complete
    else if(waypointReached == true and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint reached, but task still incomplete, switching to nearest visible waypoint towards target!!");
      //beliefs->getAgentState()->getCurrentTask()->setupNextWaypoint(current);
      beliefs->getAgentState()->getCurrentTask()->setupNearestWaypoint(current, beliefs->getAgentState()->getCurrentLaserEndpoints());
      //beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getCurrentTask(),current,planner,aStarOn);
    }
    else if(tier1->localExplorationTriggerLearning() and beliefs->getAgentState()->getCurrentTask()->getDecisionCount() != taskDecisionLimit){
      learnSpatialModel(beliefs->getAgentState(), false, true);
      // RCLCPP_DEBUG(this->get_logger(), "Finished Learning Spatial Model!!");
      updateSkeletonGraph(beliefs->getAgentState());
      // RCLCPP_DEBUG(this->get_logger(), "Finished Updating Skeleton Graph!!");
      if(aStarOn){
        tierTwoDecision(current, false);
        // RCLCPP_DEBUG(this->get_logger(), "New Plan Generated!!");
      }
      beliefs->getAgentState()->setGetOutTriggered(false);
      beliefs->getAgentState()->setRepositionTriggered(false);
      beliefs->getAgentState()->setRepositionCount(0);
      beliefs->getAgentState()->setFindAWayCount(0);
      beliefs->getAgentState()->setEnforcerCount(0);
      beliefs->getAgentState()->getCurrentTask()->resetPlanPositions();
    }
    // else if(isPlanActive == false and aStarOn){
    //   // RCLCPP_DEBUG(this->get_logger(), "No active plan, setting up new plan!!");
    //   tierTwoDecision(current);
    // }
    // else if(waypointReached == true and beliefs->getAgentState()->getCurrentTask()->getWaypoints().size() == 1){
    //   // RCLCPP_DEBUG(this->get_logger(), "Temporary Waypoint reached!!");
    //   beliefs->getAgentState()->getCurrentTask()->setIsPlanActive(false);
    //   beliefs->getAgentState()->getCurrentTask()->clearWaypoints();
    // }
    // otherwise if task Decision limit reached, skip task 
    if(beliefs->getAgentState()->getCurrentTask() != NULL){
      if(beliefs->getAgentState()->getCurrentTask()->getDecisionCount() > taskDecisionLimit){
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Controller.cpp decisionCount > " << taskDecisionLimit << " , skipping task");
        beliefs->getAgentState()->setGetOutTriggered(false);
        beliefs->getAgentState()->setRepositionTriggered(false);
        beliefs->getAgentState()->setRepositionCount(0);
        beliefs->getAgentState()->setFindAWayCount(0);
        beliefs->getAgentState()->setEnforcerCount(0);
        beliefs->getAgentState()->getCurrentTask()->resetPlanPositions();
        tier1->resetLocalExploration();
        // beliefs->getAgentState()->resetDirections();
        // circumnavigator->resetCircumnavigate();
        learnSpatialModel(beliefs->getAgentState(), false, false);
        cout << "Learned spatial model" << endl;
        // RCLCPP_DEBUG(this->get_logger(), "Finished Learning Spatial Model!!");
        updateSkeletonGraph(beliefs->getAgentState());
        cout << "Updated skeleton graph" << endl;
        // RCLCPP_DEBUG(this->get_logger(), "Finished Updating Skeleton Graph!!");
        //beliefs->getAgentState()->skipTask();
        // if(beliefs->getAgentState()->getAllAgenda().size() < planLimit +1){
        //   beliefs->getAgentState()->addTask(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
        // }
        beliefs->getAgentState()->finishTask(true);
        if(beliefs->getAgentState()->getAgenda().size() > 0){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Controller.cpp taskCount > " << (beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) << " planLimit " << (planLimit - 1));
          if((beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) > (planLimit - 1)){
            aStarOn = false;
          }
          if(aStarOn){
            tierTwoDecision(current, true);
          }
          else{
            beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
          }
        }
      }
    }
  }
  
  // // RCLCPP_DEBUG(this->get_logger(), "End Of UpdateState");
}


// Function which returns the mission status
bool Controller::isMissionComplete(){
  return beliefs->getAgentState()->isMissionComplete();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main robot decision making engine, return decisions that would lead the robot to complete its mission
// Manages switching tasks and stops if the robot is taking too long
//
FORRAction Controller::decide() {
  // RCLCPP_DEBUG(this->get_logger(), "Entering decision loop");
  cout << "Entering decision loop" << endl;
  FORRAction decidedAction;
  if(!highwayExploration->getHighwaysComplete() and highwaysOn){
    cout << "highway decision " << endl;
    decidedAction = highwayExploration->exploreDecision(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
    decisionStats.decisionTier = 1.7;
  }
  else if(!frontierExploration->getFrontiersComplete() and frontiersOn){
    cout << "frontier decision " << endl;
    decidedAction = frontierExploration->exploreDecision(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
    cout << "frontier decision " << decidedAction.type << " " << decidedAction.parameter << endl;
    decisionStats.decisionTier = 1.8;
  }
  else{
    cout << "forr decision " << endl;
    if(highwayFinished < 3){
      highwayFinished++;
    }
    if(frontierFinished < 3){
      frontierFinished++;
    }
    decidedAction = FORRDecision();
  }
  //// RCLCPP_DEBUG(this->get_logger(), "After decision made");
  cout << "Decided Action: " << decidedAction.type << " " << decidedAction.parameter << endl;
  beliefs->getAgentState()->getCurrentTask()->incrementDecisionCount();
  //// RCLCPP_DEBUG(this->get_logger(), "After incrementDecisionCount");
  cout << "Decision Count: " << beliefs->getAgentState()->getCurrentTask()->getDecisionCount() << endl;
  beliefs->getAgentState()->getCurrentTask()->saveDecision(decidedAction);
  //// RCLCPP_DEBUG(this->get_logger(), "After saveDecision");
  cout << "Clearing vetoed actions" << endl;
  beliefs->getAgentState()->clearVetoedActions();
  //// RCLCPP_DEBUG(this->get_logger(), "After clearVetoedActions");
  cout << "Exiting decision loop" << endl;
  return decidedAction;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Update spatial model after every task 
//
//

void Controller::learnSpatialModel(AgentState* agentState, bool taskStatus, bool earlyLearning){
  double computationTimeSec=0.0;
  timeval cv;
  double start_timecv;
  double end_timecv;
  gettimeofday(&cv,NULL);
  start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);

  Task* completedTask = agentState->getCurrentTask();
  vector<Position> *pos_hist = completedTask->getPositionHistory();
  vector< vector<CartesianPoint> > *laser_hist = completedTask->getLaserHistory();
  vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
  // vector< vector<CartesianPoint> > exit_traces = beliefs->getAgentState()->getInitialExitTraces();
  // vector< vector<CartesianPoint> > all_laser_hist = beliefs->getAgentState()->getAllLaserHistory();
  vector< vector < vector<CartesianPoint> > > all_laser_trace = beliefs->getAgentState()->getAllLaserTrace();
  vector<CartesianPoint> trace;
  for(int i = 0 ; i < pos_hist->size() ; i++){
    trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
  }
  all_trace.push_back(trace);
  // for(int i = 0; i < exit_traces.size(); i++){
  //   all_trace.insert(all_trace.begin(), exit_traces[i]);
  // }
  vector < vector<CartesianPoint> > laser_trace;
  for(int i = 0 ; i < laser_hist->size() ; i++){
    laser_trace.push_back((*laser_hist)[i]);
  }
  all_laser_trace.push_back(laser_trace);

  if(trailsOn and !earlyLearning){
    beliefs->getSpatialModel()->getTrails()->updateTrails(agentState);
    beliefs->getSpatialModel()->getTrails()->resetChosenTrail();
    // RCLCPP_DEBUG(this->get_logger(), "Trails Learned");
  }
  vector< vector<CartesianPoint> > trails_trace = beliefs->getSpatialModel()->getTrails()->getTrailsPoints();
  if(conveyorsOn and taskStatus){
    //beliefs->getSpatialModel()->getConveyors()->populateGridFromPositionHistory(pos_hist);
    beliefs->getSpatialModel()->getConveyors()->populateGridFromTrailTrace(trails_trace.back());
    // RCLCPP_DEBUG(this->get_logger(), "Conveyors Learned");
  }
  if(regionsOn){
    beliefs->getSpatialModel()->getRegionList()->learnRegionsAndExits(pos_hist, laser_hist, all_trace, all_laser_trace);
    // beliefs->getSpatialModel()->getRegionList()->learnRegions(pos_hist, laser_hist);
    // RCLCPP_DEBUG(this->get_logger(), "Regions Learned");
    // beliefs->getSpatialModel()->getRegionList()->clearAllExits();
    // beliefs->getSpatialModel()->getRegionList()->learnExits(all_trace);
    // beliefs->getSpatialModel()->getRegionList()->learnExits(trails_trace);
    // RCLCPP_DEBUG(this->get_logger(), "Exits Learned");
  }
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  if(doorsOn){
    beliefs->getSpatialModel()->getDoors()->clearAllDoors();
    beliefs->getSpatialModel()->getDoors()->learnDoors(regions);
    // RCLCPP_DEBUG(this->get_logger(), "Doors Learned");
  }
  if(hallwaysOn){
    //beliefs->getSpatialModel()->getHallways()->clearAllHallways();
    //beliefs->getSpatialModel()->getHallways()->learnHallways(agentState, all_trace, all_laser_hist);
    beliefs->getSpatialModel()->getHallways()->learnHallways(agentState, trace, laser_hist);
    //beliefs->getSpatialModel()->getHallways()->learnHallways(trails_trace);
    // RCLCPP_DEBUG(this->get_logger(), "Hallways Learned");
  }
  if(barrsOn){
    beliefs->getSpatialModel()->getBarriers()->updateBarriers(laser_hist, all_trace.back());
    // RCLCPP_DEBUG(this->get_logger(), "Barriers Learned");
  }
  gettimeofday(&cv,NULL);
  end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
  computationTimeSec = (end_timecv-start_timecv);
  decisionStats.learningComputationTime = computationTimeSec;
}

void Controller::updateSkeletonGraph(AgentState* agentState){
  double computationTimeSec=0.0;
  timeval cv;
  double start_timecv;
  double end_timecv;
  gettimeofday(&cv,NULL);
  start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);

  if((skeleton) or (hallwayskel and (highwayFinished >= 1 or frontierFinished >= 1))){
    cout << "Updating skeleton planner" << endl;
    PathPlanner *skeleton_planner;
    PathPlanner *hallway_skeleton_planner;
    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      if(skeleton and (*it)->getName() == "skeleton"){
        skeleton_planner = it->get();
      }
      if(hallwayskel and (*it)->getName() == "hallwayskel"){
        hallway_skeleton_planner = it->get();
      }
    }
    if(skeleton){
      skeleton_planner->resetGraph();
    }
    if(hallwayskel){
      hallway_skeleton_planner->resetOrigGraph();
    }
    // cout << "Planner reset" << endl;
    vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
    int index_val = 0;
    int hallway_index_val = 0;
    for(int i = 0 ; i < regions.size(); i++){
      int x = (int)(regions[i].getCenter().get_x()*100);
      int y = (int)(regions[i].getCenter().get_y()*100);
      // cout << "Region " << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << x << " " << y << endl;
      vector<FORRExit> exits = regions[i].getMinExits();
      // cout << "Exits " << exits.size() << endl;
      if(exits.size() > 0){
        if(skeleton){
          bool success = skeleton_planner->getGraph()->addNode(x, y, regions[i].getRadius(), index_val);
          if(success){
            index_val++;
          }
        }
        if(hallwayskel){
          bool success = hallway_skeleton_planner->getOrigGraph()->addNode(x, y, regions[i].getRadius(), hallway_index_val);
          if(success){
            hallway_index_val++;
          }
        }
      }
    }
    for(int i = 0 ; i < regions.size(); i++){
      if(skeleton){
        int region_id = skeleton_planner->getGraph()->getNodeID((int)(regions[i].getCenter().get_x()*100), (int)(regions[i].getCenter().get_y()*100));
        if(region_id != -1){
          vector<FORRExit> exits = regions[i].getMinExits();
          for(int j = 0; j < exits.size() ; j++){
            int index_val = skeleton_planner->getGraph()->getNodeID((int)(regions[exits[j].getExitRegion()].getCenter().get_x()*100), (int)(regions[exits[j].getExitRegion()].getCenter().get_y()*100));
            if(index_val != -1){
              skeleton_planner->getGraph()->addEdge(region_id, index_val, exits[j].getExitDistance()*100, exits[j].getConnectionPoints());
            }
          }
        }
      }
      if(hallwayskel){
        int region_id = hallway_skeleton_planner->getOrigGraph()->getNodeID((int)(regions[i].getCenter().get_x()*100), (int)(regions[i].getCenter().get_y()*100));
        if(region_id != -1){
          vector<FORRExit> exits = regions[i].getMinExits();
          for(int j = 0; j < exits.size() ; j++){
            int index_val = hallway_skeleton_planner->getOrigGraph()->getNodeID((int)(regions[exits[j].getExitRegion()].getCenter().get_x()*100), (int)(regions[exits[j].getExitRegion()].getCenter().get_y()*100));
            if(index_val != -1){
              hallway_skeleton_planner->getOrigGraph()->addEdge(region_id, index_val, exits[j].getExitDistance()*100, exits[j].getConnectionPoints());
            }
          }
        }
      }
    }
    if(skeleton){
      cout << "Finished updating skeleton planner" << endl;
      skeleton_planner->getGraph()->printGraph();
      cout << "Connected Graph: " << skeleton_planner->getGraph()->isConnected() << endl;
    }
    if(hallwayskel){
      cout << "Finished updating skeleton graph for passage planner" << endl;
      // skeleton_planner->getOrigGraph()->printGraph();
      beliefs->getSpatialModel()->getRegionList()->setRegionPassageValues(beliefs->getAgentState()->getPassageGrid());
      // cout << "Connected Graph: " << skeleton_planner->getOrigGraph()->isConnected() << endl;
    }
  }
  if(hallwayskel and (highwayFinished == 1 or frontierFinished == 1)){
    PathPlanner *hwskeleton_planner;
    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      if((*it)->getName() == "hallwayskel"){
        hwskeleton_planner = it->get();
      }
    }
    hwskeleton_planner->resetGraph();
    FORRPassages passages = FORRPassages(highwayExploration->getHighwayGrid(), agentState);
    Task* completedTask = agentState->getCurrentTask();
    vector<Position> *pos_hist = completedTask->getPositionHistory();
    vector< vector<CartesianPoint> > *laser_hist = completedTask->getLaserHistory();
    vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
    vector< vector < vector<CartesianPoint> > > all_laser_trace = beliefs->getAgentState()->getAllLaserTrace();
    vector<CartesianPoint> trace;
    for(int i = 0 ; i < pos_hist->size() ; i++){
      trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
    }
    all_trace.push_back(trace);
    vector < vector<CartesianPoint> > laser_trace;
    for(int i = 0 ; i < laser_hist->size() ; i++){
      laser_trace.push_back((*laser_hist)[i]);
    }
    all_laser_trace.push_back(laser_trace);
    vector<CartesianPoint> stepped_history;
    vector < vector<CartesianPoint> > stepped_laser_history;
    for(int k = 0; k < all_trace.size() ; k++){
      vector<CartesianPoint> history = all_trace[k];
      vector < vector<CartesianPoint> > laser_history = all_laser_trace[k];
      for(int j = 0; j < history.size(); j++){
        stepped_history.push_back(history[j]);
        stepped_laser_history.push_back(laser_history[j]);
      }
    }
    // cout << "stepped_history " << stepped_history.size() << " stepped_laser_history " << stepped_laser_history.size() << endl;
    passages.learnPassages(stepped_history, stepped_laser_history);
    // cout << "finished learning passages" << endl;
    int index_val = 0;
    map<int, vector< vector<int> > > graph_nodes = passages.getGraphNodes();
    vector< vector<int> > average_passage = passages.getAveragePassage();
    map<int, vector< vector<int> > >::iterator it;
    for(it = graph_nodes.begin(); it != graph_nodes.end(); it++){
      bool success = hwskeleton_planner->getGraph()->addNode(average_passage[it->first - 1][0], average_passage[it->first - 1][1], 0, index_val);
      if(success){
        hwskeleton_planner->getGraph()->getNodePtr(index_val)->setIntersectionID(it->first);
        index_val++;
      }
    }
    // cout << "finished creating nodes" << endl;
    vector< vector<int> > graph = passages.getGraph();
    for(int i = 0; i < graph.size(); i++){
      int node_a_id = hwskeleton_planner->getGraph()->getNodeID(average_passage[graph[i][0]-1][0], average_passage[graph[i][0]-1][1]);
      int node_b_id = hwskeleton_planner->getGraph()->getNodeID(average_passage[graph[i][2]-1][0], average_passage[graph[i][2]-1][1]);
      // cout << "graph " << graph[i][0] << " " << graph[i][1] << " " << graph[i][2] << " node_a_id " << node_a_id << " node_b_id " << node_b_id << endl;
      if(node_a_id != -1 and node_b_id != -1){
        double distance_ab = sqrt((average_passage[graph[i][0]-1][0] - average_passage[graph[i][2]-1][0])*(average_passage[graph[i][0]-1][0] - average_passage[graph[i][2]-1][0]) + (average_passage[graph[i][0]-1][1] - average_passage[graph[i][2]-1][1])*(average_passage[graph[i][0]-1][1] - average_passage[graph[i][2]-1][1]));
        vector<CartesianPoint> path;
        path.push_back(CartesianPoint(graph[i][0], -1));
        path.push_back(CartesianPoint(graph[i][1], -1));
        path.push_back(CartesianPoint(graph[i][2], -1));
        // cout << "distance_ab " << distance_ab << " path " << path.size() << endl;
        hwskeleton_planner->getGraph()->addEdge(node_a_id, node_b_id, distance_ab, path);
      }
    }
    // cout << "finished creating edges" << endl;
    hwskeleton_planner->getGraph()->printGraph();
    // cout << "Connected Graph: " << hwskeleton_planner->getGraph()->isConnected() << endl;
    passages.learnPassageTrails(stepped_history, stepped_laser_history);
    // cout << "finished learning passage trails" << endl;
    agentState->setPassageValues(passages.getPassages(), graph_nodes, passages.getGraphEdges(), graph, average_passage, passages.getGraphTrails(), passages.getGraphThroughIntersections(), passages.getGraphIntersectionTrails());
    beliefs->getSpatialModel()->getRegionList()->setRegionPassageValues(passages.getPassages());
    // cout << "Finished updating passage planner" << endl;
  }

  gettimeofday(&cv,NULL);
  end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
  computationTimeSec = (end_timecv-start_timecv);
  decisionStats.graphingComputationTime = computationTimeSec;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// SemaFORR decision workflow
//
//
FORRAction Controller::FORRDecision()
{  
  // RCLCPP_DEBUG(this->get_logger(), "In FORR decision");
  cout << "In FORR decision" << endl;
  FORRAction decision;
  cout << "Created decision object" << endl;
  // Basic semaFORR three tier decision making architecture 
  if(!tierOneDecision(&decision)){
  	// RCLCPP_DEBUG(this->get_logger(), "Decision to be made by t3!!");
    cout << "Decision to be made by t3!!" << endl;
  	//decision->type = FORWARD;
  	//decision->parameter = 5;
    tierThreeDecision(&decision);
  	tierThreeAdvisorInfluence();
    decisionStats.decisionTier = 3;
  }
  //cout << "decisionTier = " << decisionStats.decisionTier << endl;
  // //// RCLCPP_DEBUG(this->get_logger(), "After decision made");
  // beliefs->getAgentState()->getCurrentTask()->incrementDecisionCount();
  // //// RCLCPP_DEBUG(this->get_logger(), "After incrementDecisionCount");
  // beliefs->getAgentState()->getCurrentTask()->saveDecision(*decision);
  // //// RCLCPP_DEBUG(this->get_logger(), "After saveDecision");
  // beliefs->getAgentState()->clearVetoedActions();
  // //// RCLCPP_DEBUG(this->get_logger(), "After clearVetoedActions");
  // if(decision->type == FORWARD or decision->type == PAUSE){
  //   beliefs->getAgentState()->setRotateMode(true);
  // }
  // else{
  //   beliefs->getAgentState()->setRotateMode(false);
  // }
  cout << "Exiting FORR decision with action: " << decision.type << " " << decision.parameter << endl;
  return decision;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 1 decision
//
//
bool Controller::tierOneDecision(FORRAction *decision){
  //decision making tier1 advisor
  bool decisionMade = false;
  // // RCLCPP_INFO(this->get_logger(), "Advisor circumnavigate will create subplan");
  // tier1->advisorCircumnavigate(decision);
  cout << "Tier 1 Decision Making" << endl;
  CartesianPoint current_position = CartesianPoint(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
  cout << "Inside tier 1 decision. Current Position: " << current_position.get_x() << " " << current_position.get_y() << endl;
  if(current_position.get_distance(beliefs->getAgentState()->getFarthestPoint()) <= 0.75){
    cout << "if statement 1 triggered" << endl;
    beliefs->getAgentState()->setGetOutTriggered(false);
  }
  if(current_position.get_distance(beliefs->getAgentState()->getRepositionPoint()) <= 0.75 or beliefs->getAgentState()->getRepositionCount() >= 20){
    cout << "if statement 2 triggered" << endl;
    beliefs->getAgentState()->setRepositionTriggered(false);
    beliefs->getAgentState()->setRepositionCount(0);
  }
  if(tier1->advisorVictory(decision)){ 
    cout << "if statement 3 triggered" << endl;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor Victory has made a decision " << decision->type << " " << decision->parameter);
    // circumnavigator->addToStack(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
    decisionStats.decisionTier = 1.1;
    decisionMade = true;
  }
  else{
    cout << "else statement triggered" << endl;
    // RCLCPP_INFO(this->get_logger(), "Advisor AvoidObstacles will veto actions");
    tier1->advisorAvoidObstacles();
    cout << "Advisor AvoidObstacles vetoed actions" << endl;
    vector<FORRAction> AOVetoedActions;
    set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
    set<FORRAction>::iterator it;
    for(it = vetoedActions->begin(); it != vetoedActions->end(); it++){
      AOVetoedActions.push_back(*it);
    }
    // RCLCPP_INFO(this->get_logger(), "Advisor NotOpposite will veto actions");
    tier1->advisorNotOpposite();
    cout << "Advisor NotOpposite vetoed actions" << endl;
    vector<FORRAction> NOVetoedActions;
    vetoedActions = beliefs->getAgentState()->getVetoedActions();
    for(it = vetoedActions->begin(); it != vetoedActions->end(); it++){
      if(find(AOVetoedActions.begin(), AOVetoedActions.end(), *it) == AOVetoedActions.end()){
        NOVetoedActions.push_back(*it);
      }
    }
    if(tier1->advisorEnforcer(decision)){  // running advisorEnforcer will print PlannerName and PlanSize
      cout << "if statement 1 inside else statement triggered" << endl;
      // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor Enforcer has made a decision " << decision->type << " " << decision->parameter);
      // circumnavigator->addToStack(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
      if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "skeleton" or beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "hallwayskel"){
        if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getCreator() == 2){
          decisionStats.decisionTier = 1.5;
        }
        else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getCreator() == 3){
          decisionStats.decisionTier = 1.6;
        }
        else{
          if(tier1->getShortcut() == true){
            decisionStats.decisionTier = 1.21;
          }
          else{
            decisionStats.decisionTier = 1.2;
          }
        }
      }
      else{
        decisionStats.decisionTier = 1.2;
      }
      decisionMade = true;
    }
    if(doorwayOn and decisionMade == false){
      if(tier1->advisorDoorway(decision)){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor Doorway has made a decision " << decision->type << " " << decision->parameter);
        decisionStats.decisionTier = 1.3;
        decisionMade = true;
      }
    }
    if(behindOn and decisionMade == false){
      if(tier1->advisorBehindYou(decision)){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor BehindYou has made a decision " << decision->type << " " << decision->parameter);
        decisionStats.decisionTier = 1.4;
        decisionMade = true;
      }
    }
    // if(circumnavigator->advisorCircumnavigate(decision)){
    //   // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor circumnavigate has made a decision " << decision->type << " " << decision->parameter);
    //   decisionStats.decisionTier = 2.5;
    //   decisionMade = true;
    // }
    // else
    if(outofhereOn and decisionMade == false){
      if(tier1->advisorGetOut(decision)){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor GetOut has made a decision " << decision->type << " " << decision->parameter);
        decisionStats.decisionTier = 1.5;
        decisionMade = true;
      }
    }
    if(findawayOn and decisionMade == false and (highwayFinished > 1 or frontierFinished > 1)){
      if(tier1->advisorFindAWay(decision)){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Advisor FindAWay has made a decision " << decision->type << " " << decision->parameter);
        decisionStats.decisionTier = 1.6;
        decisionMade = true;
      }
    }
    if(dontgobackOn){
      // RCLCPP_INFO(this->get_logger(), "Advisor don't go back will veto actions");
      tier1->advisorDontGoBack();
    }
    vector<FORRAction> DGBVetoedActions;
    vetoedActions = beliefs->getAgentState()->getVetoedActions();
    for(it = vetoedActions->begin(); it != vetoedActions->end(); it++){
      if(find(AOVetoedActions.begin(), AOVetoedActions.end(), *it) == AOVetoedActions.end() and find(NOVetoedActions.begin(), NOVetoedActions.end(), *it) == NOVetoedActions.end()){
        DGBVetoedActions.push_back(*it);
      }
    }
    vector<FORRAction> SVetoedActions;
    std::stringstream vetoList;
    for(int i = 0; i < AOVetoedActions.size(); i++){
      vetoList << AOVetoedActions[i].type << " " << AOVetoedActions[i].parameter << " 1a;";
    }
    for(int i = 0; i < NOVetoedActions.size(); i++){
      vetoList << NOVetoedActions[i].type << " " << NOVetoedActions[i].parameter << " 1b;";
    }
    for(int i = 0; i < DGBVetoedActions.size(); i++){
      vetoList << DGBVetoedActions[i].type << " " << DGBVetoedActions[i].parameter << " 1c;";
    }
    for(int i = 0; i < SVetoedActions.size(); i++){
      vetoList << SVetoedActions[i].type << " " << SVetoedActions[i].parameter << " 1d;";
    }
    decisionStats.vetoedActions = vetoList.str();
  }
  // set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  // std::stringstream vetoList;
  // set<FORRAction>::iterator it;
  // for(it = vetoedActions->begin(); it != vetoedActions->end(); it++){
  //   vetoList << it->type << " " << it->parameter << ";";
  // }
  // decisionStats.vetoedActions = vetoList.str();
  //cout << "vetoedActions = " << vetoList.str() << endl;
  tier1->resetShortcut();
  return decisionMade;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 2 decision
//
//
void Controller::tierTwoDecision(Position current, bool selectNextTask){
  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Tier 2 Decision");
  vector< list<int> > plans;
  vector<string> plannerNames;
  typedef vector< list<int> >::iterator vecIT;
  if(selectNextTask == true){
    beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
  }

  double computationTimeSec=0.0;
  timeval cv;
  double start_timecv;
  double end_timecv;
  gettimeofday(&cv,NULL);
  start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
  bool planCreated = false;
  for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
    PathPlanner *planner = it->get();
    planner->setPosHistory(beliefs->getAgentState()->getAllTrace());
    vector< vector<CartesianPoint> > trails_trace = beliefs->getSpatialModel()->getTrails()->getTrailsPoints();
    planner->setSpatialModel(beliefs->getSpatialModel()->getConveyors(),beliefs->getSpatialModel()->getRegionList()->getRegions(),beliefs->getSpatialModel()->getDoors()->getDoors(),trails_trace,beliefs->getSpatialModel()->getHallways()->getHallways());
    if(highwayFinished >= 1 or frontierFinished >= 1){
      // cout << "setting values for highways" << endl;
      planner->setPassageGrid(beliefs->getAgentState()->getPassageGrid(), beliefs->getAgentState()->getPassageGraphNodes(), beliefs->getAgentState()->getPassageGraph(), beliefs->getAgentState()->getAveragePassage());
      // cout << "set planner values" << endl;
      beliefs->getAgentState()->getCurrentTask()->setPassageValues(beliefs->getAgentState()->getPassageGrid(), beliefs->getAgentState()->getPassageGraphNodes(), beliefs->getAgentState()->getPassageGraphEdges(), beliefs->getAgentState()->getPassageGraph(), beliefs->getAgentState()->getAveragePassage(), beliefs->getAgentState()->getGraphTrails(), beliefs->getAgentState()->getGraphThroughIntersections(), beliefs->getAgentState()->getGraphIntersectionTrails());
      // cout << "set task values" << endl;
    }
    if(tier1->localExplorationStarted()){
      planner->setCoverageGrid(tier1->getLocalExploreCoverage());
    }
    //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating plans " << planner->getName());
    //gettimeofday(&cv,NULL);
    //start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
    vector< list<int> > multPlans = beliefs->getAgentState()->getPlansWaypoints(current,planner,aStarOn);
    for (int i = 0; i < multPlans.size(); i++){
      plans.push_back(multPlans[i]);
      plannerNames.push_back(planner->getName());
      if(multPlans[i].size() > 0){
        planCreated = true;
      }
    }
    //gettimeofday(&cv,NULL);
    //end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
    //computationTimeSec = (end_timecv-start_timecv);
    //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Planning time = " << computationTimeSec);
  }
  if(planCreated == true){
    vector< vector<double> > planCosts;
    typedef vector< vector<double> >::iterator costIT;

    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      PathPlanner *planner = it->get();
      vector<double> planCost;
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Computing plan cost " << planner->getName());
      for (vecIT vt = plans.begin(); vt != plans.end(); vt++){
        double costOfPlan = 0;
        if(planner->getName() != "skeleton" and planner->getName() != "hallwayskel"){
          costOfPlan = planner->calcPathCost(*vt);
        }
        planCost.push_back(costOfPlan);
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Cost = " << costOfPlan);
      }
      planCosts.push_back(planCost);
    }

    typedef vector<double>::iterator doubIT;
    vector< vector<double> > planCostsNormalized;
    for (costIT it = planCosts.begin(); it != planCosts.end(); it++){
      double max = *max_element(it->begin(), it->end());
      double min = *min_element(it->begin(), it->end());
      double norm_factor = (max - min)/10;
      vector<double> planCostNormalized;
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Computing normalized plan cost: Max = " << max << " Min = " << min << " Norm Factor = " << norm_factor);
      for (doubIT vt = it->begin(); vt != it->end(); vt++){
        if (max != min){
          planCostNormalized.push_back((*vt - min)/norm_factor);
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Original value = " << *vt << " Normalized = " << ((*vt - min)/norm_factor));
        }
        else{
          planCostNormalized.push_back(0);
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Original value = " << *vt << " Normalized = 0");
        }
      }
      planCostsNormalized.push_back(planCostNormalized);
    }
    //planCostsNormalized.pop_back();
    std::stringstream plannerCommentsList;
    vector<double> totalCosts;
    for (int i = 0; i < plans.size(); i++){
      double cost=0;
      // // RCLCPP_DEBUG_STREAM(this->get_logger(), "Computing total cost = " << cost);
      plannerCommentsList << plannerNames[i] << " ";
      for (costIT it = planCostsNormalized.begin(); it != planCostsNormalized.end(); it++){
        cost += it->at(i);
        plannerCommentsList << it->at(i) << " ";
        // // RCLCPP_DEBUG_STREAM(this->get_logger(), "cost = " << cost);
      }
      plannerCommentsList << cost << ";";
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Final cost = " << cost);
      totalCosts.push_back(cost);
    }
    double minCost=100000;
    // // RCLCPP_DEBUG_STREAM(this->get_logger(), "Computing min cost");
    for (int i=0; i < totalCosts.size(); i++){
      // // RCLCPP_DEBUG_STREAM(this->get_logger(), "Total cost = " << totalCosts[i]);
      if (totalCosts[i] < minCost){
        minCost = totalCosts[i];
      }
    }

    /*double minCombinedCost=1000;
    for (int i=18; i < totalCosts.size(); i++){
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Total cost = " << totalCosts[i]);
      if (totalCosts[i] < minCombinedCost){
        minCombinedCost = totalCosts[i];
      }
    }*/
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Min cost = " << minCost);
    //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Min Combined cost = " << minCombinedCost);

    vector<string> bestPlanNames;
    vector<int> bestPlanInds;
    for (int i=0; i < totalCosts.size(); i++){
      if(totalCosts[i] == minCost){
        bestPlanNames.push_back(plannerNames[i]);
        bestPlanInds.push_back(i);
        // // RCLCPP_DEBUG_STREAM(this->get_logger(), "Best plan " << plannerNames[i]);
      }
    }

    srand(time(NULL));
    int random_number = rand() % (bestPlanInds.size());
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of best plans = " << bestPlanInds.size() << " random_number = " << random_number);
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Selected Best plan " << bestPlanNames.at(random_number));
    decisionStats.chosenPlanner = bestPlanNames.at(random_number);
    for(int i = 0; i < plannerNames.size(); i++){
      if(plannerNames[i] != bestPlanNames.at(random_number)){
        decisionStats.chosenPlanner = decisionStats.chosenPlanner + ">" + plannerNames[i];
      }
    }
    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      PathPlanner *planner = it->get();
      if(planner->getName() == bestPlanNames.at(random_number)){
        beliefs->getAgentState()->setCurrentWaypoints(current, beliefs->getAgentState()->getCurrentLaserEndpoints(), planner, aStarOn, plans.at(bestPlanInds.at(random_number)), beliefs->getSpatialModel()->getRegionList()->getRegions());
        if(planner->getName() == "hallwayskel"){
          if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getCreator() == 0){
            decisionStats.chosenPlanner = "skeletonhall>hallwayskel";
          }
          else{
            decisionStats.chosenPlanner = "hallwayskel>skeletonhall";
          }
        }
        else if(planner->getName() == "skeleton"){
          decisionStats.chosenPlanner = "skeleton>distance";
        }
        break;
      }
    }
    decisionStats.plannerComments = plannerCommentsList.str();
  }
  for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
    PathPlanner *planner = it->get();
    planner->resetPath();
    planner->resetOrigPath();
  }
  gettimeofday(&cv,NULL);
  end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
  computationTimeSec = (end_timecv-start_timecv);
  decisionStats.planningComputationTime = computationTimeSec;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 3 decision
//
//
void Controller::tierThreeDecision(FORRAction *decision){
  std::map<FORRAction, double> comments;
  // This map will aggregate value of all advisers
  std::map<FORRAction, double> allComments;

  // typedef to make for declaration that iterates over map shorter
  typedef map<FORRAction, double>::iterator mapIt;

  // vector of all the actions that got max comment strength in iteration
  vector<FORRAction> best_decisions;
  
  double rotationBaseline, linearBaseline;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = it->get();
    //if(advisor->is_active() == true)
      //cout << advisor->get_name() << " : " << advisor->get_weight() << endl;
    if(advisor->get_name() == "RotationBaseLine") rotationBaseline = advisor->get_weight();
    if(advisor->get_name() == "BaseLine")         linearBaseline   = advisor->get_weight();
  }
       
  std::stringstream advisorsList;
  std::stringstream advisorCommentsList;
  cout << "processing advisors::"<< endl;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = it->get();
    cout << advisor->get_name() << endl;
    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      //cout << advisor->get_name() << " is inactive " << endl;
      advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";
      continue;
    }
    if(advisor->is_commenting() == false){
      //cout << advisor->get_name() << " is not commenting " << endl;
      advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";
      continue;
    }

    advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";

    cout << "Before commenting " << endl;
    comments = advisor->allAdvice();
    cout << "after commenting " << endl;
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      //cout << "comment : " << (iterator->first.type) << (iterator->first.parameter) << " " << (iterator->second) << endl;
      // If this is first advisor we need to initialize our final map
      float weight;
      //cout << "Agenda size :::::::::::::::::::::::::::::::::: " << beliefs->getAgenda().size() << endl;
      // cout << "<" << advisor->get_name() << "," << iterator->first.type << "," << iterator->first.parameter << "> : " << iterator->second << endl; 
      weight = advisor->get_weight();
      //cout << "Weight for this advisor : " << weight << endl;
      // if(advisor->get_name() == "Explorer" or advisor->get_name() == "ExplorerRotation" or advisor->get_name() == "LearnSpatialModel" or advisor->get_name() == "LearnSpatialModelRotation" or advisor->get_name() == "Curiosity" or advisor->get_name() == "CuriosityRotation"){
      //   weight = beliefs->getAgentState()->getAgenda().size()/5;
      // }

      advisorCommentsList << advisor->get_name() << " " << iterator->first.type << " " << iterator->first.parameter << " " << iterator->second << ";";
      
      cout << "Start of score aggregation" << endl;
      
      if( allComments.find(iterator->first) == allComments.end()){
	    allComments[iterator->first] =  iterator->second * weight;
      }
      else{
	    allComments[iterator->first] += iterator->second * weight;
      }
    }
  } 
  
  cout << "After score aggregation" << endl;
  // Loop through map advisor created and find command with the highest vote
  double maxAdviceStrength = -1000.0;
  double maxWeight;
  for(mapIt iterator = allComments.begin(); iterator != allComments.end(); iterator++){
    double action_weight = 1.0;
    cout << "Values are : " << iterator->first.type << " " << iterator->first.parameter << " with value: " << iterator->second << " and weight: " << action_weight << endl;
    if(action_weight * iterator->second > maxAdviceStrength){
      maxAdviceStrength = action_weight * iterator->second;
      maxWeight = action_weight;
    }
  }
  cout << "Max vote strength " << maxAdviceStrength << endl;
  
  for(mapIt iterator = allComments.begin(); iterator!=allComments.end(); iterator++){
    if(maxWeight * iterator->second == maxAdviceStrength)
      best_decisions.push_back(iterator->first);
  }
  
  cout << "There are " << best_decisions.size() << " decisions that got the highest grade " << endl;
  if(best_decisions.size() == 0){
      (*decision) = FORRAction(PAUSE,0);
  }
  //for(unsigned i = 0; i < best_decisions.size(); ++i)
      //cout << "Action type: " << best_decisions.at(i).type << " parameter: " << best_decisions.at(i).parameter << endl;
    
  //generate random number using system clock as seed
  srand(time(NULL));
  int random_number = rand() % (best_decisions.size());
    
  (*decision) = best_decisions.at(random_number);
  decisionStats.advisors = advisorsList.str();
  decisionStats.advisorComments = advisorCommentsList.str();
  cout << " advisors = " << decisionStats.advisors << "\nadvisorComments = " << decisionStats.advisorComments << endl;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Checks if an T3 advisor is active
//
//
bool Controller::
isAdvisorActive(string advisorName){
  bool isActive = false;
  
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = it->get();
    if(advisor->is_active() == true && advisor->get_name() == advisorName)
      isActive = true;
  }
  
  return isActive;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Check influence of tier 3 Advisors
//
//
void Controller::tierThreeAdvisorInfluence(){
  /*std::map<FORRAction, double> comments;
  // This map will aggregate value of all advisers
  std::map<FORRAction, double> allComments;

  // typedef to make for declaration that iterates over map shorter
  typedef map<FORRAction, double>::iterator mapIt;

  // vector of all the actions that got max comment strength in iteration
  vector<FORRAction> best_decisions;
  
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = it->get();

    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      continue;
    }
    if(advisor->is_commenting() == false){
      continue;
    }

    comments = advisor->allAdvice();
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      // If this is first advisor we need to initialize our final map
      float weight;
      weight = advisor->get_weight();

      if( allComments.find(iterator->first) == allComments.end()){
      allComments[iterator->first] =  iterator->second * weight;
      }
      else{
      allComments[iterator->first] += iterator->second * weight;
      }
    }
  } 
  
  // Loop through map advisor created and find command with the highest vote
  double maxAdviceStrength = -1000;
  for(mapIt iterator = allComments.begin(); iterator != allComments.end(); iterator++){
    cout << "Values are : " << iterator->first.type << " " << iterator->first.parameter << " with value: " << iterator->second << endl;
    if(iterator->second > maxAdviceStrength){
      maxAdviceStrength = iterator->second;
    }
  }
  //cout << "Max vote strength " << maxAdviceStrength << endl;
  
  for(mapIt iterator = allComments.begin(); iterator!=allComments.end(); iterator++){
    if(iterator->second == maxAdviceStrength)
      best_decisions.push_back(iterator->first);
  }
  
  std::stringstream advisorsInfluence;
  std::map<FORRAction, double> takeOneOutComments;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    takeOneOutComments = allComments;
    Tier3Advisor *advisor = it->get();

    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      continue;
    }
    if(advisor->is_commenting() == false){
      advisorsInfluence << advisor->get_name() << " -1;";
      continue;
    }

    comments = advisor->allAdvice();
    // aggregate all comments
    if (comments.size() > 1) {
      for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
        // If this is first advisor we need to initialize our final map
        float weight;
        weight = advisor->get_weight();
        takeOneOutComments[iterator->first]-= iterator->second * weight;
      }
      double maxAdviceStrength = -1000;
      for(mapIt iterator = takeOneOutComments.begin(); iterator != takeOneOutComments.end(); iterator++){
        if(iterator->second > maxAdviceStrength){
          maxAdviceStrength = iterator->second;
        }
      }
      bool same=0;
      for(mapIt iterator = takeOneOutComments.begin(); iterator!=takeOneOutComments.end(); iterator++){
        if(iterator->second == maxAdviceStrength){
          for(unsigned i = 0; i < best_decisions.size(); ++i){
            if(iterator->first.type == best_decisions.at(i).type and iterator->first.parameter == best_decisions.at(i).parameter) {
              same = 0;
            }
            else{
              same = 1;
            }
          }
        }
      }
      if (same == 0){
        advisorsInfluence << advisor->get_name() << " 0;";
      }
      else{
        advisorsInfluence << advisor->get_name() << " 1;";
      }
    }
    else {
      advisorsInfluence << advisor->get_name() << " -1;";
    }
  }
  decisionStats.advisorInfluence = advisorsInfluence.str();
  cout << "advisorInfluence = " << decisionStats.advisorInfluence << endl;*/
  decisionStats.advisorInfluence = "";
}
