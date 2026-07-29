/*
 * Controller mission lifecycle and top-level action orchestration.
 */

#include <semaforr/decision/Controller.hpp>
#include <semaforr/core/action_adapter.hpp>
#include <semaforr/core/FORRGeometry.hpp>
#include <unistd.h>

#include <algorithm>
#include <deque>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;
// Function which takes sensor inputs and updates it for semaforr to use for decision making, and updates task status
void Controller::updateState(
  Position current,
  const semaforr::domain::LaserScan& laser_scan,
  const semaforr::domain::CrowdState& crowd){
  beliefs->getAgentState()->setCurrentSensor(current, laser_scan);
  crowdModel.observations() = crowd;
  if (crowdLearner && crowd.current()) {
    semaforr::domain::LaserObservation laser;
    const double angle_min = std::isfinite(laser_scan.angle_min)
      ? static_cast<double>(laser_scan.angle_min) : 0.0;
    const double angle_increment =
      std::isfinite(laser_scan.angle_increment) &&
        laser_scan.angle_increment > 0.0F
      ? static_cast<double>(laser_scan.angle_increment)
      : laserScanRadianIncrement;
    const double minimum_range =
      std::isfinite(laser_scan.range_min) && laser_scan.range_min >= 0.0F
      ? static_cast<double>(laser_scan.range_min) : 0.0;
    const double maximum_range =
      std::isfinite(laser_scan.range_max) &&
        laser_scan.range_max >= minimum_range
      ? static_cast<double>(laser_scan.range_max) : maxLaserRange;
    laser.angle_min = semaforr::domain::Angle(angle_min);
    laser.angle_increment = semaforr::domain::Angle(angle_increment);
    laser.minimum_range = semaforr::domain::Distance(minimum_range);
    laser.maximum_range =
      semaforr::domain::Distance(std::max(maximum_range, minimum_range));
    laser.ranges_m.reserve(laser_scan.ranges.size());
    for (const float range : laser_scan.ranges) {
      laser.ranges_m.push_back(static_cast<double>(range));
    }
    const semaforr::domain::Pose2D pose{
      {current.getX(), current.getY()},
      semaforr::domain::Angle(current.getTheta())};
    if (crowdLearner->observe(pose, laser, *crowd.current())) {
      crowdModel.setLearned(crowdLearner->snapshot());
    }
  }
  beliefs->getAgentState()->setCrowdModel(crowdModel);
  updatePlannersModels(crowdModel);
  if(firstTaskAssigned == false){
      // if(aStarOn and (!highwaysOn or (highwaysOn and highwayExploration->getHighwaysComplete())) and (!frontiersOn or (frontiersOn and frontierExploration->getFrontiersComplete()))){
      //   planForCurrentTask(current, true);
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
        planForCurrentTask(current, true);
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
      // RCLCPP_DEBUG(this->get_logger(), "Target Achieved, moving on to next target!!");
      //Learn spatial model only on tasks completed successfully
      if(beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size() <= 2000){
        learnSpatialModel(beliefs->getAgentState(), true, false);
        // RCLCPP_DEBUG(this->get_logger(), "Finished Learning Spatial Model!!");
        updateSkeletonGraph(beliefs->getAgentState());
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
      //// RCLCPP_DEBUG(this->get_logger(), "Task Cleared!!");
      //cout << "Agenda Size = " << beliefs->getAgentState()->getAgenda().size() << endl;
      if(beliefs->getAgentState()->getAgenda().size() > 0){
        //Tasks the next task , current position and a planner and generates a sequence of waypoints if astaron is true
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Controller.cpp taskCount > " << (beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) << " planLimit " << (planLimit - 1));
        if((beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) > (planLimit - 1)){
          aStarOn = false;
        }
        // RCLCPP_DEBUG(this->get_logger(), "Selecting Next Task");
        if(aStarOn){
          planForCurrentTask(current, true);
          // RCLCPP_DEBUG(this->get_logger(), "Next Plan Generated!!");
        }
        else{
          beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
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
        planForCurrentTask(current, false);
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
    //   planForCurrentTask(current, false);
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
        // RCLCPP_DEBUG(this->get_logger(), "Finished Learning Spatial Model!!");
        updateSkeletonGraph(beliefs->getAgentState());
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
            planForCurrentTask(current, true);
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
semaforr::decision::DecisionResult Controller::decide() {
  semaforr::decision::DecisionResult result;
  FORRAction decidedAction;
  if(!highwayExploration->getHighwaysComplete() and highwaysOn){
    decidedAction = highwayExploration->exploreDecision(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
    result.action = semaforr::core::toDomainAction(decidedAction);
    result.source = semaforr::decision::DecisionSource::Exploration;
    result.tier = semaforr::decision::DecisionTier::Exploration;
    result.selected_policy = "highway_exploration";
  }
  else if(!frontierExploration->getFrontiersComplete() and frontiersOn){
    decidedAction = frontierExploration->exploreDecision(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
    result.action = semaforr::core::toDomainAction(decidedAction);
    result.source = semaforr::decision::DecisionSource::Exploration;
    result.tier = semaforr::decision::DecisionTier::Exploration;
    result.selected_policy = "frontier_exploration";
  }
  else{
    if(highwayFinished < 3){
      highwayFinished++;
    }
    if(frontierFinished < 3){
      frontierFinished++;
    }
    result = FORRDecision();
    decidedAction = semaforr::core::toLegacyAction(result.action);
  }

  AgentState* const agent_state = beliefs->getAgentState();
  const Position position = agent_state->getCurrentPosition();
  result.sequence = ++decisionSequence;
  result.robot_pose = {
    {position.getX(), position.getY()},
    semaforr::domain::Angle(position.getTheta())};
  result.candidates.reserve(agent_state->getActionSet()->size());
  for (const FORRAction& candidate : *agent_state->getActionSet()) {
    result.candidates.push_back(semaforr::core::toDomainAction(candidate));
  }
  std::sort(result.candidates.begin(), result.candidates.end());
  result.candidates.erase(
    std::unique(result.candidates.begin(), result.candidates.end()),
    result.candidates.end());

  Task* const task = agent_state->getCurrentTask();
  task->incrementDecisionCount();
  task->saveDecision(decidedAction);
  semaforr::decision::TaskDiagnostic task_diagnostic;
  const std::size_t all_tasks = agent_state->getAllAgenda().size();
  const std::size_t pending_tasks = agent_state->getAgenda().size();
  task_diagnostic.task_index =
    static_cast<std::uint64_t>(
      all_tasks >= pending_tasks ? all_tasks - pending_tasks : 0U);
  task_diagnostic.decision_count =
    static_cast<std::uint64_t>(task->getDecisionCount());
  task_diagnostic.target = {task->getTaskX(), task->getTaskY()};
  if (task->getIsPlanActive()) {
    task_diagnostic.waypoint =
      semaforr::domain::Point2D{task->getX(), task->getY()};
  }
  result.task = task_diagnostic;

  if (!decisionStats.chosenPlanner.empty() &&
      decisionStats.chosenPlanner.find_first_not_of(' ') !=
        std::string::npos) {
    result.planner = decisionStats.chosenPlanner;
  } else {
    const std::string planner_name = task->getPlannerName();
    if (!planner_name.empty() && planner_name != "none") {
      result.planner = planner_name;
    }
  }

  agent_state->clearVetoedActions();
  decisionStats = FORRActionStats();
  return result;
}
