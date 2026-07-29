/*
 * Controller mission lifecycle and top-level action orchestration.
 */

#include <semaforr/decision/Controller.h>
#include <semaforr/core/FORRGeometry.h>
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
  const semaforr::domain::PoseArray& crowdpose,
  const semaforr::domain::PoseArray& crowdposeall){
  cout << "In update state" << endl;
  beliefs->getAgentState()->setCurrentSensor(current, laser_scan);
  beliefs->getAgentState()->setCrowdPose(crowdpose);
  beliefs->getAgentState()->setCrowdPoseAll(crowdposeall);
  if(firstTaskAssigned == false){
      cout << "Set first task" << endl;
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
          planForCurrentTask(current, true);
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
