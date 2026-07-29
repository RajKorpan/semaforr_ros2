/*
 * Controller tier-two planning and plan selection.
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
