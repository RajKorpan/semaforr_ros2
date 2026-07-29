/*
 * Controller tier-one and tier-three decision pipeline.
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
