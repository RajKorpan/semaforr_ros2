#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <map>
#include <set>
#include <string>
#include <time.h>
#include <limits.h>
#include <cstdlib>
#include <time.h>
#include <math.h>
#include <memory>
#include <vector>
#include <utility>

// SemaFORR
#include <semaforr/config/Configuration.hpp>
#include <semaforr/decision/Beliefs.hpp>
#include <semaforr/decision/DecisionTier.hpp>
#include <semaforr/decision/Tier1Advisor.hpp>
#include <semaforr/decision/Tier3Advisor.hpp>
#include <semaforr/core/FORRActionStats.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/navigation/PathPlanner.hpp>
#include <semaforr/exploration/HighwayExplore.hpp>
#include <semaforr/exploration/FrontierExplore.hpp>
#include <semaforr/exploration/Circumnavigate.hpp>
#include <semaforr/spatial/FORRPassages.hpp>

#include <semaforr/domain/SensorTypes.hpp>
#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/social/crowd_field_learner.hpp>


// Public navigation controller facade.
class Controller {
public:

  Controller(string, string, string, string, string);
  explicit Controller(semaforr::config::Configuration configuration);
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  
  //main sense decide loop, receives the input messages and calls the FORRDecision function
  semaforr::decision::DecisionResult decide();

  //Update state of the agent using sensor readings 
  void updateState(
    Position current,
    const semaforr::domain::LaserScan& laserscan,
    const semaforr::domain::CrowdState& crowd);

  //Returns the state of the robots mission (True 
  bool isMissionComplete();

  // getter for beliefs
  Beliefs *getBeliefs() { return beliefs.get(); }

  // getter for planner
  PathPlanner *getPlanner() { return planner; }

  std::vector<PathPlanner*> getPlanners() {
    std::vector<PathPlanner*> planners;
    planners.reserve(tier2Planners.size());
    for (const auto& ownedPlanner : tier2Planners) {
      planners.push_back(ownedPlanner.get());
    }
    return planners;
  }

  void updatePlannersModels(const semaforr::domain::CrowdModel& c) {
    for (const auto& ownedPlanner : tier2Planners) {
      ownedPlanner->setCrowdModel(c);
    }
  }

  const semaforr::domain::CrowdModel& getCrowdModel() const noexcept {
    return crowdModel;
  }

  HighwayExplorer *gethighwayExploration() { return highwayExploration.get(); }
  FrontierExplorer *getfrontierExploration() { return frontierExploration.get(); }

  bool getHighwayFinished(){
    if(highwayFinished >= 1){
      return true;
    }
    else{
      return false;
    }
  }

  bool getFrontierFinished(){
    if(frontierFinished >= 1){
      return true;
    }
    else{
      return false;
    }
  }

  int getHighwaysOn(){
    if(highwaysOn){
      return 1;
    }
    else if(frontiersOn){
      return 2;
    }
    else{
      return 0;
    }
  }

private:

  //FORR decision loop and tiers
  FORRAction FORRDecision();

  FORRActionStats decisionStats;
  
  void planForCurrentTask(Position current, bool selectNextTask);

  // learns the spatial model and updates the beliefs
  void learnSpatialModel(AgentState *agentState, bool taskStatus, bool earlyLearning);
  void updateSkeletonGraph(AgentState* agentState);

  void initialize_advisors(
    const std::vector<semaforr::config::AdvisorConfiguration>& advisors);
  void initialize_tasks(
    const std::vector<semaforr::config::TaskConfiguration>& tasks,
    int length,
    int height);
  void initialize_planner(
    const semaforr::config::MapDimensions& dimensions);
  void initialize_crowd_learning(
    const semaforr::config::Configuration& configuration);
  
  // Knowledge component of robot
  std::unique_ptr<Beliefs> beliefs;

  std::unique_ptr<HighwayExplorer> highwayExploration;
  std::unique_ptr<FrontierExplorer> frontierExploration;
  std::unique_ptr<Circumnavigate> circumnavigator;

  // An ordered list of advisors that are consulted by Controller::FORRDecision
  std::unique_ptr<Tier1Advisor> tier1;
  PathPlanner *planner = nullptr;
  std::vector<std::unique_ptr<PathPlanner>> tier2Planners;
  std::vector<std::unique_ptr<Tier3Advisor>> tier3Advisors;
  semaforr::domain::CrowdModel crowdModel;
  std::unique_ptr<semaforr::social::CrowdFieldLearner> crowdLearner;
  
  double canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle, highwayDistanceThreshold, highwayTimeThreshold, highwayDecisionThreshold;
  double arrMove[300];
  double arrRotate[300];
  int moveArrMax, rotateArrMax;
  int taskDecisionLimit;
  int planLimit;
  bool trailsOn;
  bool conveyorsOn;
  bool regionsOn;
  bool doorsOn;
  bool hallwaysOn;
  bool barrsOn;
  bool aStarOn;
  bool highwaysOn;
  bool frontiersOn;
  bool outofhereOn;
  bool doorwayOn;
  bool findawayOn;
  bool behindOn;
  bool dontgobackOn;
  bool firstTaskAssigned;
  int highwayFinished;
  int frontierFinished;
  bool skeleton;
  bool hallwayskel;
  semaforr::config::PlannerConfiguration plannerConfiguration;

  std::unique_ptr<semaforr::decision::TierOneDecision> tierOneDecision;
  std::unique_ptr<semaforr::decision::TierTwoDecision> tierTwoDecision;
  std::unique_ptr<semaforr::decision::TierThreeDecision> tierThreeDecision;
};
  
#endif /* CONTROLLER_H */
