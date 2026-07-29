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
#include <semaforr/config/Configuration.h>
#include <semaforr/decision/Beliefs.h>
#include <semaforr/decision/Tier1Advisor.h>
#include <semaforr/decision/Tier3Advisor.h>
#include <semaforr/core/FORRActionStats.h>
#include <semaforr/navigation/PathPlanner.h>
#include <semaforr/exploration/HighwayExplore.h>
#include <semaforr/exploration/FrontierExplore.h>
#include <semaforr/exploration/Circumnavigate.h>
#include <semaforr/spatial/FORRPassages.h>

#include <semaforr/domain/SensorTypes.h>


// Forward-declare Controller so the typedef below can reference it
class Controller;
typedef std::vector<std::unique_ptr<Tier3Advisor>>::iterator advisor3It;
typedef std::vector<std::unique_ptr<PathPlanner>>::iterator planner2It;

// Public navigation controller facade.
class Controller {
public:

  Controller(string, string, string, string, string);
  explicit Controller(semaforr::config::Configuration configuration);
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  
  //main sense decide loop, receives the input messages and calls the FORRDecision function
  FORRAction decide();

  FORRActionStats *getCurrentDecisionStats() { return &decisionStats; }
  void clearCurrentDecisionStats() { decisionStats = FORRActionStats();}

  //Update state of the agent using sensor readings 
  void updateState(
    Position current,
    const semaforr::domain::LaserScan& laserscan,
    const semaforr::domain::PoseArray& crowdpose,
    const semaforr::domain::PoseArray& crowdposeall);

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
  
  //Tier 1 advisors are called here
  bool tierOneDecision(FORRAction *decision);

  //Tier 2 planners are called here
  void tierTwoDecision(Position current, bool selectNextTask);

  //Tier 3 advisors are called here
  void tierThreeDecision(FORRAction *decision);

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
};
  
#endif /* CONTROLLER_H */
