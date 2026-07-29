/*
 * Controller spatial learning and graph maintenance.
 */

#include <semaforr/decision/Controller.hpp>
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

namespace {

bool isCrowdCostPlanner(const std::string& name)
{
  return name == "density" || name == "risk" ||
    name == "flow" || name == "combined";
}

void copyNavigationGraph(const Graph& source, Graph& destination)
{
  destination.resetGraph();
  for (Node* node : source.getNodes()) {
    destination.addNode(
      node->getX(), node->getY(), node->getRadius(), node->getID());
  }
  for (Edge* edge : source.getEdges()) {
    destination.addEdge(
      edge->getFrom(), edge->getTo(), edge->getDistCost(),
      edge->getEdgePath(true));
  }
}

}  // namespace

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
    (void)0;
    PathPlanner *skeleton_planner;
    PathPlanner *hallway_skeleton_planner;
    for (const auto& ownedPlanner : tier2Planners){
      if(skeleton and ownedPlanner->getName() == "skeleton"){
        skeleton_planner = ownedPlanner.get();
      }
      if(hallwayskel and ownedPlanner->getName() == "hallwayskel"){
        hallway_skeleton_planner = ownedPlanner.get();
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
      (void)0;
      skeleton_planner->getGraph()->printGraph();
      (void)0;
      for (const auto& ownedPlanner : tier2Planners) {
        if (isCrowdCostPlanner(ownedPlanner->getName())) {
          copyNavigationGraph(
            *skeleton_planner->getGraph(), *ownedPlanner->getGraph());
        }
      }
    }
    if(hallwayskel){
      (void)0;
      // skeleton_planner->getOrigGraph()->printGraph();
      beliefs->getSpatialModel()->getRegionList()->setRegionPassageValues(beliefs->getAgentState()->getPassageGrid());
      // cout << "Connected Graph: " << skeleton_planner->getOrigGraph()->isConnected() << endl;
    }
  }
  if(hallwayskel and (highwayFinished == 1 or frontierFinished == 1)){
    PathPlanner *hwskeleton_planner;
    for (const auto& ownedPlanner : tier2Planners){
      if(ownedPlanner->getName() == "hallwayskel"){
        hwskeleton_planner = ownedPlanner.get();
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
