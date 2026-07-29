/*!
  \file PathPlanner.cpp
  \addtogroup PathPlanner
  @{
 */

#include <semaforr/navigation/PathPlanner.hpp>
#include <limits.h>
#include <algorithm>

#define PATH_DEBUG true

/*!
  \brief Calculates the shortest path from a start point to a destination point on the navigation graph.

  This function makes necessary calls to populate the PathPlanner::path list, which is a list of node indexes. The list doesn't contain the source (start point) and the target (destination point), only the nodes that the robot needs to get to in sequence, in order to reach its destination.

  First A* algorithm is run on the navigation graph and then the path is smoothed by removing unnecessary waypoints.

  \b Warning a source and a target must be specified prior to this function call.

  \return 0 if path is calculated, 1 if source is not a node or accessible to a valid node, 2 if target is not a node or accessible to a valid node, 3 if no path is found between the source and the target

 */
int PathPlanner::calcPath(bool cautious){
  const string signature = "PathPlanner::calcPath()> ";

  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){

    if(PATH_DEBUG) {
      cout << signature << "Source:";
      source.printNode();
      cout << endl;
      cout << signature << "Target:";
      target.printNode();
      cout << endl;
    }
    if(name == "skeleton" or name == "hallwayskel"){
      if(!navGraph->isConnected()){
        return 5;
      }
    }

    Node s, t;
    Node rs, rt, ts, tt;
    if ( navGraph->isNode(source) ) {
      if(PATH_DEBUG)
        cout << signature << "Source is a valid Node in the navigation graph" << endl;
      s = source ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Source is not a valid Node in the navigation graph. Getting closest valid node." << endl;
      if(name == "skeleton"){
        s = getClosestNode(source, target, false);
      }
      else if(name == "hallwayskel"){
        vector<Node> start_nodes = getClosestNodes(source, target, true);
        s = start_nodes[0];
        rs = start_nodes[1];
        rt = start_nodes[2];
      }
      else{
        s = getClosestNode(source, target, false);
      }
    }
    //cout << signature << "Checking if source node is invalid" << endl;
    if ( s.getID() == Node::invalid_node_index )
      return 1;

    if ( navGraph->isNode(target) ) {
      if(PATH_DEBUG)
        cout << signature << "Target is a valid Node in the navigation graph" << endl;
      t = target ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Target is not a valid Node in the navigation graph. Getting closest valid node." << endl;
      if(name == "skeleton"){
        t = getClosestNode(target, source, true);
      }
      else if(name == "hallwayskel"){
        vector<Node> end_nodes = getClosestNodes(target, source, true);
        t = end_nodes[0];
        ts = end_nodes[2];
        tt = end_nodes[1];
      }
      else{
        t = getClosestNode(target, source, false);
      }
    }
    //cout << signature << "Checking if target node is invalid" << endl;
    if ( t.getID() == Node::invalid_node_index )
      return 2;

    //cout << signature << "Completed finding source and destination nodes" << endl;
    if(PATH_DEBUG) {
      cout << signature << "s:";
      s.printNode();
      cout << endl;
      cout << signature << "t:";
      t.printNode();
      cout << endl;
    }
    if(s.getID() == t.getID() and name != "hallwayskel")
      return 4;
    //cout << signature << "Updating nav graph" << endl;
    // update the nav graph with the latest crowd model to change the edge weights
    if (name != "distance" and name != "skeleton" and name != "hallwayskel") {
      cout << "Updating nav graph for non-distance planners" << endl;
      updateNavGraph();
      cout << "Finished nav graph update" << endl;
    }
    astar newsearch(*navGraph, s, t, name);
    cout << "Finished search" << endl;
    // cout << "finished search" << endl;
    if ( newsearch.isPathFound() ) {
      path = newsearch.getPathToTarget();
      // cout << "got path" << endl;
      paths = newsearch.getPathsToTarget();
      // cout << "got paths" << endl;
      objectiveSet = false;
      pathCompleted = false;

      // for(int i = 0; i < otherIntersection.size(); i++){
      //   if(usedOtherIntersection[i] == true){
      //     bool otherfound = false;
      //     list<int>::iterator iter;
      //     for ( iter = path.begin(); iter != path.end(); iter++ ){
      //       if((*iter) == otherIntersection[i].getID()){
      //         otherfound = true;
      //         break;
      //       }
      //     }
      //     if(otherfound == false){
      //       if(i == 0){
      //         path.insert(path.begin(), otherIntersection[i].getID());
      //       }
      //       else if(i == 1){
      //         path.push_back(otherIntersection[i].getID());
      //       }
      //     }
      //     paths.clear();
      //     paths.push_back(path);
      //   }
      // }

      if(!cautious)
        smoothPath(path, s, t);
      pathCost = 0;
      pathCosts.clear();
      pathCost = calcPathCost(path);
      // cout << "calculated path cost" << endl;
      pathCalculated = true;
      for (int i=0; i<paths.size(); i++){
        pathCosts.push_back(calcPathCost(paths[i]));
      }
      if(name == "hallwayskel"){
        origPathCost = 0;
        origPathCosts.clear();
        if(rs.getID() != Node::invalid_node_index and rt.getID() != Node::invalid_node_index){
          if(PATH_DEBUG) {
            cout << signature << "rs:";
            rs.printNode();
            cout << endl;
            cout << signature << "rt:";
            rt.printNode();
            cout << endl;
          }
          astar rnewsearch(*originalNavGraph, rs, rt, name);
          if ( rnewsearch.isPathFound()) {
            origPath1 = rnewsearch.getPathToTarget();
            origPaths.push_back(origPath1);
            // cout << "prologue path found " << origPath1.size() << " " << origPaths.size() << endl;
            origObjectiveSet = false;
            origPathCompleted = false;

            if(!cautious)
              smoothPath(origPath1, rs, rt);

            origPathCost += calcOrigPathCost(origPath1);
            origPathCalculated = true;
            origPathCosts.push_back(calcOrigPathCost(origPath1));
          }
          else{
            origPaths.push_back(list<int>());
            origPathCost += 0;
            origPathCosts.push_back(0);
          }
        }
        else{
          origPaths.push_back(list<int>());
          origPathCost += 0;
          origPathCosts.push_back(0);
        }
        if(ts.getID() != Node::invalid_node_index and tt.getID() != Node::invalid_node_index){
          if(PATH_DEBUG) {
            cout << signature << "ts:";
            ts.printNode();
            cout << endl;
            cout << signature << "tt:";
            tt.printNode();
            cout << endl;
          }
          astar tnewsearch(*originalNavGraph, ts, tt, name);
          if ( tnewsearch.isPathFound()) {
            origPath2 = tnewsearch.getPathToTarget();
            origPaths.push_back(origPath2);
            // cout << "epilogue path found " << origPath2.size() << " " << origPaths.size() << endl;
            origObjectiveSet = false;
            origPathCompleted = false;

            if(!cautious)
              smoothPath(origPath2, ts, tt);

            origPathCost += calcOrigPathCost(origPath2);
            origPathCalculated = true;
            origPathCosts.push_back(calcOrigPathCost(origPath2));
          }
          else{
            origPaths.push_back(list<int>());
            origPathCost += 0;
            origPathCosts.push_back(0);
          }
        }
        else{
          origPaths.push_back(list<int>());
          origPathCost += 0;
          origPathCosts.push_back(0);
        }
        if(rs.getID() != Node::invalid_node_index and tt.getID() != Node::invalid_node_index){
          if(PATH_DEBUG) {
            cout << signature << "rs:";
            rs.printNode();
            cout << endl;
            cout << signature << "tt:";
            tt.printNode();
            cout << endl;
          }
          astar rtnewsearch(*originalNavGraph, rs, tt, name);
          if ( rtnewsearch.isPathFound()) {
            origPath3 = rtnewsearch.getPathToTarget();
            origPaths.push_back(origPath3);
            // cout << "epilogue path found " << origPath3.size() << " " << origPaths.size() << endl;

            if(!cautious)
              smoothPath(origPath3, rs, tt);

            origPathCosts.push_back(calcOrigPathCost(origPath3));
          }
          else{
            origPaths.push_back(list<int>());
            origPathCost += 0;
            origPathCosts.push_back(0);
          }
        }
        else{
          origPaths.push_back(list<int>());
          origPathCost += 0;
          origPathCosts.push_back(0);
        }
        cout << "Plan " << name << " cost = " << pathCost << " origPathCost " << origPathCost << endl;
      }
      else{
        cout << "Plan " << name << " cost = " << pathCost << endl;
      }
    }
    else {
      return 3;
    }
  }
  return 0;
}

int PathPlanner::calcOrigPath(bool cautious){
  const string signature = "PathPlanner::calcOrigPath()> ";

  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){

    if(PATH_DEBUG) {
      cout << signature << "Source:"; 
      source.printNode(); 
      cout << endl;
      cout << signature << "Target:"; 
      target.printNode();
      cout << endl;
    }

    Node s, t;
    if ( originalNavGraph->isNode(source) ) {
      if(PATH_DEBUG)
        cout << signature << "Source is a valid Node in the navigation graph" << endl; 
      s = source ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Source is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
      s = getClosestNode(source, target, false);
    }
    //cout << signature << "Checking if source node is invalid" << endl;
    if ( s.getID() == Node::invalid_node_index )
      return 1;

    if ( originalNavGraph->isNode(target) ) {
      if(PATH_DEBUG)
        cout << signature << "Target is a valid Node in the navigation graph" << endl; 
      t = target ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Target is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
      t = getClosestNode(target, source, false);
    }
    //cout << signature << "Checking if target node is invalid" << endl;
    if ( t.getID() == Node::invalid_node_index )
      return 2;

    //cout << signature << "Completed finding source and destination nodes" << endl;
    if(PATH_DEBUG) {
      cout << signature << "s:"; 
      s.printNode(); 
      cout << endl;
      cout << signature << "t:"; 
      t.printNode();
      cout << endl;
    }

    astar newsearch(*originalNavGraph, s, t, "distance");
    if ( newsearch.isPathFound() ) {
      origPath = newsearch.getPathToTarget();
      origObjectiveSet = false;
      origPathCompleted = false;

      if(!cautious)
        smoothPath(origPath, s, t);

      origPathCost = calcOrigPathCost(origPath); 
      origPathCalculated = true;
    }
    else {
      return 3;
    }
  }
  return 0;
}

void PathPlanner::updateNavGraph(){
	cout << "Updating nav graph before" << endl;
	if(!crowdModel.learnedAvailable() and
      (name == "density" or name == "risk" or name == "flow")){
		cout << "learned crowd field not available" << endl;
	}
	else{
		vector<Edge*> edges = navGraph->getEdges();
		// compute the extra cost imposed by crowd model on each edge in navGraph
		for(int i = 0; i < edges.size(); i++){
			Node toNode = navGraph->getNode(edges[i]->getTo());
			Node fromNode = navGraph->getNode(edges[i]->getFrom());
			double oldcost = edges[i]->getDistCost();
			double newEdgeCostft = computeNewEdgeCost(fromNode, toNode, true, oldcost);
			double newEdgeCosttf = computeNewEdgeCost(fromNode, toNode, false, oldcost); 
			navGraph->updateEdgeCost(i, newEdgeCostft, newEdgeCosttf);
			//cout << "Edge Cost " << oldcost << " -> " << newEdgeCostft << " -> " << newEdgeCosttf << endl;
		}
	}
}


double PathPlanner::computeNewEdgeCost(Node s, Node d, bool direction, double oldcost){
	int b = 30;
  // weights that balance distance, crowd density and crowd flow
  int w1 = 1;
  int w2 = 500;
  int w3 = 500;
  int w4 = 500;
  int w5 = 500;
  int w6 = 1;
  int w7 = 1;
  int w8 = 1;
  if (name == "safe"){
    //cout << "Updating smooth nav graph" << endl;
    //double smooth_cost = (oldcost * 5);
    // double smooth_cost = 1;
    // return (w6 * smooth_cost);
    double s_cost = s.getDistWall();
    double d_cost = d.getDistWall();
    if(s_cost > 0 or d_cost > 0){
      return (w1 * oldcost + (w6 * 10/((s_cost + d_cost)/2)));
    }
    else{
      return (w1 * oldcost) * 10;
    }
  }
  if (name == "explore"){
    double ns_cost = novelCost(s.getX(), s.getY());
    double nd_cost = novelCost(d.getX(), d.getY());
    /*if (ns_cost > 0 or nd_cost > 0) {
      cout << "old cost = " << oldcost << " novelCost = " << (ns_cost+nd_cost)/2 << " combined cost = " << (w1 * oldcost) + (w5 * (ns_cost+nd_cost)/2) << endl;
    }*/
    //return (w1 * oldcost) + (w5 * (ns_cost+nd_cost)/2);
    if (ns_cost > 0 or nd_cost > 0){
      return (w1 * oldcost) + (w1 * (ns_cost+nd_cost)/2);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "novel"){
    int sRegion=-1,dRegion=-1;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].inRegion(s.getX()/100.0, s.getY()/100.0)){
        sRegion = i;
      }
      if(regions[i].inRegion(d.getX()/100.0, d.getY()/100.0)){
        dRegion = i;
      }
      if(sRegion >= 0 and dRegion >= 0){
        break;
      }
    }

    double s_door_min_distance = std::numeric_limits<double>::infinity();
    double s_exit_min_distance = std::numeric_limits<double>::infinity();
    if(sRegion >= 0){
      CartesianPoint sPoint = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
      for(int i = 0; i < doors[sRegion].size(); i++) {
        double doorDistance = doors[sRegion][i].distanceToDoor(sPoint, regions[sRegion]);
        if (doorDistance < s_door_min_distance){
          s_door_min_distance = doorDistance;
        }
        if(s_door_min_distance <= 0.5){
          break;
        }
      }
      vector<FORRExit> exits = regions[sRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = sPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < s_exit_min_distance){
          s_exit_min_distance = exitDistance;
        }
        if(s_exit_min_distance <= 0.5){
          break;
        }
      }
    }

    double d_door_min_distance = std::numeric_limits<double>::infinity();
    double d_exit_min_distance = std::numeric_limits<double>::infinity();
    if(dRegion >= 0){
      CartesianPoint dPoint = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
      for(int i = 0; i < doors[dRegion].size(); i++) {
        double doorDistance = doors[dRegion][i].distanceToDoor(dPoint, regions[dRegion]);
        if (doorDistance < d_door_min_distance){
          d_door_min_distance = doorDistance;
        }
        if(d_door_min_distance <= 0.5){
          break;
        }
      }
      vector<FORRExit> exits = regions[dRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = dPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < d_exit_min_distance){
          d_exit_min_distance = exitDistance;
        }
        if(d_exit_min_distance <= 0.5){
          break;
        }
      }
    }
    double regioncost;
    if (sRegion >= 0 and dRegion >= 0){
      regioncost = (w1 * oldcost) * 10;
    }
    else if (sRegion >= 0 and dRegion == -1){
      if(s_door_min_distance <= 0.5 and s_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 7;
      }
      else if(s_door_min_distance <= 0.5 or s_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 5;
      }
      else{
        regioncost = (w1 * oldcost) * 3;
      }
    }
    else if (sRegion ==-1 and dRegion >= 0){
      if(d_door_min_distance <= 0.5 and d_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 7;
      }
      else if(d_door_min_distance <= 0.5 or d_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 5;
      }
      else{
        regioncost = (w1 * oldcost) * 3;
      }
    }
    else{
      regioncost = (w1 * oldcost) * 1;
    }
    CartesianPoint snode = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
    CartesianPoint dnode = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
    double sHallway=0, dHallway=0;
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(snode)){
        sHallway++;
      }
      if(hallways[i].pointInAggregate(dnode)){
        dHallway++;
      }
      if(sHallway > 0 and dHallway > 0){
        break;
      }
    }
    double hallwaycost;
    if (sHallway > 0 and dHallway > 0){
      hallwaycost = (w1 * oldcost) * 10;
    }
    else if (sHallway > 0 or dHallway > 0){
      hallwaycost = (w1 * oldcost) * 7;
    }
    else{
      hallwaycost = (w1 * oldcost) * 1;
    }
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    double conveycost;
    if (sconveycost > 0 and dconveycost > 0){
      conveycost = (w7 * oldcost * ((sconveycost + dconveycost)/2));
    }
    else{
      conveycost = (w7 * oldcost * 1);
    }
    double strailcount = 0;
    double dtrailcount = 0;
    //cout << "trails.size() = " << trails.size() << endl;
    for(int i = 0; i < trails.size(); i++){
      //cout << "trails[i].size() = " << trails[i].size() << endl;
      for(int j = 0; j < trails[i].size(); j++){
        if(trails[i][j].get_distance(snode) <= 0.5){
          strailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "s = " << s.getX()/100.0 << ", " << s.getY()/100.0 << endl;
        }
        if(trails[i][j].get_distance(dnode) <= 0.5){
          dtrailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "d = " << d.getX()/100.0 << ", " << d.getY()/100.0 << endl;
        }
      }
      if(strailcount > 0 and dtrailcount > 0){
        break;
      }
    }
    double trailcost;
    if (strailcount > 0 and dtrailcount > 0){
      trailcost = (w7 * oldcost * 10);
    }
    else if (strailcount > 0 or dtrailcount > 0){
      trailcost = (w7 * oldcost * 7);
    }
    else{
      trailcost = (w7 * oldcost * 1);
    }
    double finalcost = (w1 * oldcost) * 1;
    if(regioncost > finalcost)
      finalcost = regioncost;
    if(hallwaycost > finalcost)
      finalcost = hallwaycost;
    if(conveycost > finalcost)
      finalcost = conveycost;
    if(trailcost > finalcost)
      finalcost = trailcost;
    return finalcost;
  }
  if (name == "density"){
    //cout << "Updating density nav graph" << endl;
    double s_cost = cellCost(s.getX(), s.getY(), b);
    double d_cost = cellCost(d.getX(), d.getY(), b);
    //return (w1 * oldcost) + (w2 * (s_cost+d_cost)/2);
    if (s_cost > 0 or d_cost > 0){
      return (w1 * oldcost) + (w2 * (s_cost+d_cost)/2);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "risk"){
    //cout << "Updating risk nav graph" << endl;
    double s_risk_cost = riskCost(s.getX(), s.getY(), b);
    double d_risk_cost = riskCost(d.getX(), d.getY(), b);
    //return (w1 * oldcost) + (w4 * (s_risk_cost+d_risk_cost)/2);
    if (s_risk_cost > 0 or d_risk_cost > 0){
      return (w1 * oldcost) + (w4 * (s_risk_cost+d_risk_cost)/2);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "flow"){
    //cout << "updating flow nav graph" << endl;
    double flowcost = computeCrowdFlow(s,d);
    if(direction == true){
      flowcost = flowcost * (-1);
    }
    if(flowcost < 0){
      flowcost = 0;
    }
    //return (w1 * oldcost) + (w3 * flowcost);
    if (flowcost > 0){
      return (w1 * oldcost) + (w3 * flowcost);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "spatial"){
    int sRegion=-1,dRegion=-1;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].inRegion(s.getX()/100.0, s.getY()/100.0)){
        sRegion = i;
      }
      if(regions[i].inRegion(d.getX()/100.0, d.getY()/100.0)){
        dRegion = i;
      }
      if(sRegion >= 0 and dRegion >= 0){
        break;
      }
    }

    double s_door_min_distance = std::numeric_limits<double>::infinity();
    double s_exit_min_distance = std::numeric_limits<double>::infinity();
    if(sRegion >= 0){
      CartesianPoint sPoint = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
      for(int i = 0; i < doors[sRegion].size(); i++) {
        double doorDistance = doors[sRegion][i].distanceToDoor(sPoint, regions[sRegion]);
        if (doorDistance < s_door_min_distance){
          s_door_min_distance = doorDistance;
        }
        if(s_door_min_distance <= 0.5){
          break;
        }
      }
      vector<FORRExit> exits = regions[sRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = sPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < s_exit_min_distance){
          s_exit_min_distance = exitDistance;
        }
        if(s_exit_min_distance <= 0.5){
          break;
        }
      }
    }

    double d_door_min_distance = std::numeric_limits<double>::infinity();
    double d_exit_min_distance = std::numeric_limits<double>::infinity();
    if(dRegion >= 0){
      CartesianPoint dPoint = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
      for(int i = 0; i < doors[dRegion].size(); i++) {
        double doorDistance = doors[dRegion][i].distanceToDoor(dPoint, regions[dRegion]);
        if (doorDistance < d_door_min_distance){
          d_door_min_distance = doorDistance;
        }
        if(d_door_min_distance <= 0.5){
          break;
        }
      }
      vector<FORRExit> exits = regions[dRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = dPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < d_exit_min_distance){
          d_exit_min_distance = exitDistance;
        }
        if(d_exit_min_distance <= 0.5){
          break;
        }
      }
    }

    if (sRegion >= 0 and dRegion >= 0){
      return (w1 * oldcost) * 1;
    }
    else if (sRegion >= 0 and dRegion == -1){
      if(s_door_min_distance <= 0.5 and s_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 1.5;
      }
      else if(s_door_min_distance <= 0.5 or s_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 1.75;
      }
      else{
        return (w1 * oldcost) * 2;
      }
    }
    else if (sRegion ==-1 and dRegion >= 0){
      if(d_door_min_distance <= 0.5 and d_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 1.5;
      }
      else if(d_door_min_distance <= 0.5 or d_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 1.75;
      }
      else{
        return (w1 * oldcost) * 2;
      }
    }
    else{
      return (w1 * oldcost) * 10;
    }
  }
  if (name == "hallwayer"){
    double sHallway=0, dHallway=0;
    CartesianPoint snode = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
    CartesianPoint dnode = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(snode)){
        sHallway++;
      }
      if(hallways[i].pointInAggregate(dnode)){
        dHallway++;
      }
      if(sHallway > 0 and dHallway > 0){
        break;
      }
    }
    if (sHallway > 0 and dHallway > 0){
      return (w1 * oldcost);
    }
    else if (sHallway > 0 or dHallway > 0){
      return (w1 * oldcost * 1.5);
    }
    else{
      return (w1 * oldcost) * 10;
    }
  }
  if (name == "conveys"){
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    //return (w7 * oldcost*pow(0.25,((sconveycost + dconveycost)/2)));
    if (sconveycost > 0 and dconveycost > 0){
      return (w7 * oldcost + 1/((sconveycost + dconveycost)/2));
    }
    else{
      return (w7 * oldcost * 10);
    }
  }
  if (name == "trailer"){
    //cout << "updating trailer nav graph" << endl;
    double strailcount = 0;
    double dtrailcount = 0;
    //cout << "trails.size() = " << trails.size() << endl;
    CartesianPoint snode = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
    CartesianPoint dnode = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
    for(int i = 0; i < trails.size(); i++){
      //cout << "trails[i].size() = " << trails[i].size() << endl;
      for(int j = 0; j < trails[i].size(); j++){
        if(trails[i][j].get_distance(snode) <= 0.5){
          strailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "s = " << s.getX()/100.0 << ", " << s.getY()/100.0 << endl;
        }
        if(trails[i][j].get_distance(dnode) <= 0.5){
          dtrailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "d = " << d.getX()/100.0 << ", " << d.getY()/100.0 << endl;
        }
      }
      if(strailcount > 0 and dtrailcount > 0){
        break;
      }
    }
    //cout << "strailcount = " << strailcount << " dtrailcount = " << dtrailcount << endl;
    //return (w8 * oldcost*pow(0.25,((strailcount + dtrailcount)/2)));
    if (strailcount > 0 and dtrailcount > 0){
      return (w7 * oldcost);
    }
    else if (strailcount > 0 or dtrailcount > 0){
      return (w7 * oldcost * 1.5);
    }
    else{
      return (w7 * oldcost * 10);
    }
  }
  if (name == "combined"){
    /*double s_cost = cellCost(s.getX(), s.getY(), b);
    double d_cost = cellCost(d.getX(), d.getY(), b);
    double s_risk_cost = riskCost(s.getX(), s.getY(), b);
    double d_risk_cost = riskCost(d.getX(), d.getY(), b);
    double flowcost = computeCrowdFlow(s,d);
    double ns_cost = novelCost(s.getX(), s.getY());
    double nd_cost = novelCost(d.getX(), d.getY());
    double smooth_cost = (oldcost * 5);
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    if(direction == true){
      flowcost = flowcost * (-1);
    }
    if(flowcost < 0){
      flowcost = 0;
    }
    return (w1 * oldcost) + (w2 * (s_cost+d_cost)/2) + (w3 * flowcost) + (w4 * (s_risk_cost+d_risk_cost)/2) + (w5 * (ns_cost+nd_cost)/2) + (w6 * smooth_cost) + (w7 * oldcost*pow(0.25,((sconveycost + dconveycost)/2)));*/
    double distcost = oldcost;

    double novelcost = (w1 * oldcost) * 1;

    double explorecost;
    double exs_cost = novelCost(s.getX(), s.getY());
    double exd_cost = novelCost(d.getX(), d.getY());
    if (exs_cost > 0 or exd_cost > 0){
      explorecost = (w1 * oldcost) + (w1 * (exs_cost+exd_cost)/2);
    }
    else{
      explorecost = (w1 * oldcost);
    }

    double safecost;
    double safes_cost = s.getDistWall();
    double safed_cost = d.getDistWall();
    if(safes_cost > 0 or safed_cost > 0){
      safecost = (w1 * oldcost + (w6 * 10/((safes_cost + safed_cost)/2)));
    }
    else{
      safecost = (w1 * oldcost) * 10;
    }

    int sRegion=-1,dRegion=-1;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].inRegion(s.getX()/100.0, s.getY()/100.0)){
        sRegion = i;
      }
      if(regions[i].inRegion(d.getX()/100.0, d.getY()/100.0)){
        dRegion = i;
      }
      if(sRegion >= 0 and dRegion >= 0){
        break;
      }
    }

    double s_door_min_distance = std::numeric_limits<double>::infinity();
    double s_exit_min_distance = std::numeric_limits<double>::infinity();
    if(sRegion >= 0){
      CartesianPoint sPoint = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
      for(int i = 0; i < doors[sRegion].size(); i++) {
        double doorDistance = doors[sRegion][i].distanceToDoor(sPoint, regions[sRegion]);
        if (doorDistance < s_door_min_distance){
          s_door_min_distance = doorDistance;
        }
        if(s_door_min_distance <= 0.5){
          break;
        }
      }
      vector<FORRExit> exits = regions[sRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = sPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < s_exit_min_distance){
          s_exit_min_distance = exitDistance;
        }
        if(s_exit_min_distance <= 0.5){
          break;
        }
      }
    }

    double d_door_min_distance = std::numeric_limits<double>::infinity();
    double d_exit_min_distance = std::numeric_limits<double>::infinity();
    if(dRegion >= 0){
      CartesianPoint dPoint = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
      for(int i = 0; i < doors[dRegion].size(); i++) {
        double doorDistance = doors[dRegion][i].distanceToDoor(dPoint, regions[dRegion]);
        if (doorDistance < d_door_min_distance){
          d_door_min_distance = doorDistance;
        }
        if(d_door_min_distance <= 0.5){
          break;
        }
      }
      vector<FORRExit> exits = regions[dRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = dPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < d_exit_min_distance){
          d_exit_min_distance = exitDistance;
        }
        if(d_exit_min_distance <= 0.5){
          break;
        }
      }
    }
    double regioncost;
    double novelregioncost;
    if (sRegion >= 0 and dRegion >= 0){
      regioncost = (w1 * oldcost) * 1;
      novelregioncost = (w1 * oldcost) * 10;
    }
    else if (sRegion >= 0 and dRegion == -1){
      if(s_door_min_distance <= 0.5 and s_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 1.5;
        novelregioncost = (w1 * oldcost) * 7;
      }
      else if(s_door_min_distance <= 0.5 or s_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 1.75;
        novelregioncost = (w1 * oldcost) * 5;
      }
      else{
        regioncost = (w1 * oldcost) * 2;
        novelregioncost = (w1 * oldcost) * 3;
      }
    }
    else if (sRegion ==-1 and dRegion >= 0){
      if(d_door_min_distance <= 0.5 and d_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 1.5;
        novelregioncost = (w1 * oldcost) * 7;
      }
      else if(d_door_min_distance <= 0.5 or d_exit_min_distance <= 0.5){
        regioncost = (w1 * oldcost) * 1.75;
        novelregioncost = (w1 * oldcost) * 5;
      }
      else{
        regioncost = (w1 * oldcost) * 2;
        novelregioncost = (w1 * oldcost) * 3;
      }
    }
    else{
      regioncost = (w1 * oldcost) * 10;
      novelregioncost = (w1 * oldcost) * 1;
    }
    CartesianPoint snode = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
    CartesianPoint dnode = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
    double sHallway=0, dHallway=0;
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(snode)){
        sHallway++;
      }
      if(hallways[i].pointInAggregate(dnode)){
        dHallway++;
      }
      if(sHallway > 0 and dHallway > 0){
        break;
      }
    }
    double hallwaycost;
    double novelhallwaycost;
    if (sHallway > 0 and dHallway > 0){
      hallwaycost = (w1 * oldcost);
      novelhallwaycost = (w1 * oldcost) * 10;
    }
    else if(sHallway > 0 or dHallway > 0){
      hallwaycost = (w1 * oldcost * 1.5);
      novelhallwaycost = (w1 * oldcost) * 7;
    }
    else{
      hallwaycost = (w1 * oldcost) * 10;
      novelhallwaycost = (w1 * oldcost);
    }
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    double conveycost;
    double novelconveycost;
    if (sconveycost > 0 and dconveycost > 0){
      conveycost = (w7 * oldcost + 1/((sconveycost + dconveycost)/2));
      novelconveycost = (w7 * oldcost * ((sconveycost + dconveycost)/2));
    }
    else{
      conveycost = (w7 * oldcost * 10);
      novelconveycost = (w7 * oldcost * 1);
    }
    double strailcount = 0;
    double dtrailcount = 0;
    //cout << "trails.size() = " << trails.size() << endl;
    for(int i = 0; i < trails.size(); i++){
      //cout << "trails[i].size() = " << trails[i].size() << endl;
      for(int j = 0; j < trails[i].size(); j++){
        if(trails[i][j].get_distance(snode) <= 0.5){
          strailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "s = " << s.getX()/100.0 << ", " << s.getY()/100.0 << endl;
        }
        if(trails[i][j].get_distance(dnode) <= 0.5){
          dtrailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "d = " << d.getX()/100.0 << ", " << d.getY()/100.0 << endl;
        }
      }
      if(strailcount > 0 and dtrailcount > 0){
        break;
      }
    }
    double trailcost;
    double noveltrailcost;
    if (strailcount > 0 and dtrailcount > 0){
      trailcost = (w7 * oldcost);
      noveltrailcost = (w7 * oldcost * 10);
    }
    else if (strailcount > 0 or dtrailcount > 0){
      trailcost = (w7 * oldcost * 1.5);
      noveltrailcost = (w7 * oldcost * 7);
    }
    else{
      trailcost = (w7 * oldcost * 10);
      noveltrailcost = (w7 * oldcost);
    }
    // double finalcost = (w1 * oldcost) * 10;
    // if(regioncost < finalcost)
    //   finalcost = regioncost;
    // if(hallwaycost < finalcost)
    //   finalcost = hallwaycost;
    // if(conveycost < finalcost)
    //   finalcost = conveycost;
    // if(trailcost < finalcost)
    //   finalcost = trailcost;

    if(regioncost > novelcost)
      novelcost = regioncost;
    if(hallwaycost > novelcost)
      novelcost = hallwaycost;
    if(conveycost > novelcost)
      novelcost = conveycost;
    if(trailcost > novelcost)
      novelcost = trailcost;
    
    double finalcost = (distcost + novelcost + explorecost + safecost + regioncost + hallwaycost + trailcost + conveycost) / 8.0;
    return finalcost;
  }

  //double newEdgeCost = (oldcost * flowcost);
  //double newEdgeCost = (w1 * oldcost) + (w2 * (s_cost+d_cost)/2) + (w3 * flowcost) + (w4 * (s_risk_cost+d_risk_cost)/2);

	//cout << "Flow cost --------- " << endl;
  /*if (s_cost > 0 or d_cost > 0 or flowcost > 0 or s_risk_cost > 0 or d_risk_cost > 0){
    cout << "Dist cost    :" << oldcost << endl;
  	cout << "Node penalty : " << s_cost << " + " << d_cost << endl;
    cout << "Flow cost    :" << flowcost << endl;
    cout << "Risk penalty : " << s_risk_cost << " + " << d_risk_cost << endl;
  	//cout << "Old cost : " << oldcost << " new cost : " << newEdgeCost << std::endl;
    cout << "New cost    :" << newEdgeCost << endl;
  }*/
	//return newEdgeCost;
  return oldcost;
}
