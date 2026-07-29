/* Path cost and traversal-cost calculations. */

#include <semaforr/navigation/PathPlanner.hpp>

#include <algorithm>
#include <cmath>
#include <limits.h>

#define PATH_DEBUG true

// The compatibility graph stores planar coordinates in centimetres.
constexpr double kGraphUnitsPerMeter = 100.0;

double PathPlanner::cellCost(int nodex, int nodey, int buffer){
  const double x = nodex / kGraphUnitsPerMeter;
  const double y = nodey / kGraphUnitsPerMeter;
  const double offset = std::fabs(buffer / kGraphUnitsPerMeter);
  return std::max({
    crowdModel.densityAt({x, y}),
    crowdModel.densityAt({x + offset, y}),
    crowdModel.densityAt({x - offset, y}),
    crowdModel.densityAt({x, y + offset}),
    crowdModel.densityAt({x, y - offset})});
}

double PathPlanner::riskCost(int nodex, int nodey, int buffer){
  const double x = nodex / kGraphUnitsPerMeter;
  const double y = nodey / kGraphUnitsPerMeter;
  const double offset = std::fabs(buffer / kGraphUnitsPerMeter);
  return std::max({
    crowdModel.navigationRiskAt({x, y}),
    crowdModel.navigationRiskAt({x + offset, y}),
    crowdModel.navigationRiskAt({x - offset, y}),
    crowdModel.navigationRiskAt({x, y + offset}),
    crowdModel.navigationRiskAt({x, y - offset})});
}

double PathPlanner::computeCrowdFlow(Node s, Node d){
  if (!crowdModel.learnedAvailable()) {
    return 0.0;
  }
  const double sx = s.getX() / kGraphUnitsPerMeter;
  const double sy = s.getY() / kGraphUnitsPerMeter;
  const double dx = d.getX() / kGraphUnitsPerMeter;
  const double dy = d.getY() / kGraphUnitsPerMeter;
  const double length = std::hypot(dx - sx, dy - sy);
  if (length <= 1.0e-9) {
    return 0.0;
  }
  const double midpoint_x = (sx + dx) / 2.0;
  const double midpoint_y = (sy + dy) / 2.0;
  const semaforr::domain::Angle direction(std::atan2(dy - sy, dx - sx));
  // Positive alignment is helpful; legacy edge costs penalize opposing flow.
  return -crowdModel.flowAlignmentAt(
    {midpoint_x, midpoint_y}, direction);
}

double PathPlanner::novelCost(int nodex, int nodey){
  //cout << "Inside novelCost : Node x = " << (nodex/100.0) << " Node y = " << (nodey/100.0) << endl;
  //cout << "Modified Node x = " << (int)(((nodex/100.0)/(map_width*1.0)) * boxes_width) << " Modified Node y = " << (int)(((nodey/100.0)/(map_height*1.0)) * boxes_height) << endl;
  //cout << "novelCost = " << posHistMapNorm[(int)(((nodex/100.0)/(map_width*1.0)) * boxes_width)][(int)(((nodey/100.0)/(map_height*1.0)) * boxes_height)] << endl;
  return posHistMap[(int)(((nodex/100.0)/(map_width*1.0)) * boxes_width)][(int)(((nodey/100.0)/(map_height*1.0)) * boxes_height)];
}

double PathPlanner::computeConveyorCost(int nodex, int nodey){
  //cout << "Inside computeConveyorCost : Node x = " << (nodex/100.0) << " Node y = " << (nodey/100.0) << endl;
  //cout << "ConveyorCost = " << conveyors->getGridValue((nodex/100.0),(nodey/100.0)) << endl;
  return conveyors->getGridValue((nodex/100.0),(nodey/100.0));
}


double PathPlanner::projection(double flow_angle, double flow_length, double xs, double ys, double xd, double yd){
	//double pi = 3.145;
	double edge_angle = atan2((ys - yd),(xs - xd))+M_PI;
	//edge_angle = (edge_angle > 0 ? edge_angle : (2*pi + edge_angle));
  edge_angle = (edge_angle < (2*M_PI) ? edge_angle : (0.0));
	double theta = flow_angle - edge_angle;
  if(theta>M_PI){
    theta = theta-2*M_PI;
  }
  else if(theta<-M_PI){
    theta = theta+2*M_PI;
  }
	//cout << "Flow angle: " << flow_angle << " Edge angle: " << edge_angle << " Diff: " << theta << endl;
	return flow_length * cos(theta);
}




bool PathPlanner::isAccessible(Node s, Node t) {
  if ( !navGraph->isNode(s) )
    s = getClosestNode(s, t, false);

  if ( !navGraph->isNode(t) )
    t = getClosestNode(t, s, false);

  if ( s.getID() == Node::invalid_node_index || t.getID() == Node::invalid_node_index )
    return false;

  astar newsearch(*navGraph, s, t, name);
  if ( newsearch.isPathFound() )
    return true;

  return false;
}


list<pair<int,int> > PathPlanner::getPathXYBetween(int x1, int y1, int x2, int y2){

  // flags show if the (x1, y1) and (x2, y2) are not valid nodes in the navgraph
  bool s_invalid = false;
  bool t_invalid = false;

  // flags represent what happens after attempting to get the closest valid node to (x1, y1) and (x2, y2)
  bool no_source = false;
  bool no_target = false;

  // create temp nodes for pairs
  Node temp_s(1, x1, y1);
  Node temp_t(1, x2, y2);

  /* check if the (x1, y1) is a valid node in the graph, if so get a copy of the node and assign it to s
   * else attempt to get the closest valid node from the graph, if it fails set the no_source flag to true
   * if it succeeds assign the closest valid node to s and set the s_invalid flag to true
   */
  Node s;
  int s_id = navGraph->getNodeID(x1, y1);
  if(s_id == Node::invalid_node_index) {
    s = getClosestNode(temp_s, temp_t, false);

    if(!navGraph->isNode(s))
      no_source = true;

    s_invalid = true;
  }
  else {
    s = navGraph->getNode(s_id);
  }

  /* check if the (x2, y2) is a valid node in the graph, if so get a copy of the node and assign it to t
   * else attempt to get the closest valid node from the graph, if it fails set the no_target flag to true
   * if it succeeds assign the closest valid node to t and set the t_invalid flag to true
   */
  Node t;
  int t_id = navGraph->getNodeID(x2, y2);
  if(t_id == Node::invalid_node_index) {
    t = getClosestNode(temp_t, temp_s, false);

    if(!navGraph->isNode(t))
      no_target = true;

    t_invalid = true;
  }

  // computed path list containing node ids
  list<int> path_c;

  // computed path list containing (x,y) values of the nodes
  list< pair<int,int> > path_c_points;

  /* if either source or the target is invalid, astar can't find a path.
   * to inform the caller of this function if this situation arises, a special pair <source, target>
   * will be added to the path and the function will return. if any of the values in the pair is -1
   * it will mean the path wasn't found due to invalid source, target or both
   */
  if(no_source || no_target) {
    int sval = (no_source) ? -1 : 0;
    int tval = (no_target) ? -1 : 0;
    pair<int,int> p(sval, tval);
    path_c_points.push_back(p);

    return path_c_points;
  }

  // calculate the path. if no path is found return the path_points list empty
  astar newsearch(*navGraph, s, t, name);
  if(newsearch.isPathFound()) {
    path_c = newsearch.getPathToTarget();
    smoothPath(path_c, s, t);
  }
  else {
    return path_c_points;
  }

  // compute the distance
  if(s_invalid) {
    pair<int, int> st(x1,y1);
    path_c_points.push_back(st);
  }

  list<int>::iterator iter;
  for ( iter = path_c.begin(); iter != path_c.end(); iter++ ){
    Node d = navGraph->getNode(*iter);
    pair<int, int> p(d.getX(), d.getY());
    path_c_points.push_back(p);
  }

  if(t_invalid) {
    pair<int, int> tg(x2,y2);
    path_c_points.push_back(tg);
  }

  return path_c_points;
}


int PathPlanner::getPathLength(list<pair<int,int> > path){
  double length = 0;
  list<pair<int,int> >::iterator iter, iter_next;
  for ( iter = path.begin(); iter != path.end(); iter++ ){
    iter_next = iter;
    iter_next++;
    if ( iter_next != path.end() ) {
      length += Map::distance(iter->first, iter->second, iter_next->first, iter_next->second);
    }
  }
  return static_cast<int>(length);
}


double PathPlanner::calcPathCost(list<int> p){
  double pcost = 0;
  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = navGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  // add reaching from source and to target costs
  // Note: This will double count when called from estimateCost() and calcPath()

  /*if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    if ( !p.empty() ){
      pcost += Map::distance(source.getX(), source.getY(),
					     navGraph->getNode(p.front()).getX(),
					     navGraph->getNode(p.front()).getY());
      pcost += Map::distance(navGraph->getNode(p.back()).getX(),
					     navGraph->getNode(p.back()).getY(),
					     target.getX(), target.getY());
    }
    else
      pcost += Map::distance(source.getX(), source.getY(),
					     target.getX(), target.getY());
  }*/

  return pcost;
}

double PathPlanner::calcOrigPathCost(list<int> p){
  double pcost = 0;
  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = originalNavGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  // add reaching from source and to target costs
  // Note: This will double count when called from estimateCost() and calcPath()

  /*if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    if ( !p.empty() ){
      pcost += Map::distance(source.getX(), source.getY(),
               navGraph->getNode(p.front()).getX(),
               navGraph->getNode(p.front()).getY());
      pcost += Map::distance(navGraph->getNode(p.back()).getX(),
               navGraph->getNode(p.back()).getY(),
               target.getX(), target.getY());
    }
    else
      pcost += Map::distance(source.getX(), source.getY(),
               target.getX(), target.getY());
  }*/

  return pcost;
}

double PathPlanner::calcPathCost(vector<CartesianPoint> waypoints, Position source, Position target){
  double pcost = 0;
  list<int> p;

  vector<CartesianPoint>::iterator it;
  for (it = waypoints.begin(); it != waypoints.end(); it++ ){
    Node s;
    int s_id = navGraph->getNodeID((*it).get_x()*100.0, (*it).get_y()*100.0);
    if(s_id == Node::invalid_node_index) {
      Node temp_s(1, (*it).get_x()*100.0, (*it).get_y()*100.0); 
      Node temp_t(1, (*it).get_x()*100.0, (*it).get_y()*100.0);
      s = getClosestNode(temp_s, temp_t, false);
      s_id = navGraph->getNodeID(s.getX(), s.getY());
    }
    p.push_back(s_id);
  }

  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = navGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  
  pcost += (estimateCost(source.getX(), source.getY(), waypoints[0].get_x(), waypoints[0].get_y()) + estimateCost(target.getX(), target.getY(), waypoints[waypoints.size()-1].get_x(), waypoints[waypoints.size()-1].get_y()));

  return pcost;
}

double PathPlanner::estimateCost(int x1, int y1, int x2, int y2) {
  Node source(1, x1, y1);
  Node target(1, x2, y2);

  return estimateCost(source, target, 1);
}

double PathPlanner::estimateCost(Node s, Node t, int l){
  Node sn = getClosestNode(s, t, false);
  Node tn = getClosestNode(t, s, false);

  if(tn.getID() < 0 || sn.getID() < 0)
    return INT_MAX;

  double pathCost = Map::distance(s.getX(), s.getY(), sn.getX(), sn.getY());
  pathCost += Map::distance(t.getX(), t.getY(), tn.getX(), tn.getY());

  astar nsearch(*navGraph, sn, tn, name);
  if ( nsearch.isPathFound() ){
    list<int> p = nsearch.getPathToTarget();
    pathCost += calcPathCost(p);
  }
  else {
    if(PATH_DEBUG)
      cout << "PathPlanner::estimateCost> no path found to target! "
	   << "Source accessible: " << sn.isAccessible()
	   << ", target accessible: " << tn.isAccessible() << endl;
    pathCost = INT_MAX;
  }

  return pathCost;
}
