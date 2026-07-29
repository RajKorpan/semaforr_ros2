/* Path queries and closest-node selection. */

#include <semaforr/navigation/PathPlanner.hpp>

#include <algorithm>
#include <limits.h>

#define PATH_DEBUG true

double PathPlanner::getRemainingPathLength(double x, double y) {
  list<pair<int,int> > path_xy;

  pair<int,int> s(x, y);
  path_xy.push_back(s);

  list<int>::iterator iter;
  for(iter = path.begin(); iter != path.end(); iter++) {
    Node d = navGraph->getNode(*iter);
    pair<int, int> p(d.getX(), d.getY());
    path_xy.push_back(p);
  }

  pair<int,int> t(target.getX(), target.getY());
  path_xy.push_back(t);

  return getPathLength(path_xy);
}


/*!
  \brief Returns the closest Node in the navigation graph to an arbirary \f$(x, y)\f$ position.

  \param Node \c n, any \f$(x, y)\f$ on the map which we are searching for the closest node
  \param Node \c ref, any \f$(x,y)\f$ on the map, which we are using as a guiding point
  \return Node, this is a member node of the navigation graph, or an invalid node if not found

  This function is used to determine the closest member nodes of the navigation graph to target and the source,
  since the A* runs only over the member nodes.

  It first asks for the nodes within a region from the navigation graph and returns an accessible node in
  that area, that has the minimum total distances from the itself to \c n and \c ref nodes.

  The radius of the search region is initially set to the \c proximity defined by the \c Graph class.
  If no accessible nodes are found, the radius of the search area is increased and the search is repeated.
  The maximum search radius is set to 1.5 * proximity. If there are no accessible nodes within that distance
  the robot is probably surrounded by obstacles, therefore this function returns an invalid node.

 */
Node PathPlanner::getClosestNode(Node n, Node ref, bool isTarget){
  const string signature = "PathPlanner::getClosestNode()> ";
  if(name == "skeleton"){
    Node temp;
    if(PATH_DEBUG)
      (void)0;

    int nRegion=-1;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].inRegion(n.getX()/100.0, n.getY()/100.0) and regions[i].getMinExits().size() > 0){
        nRegion = i;
      }
      if(nRegion >= 0){
        break;
      }
    }
    if(nRegion >= 0){
      int x = (int)(regions[nRegion].getCenter().get_x()*100);
      int y = (int)(regions[nRegion].getCenter().get_y()*100);
      (void)0;
      temp = navGraph->getNode(navGraph->getNodeID(x, y));
      return temp;
    }
    if(isTarget and use_coverage_grid){
      int vRegion=-1;
      double vDist=1000000;
      for(int i = 0; i < regions.size() ; i++){
        if(coverage_grid[(int)(regions[i].getCenter().get_x())][(int)(regions[i].getCenter().get_y())] != 0 and regions[i].visibleFromRegion(CartesianPoint(n.getX()/100.0, n.getY()/100.0), 20) and regions[i].getMinExits().size() > 0){
          double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0));
          if(dist_to_region < vDist){
            (void)0;
            vRegion = i;
            vDist = dist_to_region;
          }
        }
      }
      if(vRegion >= 0){
        int x = (int)(regions[vRegion].getCenter().get_x()*100);
        int y = (int)(regions[vRegion].getCenter().get_y()*100);
        (void)0;
        temp = navGraph->getNode(navGraph->getNodeID(x, y));
        return temp;
      }
      (void)0;
      vector<Node*> nodes = navGraph->getNodes();
      vector<Node*>::iterator iter;
      // double min_distance = 100000000.0;
      double max_score = -100000000.0;
      for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
        double d = -3.0 * ((Map::distance( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY() ) / 100.0) - (*iter)->getRadius());
        double neighbors = (*iter)->numNeighbors();
        double score = d + neighbors;
        // if(d < min_distance){
          // min_distance = d;
        if(score > max_score and coverage_grid[(int)((*iter)->getX()/100.0)][(int)((*iter)->getY()/100.0)] != 0){
          max_score = score;
          temp = (*(*iter));
          if(PATH_DEBUG) {
            (void)0;
            temp.printNode();
            (void)0;
            (void)0;
          }
        }
      }
      return temp;
    }
    else{
      int vRegion=-1;
      double vDist=1000000;
      for(int i = 0; i < regions.size() ; i++){
        if(regions[i].visibleFromRegion(CartesianPoint(n.getX()/100.0, n.getY()/100.0), 20) and regions[i].getMinExits().size() > 0){
          double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0));
          if(dist_to_region < vDist){
            (void)0;
            vRegion = i;
            vDist = dist_to_region;
          }
        }
      }
      if(vRegion >= 0){
        int x = (int)(regions[vRegion].getCenter().get_x()*100);
        int y = (int)(regions[vRegion].getCenter().get_y()*100);
        (void)0;
        temp = navGraph->getNode(navGraph->getNodeID(x, y));
        return temp;
      }
      (void)0;
      vector<Node*> nodes = navGraph->getNodes();
      vector<Node*>::iterator iter;
      // double min_distance = 100000000.0;
      double max_score = -100000000.0;
      for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
        double d = -3.0 * ((Map::distance( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY() ) / 100.0) - (*iter)->getRadius());
        double neighbors = (*iter)->numNeighbors();
        double score = d + neighbors;
        // if(d < min_distance){
          // min_distance = d;
        if(score > max_score){
          max_score = score;
          temp = (*(*iter));
          if(PATH_DEBUG) {
            (void)0;
            temp.printNode();
            (void)0;
            (void)0;
          }
        }
      }
      return temp;
    }
  }
  else{
    Node temp;
    double s_radius = navGraph->getProximity();
    double max_radius = navGraph->getProximity() * 3;

    do {

      if(PATH_DEBUG)
        (void)0;

      vector<Node*> nodes = navGraph->getNodesInRegion(n.getX(), n.getY(), s_radius);

      double dist = INT_MAX;

      vector<Node*>::iterator iter;
      for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
        double d = Map::distance( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY() );

        if(PATH_DEBUG){
          (void)0;
          (*iter)->printNode();
          (void)0;
          (void)0;
        }

        double d_t = 0.0;
        if(ref.getID() != Node::invalid_node_index)
          d_t = Map::distance((*iter)->getX(), (*iter)->getY(), ref.getX(), ref.getY());

        if(PATH_DEBUG)
          (void)0;

        if(name != "skeleton" and name != "hallwayskel"){
          if (( d + d_t < dist ) && !map.isPathObstructed( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY()) && (*iter)->isAccessible()) {
            //cout << "Checking if node is accesible : " << (*iter)->getX() << " " << (*iter)->getY()  << endl;
            dist = d + d_t;
            temp = (*(*iter));
            if(PATH_DEBUG) {
              (void)0;
              temp.printNode();
              (void)0;
            }
          }
        }
        else{
          if (( d + d_t < dist ) && (*iter)->isAccessible()) {
            //cout << "Checking if node is accesible : " << (*iter)->getX() << " " << (*iter)->getY()  << endl;
            dist = d + d_t;
            temp = (*(*iter));
            if(PATH_DEBUG) {
              (void)0;
              temp.printNode();
              (void)0;
            }
          }
        }
      }

      if(temp.getID() == Node::invalid_node_index) {
        s_radius += 0.1 * s_radius;
        if(PATH_DEBUG)
          (void)0;
      }

    } while(temp.getID() == Node::invalid_node_index && s_radius <= max_radius);

    return temp;
  }
}

vector<Node> PathPlanner::getClosestNodes(Node n, Node ref, bool findAny){
  const string signature = "PathPlanner::getClosestNodes()> ";
  Node temp, region_temp, lregion_temp, otemp;
  bool otemp_created = false;
  if(PATH_DEBUG)
    (void)0;
  vector<Node> nodes_for_point;
  // cout << "node n " << ((int)(n.getX()/100.0)) << " " << ((int)(n.getY()/100.0)) << endl;
  // cout << "passage_grid " << passage_grid.size() << " " << passage_grid[0].size() << endl;
  int nPassage = passage_grid[(int)(n.getX()/100.0)][(int)(n.getY()/100.0)];
  // cout << "Point in passage_grid " << nPassage << " graph_node " << passage_graph_nodes.count(nPassage) << endl;
  if(passage_graph_nodes.count(nPassage) != 0){
    // cout << "Point on intersection " << nPassage - 1 << endl;
    int x = passage_average_values[nPassage - 1][0];
    int y = passage_average_values[nPassage - 1][1];
    // cout << "x " << x << " y " << y << endl;
    temp = navGraph->getNode(navGraph->getNodeID(x, y));
    // Node placeHolder;
    if(PATH_DEBUG) {
      (void)0;
      temp.printNode();
      // placeHolder.printNode();
      // placeHolder.printNode();
      (void)0;
    }
    nodes_for_point.push_back(temp);
    // nodes_for_point.push_back(placeHolder);
    // nodes_for_point.push_back(placeHolder);
    // otherIntersection.push_back(otemp);
    // usedOtherIntersection.push_back(otemp_created);
    // return nodes_for_point;
  }
  else if(nPassage > 0){
    // cout << "Point on passage" << endl;
    vector<int> nearby_intersections;
    for(int i = 0; i < passage_graph.size(); i++){
      if(passage_graph[i][1] == nPassage){
        if(find(nearby_intersections.begin(), nearby_intersections.end(), passage_graph[i][0]) == nearby_intersections.end()){
          nearby_intersections.push_back(passage_graph[i][0]);
          // cout << "nearby intersection " << passage_graph[i][0] << endl;
        }
        if(find(nearby_intersections.begin(), nearby_intersections.end(), passage_graph[i][2]) == nearby_intersections.end()){
          nearby_intersections.push_back(passage_graph[i][2]);
          // cout << "nearby intersection " << passage_graph[i][2] << endl;
        }
      }
    }
    double dist_to_nearby = 1000000.0;
    int closest_intersection;
    for(int i = 0; i < nearby_intersections.size(); i++){
      double dist_to_int = CartesianPoint(passage_average_values[nearby_intersections[i] - 1][0], passage_average_values[nearby_intersections[i] - 1][1]).get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0));
      if(dist_to_int < dist_to_nearby){
        dist_to_nearby = dist_to_int;
        closest_intersection = nearby_intersections[i] - 1;
      }
    }
    // cout << "closest intersection " << closest_intersection << endl;
    int x = passage_average_values[closest_intersection][0];
    int y = passage_average_values[closest_intersection][1];
    // cout << "x " << x << " y " << y << endl;
    temp = navGraph->getNode(navGraph->getNodeID(x, y));
    // Node placeHolder;
    if(PATH_DEBUG) {
      (void)0;
      temp.printNode();
      // placeHolder.printNode();
      // placeHolder.printNode();
      (void)0;
    }
    nodes_for_point.push_back(temp);
    // nodes_for_point.push_back(placeHolder);
    // nodes_for_point.push_back(placeHolder);
    // for(int i = 0; i < nearby_intersections.size(); i++){
    //   if(nearby_intersections[i] - 1 != closest_intersection){
    //     int cx = passage_average_values[nearby_intersections[i] - 1][0];
    //     int cy = passage_average_values[nearby_intersections[i] - 1][1];
    //     // cout << "cx " << cx << " cy " << cy << endl;
    //     otemp = navGraph->getNode(navGraph->getNodeID(cx, cy));
    //     otemp_created = true;
    //     break;
    //   }
    // }
    // otherIntersection.push_back(otemp);
    // usedOtherIntersection.push_back(otemp_created);
    // return nodes_for_point;
  }
  // cout << "nodes_for_point " << nodes_for_point.size() << endl;
  // cout << "Find region associated with n" << endl;
  int nRegion = -1;
  for(int i = 0; i < regions.size() ; i++){
    if(regions[i].inRegion(n.getX()/100.0, n.getY()/100.0) and regions[i].getMinExits().size() > 0){
      // cout << "nRegion " << i << endl;
      nRegion = i;
    }
    if(nRegion >= 0){
      break;
    }
  }
  if(nRegion == -1){
    int vRegion = -1;
    double vDist=1000000;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].visibleFromRegion(CartesianPoint(n.getX()/100.0, n.getY()/100.0), 20) and regions[i].getMinExits().size() > 0){
        double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0));
        if(dist_to_region < vDist){
          // cout << "vRegion " << i << " visible to point and distance " << dist_to_region << endl;
          vRegion = i;
          vDist = dist_to_region;
        }
      }
    }
    nRegion = vRegion;
  }
  if(nRegion == -1){
    int cRegion = -1;
    double max_score = -100000000.0;
    for(int i = 0; i < regions.size() ; i++){
      double d = -3.0 * (regions[i].getCenter().get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0)) - regions[i].getRadius());
      double neighbors = regions[i].getMinExits().size();
      double score = d + neighbors;
      if(score > max_score){
        // cout << "cRegion " << i << " with score " << score << endl;
        cRegion = i;
        max_score = score;
      }
    }
    nRegion = cRegion;
  }
  // cout << "nRegion " << nRegion << endl;
  if(nRegion >= 0){
    int rx = (int)(regions[nRegion].getCenter().get_x()*100);
    int ry = (int)(regions[nRegion].getCenter().get_y()*100);
    // cout << "Point in region " << nRegion << " rx " << rx << " ry " << ry << " ID " << originalNavGraph->getNodeID(rx, ry) << endl;
    region_temp = originalNavGraph->getNode(originalNavGraph->getNodeID(rx, ry));
    vector<int> passage_values = regions[nRegion].getPassageValues();
    for(int i = 0; i < passage_values.size(); i++){
      // cout << "passage_values " << passage_values[i] << endl;
      if(passage_graph_nodes.count(passage_values[i]) != 0){
        // cout << "nRegion on intersection " << passage_values[i] - 1 << endl;
        int x = passage_average_values[passage_values[i] - 1][0];
        int y = passage_average_values[passage_values[i] - 1][1];
        // cout << "x " << x << " y " << y << endl;
        temp = navGraph->getNode(navGraph->getNodeID(x, y));
        if(PATH_DEBUG) {
          (void)0;
          temp.printNode();
          region_temp.printNode();
          (void)0;
        }
        if(nodes_for_point.size() == 0){
          nodes_for_point.push_back(temp);
        }
        nodes_for_point.push_back(region_temp);
        nodes_for_point.push_back(region_temp);
        // otherIntersection.push_back(otemp);
        // usedOtherIntersection.push_back(otemp_created);
        // return nodes_for_point;
        break;
      }
    }
    // cout << "nodes_for_point " << nodes_for_point.size() << endl;
    if(nodes_for_point.size() < 3){
      if(passage_values.size() > 0){
        // cout << "nRegion on passage" << endl;
        vector<int> nearby_intersections;
        for(int i = 0; i < passage_graph.size(); i++){
          for(int j = 0; j < passage_values.size(); j++){
            if(passage_graph[i][1] == passage_values[j]){
              if(find(nearby_intersections.begin(), nearby_intersections.end(), passage_graph[i][0]) == nearby_intersections.end()){
                nearby_intersections.push_back(passage_graph[i][0]);
                // cout << "nearby intersection " << passage_graph[i][0] << endl;
              }
              if(find(nearby_intersections.begin(), nearby_intersections.end(), passage_graph[i][2]) == nearby_intersections.end()){
                nearby_intersections.push_back(passage_graph[i][2]);
                // cout << "nearby intersection " << passage_graph[i][2] << endl;
              }
            }
          }
        }
        double dist_to_nearby = 1000000.0;
        int closest_intersection;
        for(int i = 0; i < nearby_intersections.size(); i++){
          double dist_to_int = CartesianPoint(passage_average_values[nearby_intersections[i] - 1][0], passage_average_values[nearby_intersections[i] - 1][1]).get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0));
          // cout << "dist_to_int " << dist_to_int << " dist_to_nearby " << dist_to_nearby << endl;
          if(dist_to_int < dist_to_nearby){
            dist_to_nearby = dist_to_int;
            closest_intersection = nearby_intersections[i] - 1;
          }
        }
        // cout << "closest intersection " << closest_intersection << endl;
        int x = passage_average_values[closest_intersection][0];
        int y = passage_average_values[closest_intersection][1];
        // cout << "x " << x << " y " << y << endl;
        temp = navGraph->getNode(navGraph->getNodeID(x, y));
        if(PATH_DEBUG) {
          (void)0;
          temp.printNode();
          (void)0;
          region_temp.printNode();
          (void)0;
          region_temp.printNode();
          (void)0;
        }
        if(nodes_for_point.size() == 0){
          nodes_for_point.push_back(temp);
        }
        nodes_for_point.push_back(region_temp);
        nodes_for_point.push_back(region_temp);
        // for(int i = 0; i < nearby_intersections.size(); i++){
        //   if(nearby_intersections[i] - 1 != closest_intersection){
        //     int cx = passage_average_values[nearby_intersections[i] - 1][0];
        //     int cy = passage_average_values[nearby_intersections[i] - 1][1];
        //     // cout << "cx " << cx << " cy " << cy << endl;
        //     otemp = navGraph->getNode(navGraph->getNodeID(cx, cy));
        //     otemp_created = true;
        //     break;
        //   }
        // }
        // otherIntersection.push_back(otemp);
        // usedOtherIntersection.push_back(otemp_created);
        // return nodes_for_point;
        // cout << "nodes_for_point " << nodes_for_point.size() << endl;
      }
      else{
        // cout << "nRegion on neither intersection nor passage" << endl;
        priority_queue<RegionNode, vector<RegionNode>, greater<RegionNode> > rn_queue;
        RegionNode start_rn = RegionNode(regions[nRegion], nRegion, 0);
        // cout << "nRegion exits " << regions[nRegion].getMinExits().size() << endl;
        for(int i = 0; i < regions[nRegion].getMinExits().size(); i++){
          RegionNode neighbor = RegionNode(regions[regions[nRegion].getMinExits()[i].getExitRegion()], regions[nRegion].getMinExits()[i].getExitRegion(), regions[nRegion].getMinExits()[i].getExitDistance());
          // neighbor.regionSequence.push_back(start_rn);
          // cout << "neighbor " << i << " ID " << neighbor.regionID << endl;
          rn_queue.push(neighbor);
        }
        vector<RegionNode> already_searched;
        already_searched.push_back(start_rn);
        // cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
        RegionNode final_rn;
        int count = 0;
        while(rn_queue.size() > 0 and count < 1000){
          RegionNode current_neighbor = rn_queue.top();
          // cout << "current_neighbor " << current_neighbor.regionID << " passagevalues " << current_neighbor.region.getPassageValues().size() << " cost " << current_neighbor.nodeCost << endl;
          already_searched.push_back(current_neighbor);
          rn_queue.pop();
          if(current_neighbor.region.getPassageValues().size() > 0){
            final_rn = current_neighbor;
            break;
          }
          for(int i = 0; i < current_neighbor.region.getMinExits().size(); i++){
            RegionNode eRegion = RegionNode(regions[current_neighbor.region.getMinExits()[i].getExitRegion()], current_neighbor.region.getMinExits()[i].getExitRegion(), current_neighbor.nodeCost + current_neighbor.region.getMinExits()[i].getExitDistance());
            // eRegion.regionSequence.push_back(current_neighbor);
            // cout << "eRegion " << i << " ID " << eRegion.regionID << " cost " << eRegion.nodeCost << endl;
            if(find(already_searched.begin(), already_searched.end(), eRegion) == already_searched.end()){
              rn_queue.push(eRegion);
            }
          }
          // cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
          // if(rn_queue.size() == 0){
          //   final_rn = current_neighbor;
          // }
          count = count + 1;
        }
        // cout << "final_rn " << final_rn.regionID << " rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
        if(final_rn.regionID == -1){
          int newRegion = -1;
          double max_score = -100000000.0;
          for(int i = 0; i < regions.size() ; i++){
            double d = -3.0 * (regions[i].getCenter().get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0)) - regions[i].getRadius());
            double neighbors = regions[i].getMinExits().size();
            double score = d + neighbors;
            if(score > max_score and regions[i].getPassageValues().size() > 0){
              // cout << "newRegion " << i << " with score " << score << endl;
              newRegion = i;
              max_score = score;
            }
          }
          final_rn.regionID = newRegion;
        }
        int lx = (int)(regions[final_rn.regionID].getCenter().get_x()*100);
        int ly = (int)(regions[final_rn.regionID].getCenter().get_y()*100);
        // cout << "Point in lregion " << final_rn.regionID << " lx " << lx << " ly " << ly << " ID " << originalNavGraph->getNodeID(lx, ly) << endl;
        lregion_temp = originalNavGraph->getNode(originalNavGraph->getNodeID(lx, ly));
        vector<int> lpassage_values = regions[final_rn.regionID].getPassageValues();
        for(int i = 0; i < lpassage_values.size(); i++){
          // cout << "lpassage_values " << lpassage_values[i] << endl;
          if(passage_graph_nodes.count(lpassage_values[i]) != 0){
            // cout << "lRegion on intersection " << lpassage_values[i] - 1 << endl;
            int x = passage_average_values[lpassage_values[i] - 1][0];
            int y = passage_average_values[lpassage_values[i] - 1][1];
            // cout << "x " << x << " y " << y << endl;
            temp = navGraph->getNode(navGraph->getNodeID(x, y));
            if(PATH_DEBUG) {
              (void)0;
              temp.printNode();
              (void)0;
              region_temp.printNode();
              (void)0;
              lregion_temp.printNode();
              (void)0;
            }
            if(nodes_for_point.size() == 0){
              nodes_for_point.push_back(temp);
            }
            nodes_for_point.push_back(region_temp);
            nodes_for_point.push_back(lregion_temp);
            // otherIntersection.push_back(otemp);
            // usedOtherIntersection.push_back(otemp_created);
            // return nodes_for_point;
            break;
          }
        }
        // cout << "nodes_for_point " << nodes_for_point.size() << endl;
        if(lpassage_values.size() > 0 and nodes_for_point.size() < 3){
          // cout << "Region on passage" << endl;
          vector<int> nearby_intersections;
          for(int i = 0; i < passage_graph.size(); i++){
            for(int j = 0; j < lpassage_values.size(); j++){
              if(passage_graph[i][1] == lpassage_values[j]){
                if(find(nearby_intersections.begin(), nearby_intersections.end(), passage_graph[i][0]) == nearby_intersections.end()){
                  nearby_intersections.push_back(passage_graph[i][0]);
                  // cout << "nearby intersection " << passage_graph[i][0] << endl;
                }
                if(find(nearby_intersections.begin(), nearby_intersections.end(), passage_graph[i][2]) == nearby_intersections.end()){
                  nearby_intersections.push_back(passage_graph[i][2]);
                  // cout << "nearby intersection " << passage_graph[i][2] << endl;
                }
              }
            }
          }
          double dist_to_nearby = 1000000.0;
          int closest_intersection;
          for(int i = 0; i < nearby_intersections.size(); i++){
            double dist_to_int = CartesianPoint(passage_average_values[nearby_intersections[i] - 1][0], passage_average_values[nearby_intersections[i] - 1][1]).get_distance(CartesianPoint(n.getX()/100.0, n.getY()/100.0));
            if(dist_to_int < dist_to_nearby){
              dist_to_nearby = dist_to_int;
              closest_intersection = nearby_intersections[i] - 1;
            }
          }
          // cout << "closest intersection " << closest_intersection << endl;
          int x = passage_average_values[closest_intersection][0];
          int y = passage_average_values[closest_intersection][1];
          // cout << "x " << x << " y " << y << endl;
          temp = navGraph->getNode(navGraph->getNodeID(x, y));
          if(PATH_DEBUG) {
            (void)0;
            temp.printNode();
            (void)0;
            region_temp.printNode();
            (void)0;
            lregion_temp.printNode();
            (void)0;
          }
          if(nodes_for_point.size() == 0){
            nodes_for_point.push_back(temp);
          }
          nodes_for_point.push_back(region_temp);
          nodes_for_point.push_back(lregion_temp);
          // cout << "nodes_for_point " << nodes_for_point.size() << endl;
          // for(int i = 0; i < nearby_intersections.size(); i++){
          //   if(nearby_intersections[i] - 1 != closest_intersection){
          //     int cx = passage_average_values[nearby_intersections[i] - 1][0];
          //     int cy = passage_average_values[nearby_intersections[i] - 1][1];
          //     // cout << "cx " << cx << " cy " << cy << endl;
          //     otemp = navGraph->getNode(navGraph->getNodeID(cx, cy));
          //     otemp_created = true;
          //     break;
          //   }
          // }
          // otherIntersection.push_back(otemp);
          // usedOtherIntersection.push_back(otemp_created);
          // return nodes_for_point;
        }
      }
    }
  }
  else{
    if(nodes_for_point.size() == 0){
      nodes_for_point.push_back(temp);
    }
    nodes_for_point.push_back(region_temp);
    nodes_for_point.push_back(lregion_temp);
    // otherIntersection.push_back(otemp);
    // usedOtherIntersection.push_back(otemp_created);
    // return nodes_for_point;
  }
  // cout << "final nodes_for_point " << nodes_for_point.size() << endl;
  return nodes_for_point;
}
