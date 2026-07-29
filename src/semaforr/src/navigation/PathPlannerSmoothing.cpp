/* Path smoothing, diagnostics, and validation. */

#include <semaforr/navigation/PathPlanner.hpp>

#include <algorithm>
#include <limits.h>

#define PATH_DEBUG true

/*!
  \brief Used to remove extra points off of the path
 */
void PathPlanner::smoothPath(list<int>& pathCalc, Node s, Node t){
  int proximity = navGraph->getProximity();

  if ( pathCalc.size() > 1 ) {
    // smooth the end points
    // if getting to second node from source is shorter and not obstructed remove first node.
    list<int>::iterator iter = pathCalc.begin();
    Node first = navGraph->getNode(*iter++);
    Node second = navGraph->getNode(*iter);
    iter--; // point back to the first element

    if ( PATH_DEBUG ) {
      (void)0;
      s.printNode();
      (void)0;
      first.printNode();
      (void)0;
      second.printNode();
      (void)0;
    }

    if ( !map.isPathObstructed(s.getX(), s.getY(), second.getX(), second.getY()) &&
	 Map::distance( s.getX(), s.getY(), second.getX(), second.getY() ) + proximity * 0.5 <
	 ( Map::distance( s.getX(), s.getY(), first.getX(), first.getY() ) +
	   Map::distance( first.getX(), first.getY(), second.getX(), second.getY() ) )){
      if ( PATH_DEBUG ) cout << "Erasing first" << endl;
      pathCalc.erase(iter);
    }
  }

  if ( pathCalc.size() > 1 ) {
    // if getting to second node from source is shorter and not obstructed remove first node.
    list<int>::iterator iter = pathCalc.end();
    Node last = navGraph->getNode(*(--iter));
    Node onebeforelast = navGraph->getNode(*(--iter));
    iter++;

    if ( PATH_DEBUG ){
      (void)0;
      onebeforelast.printNode();
      (void)0;
      last.printNode();
      (void)0;
      t.printNode();
      (void)0;
    }

    if ( !map.isPathObstructed(onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY()) &&
	 Map::distance( onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY() ) + proximity * 0.5 <
	 ( Map::distance( onebeforelast.getX(), onebeforelast.getY(), last.getX(), last.getY() ) +
	   Map::distance( last.getX(), last.getY(), t.getX(), t.getY() ) )){
      if ( PATH_DEBUG ) cout << "Erasing last" << endl ;
      pathCalc.erase(iter);
    }
  }

  if ( PATH_DEBUG ) {
    (void)0;
    printPath(pathCalc);
  }
}

/*!
  \brief Prints the path node by node
 */
void PathPlanner::printPath(){
  list<int>::iterator it;
  for ( it = path.begin(); it != path.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    (void)0;
  }
}

void PathPlanner::printPath(list<int> p){
  list<int>::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    (void)0;
  }
}


void PathPlanner::printPath(list<pair<int,int> > p) {
  list<pair<int,int> >::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    int nodeId = navGraph->getNodeID(it->first, it->second);
    if ( nodeId == -1 ) {
      (void)0;
    }
    else {
      navGraph->getNode(nodeId).printNode() ;
      (void)0;
    }
  }
}


bool PathPlanner::allWaypointsValid() {
  list<int>::iterator it;
  for(it = path.begin(); it != path.end(); ++it) {
    if(!navGraph->getNode(*it).isAccessible())
      return false;
  }
  return true;
}

/*! @} */
