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
      cout << "source: ";
      s.printNode();
      cout << " - first: " ;
      first.printNode();
      cout << " - second: " ;
      second.printNode();
      cout << endl ;
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
      cout << "onebeforelast: ";
      onebeforelast.printNode();
      cout << " - last: " ;
      last.printNode();
      cout << " - target: " ;
      t.printNode();
      cout << endl;
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
    cout << "after smoothing: " << endl;
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
    cout << endl;
  }
}

void PathPlanner::printPath(list<int> p){
  list<int>::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    cout << endl;
  }
}


void PathPlanner::printPath(list<pair<int,int> > p) {
  list<pair<int,int> >::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    int nodeId = navGraph->getNodeID(it->first, it->second);
    if ( nodeId == -1 ) {
      cout << "Not a graph node - (" << it->first << ", " << it->second << ")" << endl;
    }
    else {
      navGraph->getNode(nodeId).printNode() ;
      cout << endl;
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
