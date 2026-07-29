/*
 * Map.h
 *
 *  Created on: June 17, 2017
 *      Author: Anoop Aroor
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <math.h>
#include <stdio.h>
#include <string>
#include <algorithm>

using namespace std;

class Wall{
	public:
	double x1;
	double y1;
	double x2;
	double y2;
};


class Map {
public:
  Map();
  Map(double, double);
  
  void addWall(double, double, double, double); 
  const vector<Wall>& getWalls() const noexcept { return walls; }
  
  double getLength() const noexcept { return length; }
  double getHeight() const noexcept { return height; }
  
  bool isWithinBorders( double, double );
  bool isPathObstructed( double, double, double, double );
  bool isAccessible(double x, double y);
  bool isPointInBuffer(double x, double y); 

  const vector< vector <bool> >& getOccupancyGrid() const noexcept {return occupancyGrid;}
  int getOccupancySize() const noexcept { return occupancySize; }

  bool readMapFromXML(string);
  
  static double distance(double x1, double y1, double x2, double y2);
  double distanceFromWall(double x, double y, int wallIndex);
  double distanceFromSegment(double x1, double y1, double x2, double y2, double pointX, double pointY);
  double getDistanceClosestWall(double x, double y);
  
protected:
  vector<Wall> walls;
  vector< vector <bool> > occupancyGrid;
  int occupancySize{20};
  
  double length{0.0};
  double height{0.0};
  
};

#endif /* MAP_H_ */
