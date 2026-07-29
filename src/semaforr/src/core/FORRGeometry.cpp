/*
 * Implementation file for our FORRGeometry module.
 *
 * Last modified: March 25, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 *
 */

#include <iostream>
#include <semaforr/core/FORRGeometry.h>

using std::cout;
using std::endl;
using std::min;
using std::max;

/**********************************************************************
                 CartesianPoint constructor section
***********************************************************************/
CartesianPoint::CartesianPoint(): x(0), y(0) {}

CartesianPoint::CartesianPoint(double x_c, double y_c): x(x_c), y(y_c) {}

/**********************************************************************
                 Cartesian Point getters and setters
**********************************************************************/
void CartesianPoint::set_x(double new_x) { x = new_x; }

void CartesianPoint::set_y(double new_y) { y = new_y; }

double CartesianPoint::get_x() const { return x; }

double CartesianPoint::get_y() const { return y; }

double CartesianPoint::get_distance(const CartesianPoint& point) const{
  return std::hypot(x - point.x, y - point.y);
}

/**********************************************************************
                  equality operator
**********************************************************************/
bool CartesianPoint::operator==(const CartesianPoint& lhs) const{
  return (
    std::abs(this->x - lhs.x) < kGeometryTolerance &&
    std::abs(this->y - lhs.y) < kGeometryTolerance);
}

/********************************************************************
                   other operators
********************************************************************/
bool CartesianPoint::operator<(const CartesianPoint& lhs) const{
  return x < lhs.x || (x == lhs.x && y < lhs.y);
}


bool CartesianPoint::operator>(const CartesianPoint& lhs) const{
  return lhs < *this;
}


/***********************************************************************
                Line class implementation
***********************************************************************/
Line::Line(CartesianPoint first, CartesianPoint second): 
  coefficient_a(second.get_y() - first.get_y()),
  coefficient_b(first.get_x() - second.get_x()),
  coefficient_c(coefficient_a * first.get_x() + coefficient_b * first.get_y()) {}

Line::Line(double x1, double y1, double x2, double y2):
  coefficient_a(y2 - y1),
  coefficient_b(x1 - x2),
  coefficient_c(coefficient_a * x1 + coefficient_b * y1) {}

/**********************************************************************
                  Line getters and setters
***********************************************************************/
double Line::get_value_a() const { return coefficient_a; }
double Line::get_value_b() const { return coefficient_b; }
double Line::get_value_c() const { return coefficient_c; }

void Line::set_value_a(double new_a) { coefficient_a = new_a; }
void Line::set_value_b(double new_b) { coefficient_b = new_b; }
void Line::set_value_c(double new_c) { coefficient_c = new_c; }

double Line::get_slope() const{
  double answer;
  if(coefficient_a == 0)
    answer = 0;
  else if(coefficient_b == 0)   // don't divide by zero
    answer = std::numeric_limits<double>::infinity();
  else
    answer = -coefficient_a / coefficient_b;

  return answer;
}

bool Line::is_degenerate() const {
  return std::abs(coefficient_a) < kGeometryTolerance &&
         std::abs(coefficient_b) < kGeometryTolerance;
}
	      


/**********************************************************************
                  LineSegment constructors
**********************************************************************/
LineSegment::LineSegment(CartesianPoint first, CartesianPoint second): Line(first, second), 
								       end_point_1(first), 
								       end_point_2(second) {}

LineSegment::LineSegment(double x1, double y1, double x2, double y2): 
  Line(x1, y1, x2, y2), 
  end_point_1(CartesianPoint(x1, y1)), 
  end_point_2(CartesianPoint(x2, y2)) {}


/*********************************************************************
                        Accessors
**********************************************************************/
/*
 * Does not make sense to return single end point of LineSegment
 * because there is no way we can consistently distinguish them.
 * Well there is less than operator, but I did not implement it 
 * and that would just transfer problem to higher level of abstraction.
 * This way we get the end points and we can do whatever needs to
 * be done.
 */
pair<CartesianPoint, CartesianPoint> LineSegment::get_endpoints() const{
  return std::make_pair(end_point_1, end_point_2);
}
/*LineSegment::LineSegment(const LineSegment& other): 
  coefficient_a(other.coefficient_a), 
  coefficient_b(other.coefficient_b), 
  coefficient_c(other.coefficient_c), 
  end_point_1(other.end_point_1), 
  end_point_2(other.end_point_2) {}*/

double LineSegment::get_length() const{
  return end_point_1.get_distance(end_point_2);
}

/**********************************************************************
                  Vector constructors
**********************************************************************/
Vector::Vector(CartesianPoint point, double angle, double intensty): 
  origin(point), theta(angle), intensity(intensty) {
  int ratio;
  if((angle > (2 * M_PI)) || (angle < -2 * M_PI)){
    ratio = static_cast<int>(angle / (2 * M_PI));
    theta = angle - ratio * 2 * M_PI;
  } 
 else
     theta = angle;
}

Vector::Vector(double x1, double y1, double angle, double intensty):
  origin(CartesianPoint(x1, y1)), theta(angle), intensity(intensty) {}


/**********************************************************************
                  accessors and mutators
**********************************************************************/
CartesianPoint Vector::get_origin() const{ return origin; }

double Vector::get_angle() const { return theta; }

double Vector::get_intensity() const { return intensity; }

double Vector::turn_vector(double angle){
  
   double new_angle = this->get_angle() + angle;
   //new_angle = Utils::normalizeAngle(new_angle);
   return new_angle;
}

CartesianPoint Vector::get_endpoint() const{
  return get_point(this->get_intensity(), 0);
}

CartesianPoint Vector::get_point(double distance, double angle) const{

  double turned_angle = this->get_angle() + angle;
  CartesianPoint point = this->get_origin();

  //cout << "Casting new points at angle: " << normal_angle << " and from " << point.get_x() << " " << point.get_y() << endl;
  return CartesianPoint(point.get_x() + distance*cos(turned_angle), point.get_y() + distance*sin(turned_angle));
}



/**********************************************************************
     friend functions for common interclass computations
*********************************************************************/
double distance(CartesianPoint first, CartesianPoint second){
  return std::hypot(first.x - second.x, first.y - second.y);
}


CartesianPoint get_perpendicular(CartesianPoint point, Line line){
  if (line.is_degenerate()) {
    return point;
  }
  // cout << "get_perpendicular " << line.get_value_a() << " " << line.get_value_b() << " " << line.get_value_c() << endl;
  if(line.get_value_b() != 0 and line.get_value_a() != 0){
    double orig_slope = -line.get_value_a() / line.get_value_b();
    double orig_intercept = line.get_value_c() / line.get_value_b();
    double perp_slope = line.get_value_b() / line.get_value_a();
    double perp_intercept = point.get_y() - perp_slope * point.get_x();
    double intersection_x = (perp_intercept - orig_intercept) / (orig_slope - perp_slope);
    double intersection_y = orig_slope * intersection_x + orig_intercept;
    // cout << orig_slope << " " << orig_intercept << " " << perp_slope << " " << perp_intercept << " " << intersection_x << " " << intersection_y << endl;
    return CartesianPoint(intersection_x, intersection_y);
  }
  else if(line.get_value_b() == 0){
    // cout << line.get_value_c() / line.get_value_a() << " " << point.get_y() << endl;
    return CartesianPoint(line.get_value_c() / line.get_value_a(), point.get_y());
  }
  else if(line.get_value_a() == 0){
    return CartesianPoint(point.get_x(), line.get_value_c() / line.get_value_b());
  }
}

// Formula taken from Wikipedia
double distance(CartesianPoint point, Line line){
  const double denominator =
    std::hypot(line.coefficient_a, line.coefficient_b);
  if (denominator < kGeometryTolerance) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(
    line.coefficient_a * point.x +
    line.coefficient_b * point.y -
    line.coefficient_c) / denominator;
}


double distance(CartesianPoint point, LineSegment segment){
  double tmp_distance_1 = -1;
  double tmp_distance_2 = -1;
  // if((point.x > min(segment.end_point_1.x , segment.end_point_2.x) && point.x < max(segment.end_point_1.x , segment.end_point_2.x)) || (point.y > min(segment.end_point_1.y , segment.end_point_2.y) && point.y < max(segment.end_point_1.y , segment.end_point_2.y))){
  //   // In this case we can reuse distance between point and line function
  //   tmp_distance_1 = distance(point, static_cast<Line>(segment));
  // }
  CartesianPoint perpendicular_point = get_perpendicular(point, static_cast<Line>(segment));
  if(is_point_in_segment(perpendicular_point, segment)){
    tmp_distance_1 = distance(point, static_cast<Line>(segment));
  }
  // else
  tmp_distance_2 = min(distance(point, segment.end_point_1), distance(point, segment.end_point_2));
  if(tmp_distance_1 > -1 and tmp_distance_1 < tmp_distance_2)
    return tmp_distance_1;
  else
    return tmp_distance_2;

  // return tmp_distance;
}

double distance_to_intersection(CartesianPoint point, LineSegment segment){
  // cout << "in distance_to_intersection" << endl;
  double tmp_distance_1 = -1;
  double tmp_distance_2 = -1;
  // if((point.x > min(segment.end_point_1.x , segment.end_point_2.x) && point.x < max(segment.end_point_1.x , segment.end_point_2.x)) || (point.y > min(segment.end_point_1.y , segment.end_point_2.y) && point.y < max(segment.end_point_1.y , segment.end_point_2.y))){
  //   // In this case we can reuse distance between point and line function
  //   tmp_distance_1 = distance(point, static_cast<Line>(segment));
  // }
  CartesianPoint perpendicular_point = get_perpendicular(point, static_cast<Line>(segment));
  if(is_point_in_segment(perpendicular_point, segment)){
    // cout << distance(point, static_cast<Line>(segment)) << " " << distance(perpendicular_point, segment.end_point_1) << endl;
    tmp_distance_1 = distance(point, static_cast<Line>(segment)) + distance(perpendicular_point, segment.end_point_1);
  }
  // cout << "tmp_distance_1 " << tmp_distance_1 << endl;
  // else
  // cout << distance(point, segment.end_point_1) << " " << distance(point, segment.end_point_2) << " " << segment.get_length() << endl;
  tmp_distance_2 = min(distance(point, segment.end_point_1), distance(point, segment.end_point_2)) + segment.get_length();
  // cout << "tmp_distance_2 " << tmp_distance_2 << endl;
  if(tmp_distance_1 > -1 and tmp_distance_1 < tmp_distance_2)
    return tmp_distance_1;
  else
    return tmp_distance_2;

  // return tmp_distance;
}



bool is_point_on_line(CartesianPoint point, Line line){
  if (line.is_degenerate()) {
    return false;
  }
  return std::abs(
    line.coefficient_a * point.x +
    line.coefficient_b * point.y -
    line.coefficient_c) < kGeometryTolerance;
}


bool is_point_in_segment(CartesianPoint point, LineSegment segment){
  // cout << "is_point_in_segment " << distance(point, segment.end_point_1) << " " << distance(point, segment.end_point_2) << " " << distance(segment.end_point_1, segment.end_point_2) << " " << abs(distance(point, segment.end_point_1) + distance(point, segment.end_point_2) - distance(segment.end_point_1, segment.end_point_2)) << endl;
  // when all else failed I used triangular equation  
  return (
    std::abs(
      distance(point, segment.end_point_1) +
      distance(point, segment.end_point_2) -
      distance(segment.end_point_1, segment.end_point_2)) <
    kGeometryTolerance);
}

//constructors
Circle::Circle(CartesianPoint center, double radius)
  : center(center), radius(std::max(0.0, radius)) {}

//getters and setters
CartesianPoint Circle::get_center() const{
  return center;
}

double Circle::get_radius() const{
  return radius;
}

CartesianPoint intersection_point(Circle circle, LineSegment line_segment){
  const double radius = circle.get_radius();
  const CartesianPoint center = circle.get_center();
  const auto endpoints = line_segment.get_endpoints();
  const double dx =
    endpoints.second.get_x() - endpoints.first.get_x();
  const double dy =
    endpoints.second.get_y() - endpoints.first.get_y();
  const double fx = endpoints.first.get_x() - center.get_x();
  const double fy = endpoints.first.get_y() - center.get_y();
  const double a = dx * dx + dy * dy;

  if (a >= kGeometryTolerance * kGeometryTolerance) {
    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - radius * radius;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      double first_t = (-b - root) / (2.0 * a);
      double second_t = (-b + root) / (2.0 * a);
      if (second_t < first_t) {
        std::swap(first_t, second_t);
      }
      for (const double candidate : {first_t, second_t}) {
        if (candidate >= -kGeometryTolerance &&
            candidate <= 1.0 + kGeometryTolerance) {
          const double t = std::max(0.0, std::min(1.0, candidate));
          return CartesianPoint(
            endpoints.first.get_x() + t * dx,
            endpoints.first.get_y() + t * dy);
        }
      }
    }
  }

  const double angle = std::atan2(
    endpoints.first.get_y() - center.get_y(),
    endpoints.first.get_x() - center.get_x());
  return CartesianPoint(
    center.get_x() + radius * std::cos(angle),
    center.get_y() + radius * std::sin(angle));
}

bool do_intersect(Circle circle, Line line){
  if (line.is_degenerate()) {
    return false;
  }
  return distance(circle.get_center(), line) <=
         circle.get_radius() + kGeometryTolerance;
}

bool do_intersect(Circle circle, LineSegment line_segment){
  return distance(circle.get_center(), line_segment) <=
         circle.get_radius() + kGeometryTolerance;
}

bool do_intersect (Line first, Line second, CartesianPoint& point_of_intersection){
  bool answer;
  double determinant = 1.0 * first.coefficient_a * second.coefficient_b - second.coefficient_a * first.coefficient_b;

  if(std::abs(determinant) < kGeometryTolerance)
    answer = false;
  
  else{
    // coordinates of the intersection point
    point_of_intersection.x = (second.coefficient_b * first.coefficient_c - first.coefficient_b * second.coefficient_c) / determinant;
    point_of_intersection.y = (second.coefficient_c * first.coefficient_a - first.coefficient_c * second.coefficient_a) / determinant;
    answer = true;
  }
  
  return answer;
}


bool do_intersect(LineSegment first, LineSegment second, CartesianPoint& intersection){
  bool answer;
  if(do_intersect(static_cast<Line>(first), static_cast<Line>(second), intersection)){
    if((is_point_in_segment(intersection, first))  && (is_point_in_segment(intersection, second)))
      answer = true;
    else
      answer = false;
  }
  else
    answer = false;

  return answer;
}


bool do_intersect(Vector vector, Line line, CartesianPoint& intersection){
  /* LineSegment from_vector = LineSegment(vector.origin, 
					CartesianPoint(vector.origin.x+cos(vector.theta)*vector.intensity, 
						       vector.origin.y + sin(vector.theta) * vector.intensity));

  
						       return do_line_segments_intersect(from_vector, line_segment, intersection);*/
   return false;
}


bool do_intersect(Vector vector, LineSegment line_segment, CartesianPoint& intersection){
  
  LineSegment from_vector = LineSegment(vector.origin, CartesianPoint(vector.origin.x + cos(vector.theta) * vector.intensity, vector.origin.y + sin(vector.theta) * vector.intensity));

  return do_intersect(line_segment,from_vector, intersection);
}


bool do_intersect(Vector vector1, Vector vector2, CartesianPoint& intersection){
  
  LineSegment from_vector1 = LineSegment(vector1.origin, CartesianPoint(vector1.origin.x + cos(vector1.theta) * vector1.intensity, vector1.origin.y + sin(vector1.theta) * vector1.intensity));

  LineSegment from_vector2 = LineSegment(vector2.origin, CartesianPoint(vector2.origin.x + cos(vector2.theta) * vector2.intensity, vector2.origin.y + sin(vector2.theta) * vector2.intensity));
  
  return do_intersect(from_vector1, from_vector2, intersection);
}


//returns true if there is a point that is "visible" by the wall distance vectors.  
//A point is visible if the distance to the nearest wall distance vector lines is > distance to the point.
bool canAccessPoint(std::vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double distanceLimit){
  // cout << "AgentState:canAccessPoint() , robot pos " << laserPos.get_x() << "," << laserPos.get_y() << " target " << point.get_x() << "," << point.get_y() << endl; 
  // cout << "Number of laser endpoints " << givenLaserEndpoints.size() << endl; 
  bool canAccessPoint = false;
  if (givenLaserEndpoints.size() < 5 || distanceLimit < 0.0) {
    return false;
  }
  double distLaserPosToPoint = laserPos.get_distance(point);
  if(distLaserPosToPoint > distanceLimit){
    // cout << "Cannot access, too far away" << endl;
    return false;
  }
  double point_direction = atan2((point.get_y() - laserPos.get_y()), (point.get_x() - laserPos.get_x()));
  int index = 0;
  double min_angle = 100000;
  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //cout << "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y() << endl;
    double laser_direction = atan2((givenLaserEndpoints[i].get_y() - laserPos.get_y()), (givenLaserEndpoints[i].get_x() - laserPos.get_x()));
    double angle_diff = laser_direction - point_direction;
    if(angle_diff > M_PI)
      angle_diff = angle_diff - 2*M_PI;
    if(angle_diff < -M_PI)
      angle_diff = angle_diff + 2*M_PI;
    angle_diff = fabs(angle_diff);
    // cout << "point_direction " << point_direction << " laser_direction " << laser_direction << " angle_diff " << angle_diff << endl;
    if(angle_diff < min_angle){
      // cout << "Laser Direction : " << laser_direction << ", Point Direction : " << point_direction << endl;
      min_angle = angle_diff;
      index = i;
    }
  }
  while (index-2 < 0){
    index = index + 1;
  }
  while (index+2 > givenLaserEndpoints.size()-1){
    index = index - 1;
  }
  // cout << "Min angle : " << min_angle << ", " << index << endl;
  int numFree = 0;
  for(int i = -2; i < 3; i++) {
    double distLaserEndPointToLaserPos = givenLaserEndpoints[index+i].get_distance(laserPos);
    // cout << "Distance Laser EndPoint to Laser Pos : " << distLaserEndPointToLaserPos << ", Distance Laser Pos to Point : " << distLaserPosToPoint << endl;
    if (distLaserEndPointToLaserPos > distLaserPosToPoint) {
      numFree++;
    }
  }
  // cout << "Number farther than point : " << numFree << endl;
  if (numFree > 3) {
    canAccessPoint = true;
  }
  else{
    return false;
  }
  //else, not visible
  //return canAccessPoint;
  double epsilon = 0.005;
  bool canSeePoint = false;
  double ab = laserPos.get_distance(point);
  for(int i = -2; i < 3; i++) {
    //cout << "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y() << endl;
    double ac = laserPos.get_distance(givenLaserEndpoints[index+i]);
    double bc = givenLaserEndpoints[index+i].get_distance(point);
    if(((ab + bc) - ac) < epsilon){
      // cout << "Distance vector endpoint visible: ("<<givenLaserEndpoints[index+i].get_x()<<","<< givenLaserEndpoints[index+i].get_y()<<")"<<endl; 
      // cout << "Distance: "<<distance_to_point<<endl;
      canSeePoint = true;
      break;
    }
  }
  if(canSeePoint == false){
    for(int i = 0; i < givenLaserEndpoints.size(); i++){
      //cout << "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y() << endl;
      double ac = laserPos.get_distance(givenLaserEndpoints[i]);
      double bc = givenLaserEndpoints[i].get_distance(point);
      if(((ab + bc) - ac) < epsilon){
        // cout << "Distance vector endpoint visible: ("<<givenLaserEndpoints[i].get_x()<<","<< givenLaserEndpoints[i].get_y()<<")"<<endl; 
        // cout << "Distance: "<<distance_to_point<<endl;
        canSeePoint = true;
        break;
      }
    }
  }
  if(canSeePoint and canAccessPoint){
    return true;
  }
  else{
    return false;
  }
}
