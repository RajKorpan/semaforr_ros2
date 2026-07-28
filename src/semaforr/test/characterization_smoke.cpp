#include <semaforr/core/FORRAction.h>
#include <semaforr/core/Position.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>

namespace
{
constexpr double kTolerance = 1e-12;

void require(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << "characterization failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void characterize_actions()
{
  const FORRAction default_action;
  require(default_action.type == PAUSE, "default action type is PAUSE");
  require(default_action.parameter == 0, "default action parameter is zero");

  const FORRAction forward_short(FORWARD, 1);
  const FORRAction forward_long(FORWARD, 2);
  const FORRAction right(RIGHT_TURN, 1);
  const FORRAction left(LEFT_TURN, 1);
  const FORRAction pause(PAUSE, 0);

  require(forward_short == FORRAction(FORWARD, 1),
    "equal action values compare equal");
  require(!(forward_short == forward_long),
    "different magnitudes compare unequal");
  require(forward_short < forward_long,
    "actions of one type are ordered by magnitude");
  require(forward_long < right,
    "action ordering follows the action type enumeration");
  require(right < left, "RIGHT_TURN sorts before LEFT_TURN");
  require(left < pause, "LEFT_TURN sorts before PAUSE");

  const std::set<FORRAction> actions{
    pause, right, forward_long, left, forward_short, forward_short
  };
  require(actions.size() == 5, "action ordering supports set de-duplication");

  const std::map<FORRAction, double> scores{
    {forward_short, 1.0},
    {forward_long, 2.0},
    {right, 3.0},
  };
  require(scores.at(FORRAction(FORWARD, 2)) == 2.0,
    "action values are stable map keys");
}

void characterize_positions()
{
  Position origin(0.0, 0.0, 0.0);
  Position point(3.0, 4.0, 1.25);

  require(std::fabs(origin.getDistance(point) - 5.0) < kTolerance,
    "position distance uses Euclidean x/y distance");
  require(std::fabs(point.getDistance(0.0, 0.0) - 5.0) < kTolerance,
    "coordinate distance uses Euclidean x/y distance");
  require(point == Position(3.0, 4.0, 1.25),
    "position equality compares x, y, and theta exactly");
  require(!(point == Position(3.0, 4.0, 1.2500001)),
    "position equality has no floating-point tolerance");

  point.setX(-2.0);
  point.setY(7.0);
  point.setTheta(-0.5);
  require(point.getX() == -2.0, "setX updates x");
  require(point.getY() == 7.0, "setY updates y");
  require(point.getTheta() == -0.5, "setTheta updates theta");
}
}  // namespace

int main()
{
  characterize_actions();
  characterize_positions();
  std::cout << "SemaFORR characterization checks passed\n";
  return EXIT_SUCCESS;
}
