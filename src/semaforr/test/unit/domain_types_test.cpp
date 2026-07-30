#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/mission.hpp>
#include <semaforr/domain/world_model.hpp>
#include <stdexcept>
#include <vector>

namespace {

using semaforr::domain::Action;
using semaforr::domain::ActionSpace;
using semaforr::domain::ActionType;
using semaforr::domain::Angle;
using semaforr::domain::Mission;
using semaforr::domain::NavigationTask;
using semaforr::domain::Point2D;

TEST(Action, ValidatesMagnitudeAtConstruction) {
  EXPECT_EQ(Action::pause().type(), ActionType::Pause);
  EXPECT_THROW(Action(ActionType::Pause, 1U), std::invalid_argument);
  EXPECT_THROW(Action(ActionType::Forward, 0U), std::invalid_argument);
  EXPECT_THROW(
      Action(ActionType::Forward, Action::maximum_magnitude_index + 1U),
      std::out_of_range);
}

TEST(Action, OrdersByTypeThenMagnitude) {
  const std::vector<Action> expected{
      Action(ActionType::Forward, 1U), Action(ActionType::Forward, 2U),
      Action(ActionType::TurnRight, 1U), Action(ActionType::TurnLeft, 1U),
      Action::pause()};
  auto shuffled = std::vector<Action>{expected[4], expected[2], expected[1],
                                      expected[3], expected[0]};
  std::sort(shuffled.begin(), shuffled.end());
  EXPECT_EQ(shuffled, expected);
}

TEST(ActionSpace, RequiresStrictlyIncreasingPositiveValues) {
  EXPECT_NO_THROW(ActionSpace(std::vector<double>{0.1, 0.2, 0.4},
                              std::vector<double>{0.1, 0.5}));
  EXPECT_THROW(
      ActionSpace(std::vector<double>{0.1, 0.1}, std::vector<double>{0.2}),
      std::invalid_argument);
  EXPECT_THROW(
      ActionSpace(std::vector<double>{0.0, 0.1}, std::vector<double>{0.2}),
      std::invalid_argument);
}

TEST(Geometry, NormalizesAnglesOnce) {
  const Angle angle(3.0 * std::numbers::pi);
  EXPECT_NEAR(angle.radians(), std::numbers::pi, 1.0e-12);
  EXPECT_THROW(Angle(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(Mission, SeparatesPendingActiveCompletedAndSkippedTasks) {
  Mission mission({NavigationTask{1U, Point2D{1.0, 2.0}},
                   NavigationTask{2U, Point2D{3.0, 4.0}}},
                  2U);

  ASSERT_TRUE(mission.activate_next());
  ASSERT_TRUE(mission.active().has_value());
  EXPECT_EQ(mission.active()->id, 1U);
  EXPECT_TRUE(mission.record_decision());
  EXPECT_TRUE(mission.complete_active());
  EXPECT_EQ(mission.completed().size(), 1U);

  ASSERT_TRUE(mission.activate_next());
  EXPECT_TRUE(mission.skip_active());
  EXPECT_EQ(mission.skipped().size(), 1U);
  EXPECT_TRUE(mission.finished());
}

}  // namespace
