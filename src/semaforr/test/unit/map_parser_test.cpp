#include <gtest/gtest.h>

#include <semaforr/planning/map_parser.hpp>
#include <sstream>

TEST(MapParser, ParsesValidatedMeterBasedSegments) {
  std::istringstream input(R"xml(
    <Experiment>
      <ObstacleSet>
        <Obstacle closed="1">
          <Vertex p_x="1.0" p_y="2.0"/>
          <Vertex p_x="3.0" p_y="4.0"/>
        </Obstacle>
      </ObstacleSet>
    </Experiment>)xml");
  const auto map = semaforr::planning::parseMapXml(input, "inline map");
  ASSERT_EQ(map.walls.size(), 1U);
  EXPECT_DOUBLE_EQ(map.walls.front().start.x_m, 1.0);
  EXPECT_DOUBLE_EQ(map.walls.front().end.y_m, 4.0);
}

TEST(MapParser, RejectsMalformedVerticesPrecisely) {
  std::istringstream input(R"xml(
    <ObstacleSet><Obstacle>
      <Vertex p_x="one" p_y="2"/>
      <Vertex p_x="3" p_y="4"/>
    </Obstacle></ObstacleSet>)xml");
  EXPECT_THROW(semaforr::planning::parseMapXml(input, "bad map"),
               std::runtime_error);
}
