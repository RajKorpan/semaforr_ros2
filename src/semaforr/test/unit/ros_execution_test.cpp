#include <gtest/gtest.h>
#include <rcl/time.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <numbers>
#include <rclcpp/time.hpp>
#include <semaforr/ros/command_executor.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

namespace {

rclcpp::Time at(double seconds) {
  return rclcpp::Time(static_cast<std::int64_t>(seconds * 1.0e9), RCL_ROS_TIME);
}

geometry_msgs::msg::PoseStamped pose(double stamp_s, double x, double y,
                                     double yaw, std::string frame = "map") {
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = std::move(frame);
  message.header.stamp = at(stamp_s);
  message.pose.position.x = x;
  message.pose.position.y = y;
  message.pose.orientation.z = std::sin(yaw / 2.0);
  message.pose.orientation.w = std::cos(yaw / 2.0);
  return message;
}

sensor_msgs::msg::LaserScan scan(double stamp_s,
                                 std::string frame = "base_laser_link") {
  sensor_msgs::msg::LaserScan message;
  message.header.frame_id = std::move(frame);
  message.header.stamp = at(stamp_s);
  message.angle_min = -1.0F;
  message.angle_max = 1.0F;
  message.angle_increment = 1.0F;
  message.range_min = 0.05F;
  message.range_max = 5.0F;
  message.ranges = {1.0F, 2.0F, 1.0F};
  return message;
}

semaforr::domain::Pose2D domainPose(double x, double y, double yaw) {
  return {{x, y}, semaforr::domain::Angle(yaw)};
}

}  // namespace

TEST(SensorSynchronizer, ProducesOnlyCoherentFreshObservations) {
  using namespace semaforr::ros;
  SensorSynchronizer synchronizer({"map", "base_laser_link", 0.5, 0.05});

  EXPECT_TRUE(synchronizer.acceptPose(pose(1.0, 2.0, 3.0, 0.4), at(1.0)));
  EXPECT_EQ(synchronizer.status(at(1.0)), SensorStatus::WaitingForScan);
  EXPECT_TRUE(synchronizer.acceptScan(scan(1.02), at(1.02)));

  const auto observation = synchronizer.snapshot(at(1.03));
  ASSERT_TRUE(observation);
  EXPECT_DOUBLE_EQ(observation->pose.position.x_m, 2.0);
  EXPECT_DOUBLE_EQ(observation->pose.position.y_m, 3.0);
  EXPECT_NEAR(observation->pose.heading.radians(), 0.4, 1.0e-9);
  EXPECT_EQ(observation->scan.ranges_m.size(), 3U);
  EXPECT_EQ(synchronizer.status(at(1.6)), SensorStatus::PoseStale);
}

TEST(SensorSynchronizer, RejectsFramesAndExcessiveSkew) {
  using namespace semaforr::ros;
  SensorSynchronizer synchronizer({"map", "base_laser_link", 1.0, 0.05});

  EXPECT_FALSE(
      synchronizer.acceptPose(pose(1.0, 0.0, 0.0, 0.0, "odom"), at(1.0)));
  EXPECT_EQ(synchronizer.status(at(1.0)), SensorStatus::PoseFrameMismatch);

  EXPECT_TRUE(synchronizer.acceptPose(pose(1.0, 0.0, 0.0, 0.0), at(1.0)));
  EXPECT_TRUE(synchronizer.acceptScan(scan(1.2), at(1.2)));
  EXPECT_EQ(synchronizer.status(at(1.2)), SensorStatus::Unsynchronized);
}

TEST(SensorSynchronizer, RejectsStaleSourceStampsAndClockResets) {
  using namespace semaforr::ros;
  SensorSynchronizer stale({"map", "base_laser_link", 0.5, 2.0});
  EXPECT_TRUE(stale.acceptPose(pose(1.0, 0.0, 0.0, 0.0), at(2.0)));
  EXPECT_TRUE(stale.acceptScan(scan(2.0), at(2.0)));
  EXPECT_EQ(stale.status(at(2.0)), SensorStatus::PoseStale);

  SensorSynchronizer future({"map", "base_laser_link", 0.5, 0.05});
  EXPECT_TRUE(future.acceptPose(pose(3.0, 0.0, 0.0, 0.0), at(2.0)));
  EXPECT_TRUE(future.acceptScan(scan(3.0), at(2.0)));
  EXPECT_EQ(future.status(at(2.0)), SensorStatus::ClockReset);
}

TEST(CommandExecutor, SeparatesTargetDistanceFromVelocity) {
  using namespace semaforr;
  using namespace semaforr::ros;
  CommandExecutor executor(CommandExecutorConfiguration{});
  const ActionExecutionRequest request{
      domain::Action(domain::ActionType::Forward, 1U), 0.2, 0.0};

  const auto started =
      executor.start(request, domainPose(0.0, 0.0, 0.0), at(1.0));
  EXPECT_EQ(started.status, ActionExecutionStatus::Executing);
  EXPECT_DOUBLE_EQ(started.command.linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(started.target, 0.2);

  const auto completed = executor.update(domainPose(0.15, 0.0, 0.0), at(1.2));
  EXPECT_EQ(completed.status, ActionExecutionStatus::Completed);
  EXPECT_DOUBLE_EQ(completed.command.linear_mps, 0.0);
}

TEST(CommandExecutor, SmallTurnsRequireMeasuredProgressAndWrapAngles) {
  using namespace semaforr;
  using namespace semaforr::ros;
  CommandExecutor executor(CommandExecutorConfiguration{});
  const ActionExecutionRequest request{
      domain::Action(domain::ActionType::TurnLeft, 1U), 0.0, 0.0873};

  executor.start(request, domainPose(0.0, 0.0, 3.10), at(1.0));
  EXPECT_EQ(executor.update(domainPose(0.0, 0.0, 3.10), at(1.01)).status,
            ActionExecutionStatus::Executing);
  EXPECT_EQ(executor.update(domainPose(0.0, 0.0, -3.10), at(1.2)).status,
            ActionExecutionStatus::Completed);
}

TEST(CommandExecutor, ReportsTimeoutsAndOdometryResetsSafely) {
  using namespace semaforr;
  using namespace semaforr::ros;
  const ActionExecutionRequest request{
      domain::Action(domain::ActionType::Forward, 1U), 0.1, 0.0};

  CommandExecutor timeout_executor(CommandExecutorConfiguration{});
  timeout_executor.start(request, domainPose(0.0, 0.0, 0.0), at(1.0));
  const auto timed_out =
      timeout_executor.update(domainPose(0.0, 0.0, 0.0), at(2.1));
  EXPECT_EQ(timed_out.status, ActionExecutionStatus::TimedOut);
  EXPECT_DOUBLE_EQ(timed_out.command.linear_mps, 0.0);

  CommandExecutor reset_executor(CommandExecutorConfiguration{});
  reset_executor.start(request, domainPose(0.0, 0.0, 0.0), at(1.0));
  const auto reset = reset_executor.update(domainPose(5.0, 0.0, 0.0), at(1.1));
  EXPECT_EQ(reset.status, ActionExecutionStatus::OdometryReset);
  EXPECT_DOUBLE_EQ(reset.command.linear_mps, 0.0);
}

TEST(CommandExecutor, EnforcesActionVelocityAndAccelerationBounds) {
  using namespace semaforr;
  using namespace semaforr::ros;
  CommandExecutorConfiguration configuration;
  configuration.maximum_move_action_index = 1U;
  configuration.maximum_rotation_action_index = 1U;
  configuration.maximum_linear_acceleration_mps2 = 1.0;
  CommandExecutor executor(configuration);

  EXPECT_THROW(
      executor.start(
          {domain::Action(domain::ActionType::Forward, 2U), 0.2, 0.0},
          domainPose(0.0, 0.0, 0.0), at(1.0)),
      std::invalid_argument);

  executor.start(
      {domain::Action(domain::ActionType::Forward, 1U), 1.0, 0.0},
      domainPose(0.0, 0.0, 0.0), at(1.0));
  const auto first =
      executor.update(domainPose(0.0, 0.0, 0.0), at(1.1));
  EXPECT_NEAR(first.command.linear_mps, 0.1, 1.0e-9);
  const auto second =
      executor.update(domainPose(0.0, 0.0, 0.0), at(1.2));
  EXPECT_NEAR(second.command.linear_mps, 0.2, 1.0e-9);

  auto invalid = configuration;
  invalid.linear_velocity_mps = 0.6;
  EXPECT_THROW(
      {
        const CommandExecutor rejected(invalid);
        static_cast<void>(rejected);
      },
      std::invalid_argument);
}

TEST(CommandExecutor, SensorTimeoutCancellationPublishesAZeroCommand) {
  using namespace semaforr;
  using namespace semaforr::ros;

  SensorSynchronizer synchronizer({"map", "base_laser_link", 0.25, 0.05});
  ASSERT_TRUE(synchronizer.acceptPose(pose(1.0, 0.0, 0.0, 0.0), at(1.0)));
  ASSERT_TRUE(synchronizer.acceptScan(scan(1.0), at(1.0)));

  CommandExecutor executor(CommandExecutorConfiguration{});
  executor.start({domain::Action(domain::ActionType::Forward, 1U), 1.0, 0.0},
                 domainPose(0.0, 0.0, 0.0), at(1.0));
  ASSERT_NE(synchronizer.status(at(1.3)), SensorStatus::Ready);
  const auto cancelled = executor.cancel();
  EXPECT_EQ(cancelled.status, ActionExecutionStatus::Cancelled);
  EXPECT_DOUBLE_EQ(cancelled.command.linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(cancelled.command.angular_radps, 0.0);
}
