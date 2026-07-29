#include <semaforr/ros/visualization_publisher.hpp>

#include <cstdint>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <social_context_msgs/msg/crowd_field.hpp>

#include <semaforr/decision/Controller.hpp>
#include <semaforr/ros/Visualizer.hpp>

namespace semaforr::ros {
namespace {

builtin_interfaces::msg::Time toRosTime(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
  result.nanosec = static_cast<std::uint32_t>(
    nanoseconds % 1'000'000'000LL);
  return result;
}

}  // namespace

class VisualizationPublisher::Impl {
public:
  Impl(rclcpp::Node& node, Controller& controller)
    : node_(node),
      controller_(controller),
      visualizer_(node, controller),
      crowd_field_publisher_(
        node.create_publisher<social_context_msgs::msg::CrowdField>(
          node.get_parameter("topics.crowd_field").as_string(),
          rclcpp::QoS(1).transient_local().reliable()))
  {
  }

  void publishCrowdField()
  {
    const auto& snapshot = controller_.getCrowdModel().learned();
    if (!snapshot.available() || snapshot.version == last_crowd_version_) {
      return;
    }
    social_context_msgs::msg::CrowdField message;
    message.header.frame_id = snapshot.geometry.frame_id;
    message.header.stamp = toRosTime(snapshot.generated_at.count());
    message.width_m = snapshot.geometry.width_m;
    message.height_m = snapshot.geometry.height_m;
    message.resolution_m = snapshot.geometry.resolution_m;
    message.origin_x_m = snapshot.geometry.origin_x_m;
    message.origin_y_m = snapshot.geometry.origin_y_m;
    message.columns =
      static_cast<std::uint32_t>(snapshot.geometry.columns());
    message.rows = static_cast<std::uint32_t>(snapshot.geometry.rows());
    message.estimator = snapshot.estimator;
    message.version = snapshot.version;
    message.cells.reserve(snapshot.cells.size());
    for (const auto& source : snapshot.cells) {
      social_context_msgs::msg::CrowdFieldCell cell;
      cell.density = source.density;
      cell.learned_encounter_risk = source.learned_encounter_risk;
      cell.directional_flow = source.directional_flow;
      cell.visibility_exposures = source.visibility_exposures;
      cell.pedestrian_hits = source.pedestrian_hits;
      cell.risk_encounters = source.risk_encounters;
      cell.risk_experiences = source.risk_experiences;
      cell.last_updated = toRosTime(source.last_updated.count());
      cell.confidence = source.confidence;
      message.cells.push_back(std::move(cell));
    }
    crowd_field_publisher_->publish(message);
    last_crowd_version_ = snapshot.version;
  }

  rclcpp::Node& node_;
  Controller& controller_;
  Visualizer visualizer_;
  rclcpp::Publisher<social_context_msgs::msg::CrowdField>::SharedPtr
    crowd_field_publisher_;
  std::uint64_t last_crowd_version_{0U};
};

VisualizationPublisher::VisualizationPublisher(
  rclcpp::Node& node,
  Controller& controller)
  : impl_(std::make_unique<Impl>(node, controller))
{
}

VisualizationPublisher::~VisualizationPublisher() = default;
VisualizationPublisher::VisualizationPublisher(
  VisualizationPublisher&&) noexcept = default;
VisualizationPublisher& VisualizationPublisher::operator=(
  VisualizationPublisher&&) noexcept = default;

void VisualizationPublisher::publishSnapshot()
{
  impl_->visualizer_.publish();
  impl_->publishCrowdField();
}

void VisualizationPublisher::publishDecision(
  const decision::DecisionResult& result,
  double mission_time_s,
  double computation_time_s)
{
  impl_->visualizer_.publishLog(
    result, mission_time_s, computation_time_s);
}

}  // namespace semaforr::ros
