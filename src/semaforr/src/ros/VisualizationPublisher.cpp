#include <semaforr/ros/visualization_publisher.hpp>

#include <utility>

#include <semaforr/decision/Controller.hpp>
#include <semaforr/ros/Visualizer.hpp>

namespace semaforr::ros {

class VisualizationPublisher::Impl {
public:
  Impl(rclcpp::Node& node, Controller& controller)
    : visualizer_(node, controller)
  {
  }

  Visualizer visualizer_;
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
