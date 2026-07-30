#ifndef SEMAFORR_ROS_VISUALIZATION_PUBLISHER_HPP
#define SEMAFORR_ROS_VISUALIZATION_PUBLISHER_HPP

#include <memory>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/world_model.hpp>

namespace rclcpp {
class Node;
}

namespace semaforr::ros {

class VisualizationPublisher {
 public:
  VisualizationPublisher(rclcpp::Node& node, const domain::WorldModel& world);
  ~VisualizationPublisher();

  VisualizationPublisher(const VisualizationPublisher&) = delete;
  VisualizationPublisher& operator=(const VisualizationPublisher&) = delete;
  VisualizationPublisher(VisualizationPublisher&&) noexcept;
  VisualizationPublisher& operator=(VisualizationPublisher&&) noexcept;

  void publishSnapshot();
  void publishDecision(const decision::DecisionResult& result);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_VISUALIZATION_PUBLISHER_HPP
