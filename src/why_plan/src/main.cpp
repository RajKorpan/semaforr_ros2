#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <semaforr_msgs/msg/decision_record.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <string_view>

namespace {

using semaforr_msgs::msg::DecisionRecord;

std::string_view plannerObjective(std::string_view planner) {
  if (planner == "density") {
    return "reduce learned crowd density";
  }
  if (planner == "risk") {
    return "reduce predicted encounter risk";
  }
  if (planner == "flow") {
    return "align motion with pedestrian flow";
  }
  if (planner == "skeleton") {
    return "follow the learned spatial skeleton";
  }
  return "minimize metric path distance";
}

std::string_view outcomeText(std::uint8_t outcome) {
  switch (outcome) {
  case DecisionRecord::OUTCOME_COMPLETED:
    return "completed";
  case DecisionRecord::OUTCOME_TIMED_OUT:
    return "timed out";
  case DecisionRecord::OUTCOME_ODOMETRY_RESET:
    return "stopped after an odometry reset";
  case DecisionRecord::OUTCOME_CLOCK_RESET:
    return "stopped after a clock reset";
  case DecisionRecord::OUTCOME_CANCELLED:
    return "was cancelled";
  case DecisionRecord::OUTCOME_SENSOR_LOST:
    return "stopped because sensors became stale";
  case DecisionRecord::OUTCOME_SHUTDOWN:
    return "stopped during shutdown";
  case DecisionRecord::OUTCOME_PENDING:
    return "is pending";
  default:
    return "has an unknown outcome";
  }
}

class PlanExplanationNode final : public rclcpp::Node {
public:
  PlanExplanationNode() : Node("why_plan") {
    const auto input_topic = declare_parameter<std::string>(
        "decision_records_topic", "decision_records");
    const auto output_topic = declare_parameter<std::string>(
        "plan_explanations_topic", "plan_explanations");
    publisher_ = create_publisher<std_msgs::msg::String>(output_topic, 10);
    subscription_ = create_subscription<DecisionRecord>(
        input_topic, rclcpp::QoS(10).reliable(),
        [this](DecisionRecord::ConstSharedPtr record) { explain(*record); });
  }

private:
  void explain(const DecisionRecord &record) {
    if (!record.has_planner) {
      RCLCPP_DEBUG(get_logger(),
                   "Decision %lu did not select a planner; no plan "
                   "explanation published",
                   static_cast<unsigned long>(record.sequence));
      return;
    }

    std_msgs::msg::String output;
    std::ostringstream explanation;
    explanation << "Decision " << record.sequence << " used the '"
                << record.selected_planner << "' planner to "
                << plannerObjective(record.selected_planner) << '.';
    if (record.has_task) {
      const auto &target =
          record.task.has_waypoint ? record.task.waypoint : record.task.target;
      explanation << " The active navigation target is (" << target.x << ", "
                  << target.y << ") in the decision record frame.";
    }
    explanation << " Planning and arbitration completed in "
                << record.decision_latency_s << " seconds.";
    if (record.action_outcome != DecisionRecord::OUTCOME_PENDING) {
      explanation << " The selected action "
                  << outcomeText(record.action_outcome) << '.';
    }
    output.data = explanation.str();
    publisher_->publish(output);
    RCLCPP_INFO(get_logger(), "%s", output.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<DecisionRecord>::SharedPtr subscription_;
};

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanExplanationNode>());
  rclcpp::shutdown();
  return 0;
}
