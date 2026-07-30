#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <semaforr_msgs/msg/decision_action.hpp>
#include <semaforr_msgs/msg/decision_record.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <string_view>

namespace {

using semaforr_msgs::msg::DecisionAction;
using semaforr_msgs::msg::DecisionRecord;

std::string actionText(const DecisionAction &action) {
  std::ostringstream text;
  switch (action.type) {
  case DecisionAction::FORWARD:
    text << "move forward";
    break;
  case DecisionAction::TURN_RIGHT:
    text << "turn right";
    break;
  case DecisionAction::TURN_LEFT:
    text << "turn left";
    break;
  case DecisionAction::PAUSE:
    return "pause";
  default:
    return "stop safely";
  }
  text << " using action magnitude " << action.magnitude_index;
  return text.str();
}

std::string_view tierText(std::uint8_t tier) {
  switch (tier) {
  case DecisionRecord::TIER_ONE:
    return "a mandatory safety or goal rule";
  case DecisionRecord::TIER_TWO:
    return "the selected plan";
  case DecisionRecord::TIER_THREE:
    return "weighted advisor arbitration";
  case DecisionRecord::EXPLORATION:
    return "the exploration policy";
  case DecisionRecord::FALLBACK:
    return "the configured fallback";
  case DecisionRecord::SAFE_STOP:
    return "the safe-stop policy";
  default:
    return "an unknown decision source";
  }
}

std::string confidenceText(const DecisionRecord &record) {
  if (record.selected_tier == DecisionRecord::SAFE_STOP) {
    return "Confidence is deliberately conservative because no safe motion "
           "candidate was available.";
  }
  using ActionKey = std::pair<std::uint8_t, std::uint32_t>;
  std::map<ActionKey, double> totals;
  for (const auto &contribution : record.advisor_contributions) {
    totals[{contribution.action.type, contribution.action.magnitude_index}] +=
        contribution.weighted_score;
  }
  const ActionKey selected_key{record.selected_action.type,
                               record.selected_action.magnitude_index};
  const auto selected_entry = totals.find(selected_key);
  if (selected_entry == totals.end()) {
    return "The decision was produced directly by policy rather than an "
           "advisor score.";
  }
  double alternative = -std::numeric_limits<double>::infinity();
  for (const auto &[action, score] : totals) {
    if (action != selected_key) {
      alternative = std::max(alternative, score);
    }
  }
  if (!std::isfinite(alternative)) {
    return "The selected action was the only action scored by advisors.";
  }
  const double margin = selected_entry->second - alternative;
  if (margin > 1.0) {
    return "Advisor support was substantially stronger than the alternatives.";
  }
  if (margin > 0.1) {
    return "Advisor support was moderately stronger than the alternatives.";
  }
  return "Advisor support was close, so deterministic tie policy was "
         "important.";
}

class ActionExplanationNode final : public rclcpp::Node {
public:
  ActionExplanationNode() : Node("why") {
    const auto input_topic = declare_parameter<std::string>(
        "decision_records_topic", "decision_records");
    const auto output_topic =
        declare_parameter<std::string>("explanations_topic", "explanations");
    publisher_ = create_publisher<std_msgs::msg::String>(output_topic, 10);
    subscription_ = create_subscription<DecisionRecord>(
        input_topic, rclcpp::QoS(10).reliable(),
        [this](DecisionRecord::ConstSharedPtr record) { explain(*record); });
  }

private:
  void explain(const DecisionRecord &record) {
    std_msgs::msg::String output;
    std::ostringstream explanation;
    explanation << "Decision " << record.sequence << ": I chose to "
                << actionText(record.selected_action) << " because "
                << tierText(record.selected_tier) << " selected it.";
    if (!record.vetoes.empty()) {
      explanation << ' ' << record.vetoes.size()
                  << " unsafe candidate action(s) were vetoed.";
    }
    if (record.has_task) {
      explanation << " This advances task " << record.task.task_index << '.';
    }
    explanation << ' ' << confidenceText(record);
    if (!record.outcome_detail.empty()) {
      explanation << " Execution status: " << record.outcome_detail << '.';
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
  rclcpp::spin(std::make_shared<ActionExplanationNode>());
  rclcpp::shutdown();
  return 0;
}
