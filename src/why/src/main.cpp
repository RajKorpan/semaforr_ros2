#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <semaforr_msgs/msg/decision_record.hpp>
#include <semaforr_msgs/msg/explanation_question.hpp>
#include <semaforr_msgs/msg/explanation_response.hpp>
#include <string>
#include <why/why_system.hpp>

namespace {

class WhyNode final : public rclcpp::Node {
 public:
  WhyNode() : Node("why") {
    const auto records_topic = declare_parameter<std::string>(
        "decision_records_topic", "decision_records");
    const auto questions_topic = declare_parameter<std::string>(
        "questions_topic", "why_questions");
    const auto responses_topic = declare_parameter<std::string>(
        "responses_topic", "why_responses");
    responses_ = create_publisher<semaforr_msgs::msg::ExplanationResponse>(
        responses_topic, rclcpp::QoS(10).reliable());
    records_ = create_subscription<semaforr_msgs::msg::DecisionRecord>(
        records_topic, rclcpp::QoS(100).reliable(),
        [this](const semaforr_msgs::msg::DecisionRecord::ConstSharedPtr record) {
          why_.record(*record);
        });
    questions_ = create_subscription<semaforr_msgs::msg::ExplanationQuestion>(
        questions_topic, rclcpp::QoS(10).reliable(),
        [this](
            const semaforr_msgs::msg::ExplanationQuestion::ConstSharedPtr question) {
          responses_->publish(why_.answer(*question));
        });
  }

 private:
  semaforr::why::UnifiedWhySystem why_;
  rclcpp::Publisher<semaforr_msgs::msg::ExplanationResponse>::SharedPtr
      responses_;
  rclcpp::Subscription<semaforr_msgs::msg::DecisionRecord>::SharedPtr records_;
  rclcpp::Subscription<semaforr_msgs::msg::ExplanationQuestion>::SharedPtr
      questions_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhyNode>());
  rclcpp::shutdown();
  return 0;
}
