#ifndef SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP
#define SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hunav_msgs/msg/agents.hpp>
#include <map>
#include <optional>
#include <rclcpp/time.hpp>
#include <semaforr/domain/social.hpp>
#include <semaforr/ros/message_adapters.hpp>
#include <social_context_msgs/msg/formation_group_array.hpp>
#include <social_context_msgs/msg/tracked_person_array.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace semaforr::ros {

enum class SocialInputMode { None, Tracked, Hunav };

SocialInputMode socialInputModeFromString(std::string_view value);
std::string_view toString(SocialInputMode mode) noexcept;

struct SocialObservationConfiguration {
  std::string frame{"map"};
  SocialInputMode input_mode{SocialInputMode::Tracked};
  double current_maximum_age_s{0.75};
  double prediction_maximum_age_s{6.0};
  double formation_maximum_age_s{1.0};
  double minimum_confidence{0.25};
  double minimum_formation_confidence{0.5};
  double prediction_step_s{1.0};
  std::size_t prediction_steps{5U};
  bool constant_velocity_fallback{true};
  SocialAdapterConfiguration adapter;
};

enum class SocialObservationStatus {
  NoData,
  Ready,
  SourceMismatch,
  FrameMismatch,
  Invalid,
  Stale,
  ClockReset
};

struct TrackLifecycleEvent {
  enum class Type { Appeared, Disappeared, Reappeared };
  std::string pedestrian_id;
  Type type{Type::Appeared};
};

class SocialObservationBuffer {
 public:
  explicit SocialObservationBuffer(
      SocialObservationConfiguration configuration);

  bool accept(domain::CrowdObservation observation,
              const rclcpp::Time& received_at);
  bool acceptTracked(
      const social_context_msgs::msg::TrackedPersonArray& message,
      const rclcpp::Time& received_at);
  bool acceptHunav(const hunav_msgs::msg::Agents& message,
                   const rclcpp::Time& received_at);
  bool acceptPrediction(const geometry_msgs::msg::PoseStamped& message,
                        const rclcpp::Time& received_at);
  bool acceptFormations(
      const social_context_msgs::msg::FormationGroupArray& message,
      const rclcpp::Time& received_at);

  SocialObservationStatus status(const rclcpp::Time& now) const;
  std::optional<domain::CrowdObservation> snapshot(
      const rclcpp::Time& now) const;
  std::vector<TrackLifecycleEvent> takeLifecycleEvents();
  void clear() noexcept;

 private:
  struct PredictionCycle {
    std::map<std::size_t, domain::Point2D> steps;
    std::optional<rclcpp::Time> received_at;
  };

  void recordLifecycle(const domain::CrowdObservation& observation);
  bool sourceMatches(std::string_view provenance) const noexcept;

  SocialObservationConfiguration configuration_;
  std::optional<domain::CrowdObservation> observation_;
  std::optional<rclcpp::Time> received_at_;
  std::unordered_map<std::string, PredictionCycle> predictions_;
  std::vector<domain::FormationObservation> formations_;
  std::optional<rclcpp::Time> formations_received_at_;
  std::unordered_set<std::string> active_ids_;
  std::unordered_set<std::string> seen_ids_;
  std::vector<TrackLifecycleEvent> lifecycle_events_;
  SocialObservationStatus last_status_{SocialObservationStatus::NoData};
};

std::string_view toString(SocialObservationStatus status) noexcept;
std::string_view toString(TrackLifecycleEvent::Type type) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP
