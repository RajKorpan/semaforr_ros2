/**
 * @file social_observation_buffer.hpp
 * @brief Social observation buffer responsibilities.
 *
 * @details This file defines social observation buffer behavior for the ROS 2
 * composition and message-adaptation boundary. It centers on
 * `SocialInputMode`, `SocialObservationConfiguration`,
 * `SocialObservationStatus`, `TrackLifecycleEvent`, `Type`,
 * `SocialObservationBuffer`, `PredictionCycle`. Its package-relative
 * location is `include/semaforr/ros/social_observation_buffer.hpp`.
 */
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

/**
 * @brief Enumerates the supported social input mode values used by this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class SocialInputMode { None, Tracked, Hunav };

/**
 * @brief Performs the social input mode from string operation for this
 * subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `SocialInputMode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SocialInputMode socialInputModeFromString(std::string_view value);
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SocialInputMode mode) noexcept;

/**
 * @brief Encapsulates social observation configuration state and behavior
 * for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Enumerates the supported social observation status values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class SocialObservationStatus {
  NoData,
  Ready,
  SourceMismatch,
  FrameMismatch,
  Invalid,
  Stale,
  ClockReset
};

/**
 * @brief Encapsulates track lifecycle event state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct TrackLifecycleEvent {
  /**
   * @brief Enumerates the supported type values used by this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  enum class Type { Appeared, Disappeared, Reappeared };
  std::string pedestrian_id;
  Type type{Type::Appeared};
};

/**
 * @brief Encapsulates social observation buffer state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class SocialObservationBuffer {
 public:
  /**
   * @brief Performs the social observation buffer operation for this
   * subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit SocialObservationBuffer(
      SocialObservationConfiguration configuration);

  /**
   * @brief Performs the accept operation for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   * - @p received_at: Supplies received at input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool accept(domain::CrowdObservation observation,
              const rclcpp::Time& received_at);
  /**
   * @brief Performs the accept tracked operation for this subsystem.
   *
   * Arguments:
   * - @p message: Supplies message input to the operation.
   * - @p received_at: Supplies received at input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool acceptTracked(
      const social_context_msgs::msg::TrackedPersonArray& message,
      const rclcpp::Time& received_at);
  /**
   * @brief Performs the accept hunav operation for this subsystem.
   *
   * Arguments:
   * - @p message: Supplies message input to the operation.
   * - @p received_at: Supplies received at input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool acceptHunav(const hunav_msgs::msg::Agents& message,
                   const rclcpp::Time& received_at);
  /**
   * @brief Performs the accept prediction operation for this subsystem.
   *
   * Arguments:
   * - @p message: Supplies message input to the operation.
   * - @p received_at: Supplies received at input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool acceptPrediction(const geometry_msgs::msg::PoseStamped& message,
                        const rclcpp::Time& received_at);
  /**
   * @brief Performs the accept formations operation for this subsystem.
   *
   * Arguments:
   * - @p message: Supplies message input to the operation.
   * - @p received_at: Supplies received at input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool acceptFormations(
      const social_context_msgs::msg::FormationGroupArray& message,
      const rclcpp::Time& received_at);

  /**
   * @brief Performs the status operation for this subsystem.
   *
   * Arguments:
   * - @p now: Supplies now input to the operation.
   *
   * Returns:
   * - `SocialObservationStatus` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SocialObservationStatus status(const rclcpp::Time& now) const;
  /**
   * @brief Performs the snapshot operation for this subsystem.
   *
   * Arguments:
   * - @p now: Supplies now input to the operation.
   *
   * Returns:
   * - `std::optional<domain::CrowdObservation>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<domain::CrowdObservation> snapshot(
      const rclcpp::Time& now) const;
  /**
   * @brief Performs the take lifecycle events operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<TrackLifecycleEvent>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<TrackLifecycleEvent> takeLifecycleEvents();
  /**
   * @brief Clears package content for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void clear() noexcept;

 private:
  /**
   * @brief Encapsulates prediction cycle state and behavior for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  struct PredictionCycle {
    std::map<std::size_t, domain::Point2D> steps;
    std::optional<rclcpp::Time> received_at;
  };

  /**
   * @brief Records lifecycle for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordLifecycle(const domain::CrowdObservation& observation);
  /**
   * @brief Performs the source matches operation for this subsystem.
   *
   * Arguments:
   * - @p provenance: Supplies provenance input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SocialObservationStatus status) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p type: Supplies type input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(TrackLifecycleEvent::Type type) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP
