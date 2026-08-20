#ifndef SEMAFORR_DECISION_SOCIAL_NAVIGATION_ADVISOR_HPP
#define SEMAFORR_DECISION_SOCIAL_NAVIGATION_ADVISOR_HPP

#include <chrono>
#include <semaforr/decision/advisor.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

struct SocialAdvisorConfiguration {
  std::vector<double> move_distances_m;
  std::vector<double> rotation_angles_rad;
  std::chrono::nanoseconds maximum_age{std::chrono::milliseconds(750)};
  double minimum_confidence{0.25};
  double prediction_horizon_s{2.0};
  double personal_space_m{1.2};
  double collision_distance_m{0.65};
  double weight{1.0};
  std::string advisor_name{"social_navigation"};
};

class SocialNavigationAdvisor final : public Advisor {
 public:
  explicit SocialNavigationAdvisor(SocialAdvisorConfiguration configuration);

  std::string_view name() const noexcept override {
    return configuration_.advisor_name;
  }
  std::vector<std::string_view> dependencies() const override {
    return {"live_crowd_observations"};
  }
  AdvisorMetadata metadata() const override {
    return {dependencies(),
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            true, ScoreNormalization::TenPoint,
            "predictive personal-space and collision preference"};
  }

  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;

 private:
  double score(const domain::WorldModel& world,
               const domain::CrowdObservation& crowd,
               const domain::Action& action) const;

  SocialAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_SOCIAL_NAVIGATION_ADVISOR_HPP
