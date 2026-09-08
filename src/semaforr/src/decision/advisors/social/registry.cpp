/**
 * @file registry.cpp
 * @brief Registry responsibilities.
 *
 * @details This file implements registry behavior for tiered decision making
 * and action arbitration. It records the declarations, settings, fixtures, or
 * guidance needed by that responsibility. Its package-relative location is
 * `src/decision/advisors/social/registry.cpp`.
 */
#include <memory>
#include <semaforr/decision/advisors/social/registry.hpp>
#include <utility>

namespace semaforr::decision {

/**
 * @brief Registers social advisor factories for this subsystem.
 *
 * Arguments:
 * - @p registry: Supplies registry input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void registerSocialAdvisorFactories(
    AdvisorRegistry& registry,
    SocialAdvisorRegistryConfiguration configuration) {
  configuration.live.advisor_name = "social_navigation";
  configuration.density.objective = LearnedCrowdObjective::AvoidDensity;
  configuration.density.advisor_name = "crowd_avoid";
  configuration.risk.objective = LearnedCrowdObjective::AvoidEncounterRisk;
  configuration.risk.advisor_name = "risk_avoid";
  configuration.flow.objective = LearnedCrowdObjective::PreferFollowingFlow;
  configuration.flow.advisor_name = "flow_follow";

  registry.registerFactory("social_navigation", [value = configuration.live] {
    return std::make_unique<SocialNavigationAdvisor>(value);
  });
  registry.registerFactory("crowd_avoid", [value = configuration.density] {
    return std::make_unique<LearnedCrowdAdvisor>(value);
  });
  registry.registerFactory("risk_avoid", [value = configuration.risk] {
    return std::make_unique<LearnedCrowdAdvisor>(value);
  });
  registry.registerFactory("flow_follow", [value = configuration.flow] {
    return std::make_unique<LearnedCrowdAdvisor>(value);
  });
}

}  // namespace semaforr::decision
