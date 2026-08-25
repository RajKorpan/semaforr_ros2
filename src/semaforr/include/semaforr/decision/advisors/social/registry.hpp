/**
 * @file registry.hpp
 * @brief Registry responsibilities.
 *
 * @details This file defines registry behavior for tiered decision making and
 * action arbitration. It centers on `SocialAdvisorRegistryConfiguration`.
 * Its package-relative location is
 * `include/semaforr/decision/advisors/social/registry.hpp`.
 */
#ifndef SEMAFORR_DECISION_SOCIAL_ADVISOR_REGISTRY_HPP
#define SEMAFORR_DECISION_SOCIAL_ADVISOR_REGISTRY_HPP

#include <semaforr/decision/advisors/social/learned_crowd_advisor.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/decision/advisors/social/social_navigation_advisor.hpp>

namespace semaforr::decision {

/**
 * @brief Encapsulates social advisor registry configuration state and
 * behavior for this subsystem.
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
struct SocialAdvisorRegistryConfiguration {
  SocialAdvisorConfiguration live;
  LearnedCrowdAdvisorConfiguration density;
  LearnedCrowdAdvisorConfiguration risk;
  LearnedCrowdAdvisorConfiguration flow;
};

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
    SocialAdvisorRegistryConfiguration configuration);

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_SOCIAL_ADVISOR_REGISTRY_HPP
