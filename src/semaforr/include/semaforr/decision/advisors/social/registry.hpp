#ifndef SEMAFORR_DECISION_SOCIAL_ADVISOR_REGISTRY_HPP
#define SEMAFORR_DECISION_SOCIAL_ADVISOR_REGISTRY_HPP

#include <semaforr/decision/advisors/social/learned_crowd_advisor.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/decision/advisors/social/social_navigation_advisor.hpp>

namespace semaforr::decision {

struct SocialAdvisorRegistryConfiguration {
  SocialAdvisorConfiguration live;
  LearnedCrowdAdvisorConfiguration density;
  LearnedCrowdAdvisorConfiguration risk;
  LearnedCrowdAdvisorConfiguration flow;
};

void registerSocialAdvisorFactories(
    AdvisorRegistry& registry,
    SocialAdvisorRegistryConfiguration configuration);

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_SOCIAL_ADVISOR_REGISTRY_HPP
