#include <memory>
#include <semaforr/decision/social_advisor_registry.hpp>
#include <utility>

namespace semaforr::decision {

void registerSocialAdvisorFactories(
    AdvisorRegistry& registry,
    SocialAdvisorRegistryConfiguration configuration) {
  configuration.density.objective = LearnedCrowdObjective::AvoidDensity;
  configuration.risk.objective = LearnedCrowdObjective::AvoidEncounterRisk;
  configuration.flow.objective = LearnedCrowdObjective::PreferFollowingFlow;

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
