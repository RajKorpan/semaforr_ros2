#include <algorithm>
#include <semaforr/spatial/circumstance_learner.hpp>

namespace semaforr::spatial {

CircumstanceLearner::CircumstanceLearner()
    : SpatialLearnerBase(
          SpatialRepresentation::Circumstances, "circumstance",
          UpdateMode::Incremental,
          {true, true, true, true,
           "record completed actions at target boundaries for precedent",
           {"Precedent"}, UpdateSchedule::EndOfTarget}) {}

void CircumstanceLearner::onObserve(const NavigationEpisode& episode) {
  if (!episode.action_completed || !episode.selected_action) return;
  const auto found =
      std::find_if(model_.actions.begin(), model_.actions.end(),
                   [&](const auto& item) {
                     return item.action == *episode.selected_action;
                   });
  if (found == model_.actions.end())
    model_.actions.push_back({*episode.selected_action, 1U});
  else
    ++found->occurrences;
}

void CircumstanceLearner::onRebuild() {
  publish(model_, model_.actions.empty() ? ModelStatus::Incomplete
                                         : ModelStatus::Fresh,
          model_.actions.empty() ? "no completed action circumstances"
                                 : "circumstance snapshot published");
}

}  // namespace semaforr::spatial
