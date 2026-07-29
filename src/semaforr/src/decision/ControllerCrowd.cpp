/*
 * Domain-owned crowd learning configuration.
 */

#include <semaforr/decision/Controller.hpp>

#include <memory>
#include <utility>

void Controller::initialize_crowd_learning(
  const semaforr::config::Configuration& configuration)
{
  const auto& configured = configuration.controller.crowd_learning;
  if (!configured.enabled) {
    crowdLearner.reset();
    return;
  }

  semaforr::social::CrowdFieldLearnerConfiguration learner;
  learner.geometry.frame_id = configured.frame;
  learner.geometry.width_m =
    static_cast<double>(configuration.map_dimensions.length);
  learner.geometry.height_m =
    static_cast<double>(configuration.map_dimensions.height);
  learner.geometry.resolution_m = configured.resolution_m;
  learner.geometry.origin_x_m = configured.origin_x_m;
  learner.geometry.origin_y_m = configured.origin_y_m;
  learner.strategy = semaforr::social::crowdEstimatorStrategyFromString(
    configured.estimator);
  learner.discount_factor = configured.discount_factor;
  learner.minimum_update_period_s = configured.minimum_update_period_s;
  learner.encounter_radius_m = configured.encounter_radius_m;
  learner.minimum_flow_speed_mps = configured.minimum_flow_speed_mps;
  learner.confidence_exposures = configured.confidence_exposures;
  learner.cusum_increase = configured.cusum_increase;
  learner.cusum_decrease = configured.cusum_decrease;
  learner.cusum_threshold = configured.cusum_threshold;
  learner.random_seed = configured.random_seed;
  crowdLearner = std::make_unique<semaforr::social::CrowdFieldLearner>(
    std::move(learner));
}
