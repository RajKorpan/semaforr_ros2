#ifndef SEMAFORR_VALIDATION_EXPERIMENT_METRICS_HPP
#define SEMAFORR_VALIDATION_EXPERIMENT_METRICS_HPP

#include <cstddef>
#include <map>
#include <optional>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>
#include <vector>

namespace semaforr::validation {

struct AllocationMeasurement {
  std::size_t count = 0U;
  std::size_t bytes = 0U;
};

struct ExperimentObservation {
  decision::DecisionResult decision;
  double runtime_s = 0.0;
  double planning_latency_s = 0.0;
  double model_update_cost_s = 0.0;
  AllocationMeasurement allocations;
  std::optional<std::size_t> covered_cells;
};

struct ExperimentSummary {
  std::string scenario;
  std::string profile;
  std::string configuration_fingerprint;
  std::size_t targets_attempted = 0U;
  std::size_t targets_succeeded = 0U;
  std::size_t decisions = 0U;
  double success_rate = 0.0;
  double exploration_distance_m = 0.0;
  double target_distance_m = 0.0;
  double total_distance_m = 0.0;
  double exploration_runtime_s = 0.0;
  double target_runtime_s = 0.0;
  double total_runtime_s = 0.0;
  double mean_decision_latency_s = 0.0;
  double maximum_decision_latency_s = 0.0;
  double planning_latency_s = 0.0;
  double model_update_cost_s = 0.0;
  std::size_t allocation_count = 0U;
  std::size_t allocation_bytes = 0U;
  double coverage = 0.0;
  std::map<std::string, std::size_t> intervention_counts;
  std::map<std::string, double> intervention_frequency;
};

class ExperimentMetricsCollector {
 public:
  ExperimentMetricsCollector(std::string scenario, std::string profile,
                             std::size_t freespace_cells);

  void record(const ExperimentObservation& observation);
  void recordTargetOutcome(bool succeeded);
  ExperimentSummary summary() const;
  std::string serialize() const;

  static std::size_t coveredCells(const domain::SpatialModel& model);

 private:
  std::string scenario_;
  std::string profile_;
  std::size_t freespace_cells_ = 0U;
  std::size_t targets_attempted_ = 0U;
  std::size_t targets_succeeded_ = 0U;
  std::vector<ExperimentObservation> observations_;
};

}  // namespace semaforr::validation

#endif  // SEMAFORR_VALIDATION_EXPERIMENT_METRICS_HPP
