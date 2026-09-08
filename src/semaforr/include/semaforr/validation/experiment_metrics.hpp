/**
 * @file experiment_metrics.hpp
 * @brief Experiment metrics responsibilities.
 *
 * @details This file defines experiment metrics behavior for replay,
 * experimental validation, and performance measurement. It centers on
 * `AllocationMeasurement`, `ExperimentObservation`, `ExperimentSummary`,
 * `ExperimentMetricsCollector`. Its package-relative location is
 * `include/semaforr/validation/experiment_metrics.hpp`.
 */
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

/**
 * @brief Encapsulates allocation measurement state and behavior for this
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
struct AllocationMeasurement {
  std::size_t count = 0U;
  std::size_t bytes = 0U;
};

/**
 * @brief Encapsulates experiment observation state and behavior for this
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
struct ExperimentObservation {
  decision::DecisionResult decision;
  double runtime_s = 0.0;
  double planning_latency_s = 0.0;
  double model_update_cost_s = 0.0;
  AllocationMeasurement allocations;
  std::optional<std::size_t> covered_cells;
};

/**
 * @brief Encapsulates experiment summary state and behavior for this
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

/**
 * @brief Encapsulates experiment metrics collector state and behavior for
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
class ExperimentMetricsCollector {
 public:
  /**
   * @brief Performs the experiment metrics collector operation for this
   * subsystem.
   *
   * Arguments:
   * - @p scenario: Supplies scenario input to the operation.
   * - @p profile: Supplies profile input to the operation.
   * - @p freespace_cells: Supplies freespace cells input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ExperimentMetricsCollector(std::string scenario, std::string profile,
                             std::size_t freespace_cells);

  /**
   * @brief Records package content for this subsystem.
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
  void record(const ExperimentObservation& observation);
  /**
   * @brief Records target outcome for this subsystem.
   *
   * Arguments:
   * - @p succeeded: Supplies succeeded input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordTargetOutcome(bool succeeded);
  /**
   * @brief Performs the summary operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `ExperimentSummary` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ExperimentSummary summary() const;
  /**
   * @brief Performs the serialize operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string serialize() const;

  /**
   * @brief Performs the covered cells operation for this subsystem.
   *
   * Arguments:
   * - @p model: Supplies model input to the operation.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
