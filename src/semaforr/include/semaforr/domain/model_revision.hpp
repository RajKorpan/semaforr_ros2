/**
 * @file model_revision.hpp
 * @brief Model revision responsibilities.
 *
 * @details This file defines model revision behavior for ROS-independent domain
 * state and value types. It centers on `ModelDependency`, `ModelMutation`.
 * Its package-relative location is
 * `include/semaforr/domain/model_revision.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_MODEL_REVISION_HPP
#define SEMAFORR_DOMAIN_MODEL_REVISION_HPP

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::domain {

using Revision = std::uint64_t;

/**
 * @brief Enumerates the supported model dependency values used by this
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
enum class ModelDependency {
  StaticMapGeometry,
  StaticOccupancy,
  SensedOccupancy,
  Familiarity,
  Inclusion,
  Trails,
  Conveyors,
  Regions,
  DoorsAndExits,
  Hallways,
  Barriers,
  Skeleton,
  Highways,
  HighwayGraph,
  Circumstances,
  LiveCrowdObservation,
  CrowdDensity,
  CrowdRisk,
  CrowdFlow,
  PlannerConfiguration,
  VisibilityGeometry
};

using DependencyRevisions = std::map<ModelDependency, Revision>;

/**
 * @brief Encapsulates model mutation state and behavior for this subsystem.
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
struct ModelMutation {
  Revision sequence{0U};
  ModelDependency representation{ModelDependency::Trails};
  Revision representation_revision{0U};
  std::chrono::steady_clock::time_point timestamp{};
  std::string summary;
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p dependency: Supplies dependency input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ModelDependency dependency) noexcept;

}  // namespace semaforr::domain

#endif
