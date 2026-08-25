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

struct ModelMutation {
  Revision sequence{0U};
  ModelDependency representation{ModelDependency::Trails};
  Revision representation_revision{0U};
  std::chrono::steady_clock::time_point timestamp{};
  std::string summary;
};

std::string_view toString(ModelDependency dependency) noexcept;

}  // namespace semaforr::domain

#endif
