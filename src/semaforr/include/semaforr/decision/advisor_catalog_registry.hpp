#ifndef SEMAFORR_DECISION_ADVISOR_CATALOG_REGISTRY_HPP
#define SEMAFORR_DECISION_ADVISOR_CATALOG_REGISTRY_HPP

#include <semaforr/config/Configuration.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/domain/world_model.hpp>
#include <vector>

namespace semaforr::decision {

void registerAdvisorCatalog(
    AdvisorRegistry&, const domain::ActionSpace&,
    const std::vector<config::AdvisorConfiguration>&);

}  // namespace semaforr::decision

#endif
