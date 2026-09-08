/**
 * @file catalog_registry.hpp
 * @brief Catalog registry responsibilities.
 *
 * @details This file defines catalog registry behavior for tiered decision
 * making and action arbitration. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its package-relative
 * location is `include/semaforr/decision/advisors/catalog_registry.hpp`.
 */
#ifndef SEMAFORR_DECISION_ADVISOR_CATALOG_REGISTRY_HPP
#define SEMAFORR_DECISION_ADVISOR_CATALOG_REGISTRY_HPP

#include <semaforr/config/navigation_configuration.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/domain/world_model.hpp>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Registers advisor catalog for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 * - @p ActionSpace: Supplies action space input to the operation.
 * - @p AdvisorConfiguration: Supplies advisor configuration input to the
 * operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void registerAdvisorCatalog(AdvisorRegistry&, const domain::ActionSpace&,
                            const std::vector<config::AdvisorConfiguration>&);

}  // namespace semaforr::decision

#endif
