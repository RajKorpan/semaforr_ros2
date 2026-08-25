/**
 * @file static_map_loader.hpp
 * @brief Static map loader responsibilities.
 *
 * @details This file defines static map loader behavior for path planning and
 * hierarchical plan construction. It centers on `MapSearchPaths`. Its
 * package-relative location is
 * `include/semaforr/planning/static_map_loader.hpp`.
 */
#ifndef SEMAFORR_PLANNING_STATIC_MAP_LOADER_HPP
#define SEMAFORR_PLANNING_STATIC_MAP_LOADER_HPP

#include <filesystem>
#include <map>
#include <semaforr/config/navigation_configuration.hpp>
#include <semaforr/domain/static_map.hpp>
#include <string>

namespace semaforr::planning {

/**
 * @brief Encapsulates map search paths state and behavior for this
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
struct MapSearchPaths {
  std::filesystem::path working_directory;
  std::map<std::string, std::filesystem::path> package_shares;
  std::filesystem::path example_core;
};

/**
 * @brief Performs the resolve map path operation for this subsystem.
 *
 * Arguments:
 * - @p requested: Supplies requested input to the operation.
 * - @p search_paths: Supplies search paths input to the operation.
 *
 * Returns:
 * - `std::filesystem::path` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::filesystem::path resolveMapPath(const std::string& requested,
                                     const MapSearchPaths& search_paths);

/**
 * @brief Loads static map for this subsystem.
 *
 * Arguments:
 * - @p resolved_path: Supplies resolved path input to the operation.
 * - @p dimensions: Supplies dimensions input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `domain::StaticMap` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::StaticMap loadStaticMap(
    const std::filesystem::path& resolved_path,
    const config::MapDimensions& dimensions,
    const config::StaticMapConfiguration& configuration);

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_STATIC_MAP_LOADER_HPP
