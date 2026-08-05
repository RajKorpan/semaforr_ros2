#ifndef SEMAFORR_PLANNING_STATIC_MAP_LOADER_HPP
#define SEMAFORR_PLANNING_STATIC_MAP_LOADER_HPP

#include <filesystem>
#include <map>
#include <semaforr/config/navigation_configuration.hpp>
#include <semaforr/domain/static_map.hpp>
#include <string>

namespace semaforr::planning {

struct MapSearchPaths {
  std::filesystem::path working_directory;
  std::map<std::string, std::filesystem::path> package_shares;
  std::filesystem::path example_core;
};

std::filesystem::path resolveMapPath(const std::string& requested,
                                     const MapSearchPaths& search_paths);

domain::StaticMap loadStaticMap(
    const std::filesystem::path& resolved_path,
    const config::MapDimensions& dimensions,
    const config::StaticMapConfiguration& configuration);

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_STATIC_MAP_LOADER_HPP
