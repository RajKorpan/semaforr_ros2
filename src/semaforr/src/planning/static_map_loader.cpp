#include <algorithm>
#include <cmath>
#include <filesystem>
#include <semaforr/planning/map_parser.hpp>
#include <semaforr/planning/static_map_loader.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace semaforr::planning {
namespace {

bool regularFile(const std::filesystem::path& path) {
  std::error_code error;
  return std::filesystem::is_regular_file(path, error);
}

std::filesystem::path canonicalFile(const std::filesystem::path& path) {
  std::error_code error;
  const auto canonical = std::filesystem::weakly_canonical(path, error);
  return error ? path.lexically_normal() : canonical;
}

void markWall(domain::StaticOccupancyGrid& grid,
              const domain::Segment2D& wall) {
  const double length = wall.length().meters();
  const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / (grid.resolution_m / 2.0))));
  constexpr int inflation_cells = 0;
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double t = static_cast<double>(sample) / static_cast<double>(samples);
    const double x = wall.start.x_m + t * (wall.end.x_m - wall.start.x_m);
    const double y = wall.start.y_m + t * (wall.end.y_m - wall.start.y_m);
    const int column = static_cast<int>(std::floor((x - grid.origin.x_m) /
                                                   grid.resolution_m));
    const int row = static_cast<int>(std::floor((y - grid.origin.y_m) /
                                                grid.resolution_m));
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        if (dx * dx + dy * dy > inflation_cells * inflation_cells) continue;
        const int occupied_column = column + dx;
        const int occupied_row = row + dy;
        if (occupied_column < 0 || occupied_row < 0 ||
            occupied_column >= static_cast<int>(grid.columns) ||
            occupied_row >= static_cast<int>(grid.rows))
          continue;
        grid.cells[static_cast<std::size_t>(occupied_row) * grid.columns +
                   static_cast<std::size_t>(occupied_column)] =
            domain::StaticOccupancyState::StaticOccupied;
      }
    }
  }
}

}  // namespace

std::filesystem::path resolveMapPath(const std::string& requested,
                                     const MapSearchPaths& search_paths) {
  if (requested.empty())
    throw std::runtime_error("map access is enabled but map.path is empty");
  const std::filesystem::path input(requested);
  if (input.is_absolute()) {
    if (regularFile(input)) return canonicalFile(input);
    throw std::runtime_error("requested absolute map does not exist: '" +
                             requested + "'");
  }

  constexpr std::string_view package_prefix = "package://";
  if (requested.starts_with(package_prefix)) {
    const std::string remainder = requested.substr(package_prefix.size());
    const auto slash = remainder.find('/');
    if (slash == std::string::npos)
      throw std::runtime_error(
          "package map URI must be package://<package>/<path>: '" +
          requested + "'");
    const auto package = search_paths.package_shares.find(
        remainder.substr(0U, slash));
    if (package == search_paths.package_shares.end())
      throw std::runtime_error("map package is unavailable in this install: '" +
                               remainder.substr(0U, slash) + "'");
    const auto candidate = package->second / remainder.substr(slash + 1U);
    if (regularFile(candidate)) return canonicalFile(candidate);
    throw std::runtime_error("package-relative map does not exist: '" +
                             requested + "'");
  }

  std::vector<std::filesystem::path> candidates;
  if (!search_paths.working_directory.empty())
    candidates.push_back(search_paths.working_directory / input);
  for (const auto& [name, share] : search_paths.package_shares) {
    static_cast<void>(name);
    candidates.push_back(share / input);
  }
  if (!search_paths.example_core.empty()) {
    candidates.push_back(search_paths.example_core / input);
    if (!input.has_extension()) {
      candidates.push_back(search_paths.example_core / requested /
                           (input.filename().string() + "S.xml"));
      candidates.push_back(search_paths.example_core / (requested + ".xml"));
    }
  }
  for (const auto& candidate : candidates)
    if (regularFile(candidate)) return canonicalFile(candidate);
  throw std::runtime_error(
      "unable to resolve requested map '" + requested +
      "'; use an absolute path, package:// URI, package-relative path, or "
      "an example name installed under semaforr_examples/core");
}

domain::StaticMap loadStaticMap(
    const std::filesystem::path& resolved_path,
    const config::MapDimensions& dimensions,
    const config::StaticMapConfiguration& configuration) {
  if (resolved_path.extension() != ".xml")
    throw std::runtime_error("unsupported map format '" +
                             resolved_path.extension().string() +
                             "' for '" + resolved_path.string() +
                             "'; supported format: XML ObstacleSet");
  if (dimensions.length <= 0 || dimensions.height <= 0 ||
      !std::isfinite(configuration.origin_x_m) ||
      !std::isfinite(configuration.origin_y_m) ||
      !std::isfinite(configuration.occupancy_resolution_m) ||
      configuration.occupancy_resolution_m <= 0.0 ||
      !std::isfinite(configuration.obstacle_inflation_m) ||
      configuration.obstacle_inflation_m < 0.0)
    throw std::runtime_error(
        "static map requires positive bounds and resolution, a nonnegative "
        "inflation radius, and finite origin coordinates");

  auto parsed = parseMapXmlFile(resolved_path);
  domain::StaticMap result;
  result.source = canonicalFile(resolved_path).string();
  result.format = "menge_obstacle_set_xml";
  result.bounds = {{configuration.origin_x_m, configuration.origin_y_m},
                   {configuration.origin_x_m + dimensions.length,
                    configuration.origin_y_m + dimensions.height}};
  for (const auto& wall : parsed.walls) {
    if (!result.bounds.contains(wall.start) ||
        !result.bounds.contains(wall.end))
      throw std::runtime_error("map obstacle lies outside declared bounds in '" +
                               result.source + "'");
    if (wall.length().meters() <= domain::geometry_tolerance_m)
      throw std::runtime_error("map contains a zero-length wall in '" +
                               result.source + "'");
  }
  result.walls = std::move(parsed.walls);
  result.obstacle_polygons = std::move(parsed.obstacle_polygons);
  auto& grid = result.occupancy;
  grid.resolution_m = configuration.occupancy_resolution_m;
  grid.origin = result.bounds.minimum;
  grid.columns = static_cast<std::size_t>(
      std::ceil(static_cast<double>(dimensions.length) / grid.resolution_m));
  grid.rows = static_cast<std::size_t>(
      std::ceil(static_cast<double>(dimensions.height) / grid.resolution_m));
  grid.cells.assign(grid.columns * grid.rows,
                    domain::StaticOccupancyState::StaticFree);
  for (const auto& wall : result.walls)
    markWall(grid, wall);
  if (!result.geometryAvailable() || !result.occupancyAvailable())
    throw std::runtime_error("map-derived geometry or occupancy is empty for '" +
                             result.source + "'");
  return result;
}

}  // namespace semaforr::planning
