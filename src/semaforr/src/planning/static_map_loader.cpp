/**
 * @file static_map_loader.cpp
 * @brief Static map loader responsibilities.
 *
 * @details This file implements static map loader behavior for path planning and
 * hierarchical plan construction. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/planning/static_map_loader.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <semaforr/planning/map_parser.hpp>
#include <semaforr/planning/static_map_loader.hpp>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

namespace semaforr::planning {
namespace {

/**
 * @brief Performs the regular file operation for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool regularFile(const std::filesystem::path& path) {
  std::error_code error;
  return std::filesystem::is_regular_file(path, error);
}

/**
 * @brief Performs the canonical file operation for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `std::filesystem::path` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::filesystem::path canonicalFile(const std::filesystem::path& path) {
  std::error_code error;
  const auto canonical = std::filesystem::weakly_canonical(path, error);
  return error ? path.lexically_normal() : canonical;
}

/**
 * @brief Performs the file checksum operation for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string fileChecksum(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot checksum map file");
  std::uint64_t hash = 1469598103934665603ULL;
  char byte = 0;
  while (input.get(byte)) {
    hash ^= static_cast<unsigned char>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

/**
 * @brief Performs the mark wall operation for this subsystem.
 *
 * Arguments:
 * - @p grid: Supplies grid input to the operation.
 * - @p wall: Supplies wall input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void markWall(domain::StaticOccupancyGrid& grid,
              const domain::Segment2D& wall) {
  const double length = wall.length().meters();
  const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
              std::ceil(length / (grid.geometry.resolution_m / 2.0))));
  constexpr int inflation_cells = 0;
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double t = static_cast<double>(sample) / static_cast<double>(samples);
    const double x = wall.start.x_m + t * (wall.end.x_m - wall.start.x_m);
    const double y = wall.start.y_m + t * (wall.end.y_m - wall.start.y_m);
    const int column = static_cast<int>(std::floor(
        (x - grid.geometry.origin.x_m) / grid.geometry.resolution_m));
    const int row = static_cast<int>(std::floor(
        (y - grid.geometry.origin.y_m) / grid.geometry.resolution_m));
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        if (dx * dx + dy * dy > inflation_cells * inflation_cells) continue;
        const int occupied_column = column + dx;
        const int occupied_row = row + dy;
        if (occupied_column < 0 || occupied_row < 0 ||
            occupied_column >= static_cast<int>(grid.geometry.columns) ||
            occupied_row >= static_cast<int>(grid.geometry.rows))
          continue;
        grid.cells[static_cast<std::size_t>(occupied_row) *
                       grid.geometry.columns +
                   static_cast<std::size_t>(occupied_column)] =
            domain::StaticOccupancyState::StaticOccupied;
      }
    }
  }
}

}  // namespace

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
    const config::StaticMapConfiguration& configuration) {
  if (resolved_path.extension() != ".xml")
    throw std::runtime_error("unsupported map format '" +
                             resolved_path.extension().string() +
                             "' for '" + resolved_path.string() +
                             "'; supported format: XML ObstacleSet");
  const bool infer_bounds = configuration.bounds_policy == "infer" ||
                            configuration.bounds_policy == "infer_expandable";
  if ((!infer_bounds &&
       (dimensions.length <= 0 || dimensions.height <= 0)) ||
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
  result.checksum = fileChecksum(resolved_path);
  result.format = "menge_obstacle_set_xml";
  if (infer_bounds) {
    if (parsed.walls.empty())
      throw std::runtime_error(
          "cannot infer map bounds from a map without obstacle geometry");
    double minimum_x = parsed.walls.front().start.x_m;
    double maximum_x = minimum_x;
    double minimum_y = parsed.walls.front().start.y_m;
    double maximum_y = minimum_y;
    for (const auto& wall : parsed.walls) {
      for (const auto point : {wall.start, wall.end}) {
        minimum_x = std::min(minimum_x, point.x_m);
        maximum_x = std::max(maximum_x, point.x_m);
        minimum_y = std::min(minimum_y, point.y_m);
        maximum_y = std::max(maximum_y, point.y_m);
      }
    }
    const double padding = configuration.inferred_bounds_padding_m;
    result.bounds = {{minimum_x - padding, minimum_y - padding},
                     {maximum_x + padding, maximum_y + padding}};
  } else {
    result.bounds = {{configuration.origin_x_m, configuration.origin_y_m},
                     {configuration.origin_x_m + dimensions.length,
                      configuration.origin_y_m + dimensions.height}};
  }
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
  grid.geometry = domain::GridGeometry::fromBounds(
      "map", result.bounds.minimum, result.bounds.maximum,
      configuration.occupancy_resolution_m, domain::GridExtentMode::Fixed,
      infer_bounds ? domain::GridExtentSource::InferredMapBounds
                   : domain::GridExtentSource::StaticMapBounds,
      domain::GridOutOfBoundsBehavior::NonTraversable, result.revision,
      result.source + "#" + result.checksum);
  grid.cells.assign(grid.geometry.cellCount(),
                    domain::StaticOccupancyState::StaticFree);
  for (const auto& wall : result.walls)
    markWall(grid, wall);
  if (!result.geometryAvailable() || !result.occupancyAvailable())
    throw std::runtime_error("map-derived geometry or occupancy is empty for '" +
                             result.source + "'");
  return result;
}

}  // namespace semaforr::planning
