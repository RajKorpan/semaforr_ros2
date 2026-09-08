/**
 * @file map_parser.hpp
 * @brief Map parser responsibilities.
 *
 * @details This file defines map parser behavior for path planning and
 * hierarchical plan construction. It centers on `MapRepresentation`. Its
 * package-relative location is `include/semaforr/planning/map_parser.hpp`.
 */
#ifndef SEMAFORR_PLANNING_MAP_PARSER_HPP
#define SEMAFORR_PLANNING_MAP_PARSER_HPP

#include <filesystem>
#include <istream>
#include <semaforr/domain/geometry.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {

/**
 * @brief Encapsulates map representation state and behavior for this
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
struct MapRepresentation {
  std::vector<domain::Segment2D> walls;
  std::vector<domain::Polygon> obstacle_polygons;
};

/**
 * @brief Parses map xml for this subsystem.
 *
 * Arguments:
 * - @p input: Supplies input input to the operation.
 * - @p source_name: Supplies source name input to the operation.
 *
 * Returns:
 * - `MapRepresentation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
MapRepresentation parseMapXml(std::istream& input,
                              const std::string& source_name);
/**
 * @brief Parses map xml file for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `MapRepresentation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
MapRepresentation parseMapXmlFile(const std::filesystem::path& path);

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_MAP_PARSER_HPP
