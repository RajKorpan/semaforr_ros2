#ifndef SEMAFORR_PLANNING_MAP_PARSER_HPP
#define SEMAFORR_PLANNING_MAP_PARSER_HPP

#include <filesystem>
#include <istream>
#include <semaforr/domain/geometry.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {

struct MapRepresentation {
  std::vector<domain::Segment2D> walls;
};

MapRepresentation parseMapXml(std::istream& input,
                              const std::string& source_name);
MapRepresentation parseMapXmlFile(const std::filesystem::path& path);

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_MAP_PARSER_HPP
