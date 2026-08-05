#include <cmath>
#include <fstream>
#include <regex>
#include <semaforr/planning/map_parser.hpp>
#include <sstream>
#include <stdexcept>

namespace semaforr::planning {
namespace {

double attribute(const std::string& element, const char* name,
                 const std::string& source) {
  const std::regex expression(std::string(R"re(\b)re") + name +
                              R"re(\s*=\s*"([^"]+)")re");
  std::smatch match;
  if (!std::regex_search(element, match, expression)) {
    throw std::runtime_error(
        source + ": Vertex is missing required attribute '" + name + "'");
  }
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(match[1].str(), &consumed);
  } catch (const std::exception&) {
    throw std::runtime_error(source + ": Vertex attribute '" + name +
                             "' is not numeric");
  }
  if (consumed != match[1].str().size() || !std::isfinite(value)) {
    throw std::runtime_error(source + ": Vertex attribute '" + name +
                             "' must be finite");
  }
  return value;
}

}  // namespace

MapRepresentation parseMapXml(std::istream& input,
                              const std::string& source_name) {
  const std::string xml{std::istreambuf_iterator<char>(input),
                        std::istreambuf_iterator<char>()};
  if (xml.find("<ObstacleSet") == std::string::npos) {
    throw std::runtime_error(source_name + ": missing ObstacleSet element");
  }
  if (xml.find("</ObstacleSet>") == std::string::npos &&
      !std::regex_search(xml, std::regex(R"re(<ObstacleSet\b[^>]*/>)re")))
    throw std::runtime_error(source_name +
                             ": malformed or unclosed ObstacleSet element");

  const std::regex obstacle_expression(
      R"re(<Obstacle\b([^>]*)>([\s\S]*?)</Obstacle>)re");
  const std::regex vertex_expression(R"re(<Vertex\b[^>]*/?>)re");
  MapRepresentation result;
  for (auto obstacle =
           std::sregex_iterator(xml.begin(), xml.end(), obstacle_expression);
       obstacle != std::sregex_iterator(); ++obstacle) {
    const std::string attributes = (*obstacle)[1].str();
    const std::string body = (*obstacle)[2].str();
    std::vector<domain::Point2D> vertices;
    for (auto vertex =
             std::sregex_iterator(body.begin(), body.end(), vertex_expression);
         vertex != std::sregex_iterator(); ++vertex) {
      const std::string element = vertex->str();
      vertices.push_back({attribute(element, "p_x", source_name),
                          attribute(element, "p_y", source_name)});
    }
    if (vertices.size() < 2U) {
      throw std::runtime_error(
          source_name +
          ": each Obstacle requires at least two Vertex elements");
    }
    for (std::size_t index = 1U; index < vertices.size(); ++index) {
      result.walls.push_back({vertices[index - 1U], vertices[index]});
    }
    const bool closed =
        std::regex_search(attributes, std::regex(R"re(\bclosed\s*=\s*"1")re"));
    if (closed && vertices.size() > 2U && vertices.front() != vertices.back()) {
      result.walls.push_back({vertices.back(), vertices.front()});
    }
    if (closed && vertices.size() >= 3U) {
      if (vertices.front() == vertices.back()) vertices.pop_back();
      if (vertices.size() >= 3U)
        result.obstacle_polygons.emplace_back(std::move(vertices));
    }
  }
  if (result.walls.empty()) {
    throw std::runtime_error(source_name + ": map contains no obstacle walls");
  }
  return result;
}

MapRepresentation parseMapXmlFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open map XML '" + path.string() + "'");
  }
  return parseMapXml(input, path.string());
}

}  // namespace semaforr::planning
