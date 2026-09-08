/**
 * @file learner.cpp
 * @brief Learner responsibilities.
 *
 * @details This file implements learner behavior for learned spatial
 * representations and their lifecycle. It centers on `PayloadShape`. Its
 * package-relative location is `src/spatial/learner.cpp`.
 */
#include <algorithm>
#include <iomanip>
#include <semaforr/spatial/learner_base.hpp>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace semaforr::spatial {
namespace {

/**
 * @brief Performs the changed sparse indices operation for this subsystem.
 *
 * Arguments:
 * - @p before: Supplies before input to the operation.
 * - @p after: Supplies after input to the operation.
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `std::vector<std::size_t>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
template <typename Cell, typename Value>
std::vector<std::size_t> changedSparseIndices(const std::vector<Cell>& before,
                                              const std::vector<Cell>& after,
                                              Value value) {
  std::vector<std::size_t> changed;
  std::size_t left = 0U, right = 0U;
  while (left < before.size() || right < after.size()) {
    if (right >= after.size() ||
        (left < before.size() && before[left].index < after[right].index)) {
      changed.push_back(before[left++].index);
    } else if (left >= before.size() ||
               after[right].index < before[left].index) {
      changed.push_back(after[right++].index);
    } else {
      if (value(before[left]) != value(after[right]))
        changed.push_back(before[left].index);
      ++left;
      ++right;
    }
  }
  return changed;
}

/**
 * @brief Performs the ranges operation for this subsystem.
 *
 * Arguments:
 * - @p indices: Supplies indices input to the operation.
 *
 * Returns:
 * - `std::vector<ChangedCellRange>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<ChangedCellRange> ranges(std::vector<std::size_t> indices) {
  if (indices.empty()) return {};
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  std::vector<ChangedCellRange> result;
  ChangedCellRange current{indices.front(), indices.front()};
  for (std::size_t position = 1U; position < indices.size(); ++position) {
    if (indices[position] == current.last + 1U) {
      current.last = indices[position];
    } else {
      result.push_back(current);
      current = {indices[position], indices[position]};
    }
  }
  result.push_back(current);
  return result;
}

/**
 * @brief Encapsulates payload shape state and behavior for this subsystem.
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
struct PayloadShape {
  std::size_t entities = 0U;
  std::size_t graph_nodes = 0U;
  std::size_t graph_edges = 0U;
  bool grid = false;
};

/**
 * @brief Performs the shape operation for this subsystem.
 *
 * Arguments:
 * - @p payload: Supplies payload input to the operation.
 *
 * Returns:
 * - `PayloadShape` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PayloadShape shape(const SpatialPayload& payload) {
  return std::visit(
      [](const auto& model) {
        using Model = std::decay_t<decltype(model)>;
        PayloadShape result;
        if constexpr (std::is_same_v<Model, TrailModel>)
          result.entities =
              std::max(model.trails.size(), model.learned_trails.size());
        else if constexpr (std::is_same_v<Model, ConveyorModel>)
          result.entities = model.flows.size();
        else if constexpr (std::is_same_v<Model, RegionModel>)
          result.entities =
              std::max(model.regions.size(), model.learned_regions.size());
        else if constexpr (std::is_same_v<Model, DoorExitModel>)
          result.entities = model.exits.size() + model.doors.size() +
                            model.sensor_openings.size();
        else if constexpr (std::is_same_v<Model, HallwayModel>)
          result.entities =
              std::max(model.centerlines.size(), model.hallways.size());
        else if constexpr (std::is_same_v<Model, BarrierModel>)
          result.entities = model.barriers.size();
        else if constexpr (std::is_same_v<Model, PassageSkeletonModel>) {
          result.graph_nodes =
              std::max(model.nodes.size(), model.region_nodes.size());
          result.graph_edges =
              std::max(model.edges.size(), model.region_edges.size());
        } else if constexpr (std::is_same_v<Model, HighwayModel>) {
          result.entities = model.highways.size();
          result.graph_nodes = model.graph.vertices.size();
          result.graph_edges = model.graph.edges.size();
        } else if constexpr (std::is_same_v<Model, CircumstanceModel>)
          result.entities = model.cases.size() + model.clusters.size();
        else if constexpr (std::is_same_v<Model, KnownGridModel> ||
                           std::is_same_v<Model, SensedOccupancyModel> ||
                           std::is_same_v<Model, InclusionGridModel>)
          result.grid = true;
        return result;
      },
      payload);
}

/**
 * @brief Performs the change set operation for this subsystem.
 *
 * Arguments:
 * - @p before: Supplies before input to the operation.
 * - @p after: Supplies after input to the operation.
 * - @p revision: Supplies revision input to the operation.
 *
 * Returns:
 * - `RepresentationChangeSet` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
RepresentationChangeSet changeSet(const SpatialPayload* before,
                                  const SpatialPayload& after,
                                  std::size_t revision) {
  RepresentationChangeSet result;
  result.revision = revision;
  std::vector<std::size_t> changed_indices;
  static const std::vector<SparseGridCell> empty_counts;
  static const std::vector<domain::SparseSensedOccupancyCell> empty_occupancy;
  if (const auto* current = std::get_if<KnownGridModel>(&after)) {
    const auto* previous =
        before ? std::get_if<KnownGridModel>(before) : nullptr;
    changed_indices = changedSparseIndices(
        previous ? previous->sparse_observations : empty_counts,
        current->sparse_observations,
        [](const auto& cell) { return cell.value; });
  } else if (const auto* current = std::get_if<SensedOccupancyModel>(&after)) {
    const auto* previous =
        before ? std::get_if<SensedOccupancyModel>(before) : nullptr;
    changed_indices = changedSparseIndices(
        previous ? previous->sparse_cells : empty_occupancy,
        current->sparse_cells, [](const auto& cell) { return cell.value; });
  } else if (const auto* current = std::get_if<InclusionGridModel>(&after)) {
    const auto* previous =
        before ? std::get_if<InclusionGridModel>(before) : nullptr;
    changed_indices = changedSparseIndices(
        previous ? previous->sparse_included : empty_counts,
        current->sparse_included, [](const auto& cell) { return cell.value; });
  }
  result.changed_cell_ranges = ranges(std::move(changed_indices));
  const auto old_shape = before ? shape(*before) : PayloadShape{};
  const auto new_shape = shape(after);
  if (!new_shape.grid) {
    result.added_entities = new_shape.entities > old_shape.entities
                                ? new_shape.entities - old_shape.entities
                                : 0U;
    result.removed_entities = old_shape.entities > new_shape.entities
                                  ? old_shape.entities - new_shape.entities
                                  : 0U;
    result.updated_entities = std::min(old_shape.entities, new_shape.entities);
    result.added_graph_nodes =
        new_shape.graph_nodes > old_shape.graph_nodes
            ? new_shape.graph_nodes - old_shape.graph_nodes
            : 0U;
    result.removed_graph_nodes =
        old_shape.graph_nodes > new_shape.graph_nodes
            ? old_shape.graph_nodes - new_shape.graph_nodes
            : 0U;
    result.updated_graph_nodes =
        std::min(old_shape.graph_nodes, new_shape.graph_nodes);
    result.added_graph_edges =
        new_shape.graph_edges > old_shape.graph_edges
            ? new_shape.graph_edges - old_shape.graph_edges
            : 0U;
    result.removed_graph_edges =
        old_shape.graph_edges > new_shape.graph_edges
            ? old_shape.graph_edges - new_shape.graph_edges
            : 0U;
    result.updated_graph_edges =
        std::min(old_shape.graph_edges, new_shape.graph_edges);
  }
  return result;
}

/**
 * @brief Performs the quote operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string quote(std::string_view value) {
  std::string result{"\""};
  for (const char character : value) {
    switch (character) {
      case '\\':
        result += "\\\\";
        break;
      case '"':
        result += "\\\"";
        break;
      case '\n':
        result += "\\n";
        break;
      case '\r':
        result += "\\r";
        break;
      case '\t':
        result += "\\t";
        break;
      default:
        result += character;
        break;
    }
  }
  result += '"';
  return result;
}

/**
 * @brief Performs the point operation for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void point(std::ostream& output, const domain::Point2D& value) {
  output << "{\"x_m\":" << value.x_m << ",\"y_m\":" << value.y_m << '}';
}

/**
 * @brief Performs the segment operation for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void segment(std::ostream& output, const domain::Segment2D& value) {
  output << "{\"start\":";
  point(output, value.start);
  output << ",\"end\":";
  point(output, value.end);
  output << '}';
}

/**
 * @brief Performs the grid geometry operation for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 * - @p geometry: Supplies geometry input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void gridGeometry(std::ostream& output, const domain::GridGeometry& geometry) {
  output << "{\"schema_version\":1,\"frame_id\":" << quote(geometry.frame_id)
         << ",\"minimum\":";
  point(output, geometry.minimum);
  output << ",\"maximum\":";
  point(output, geometry.maximum);
  output << ",\"origin\":";
  point(output, geometry.origin);
  output << ",\"resolution_m\":" << geometry.resolution_m
         << ",\"columns\":" << geometry.columns << ",\"rows\":" << geometry.rows
         << ",\"boundary_convention\":\"half_open\""
         << ",\"out_of_bounds\":" << static_cast<int>(geometry.out_of_bounds)
         << ",\"expandable\":"
         << (geometry.extent_mode == domain::GridExtentMode::Expandable
                 ? "true"
                 : "false")
         << ",\"geometry_revision\":" << geometry.geometry_revision
         << ",\"extent_source\":"
         << quote(domain::toString(geometry.extent_source))
         << ",\"map_identifier\":" << quote(geometry.map_identifier) << '}';
}

/**
 * @brief Performs the array operation for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 * - @p values: Supplies values input to the operation.
 * - @p writer: Supplies writer input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
template <typename Range, typename Writer>
void array(std::ostream& output, const Range& values, Writer writer) {
  output << '[';
  bool first = true;
  for (const auto& value : values) {
    if (!first) {
      output << ',';
    }
    first = false;
    writer(output, value);
  }
  output << ']';
}

/**
 * @brief Performs the payload operation for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void payload(std::ostream& output, const SpatialPayload& value) {
  std::visit(
      [&output](const auto& model) {
        using Model = std::decay_t<decltype(model)>;
        if constexpr (std::is_same_v<Model, std::monostate>) {
          output << "null";
        } else if constexpr (std::is_same_v<Model, TrailModel>) {
          output << "{\"trails\":";
          array(output, model.trails,
                [](std::ostream& stream, const auto& trail) {
                  array(stream, trail, point);
                });
          output << ",\"learned_trails\":";
          array(output, model.learned_trails,
                [](std::ostream& stream, const auto& trail) {
                  stream << "{\"id\":" << trail.id
                         << ",\"source_path\":" << trail.source_path
                         << ",\"length_m\":" << trail.length_m
                         << ",\"markers\":";
                  array(stream, trail.markers,
                        [](std::ostream& marker_stream, const auto& marker) {
                          marker_stream << "{\"path_point_index\":"
                                        << marker.path_point_index
                                        << ",\"position\":";
                          point(marker_stream, marker.pose.position);
                          marker_stream
                              << ",\"visibility_distance_m\":"
                              << (marker.visibility_to_next
                                      ? marker.visibility_to_next
                                            ->visible_distance.meters()
                                      : -1.0)
                              << '}';
                        });
                  stream << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, ConveyorModel>) {
          output << "{\"flows\":";
          array(output, model.flows,
                [](std::ostream& stream, const auto& flow) {
                  stream << "{\"axis\":";
                  segment(stream, flow.axis);
                  stream << ",\"traversals\":" << flow.traversals << '}';
                });
          output << ",\"grid\":";
          if (model.grid.geometry.valid()) {
            output << "{\"geometry\":";
            gridGeometry(output, model.grid.geometry);
            output << ",\"decay_factor\":" << model.grid.decay_factor
                   << ",\"maximum_frequency\":" << model.grid.maximum_frequency
                   << ",\"cells\":";
            array(output, model.grid.cells,
                  [](std::ostream& stream, const auto& cell) {
                    stream << "{\"index\":" << cell.index
                           << ",\"frequency\":" << cell.traversal_frequency
                           << ",\"direction_x\":" << cell.direction_x
                           << ",\"direction_y\":" << cell.direction_y
                           << ",\"strength\":" << cell.normalized_strength
                           << '}';
                  });
            output << '}';
          } else {
            output << "null";
          }
          output << '}';
        } else if constexpr (std::is_same_v<Model, RegionModel>) {
          output << "{\"regions\":";
          array(output, model.regions,
                [](std::ostream& stream, const auto& region) {
                  stream << "{\"center\":";
                  point(stream, region.center);
                  stream << ",\"radius_m\":" << region.radius.meters() << '}';
                });
          output << ",\"learned_regions\":";
          array(output, model.learned_regions,
                [](std::ostream& stream, const auto& region) {
                  stream << "{\"id\":" << region.id << ",\"center\":";
                  point(stream, region.boundary.center);
                  stream << ",\"radius_m\":" << region.boundary.radius.meters()
                         << ",\"supporting_decision\":"
                         << region.supporting_decision
                         << ",\"visibility_revision\":"
                         << region.visibility_revision << ",\"visibility\":";
                  array(stream, region.visibility,
                        [](std::ostream& bin_stream, const auto& bin) {
                          bin_stream
                              << (bin.known ? bin.maximum_distance_m : -1.0);
                        });
                  stream << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, DoorExitModel>) {
          output << "{\"openings\":";
          array(output, model.openings, segment);
          output << ",\"exits\":";
          array(output, model.exits,
                [](std::ostream& stream, const auto& exit) {
                  stream << "{\"id\":" << exit.id
                         << ",\"region\":" << exit.region << ",\"point\":";
                  point(stream, exit.point);
                  stream << ",\"traversals\":" << exit.traversal_count
                         << ",\"confidence\":" << exit.confidence << '}';
                });
          output << ",\"doors\":";
          array(
              output, model.doors, [](std::ostream& stream, const auto& door) {
                stream << "{\"id\":" << door.id << ",\"region\":" << door.region
                       << ",\"start_rad\":" << door.clockwise_start_rad
                       << ",\"end_rad\":" << door.clockwise_end_rad
                       << ",\"confidence\":" << door.confidence
                       << ",\"traversals\":" << door.supporting_traversals
                       << '}';
              });
          output << ",\"sensor_openings\":";
          array(output, model.sensor_openings,
                [](std::ostream& stream, const auto& opening) {
                  stream << "{\"segment\":";
                  segment(stream, opening.opening);
                  stream << ",\"decision_id\":" << opening.decision_id
                         << ",\"confidence\":" << opening.confidence << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, HallwayModel>) {
          output << "{\"centerlines\":";
          array(output, model.centerlines, segment);
          output << ",\"hallways\":";
          array(output, model.hallways,
                [](std::ostream& stream, const auto& hallway) {
                  stream << "{\"id\":" << hallway.id << ",\"direction\":"
                         << static_cast<int>(hallway.direction)
                         << ",\"centerline\":";
                  segment(stream, hallway.centerline);
                  stream << ",\"width_m\":" << hallway.width_m
                         << ",\"extent_m\":" << hallway.extent_m
                         << ",\"supporting_segments\":"
                         << hallway.supporting_segments << ",\"cells\":";
                  array(stream, hallway.connected_area,
                        [](std::ostream& cell_stream, const auto& cell) {
                          cell_stream << "{\"x\":" << cell.x
                                      << ",\"y\":" << cell.y
                                      << ",\"heat\":" << cell.heat << '}';
                        });
                  stream << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, BarrierModel>) {
          output << "{\"barriers\":";
          array(output, model.barriers, segment);
          output << '}';
        } else if constexpr (std::is_same_v<Model, PassageSkeletonModel>) {
          output << "{\"nodes\":";
          array(output, model.nodes, point);
          output << ",\"edges\":";
          array(output, model.edges,
                [](std::ostream& stream, const auto& edge) {
                  stream << "{\"from\":" << edge.from << ",\"to\":" << edge.to
                         << '}';
                });
          output << ",\"component_by_node\":";
          array(output, model.component_by_node,
                [](std::ostream& stream, auto component) {
                  stream << component;
                });
          output << ",\"connectivity_revision\":"
                 << model.connectivity_revision;
          output << ",\"region_nodes\":";
          array(output, model.region_nodes,
                [](std::ostream& stream, const auto& node) {
                  stream << "{\"id\":" << node.id
                         << ",\"region\":" << node.region << ",\"center\":";
                  point(stream, node.center);
                  stream << '}';
                });
          output << ",\"region_edges\":";
          array(output, model.region_edges,
                [](std::ostream& stream, const auto& edge) {
                  stream << "{\"from\":" << edge.from << ",\"to\":" << edge.to
                         << ",\"length_m\":" << edge.length_m
                         << ",\"source_path\":" << edge.source_path
                         << ",\"subtrail\":";
                  array(stream, edge.supporting_subtrail, point);
                  stream << '}';
                });
          output << ",\"sampled_path_nodes\":";
          array(output, model.sampled_path_nodes, point);
          output << '}';
        } else if constexpr (std::is_same_v<Model, KnownGridModel>) {
          output << "{\"geometry\":";
          gridGeometry(output, model.geometry);
          output << ",\"observations\":";
          array(output, model.observations,
                [](std::ostream& stream, auto cell) { stream << cell; });
          output << ",\"sparse_observations\":";
          array(output, model.sparse_observations,
                [](std::ostream& stream, const auto& cell) {
                  stream << "{\"index\":" << cell.index
                         << ",\"value\":" << cell.value << '}';
                });
          output << ",\"sparse_metadata\":";
          array(output, model.sparse_metadata,
                [](std::ostream& stream, const auto& cell) {
                  stream << "{\"index\":" << cell.index
                         << ",\"last_observed_sequence\":"
                         << cell.last_observed_sequence
                         << ",\"confidence\":" << cell.confidence << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, SensedOccupancyModel>) {
          output << "{\"geometry\":";
          gridGeometry(output, model.geometry);
          output << ",\"cells\":";
          array(output, model.cells,
                [](std::ostream& stream, const auto& cell) {
                  stream << "{\"state\":" << static_cast<int>(cell.state)
                         << ",\"free_evidence\":" << cell.free_evidence
                         << ",\"occupied_evidence\":" << cell.occupied_evidence
                         << ",\"confidence\":" << cell.confidence
                         << ",\"last_update_sequence\":"
                         << cell.last_update_sequence << ",\"conflicting\":"
                         << (cell.conflicting ? "true" : "false")
                         << ",\"dynamic\":" << (cell.dynamic ? "true" : "false")
                         << ",\"provenance\":" << static_cast<int>(cell.source)
                         << '}';
                });
          output << ",\"sparse_cells\":";
          array(output, model.sparse_cells,
                [](std::ostream& stream, const auto& sparse) {
                  const auto& cell = sparse.value;
                  stream << "{\"index\":" << sparse.index
                         << ",\"state\":" << static_cast<int>(cell.state)
                         << ",\"free_evidence\":" << cell.free_evidence
                         << ",\"occupied_evidence\":" << cell.occupied_evidence
                         << ",\"confidence\":" << cell.confidence
                         << ",\"last_update_sequence\":"
                         << cell.last_update_sequence << ",\"conflicting\":"
                         << (cell.conflicting ? "true" : "false")
                         << ",\"dynamic\":" << (cell.dynamic ? "true" : "false")
                         << ",\"provenance\":" << static_cast<int>(cell.source)
                         << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, InclusionGridModel>) {
          output << "{\"geometry\":";
          gridGeometry(output, model.geometry);
          output << ",\"included\":";
          array(output, model.included,
                [](std::ostream& stream, auto cell) { stream << cell; });
          output << ",\"sparse_included\":";
          array(output, model.sparse_included,
                [](std::ostream& stream, const auto& cell) {
                  stream << "{\"index\":" << cell.index
                         << ",\"value\":" << cell.value << '}';
                });
          output << '}';
        } else if constexpr (std::is_same_v<Model, HighwayModel>) {
          output << "{\"schema_version\":" << model.serialized_schema_version
                 << ",\"geometry\":";
          if (model.geometry.valid())
            gridGeometry(output, model.geometry);
          else
            output << "null";
          output << ",\"smoothing_policy\":\"" << model.smoothing_policy
                 << "\",\"component_selection_policy\":\""
                 << model.component_selection_policy << "\",\"highways\":";
          array(output, model.highways,
                [](std::ostream& stream, const auto& highway) {
                  stream << "{\"id\":" << highway.id << ",\"axis\":\""
                         << (highway.axis == domain::Axis::Horizontal
                                 ? "horizontal"
                                 : "vertical")
                         << "\",\"cells\":";
                  array(stream, highway.cells,
                        [](std::ostream& cell_stream, const auto& cell) {
                          cell_stream << "{\"row\":" << cell.row
                                      << ",\"column\":" << cell.column << '}';
                        });
                  stream << ",\"endpoints\":";
                  array(stream, highway.endpoints,
                        [](std::ostream& endpoint_stream, auto endpoint) {
                          endpoint_stream << endpoint;
                        });
                  stream << '}';
                });
          output << ",\"graph\":{\"intersections\":";
          array(output, model.graph.vertices,
                [](std::ostream& stream, const auto& intersection) {
                  stream << "{\"id\":" << intersection.id
                         << ",\"cell\":{\"row\":" << intersection.cell.row
                         << ",\"column\":" << intersection.cell.column
                         << "},\"position\":";
                  point(stream, intersection.position);
                  stream << ",\"terminal_access\":"
                         << (intersection.terminal_access ? "true" : "false")
                         << '}';
                });
          output << ",\"edges\":";
          array(output, model.graph.edges,
                [](std::ostream& stream, const auto& edge) {
                  stream << "{\"from\":" << edge.from << ",\"to\":" << edge.to
                         << ",\"highway\":" << edge.highway
                         << ",\"length_m\":" << edge.length_m
                         << ",\"trail_labels\":";
                  array(stream, edge.trail_labels,
                        [](std::ostream& label_stream, auto label) {
                          label_stream << label;
                        });
                  stream << ",\"operational_subtrail\":";
                  array(stream, edge.operational_subtrail, point);
                  stream << '}';
                });
          output << "},\"nodes\":";
          array(output, model.nodes, point);
          output << ",\"edges\":";
          array(output, model.edges,
                [](std::ostream& stream, const auto& edge) {
                  stream << "{\"from\":" << edge.from << ",\"to\":" << edge.to
                         << '}';
                });
          output << ",\"intersections\":";
          array(output, model.intersections,
                [](std::ostream& stream, const auto& intersection) {
                  stream << "{\"node\":" << intersection.node
                         << ",\"degree\":" << intersection.degree << '}';
                });
          output << ",\"grid_labels\":";
          array(output, model.grid_labels,
                [](std::ostream& stream, const auto& label) {
                  stream << "{\"row\":" << label.row
                         << ",\"column\":" << label.column
                         << ",\"label\":" << label.label << '}';
                });
          output << ",\"touched_rows\":";
          array(output, model.touched_rows,
                [](std::ostream& stream, auto value) { stream << value; });
          output << ",\"touched_columns\":";
          array(output, model.touched_columns,
                [](std::ostream& stream, auto value) { stream << value; });
          output << '}';
        } else if constexpr (std::is_same_v<Model, CircumstanceModel>) {
          const auto action = [](std::ostream& stream,
                                 const domain::Action& value) {
            stream << "{\"type\":" << static_cast<int>(value.type())
                   << ",\"magnitude\":" << value.magnitude_index() << '}';
          };
          output << "{\"schema_version\":2,\"learning_mode\":\""
                 << domain::toString(model.learning_mode)
                 << "\",\"model_version\":\"" << model.model_version
                 << "\",\"classifier_version\":\"" << model.classifier_version
                 << "\",\"feature_version\":\"" << model.feature_version
                 << "\",\"similarity_metric\":\"" << model.similarity_metric
                 << "\",\"reclustering_policy\":\"" << model.reclustering_policy
                 << "\",\"minimum_cluster_size\":" << model.minimum_cluster_size
                 << ",\"minimum_case_evidence\":" << model.minimum_case_evidence
                 << ",\"assignment_confidence_threshold\":"
                 << model.assignment_confidence_threshold
                 << ",\"similarity_l1_threshold\":"
                 << model.similarity_l1_threshold
                 << ",\"accuracy_threshold\":" << model.accuracy_threshold
                 << ",\"action_confidence_threshold\":"
                 << model.action_confidence_threshold
                 << ",\"unclustered_settings\":" << model.unclustered_settings
                 << ",\"clusters\":";
          array(
              output, model.clusters,
              [](std::ostream& stream, const auto& cluster) {
                stream << "{\"id\":" << cluster.id
                       << ",\"evidence\":" << cluster.evidence
                       << ",\"assignment_confidence\":"
                       << cluster.assignment_confidence
                       << ",\"creation_method\":\""
                       << domain::toString(cluster.creation_method)
                       << "\",\"model_version\":" << cluster.model_version
                       << ",\"last_update_sequence\":"
                       << cluster.last_update_sequence
                       << ",\"revision\":" << cluster.revision
                       << ",\"retired\":"
                       << (cluster.retired ? "true" : "false")
                       << ",\"side_cells\":" << cluster.centroid.side_cells
                       << ",\"resolution_m\":" << cluster.centroid.resolution_m
                       << ",\"radius_m\":" << cluster.centroid.radius_m
                       << ",\"freespace\":";
                array(
                    stream, cluster.centroid.freespace,
                    [](std::ostream& values, double cell) { values << cell; });
                stream << '}';
              });
          output << ",\"cases\":";
          array(
              output, model.cases, [&](std::ostream& stream, const auto& item) {
                stream << "{\"circumstance_id\":" << item.key.circumstance_id
                       << ",\"distance_bin\":" << item.key.distance_bin
                       << ",\"angle_bin\":" << item.key.angle_bin
                       << ",\"evidence\":" << item.evidence
                       << ",\"accuracy\":" << item.accuracy
                       << ",\"revision\":" << item.revision
                       << ",\"action_pairs\":";
                array(stream, item.action_pairs,
                      [&](std::ostream& pairs, const auto& pair) {
                        pairs << "{\"actual\":";
                        action(pairs, pair.actual);
                        pairs << ",\"hypothetical\":";
                        action(pairs, pair.hypothetical);
                        pairs << ",\"occurrences\":" << pair.occurrences << '}';
                      });
                stream << ",\"actions\":";
                array(stream, item.actions,
                      [&](std::ostream& actions, const auto& evidence) {
                        actions << "{\"action\":";
                        action(actions, evidence.action);
                        actions
                            << ",\"selected\":" << evidence.selected
                            << ",\"executed\":" << evidence.executed
                            << ",\"successful\":" << evidence.successful
                            << ",\"failed\":" << evidence.failed
                            << ",\"partial\":" << evidence.partial
                            << ",\"cancellations\":" << evidence.cancellations
                            << ",\"timeouts\":" << evidence.timeouts
                            << ",\"safety_interruptions\":"
                            << evidence.safety_interruptions
                            << ",\"preemptions\":" << evidence.preemptions
                            << ",\"unknown\":" << evidence.unknown
                            << ",\"effective_evidence\":"
                            << evidence.effective_evidence
                            << ",\"success_credit\":" << evidence.success_credit
                            << ",\"confidence\":" << evidence.confidence
                            << ",\"accuracy\":" << evidence.accuracy
                            << ",\"last_outcome\":\""
                            << domain::toString(evidence.last_outcome)
                            << "\",\"last_update_sequence\":"
                            << evidence.last_update_sequence << '}';
                      });
                stream << '}';
              });
          output << ",\"migrations\":";
          array(output, model.migrations,
                [](std::ostream& stream, const auto& migration) {
                  stream << "{\"previous_id\":" << migration.previous_id
                         << ",\"new_id\":" << migration.new_id
                         << ",\"operation\":\"" << migration.operation
                         << "\",\"evidence_moved\":" << migration.evidence_moved
                         << ",\"model_revision\":" << migration.model_revision
                         << '}';
                });
          output << ",\"metrics\":{\"observations\":"
                 << model.metrics.observations
                 << ",\"assignments\":" << model.metrics.assignments
                 << ",\"unmatched\":" << model.metrics.unmatched
                 << ",\"assignment_rate\":" << model.metrics.assignmentRate()
                 << ",\"average_assignment_confidence\":"
                 << model.metrics.averageAssignmentConfidence()
                 << ",\"reclusterings\":" << model.metrics.reclusterings
                 << "}}";
        }
      },
      value);
}

}  // namespace

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SpatialRepresentation representation) noexcept {
  switch (representation) {
    case SpatialRepresentation::Trails:
      return "trails";
    case SpatialRepresentation::Conveyors:
      return "conveyors";
    case SpatialRepresentation::Regions:
      return "regions";
    case SpatialRepresentation::DoorsAndExits:
      return "doors_and_exits";
    case SpatialRepresentation::Hallways:
      return "hallways";
    case SpatialRepresentation::Barriers:
      return "barriers";
    case SpatialRepresentation::PassagesAndSkeleton:
      return "passages_and_skeleton";
    case SpatialRepresentation::KnownGrid:
      return "known_grid";
    case SpatialRepresentation::SensedOccupancy:
      return "sensed_occupancy";
    case SpatialRepresentation::InclusionGrid:
      return "inclusion_grid";
    case SpatialRepresentation::Highways:
      return "highways";
    case SpatialRepresentation::Circumstances:
      return "circumstances";
  }
  return "unknown";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(UpdateMode mode) noexcept {
  return mode == UpdateMode::Incremental ? "incremental" : "rebuild_on_demand";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p schedule: Supplies schedule input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(UpdateSchedule schedule) noexcept {
  switch (schedule) {
    case UpdateSchedule::EveryObservation:
      return "every_observation";
    case UpdateSchedule::EveryDecisionCycle:
      return "every_decision_cycle";
    case UpdateSchedule::AfterActionStart:
      return "after_action_start";
    case UpdateSchedule::AfterSuccessfulActionCompletion:
      return "after_successful_action_completion";
    case UpdateSchedule::AfterAnyTerminalActionResult:
      return "after_any_terminal_action_result";
    case UpdateSchedule::EndOfTarget:
      return "end_of_target";
    case UpdateSchedule::EndOfTask:
      return "end_of_task";
    case UpdateSchedule::EndOfInitialExploration:
      return "end_of_initial_exploration";
    case UpdateSchedule::DuringHLEOnly:
      return "during_hle_only";
    case UpdateSchedule::DuringLLEOnly:
      return "during_lle_only";
    case UpdateSchedule::Periodic:
      return "periodic";
    case UpdateSchedule::OnShutdown:
      return "on_shutdown";
    case UpdateSchedule::OnDemand:
      return "on_demand";
  }
  return "on_demand";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ModelStatus status) noexcept {
  switch (status) {
    case ModelStatus::Empty:
      return "empty";
    case ModelStatus::Incomplete:
      return "incomplete";
    case ModelStatus::Fresh:
      return "fresh";
    case ModelStatus::Stale:
      return "stale";
  }
  return "unknown";
}

/**
 * @brief Performs the serialize operation for this subsystem.
 *
 * Arguments:
 * - @p update: Supplies update input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string serialize(const SpatialModelUpdate& update) {
  std::ostringstream output;
  output << std::setprecision(17)
         << "{\"representation\":" << quote(toString(update.representation))
         << ",\"learner\":" << quote(update.learner)
         << ",\"revision\":" << update.revision
         << ",\"observed_episodes\":" << update.observed_episodes
         << ",\"last_observation_sequence\":";
  if (update.last_observation_sequence) {
    output << *update.last_observation_sequence;
  } else {
    output << "null";
  }
  output << ",\"update_mode\":" << quote(toString(update.update_mode))
         << ",\"update_schedule\":" << quote(toString(update.update_schedule))
         << ",\"status\":" << quote(toString(update.status))
         << ",\"consumers\":";
  array(output, update.consumers,
        [](std::ostream& stream, const auto& consumer) {
          stream << quote(consumer);
        });
  output << ",\"diagnostic\":" << quote(update.diagnostic)
         << ",\"changes\":{\"revision\":" << update.changes.revision
         << ",\"changed_cell_ranges\":";
  array(output, update.changes.changed_cell_ranges,
        [](std::ostream& stream, const auto& range) {
          stream << "{\"first\":" << range.first << ",\"last\":" << range.last
                 << '}';
        });
  output << ",\"added_entities\":" << update.changes.added_entities
         << ",\"removed_entities\":" << update.changes.removed_entities
         << ",\"updated_entities\":" << update.changes.updated_entities
         << ",\"added_graph_nodes\":" << update.changes.added_graph_nodes
         << ",\"removed_graph_nodes\":" << update.changes.removed_graph_nodes
         << ",\"updated_graph_nodes\":" << update.changes.updated_graph_nodes
         << ",\"added_graph_edges\":" << update.changes.added_graph_edges
         << ",\"removed_graph_edges\":" << update.changes.removed_graph_edges
         << ",\"updated_graph_edges\":" << update.changes.updated_graph_edges
         << "},\"payload\":";
  payload(output, update.payload);
  output << '}';
  return output.str();
}

/**
 * @brief Performs the spatial learner base operation for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 * - @p name: Supplies name input to the operation.
 * - @p mode: Supplies mode input to the operation.
 * - @p contract: Supplies contract input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SpatialLearnerBase::SpatialLearnerBase(SpatialRepresentation representation,
                                       std::string name, UpdateMode mode,
                                       ObservationContract contract)
    : contract_(std::move(contract)) {
  if (name.empty()) {
    throw std::invalid_argument("spatial learner name must not be empty");
  }
  if (contract_.update_trigger.empty()) {
    throw std::invalid_argument(
        "spatial learner update trigger must not be empty");
  }
  if (contract_.consumers.empty()) {
    throw std::invalid_argument(
        "spatial learner must declare at least one consumer");
  }
  update_.representation = representation;
  update_.learner = std::move(name);
  update_.update_mode = mode;
  update_.update_schedule = contract_.schedule;
  update_.consumers = contract_.consumers;
}

/**
 * @brief Processes package content for this subsystem.
 *
 * Arguments:
 * - @p episode: Supplies episode input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SpatialLearnerBase::observe(const NavigationEpisode& episode) {
  if (!episode.observation.pose.position.finite()) {
    throw std::invalid_argument("navigation episode pose must be finite");
  }
  episode.observation.laser.validate();
  if (episode.sequence == 0U) {
    throw std::invalid_argument("navigation episode sequence must be positive");
  }
  if (update_.last_observation_sequence &&
      episode.sequence <= *update_.last_observation_sequence) {
    throw std::invalid_argument(
        "navigation episodes must have strictly increasing sequence numbers");
  }

  if (update_.update_mode == UpdateMode::RebuildOnDemand) {
    episodes_.push_back(episode);
  }
  ++update_.observed_episodes;
  update_.last_observation_sequence = episode.sequence;
  if (update_.update_mode == UpdateMode::RebuildOnDemand) {
    update_.status =
        update_.revision == 0U ? ModelStatus::Incomplete : ModelStatus::Stale;
    update_.diagnostic =
        update_.revision == 0U
            ? "observations collected; rebuild required"
            : "new observations collected after the last rebuild";
  }
  onObserve(episode);
}

/**
 * @brief Performs the rebuild operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SpatialLearnerBase::rebuild() { onRebuild(); }

/**
 * @brief Performs the snapshot operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `SpatialModelUpdate` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SpatialModelUpdate SpatialLearnerBase::snapshot() const {
  SpatialModelUpdate result = published_ ? *published_ : update_;
  result.observed_episodes = update_.observed_episodes;
  result.last_observation_sequence = update_.last_observation_sequence;
  result.status = update_.status;
  result.diagnostic = update_.diagnostic;
  return result;
}

/**
 * @brief Performs the shared snapshot operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `SharedSpatialSnapshot` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SharedSpatialSnapshot SpatialLearnerBase::sharedSnapshot() const {
  if (published_ && published_->revision == update_.revision &&
      published_->status == update_.status)
    return published_;
  if (!metadata_snapshot_ || metadata_snapshot_->revision != update_.revision ||
      metadata_snapshot_->status != update_.status ||
      metadata_snapshot_->observed_episodes != update_.observed_episodes)
    metadata_snapshot_ = std::make_shared<const SpatialModelUpdate>(update_);
  return metadata_snapshot_;
}

/**
 * @brief Publishes package content for this subsystem.
 *
 * Arguments:
 * - @p payload_value: Supplies payload value input to the operation.
 * - @p status: Supplies status input to the operation.
 * - @p diagnostic: Supplies diagnostic input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SpatialLearnerBase::publish(SpatialPayload payload_value,
                                 ModelStatus status, std::string diagnostic) {
  if (status == ModelStatus::Empty || status == ModelStatus::Stale) {
    throw std::invalid_argument(
        "published spatial model must be fresh or explicitly incomplete");
  }
  std::ostringstream encoded;
  payload(encoded, payload_value);
  const std::string signature = encoded.str();
  const bool changed = update_.revision == 0U ||
                       signature != published_payload_signature_ ||
                       status != update_.status;
  if (changed) ++update_.revision;
  update_.status = status;
  update_.diagnostic = std::move(diagnostic);
  metadata_snapshot_.reset();
  if (!changed) return;
  published_payload_signature_ = signature;
  auto publication = std::make_shared<SpatialModelUpdate>(update_);
  publication->changes = changeSet(published_ ? &published_->payload : nullptr,
                                   payload_value, update_.revision);
  publication->payload = std::move(payload_value);
  published_ = std::move(publication);
}

/**
 * @brief Performs the mark incomplete operation for this subsystem.
 *
 * Arguments:
 * - @p diagnostic: Supplies diagnostic input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SpatialLearnerBase::markIncomplete(std::string diagnostic) {
  publish(std::monostate{}, ModelStatus::Incomplete, std::move(diagnostic));
}

}  // namespace semaforr::spatial
