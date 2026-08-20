#include <gtest/gtest.h>

#include <memory>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/exploration/exploration_coordinator.hpp>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <semaforr/exploration/highway_explorer.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <semaforr/spatial/learners/highway_learner.hpp>

namespace {

semaforr::domain::RobotObservation observation(double x,
                                               std::vector<double> ranges = {
                                                   2.0, 2.0, 2.0, 2.0, 2.0}) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x, 0.0}, semaforr::domain::Angle::zero()};
  result.laser.angle_min = semaforr::domain::Angle(-0.4);
  result.laser.angle_increment = semaforr::domain::Angle(0.2);
  result.laser.minimum_range = semaforr::domain::Distance(0.1);
  result.laser.maximum_range = semaforr::domain::Distance(5.0);
  result.laser.ranges_m = std::move(ranges);
  return result;
}

semaforr::domain::RobotObservation compatibilityObservation(
    double x = 0.0, double y = 0.0, double heading = 0.0,
    double right_range = 4.0, double left_range = 4.0) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x, y}, semaforr::domain::Angle(heading)};
  result.laser.angle_min = semaforr::domain::Angle(-0.61);
  result.laser.angle_increment = semaforr::domain::Angle(0.01);
  result.laser.minimum_range = semaforr::domain::Distance(0.1);
  result.laser.maximum_range = semaforr::domain::Distance(10.0);
  result.laser.ranges_m.assign(123U, 0.3);
  std::fill(result.laser.ranges_m.begin(),
            result.laser.ranges_m.begin() + 41, right_range);
  std::fill(result.laser.ranges_m.end() - 41,
            result.laser.ranges_m.end(), left_range);
  return result;
}

semaforr::domain::StaticMap planningMap() {
  semaforr::domain::StaticMap map;
  map.source = "planning-test";
  map.bounds = {{0.0, -1.5}, {3.0, 1.5}};
  map.walls = {{{0.0, -1.5}, {3.0, -1.5}}};
  map.occupancy = {
      3U, 3U, 1.0, {0.0, -1.5},
      std::vector<semaforr::domain::StaticOccupancyState>(
          9U, semaforr::domain::StaticOccupancyState::StaticFree)};
  return map;
}

}  // namespace

TEST(HighwayExplore, PassageSelectionAndStateTransitionsAreDeterministic) {
  semaforr::exploration::HighwayExplorer first;
  semaforr::exploration::HighwayExplorer second;
  const semaforr::domain::ActionSpace actions({0.25, 0.75}, {0.2, 0.5});
  const auto view = observation(0.0, {2.0, 2.0, 0.2, 2.0, 2.0});
  const auto one = first.decide(view, actions);
  const auto two = second.decide(view, actions);
  EXPECT_EQ(one.action, two.action);
  EXPECT_EQ(one.state, two.state);
  ASSERT_EQ(one.candidates.size(), 2U);
  EXPECT_EQ(one.action.type(), semaforr::domain::ActionType::TurnRight);
}

TEST(HighLevelExplore,
     OwnsDeterministicCandidateLifecycleAndSparsePassageGrid) {
  semaforr::exploration::HighLevelExplorationConfiguration configuration;
  configuration.candidate_completion_distance = semaforr::domain::Distance(0.1);
  semaforr::exploration::HighLevelExplorer explorer(configuration);
  const semaforr::domain::ActionSpace actions({0.1, 0.2}, {0.25, 0.5});
  auto view = observation(0.0);

  EXPECT_EQ(explorer.update({view, actions, {}}).state,
            semaforr::exploration::HleState::DiscoverCandidate);
  const auto selected = explorer.update({view, actions, {}});
  ASSERT_EQ(selected.state,
            semaforr::exploration::HleState::ReturnToCandidateStart);
  ASSERT_TRUE(selected.candidate_id);
  ASSERT_FALSE(selected.discovered.empty());
  EXPECT_EQ(*selected.candidate_id, selected.discovered.front().id);
  EXPECT_EQ(*selected.candidate_id, 1U);

  const auto pursuing = explorer.update({view, actions, {}});
  EXPECT_EQ(pursuing.state, semaforr::exploration::HleState::PursueCandidate);
  EXPECT_EQ(pursuing.event,
            semaforr::exploration::CandidateLifecycleEvent::PursuitStarted);
  view.pose.position.x_m = 0.2;
  const auto completed = explorer.update({view, actions, {}});
  EXPECT_EQ(completed.event,
            semaforr::exploration::CandidateLifecycleEvent::Completed);
  EXPECT_GT(completed.passage_grid_revision, 0U);
  const auto passage_grid = explorer.passageGrid();
  EXPECT_TRUE(passage_grid.geometry.valid());
  EXPECT_FALSE(passage_grid.cells.empty());
  EXPECT_TRUE(std::any_of(
      passage_grid.cells.begin(), passage_grid.cells.end(),
      [](const auto& cell) {
        return cell.state == semaforr::exploration::PassageCellState::Free;
      }));
  EXPECT_TRUE(std::any_of(
      passage_grid.cells.begin(), passage_grid.cells.end(),
      [](const auto& cell) {
        return cell.state ==
               semaforr::exploration::PassageCellState::Obstructed;
      }));
  EXPECT_TRUE(std::any_of(
      passage_grid.cells.begin(), passage_grid.cells.end(),
      [&](const auto& cell) {
        return cell.state == semaforr::exploration::PassageCellState::Passage &&
               cell.passage_id == selected.candidate_id;
      }));
}

TEST(HighLevelExplore, ReportsBudgetCompletionAndFinalizesExactlyOnce) {
  semaforr::exploration::HighLevelExplorationConfiguration configuration;
  configuration.decision_budget = 1U;
  semaforr::exploration::HighLevelExplorer explorer(configuration);
  const semaforr::domain::ActionSpace actions({0.1}, {0.25});
  const auto view = observation(0.0);
  static_cast<void>(explorer.update({view, actions, {}}));
  const auto finalizing = explorer.update({view, actions, {}});
  EXPECT_EQ(finalizing.state, semaforr::exploration::HleState::FinalizeModel);
  EXPECT_EQ(finalizing.completion_reason,
            semaforr::exploration::ExplorationCompletionReason::
                DecisionBudgetExceeded);
  EXPECT_EQ(explorer.update({view, actions, {}}).state,
            semaforr::exploration::HleState::Complete);

  semaforr::exploration::ExplorationCoordinator coordinator(configuration);
  std::size_t finalizations = 0U;
  coordinator.setModelFinalizer([&finalizations] { ++finalizations; });
  coordinator.finish();
  coordinator.finish();
  EXPECT_EQ(finalizations, 1U);
}

TEST(HighLevelExplore,
     CompatibilityCuesUseFixedBundlesAndExplicitGeometricValidation) {
  using namespace semaforr;
  exploration::HighLevelExplorationConfiguration configuration;
  configuration.behavior_policy = exploration::HleBehaviorPolicy::Compatibility;
  configuration.large_room_width = domain::Distance(20.0);
  const auto view = compatibilityObservation(10.0, -2.0, 0.7);
  const auto candidates =
      exploration::HighLevelExplorer::discoverCandidates(view, configuration);
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].cue_type,
            exploration::PassageCueType::RightOpen);
  EXPECT_EQ(candidates[0].first_beam, 0U);
  EXPECT_EQ(candidates[0].last_beam, 40U);
  EXPECT_EQ(candidates[1].cue_type,
            exploration::PassageCueType::LeftOpen);
  EXPECT_EQ(candidates[1].first_beam, 82U);
  EXPECT_EQ(candidates[1].last_beam, 122U);
  for (const auto& candidate : candidates) {
    EXPECT_GT(candidate.length.meters(), candidate.width.meters());
    EXPECT_GT(candidate.confidence, 0.0);
    EXPECT_GT(domain::distance(candidate.start, candidate.endpoint).meters(),
              0.0);
  }
  auto narrow_policy = configuration;
  narrow_policy.minimum_length_to_width_ratio = 10.0;
  EXPECT_TRUE(exploration::HighLevelExplorer::discoverCandidates(
                  view, narrow_policy)
                  .empty());
  auto room_policy = configuration;
  room_policy.large_room_width = domain::Distance(3.0);
  const auto rooms = exploration::HighLevelExplorer::discoverCandidates(
      compatibilityObservation(10.0, -2.0, 0.7, 8.0, 8.0), room_policy);
  ASSERT_EQ(rooms.size(), 2U);
  EXPECT_TRUE(std::all_of(rooms.begin(), rooms.end(), [](const auto& cue) {
    return cue.kind == exploration::PassageKind::LargeRoom;
  }));

  exploration::HighLevelExplorer explorer(configuration);
  const auto validation = explorer.evaluateCue(candidates.front(), view);
  EXPECT_TRUE(validation.start_clear);
  EXPECT_TRUE(validation.midpoint_clear);
  EXPECT_TRUE(validation.endpoint_clear);
  EXPECT_TRUE(validation.geometrically_reachable);
  EXPECT_EQ(validation.passage_identities, 0U);
  EXPECT_TRUE(validation.accepted);

  auto blocked = view;
  blocked.laser.ranges_m[20] = 0.5;
  const auto blocked_validation =
      explorer.evaluateCue(candidates.front(), blocked);
  EXPECT_FALSE(blocked_validation.endpoint_clear);
  EXPECT_FALSE(blocked_validation.accepted);

  exploration::PassageGridSnapshot prior;
  prior.geometry = domain::GridGeometry::fromBounds(
      "map", {5.0, -7.0}, {15.0, 3.0}, 0.5,
      domain::GridExtentMode::Expandable,
      domain::GridExtentSource::SensorDerivedExpansion,
      domain::GridOutOfBoundsBehavior::ExpandBeforeInsert);
  prior.revision = 4U;
  for (const auto& [fraction, passage] :
       {std::pair{0.25, 7U}, std::pair{0.75, 8U}}) {
    const domain::Point2D point{
        candidates.front().start.x_m +
            fraction * (candidates.front().endpoint.x_m -
                        candidates.front().start.x_m),
        candidates.front().start.y_m +
            fraction * (candidates.front().endpoint.y_m -
                        candidates.front().start.y_m)};
    const auto cell = prior.geometry.cell(point);
    ASSERT_TRUE(cell);
    exploration::PassageCell evidence;
    evidence.row = static_cast<int>(cell->second);
    evidence.column = static_cast<int>(cell->first);
    evidence.state = exploration::PassageCellState::Passage;
    evidence.passage_id = passage;
    evidence.completion_state =
        exploration::PassageCompletionState::Completed;
    evidence.evidence_count = 1U;
    prior.cells.push_back(evidence);
  }
  exploration::HighLevelExplorer intersecting(configuration);
  intersecting.restorePassageGrid(prior);
  const auto conflict = intersecting.evaluateCue(candidates.front(), view);
  EXPECT_EQ(conflict.passage_identities, 2U);
  EXPECT_FALSE(conflict.accepted);
  EXPECT_EQ(conflict.reason, "cue_intersects_multiple_passages");

  auto overlapping = candidates.front();
  overlapping.start.x_m += 0.1;
  overlapping.endpoint.x_m += 0.1;
  EXPECT_TRUE(exploration::HighLevelExplorer::cuesSimilar(
      candidates.front(), overlapping, 0.5));
  overlapping.start.y_m += 5.0;
  overlapping.endpoint.y_m += 5.0;
  EXPECT_FALSE(exploration::HighLevelExplorer::cuesSimilar(
      candidates.front(), overlapping, 0.5));
}

TEST(HighLevelExplore,
     CompatibilityPursuitUsesGlobalHeadingExtendsAndReplaysExactly) {
  using namespace semaforr;
  exploration::HighLevelExplorationConfiguration configuration;
  configuration.behavior_policy = exploration::HleBehaviorPolicy::Compatibility;
  configuration.large_room_width = domain::Distance(20.0);
  configuration.maximum_width_change_ratio = 10.0;
  exploration::HighLevelExplorer explorer(configuration);
  const domain::ActionSpace actions({0.1, 0.4}, {0.2, 0.5, 1.0});
  auto view = compatibilityObservation();
  static_cast<void>(explorer.update({view, actions, {}}));
  const auto selected = explorer.update({view, actions, {}});
  ASSERT_TRUE(selected.candidate_id);
  const auto candidate = *std::find_if(
      selected.discovered.begin(), selected.discovered.end(),
      [&](const auto& value) { return value.id == *selected.candidate_id; });
  static_cast<void>(explorer.update({view, actions, {}}));

  view.pose.position =
      {0.5 * std::cos(candidate.direction.radians()),
       0.5 * std::sin(candidate.direction.radians())};
  std::fill(view.laser.ranges_m.begin(),
            view.laser.ranges_m.begin() + 41, 5.0);
  const auto extended = explorer.update({view, actions, {}});
  EXPECT_EQ(extended.state, exploration::HleState::PursueCandidate);
  const auto unfinished = explorer.unfinishedCandidates();
  const auto active = std::find_if(unfinished.begin(), unfinished.end(),
                                   [&](const auto& value) {
                                     return value.id == *selected.candidate_id;
                                   });
  ASSERT_NE(active, unfinished.end());
  EXPECT_GT(active->current_extension.meters(),
            candidate.current_extension.meters());

  view.pose.heading = candidate.direction;
  const auto aligned_after_movement = explorer.update({view, actions, {}});
  EXPECT_EQ(aligned_after_movement.action.type(), domain::ActionType::Forward);
  EXPECT_EQ(active->direction, candidate.direction);

  const auto replay = exploration::HighLevelExplorer::replay(explorer.trace());
  ASSERT_EQ(replay.size(), explorer.trace().size());
  ASSERT_FALSE(replay.empty());
  for (std::size_t index = 0U; index < replay.size(); ++index) {
    EXPECT_EQ(replay[index].action, explorer.trace()[index].result.action);
    EXPECT_EQ(replay[index].state, explorer.trace()[index].result.state);
    EXPECT_EQ(replay[index].event, explorer.trace()[index].result.event);
    EXPECT_EQ(replay[index].pursuit_termination_reason,
              explorer.trace()[index].result.pursuit_termination_reason);
  }
}

TEST(HighLevelExplore,
     CompatibilityTerminationAndPassageAssociationsAreExplicit) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.1, 0.4}, {0.2, 0.5, 1.0});
  const auto run = [&](domain::RobotObservation terminal_view,
                       exploration::PursuitTerminationReason expected,
                       exploration::PassageCompletionState completion) {
    exploration::HighLevelExplorationConfiguration configuration;
    configuration.behavior_policy =
        exploration::HleBehaviorPolicy::Compatibility;
    configuration.large_room_width = domain::Distance(3.0);
    exploration::HighLevelExplorer explorer(configuration);
    auto initial = compatibilityObservation();
    static_cast<void>(explorer.update({initial, actions, {}}));
    const auto selected = explorer.update({initial, actions, {}});
    ASSERT_TRUE(selected.candidate_id);
    static_cast<void>(explorer.update({initial, actions, {}}));
    const auto terminal = explorer.update({terminal_view, actions, {}});
    EXPECT_EQ(terminal.pursuit_termination_reason, expected);
    EXPECT_TRUE(terminal.event == exploration::CandidateLifecycleEvent::Completed ||
                terminal.event == exploration::CandidateLifecycleEvent::Suspended);
    EXPECT_TRUE(std::any_of(
        terminal.diagnostics.begin(), terminal.diagnostics.end(),
        [&](const auto& diagnostic) {
          return diagnostic.kind ==
                 (completion == exploration::PassageCompletionState::Suspended
                      ? exploration::CandidateDiagnosticKind::Suspended
                      : exploration::CandidateDiagnosticKind::Completed);
        }));
    const auto grid = explorer.passageGrid();
    EXPECT_TRUE(std::any_of(grid.cells.begin(), grid.cells.end(),
                            [&](const auto& cell) {
                              return cell.state ==
                                         exploration::PassageCellState::Passage &&
                                     cell.candidate_id == selected.candidate_id &&
                                     cell.passage_id.has_value() &&
                                     cell.completion_state == completion;
                            }));
  };

  auto width_change = compatibilityObservation(0.2, 0.0, 0.0, 2.0, 4.0);
  run(width_change, exploration::PursuitTerminationReason::WidthChanged,
      exploration::PassageCompletionState::Suspended);
  auto hard_turn = compatibilityObservation(0.2, 0.0, 1.0);
  run(hard_turn, exploration::PursuitTerminationReason::HardTurn,
      exploration::PassageCompletionState::Suspended);
  auto large_room = compatibilityObservation(0.2, 0.0, 0.0, 8.0, 4.0);
  run(large_room, exploration::PursuitTerminationReason::LargeRoom,
      exploration::PassageCompletionState::Completed);
  auto ended = compatibilityObservation(0.2, 0.0);
  std::fill(ended.laser.ranges_m.begin(), ended.laser.ranges_m.end(), 0.3);
  run(ended, exploration::PursuitTerminationReason::EndOfPassageClearance,
      exploration::PassageCompletionState::Completed);
}

TEST(HighLevelExplore,
     CompatibilityDiagnosticsExplainMergeRejectionAndLifecycle) {
  using namespace semaforr;
  exploration::HighLevelExplorationConfiguration configuration;
  configuration.behavior_policy = exploration::HleBehaviorPolicy::Compatibility;
  configuration.large_room_width = domain::Distance(20.0);
  exploration::HighLevelExplorer explorer(configuration);
  const domain::ActionSpace actions({0.1, 0.4}, {0.2, 0.5, 1.0});
  auto view = compatibilityObservation();
  static_cast<void>(explorer.update({view, actions, {}}));
  const auto selected = explorer.update({view, actions, {}});
  ASSERT_TRUE(selected.candidate_id);
  static_cast<void>(explorer.update({view, actions, {}}));
  auto ended = view;
  ended.pose.position.x_m = 0.2;
  std::fill(ended.laser.ranges_m.begin(), ended.laser.ranges_m.end(), 0.3);
  static_cast<void>(explorer.update({ended, actions, {}}));
  auto repeated = compatibilityObservation(0.2, 0.0);
  const auto rediscovery = explorer.update({repeated, actions, {}});
  EXPECT_TRUE(std::any_of(rediscovery.diagnostics.begin(),
                          rediscovery.diagnostics.end(), [](const auto& item) {
                            return item.kind ==
                                   exploration::CandidateDiagnosticKind::Merged;
                          }));

  exploration::HighLevelExplorationConfiguration bounded = configuration;
  bounded.passage_grid_geometry = domain::GridGeometry::fromBounds(
      "map", {-1.0, -1.0}, {1.0, 1.0}, 0.5,
      domain::GridExtentMode::Fixed,
      domain::GridExtentSource::RepresentationLocalBounds,
      domain::GridOutOfBoundsBehavior::Reject);
  exploration::HighLevelExplorer rejecting(bounded);
  static_cast<void>(rejecting.update({view, actions, {}}));
  const auto rejection = rejecting.update({view, actions, {}});
  EXPECT_TRUE(std::any_of(rejection.diagnostics.begin(),
                          rejection.diagnostics.end(), [](const auto& item) {
                            return item.kind ==
                                       exploration::CandidateDiagnosticKind::Rejected &&
                                   item.reason == "cue_outside_fixed_grid";
                          }));

  const auto& all = explorer.candidateDiagnostics();
  for (const auto kind : {exploration::CandidateDiagnosticKind::Created,
                          exploration::CandidateDiagnosticKind::Selected,
                          exploration::CandidateDiagnosticKind::Completed,
                          exploration::CandidateDiagnosticKind::Merged})
    EXPECT_TRUE(std::any_of(all.begin(), all.end(), [&](const auto& item) {
      return item.kind == kind;
    }));

  explorer.finish();
  EXPECT_TRUE(std::any_of(
      explorer.candidateDiagnostics().begin(),
      explorer.candidateDiagnostics().end(), [](const auto& item) {
        return item.kind ==
               exploration::CandidateDiagnosticKind::Abandoned;
      }));
}

TEST(HighwayLearning, BuildsVersionedGraphIncrementally) {
  semaforr::spatial::HighwayLearner learner(0.5, 0.8);
  for (std::size_t sequence = 1U; sequence <= 3U; ++sequence) {
    semaforr::spatial::NavigationEpisode episode;
    episode.sequence = sequence;
    episode.observation = observation(static_cast<double>(sequence));
    episode.selected_action = semaforr::domain::Action::pause();
    episode.initial_exploration = true;
    learner.observe(episode);
  }
  EXPECT_EQ(learner.snapshot().revision, 0U);
  learner.rebuild();
  const auto update = learner.snapshot();
  ASSERT_TRUE(update.usable());
  EXPECT_EQ(update.revision, 1U);
  const auto& model = std::get<semaforr::spatial::HighwayModel>(update.payload);
  EXPECT_EQ(model.nodes.size(), 3U);
  EXPECT_EQ(model.edges.size(), 2U);
  ASSERT_EQ(model.highways.size(), 1U);
  EXPECT_EQ(model.highways.front().axis, semaforr::domain::Axis::Horizontal);
  EXPECT_GE(model.highways.front().cells.size(), 3U);
  EXPECT_EQ(model.highways.front().endpoints.size(), 2U);
  ASSERT_EQ(model.graph.edges.size(), 1U);
  EXPECT_FALSE(model.graph.edges.front().trail_labels.empty());
  EXPECT_FALSE(model.graph.edges.front().operational_subtrail.empty());
  EXPECT_TRUE(model.geometry.valid());
  EXPECT_DOUBLE_EQ(model.geometry.resolution_m, 0.5);
  EXPECT_EQ(model.smoothing_policy, "von_neumann_three_of_four");
  EXPECT_EQ(model.component_selection_policy, "most_intersections");
  EXPECT_EQ(model.serialized_schema_version,
            semaforr::spatial::HighwayModel::schema_version);
  const auto encoded = semaforr::spatial::serialize(update);
  EXPECT_NE(encoded.find("\"schema_version\":2"), std::string::npos);
  EXPECT_NE(encoded.find("\"trail_labels\""), std::string::npos);
  EXPECT_NE(encoded.find("\"operational_subtrail\""), std::string::npos);
  EXPECT_FALSE(model.grid_labels.empty());
  EXPECT_FALSE(model.touched_rows.empty());
  EXPECT_FALSE(model.touched_columns.empty());
}

TEST(HighwayLearning, LabelsNegativeWorldCoordinatesWithoutDiscardingThem) {
  semaforr::spatial::HighwayLearner learner(0.5, 0.8);
  semaforr::spatial::NavigationEpisode episode;
  episode.sequence = 1U;
  episode.observation = observation(-2.25);
  episode.observation.pose.position.y_m = -3.25;
  episode.initial_exploration = true;
  learner.observe(episode);
  learner.rebuild();
  const auto update = learner.snapshot();
  const auto& model = std::get<semaforr::spatial::HighwayModel>(
      update.payload);
  ASSERT_FALSE(model.grid_labels.empty());
  EXPECT_GE(model.grid_labels.front().row, 0);
  EXPECT_GE(model.grid_labels.front().column, 0);
  ASSERT_TRUE(model.geometry.valid());
  EXPECT_LT(model.geometry.minimum.x_m, 0.0);
  EXPECT_LT(model.geometry.minimum.y_m, 0.0);
  EXPECT_LT(model.geometry.center(
                static_cast<std::size_t>(model.grid_labels.front().column),
                static_cast<std::size_t>(model.grid_labels.front().row))
                .x_m,
            0.0);
}

TEST(HighwayLearning, CompatibilitySmoothingUsesThreeVonNeumannNeighbors) {
  using CellSet = semaforr::spatial::HighwayCellSet;
  const CellSet free{{0, 0}, {-1, 0}, {1, 0}, {0, -1}};
  const CellSet labeled{{-1, 0}, {1, 0}};
  const auto compatible = semaforr::spatial::smoothHighwayCells(
      free, {}, labeled,
      semaforr::spatial::HighwaySmoothingPolicy::VonNeumannThreeOfFour);
  const auto adapted = semaforr::spatial::smoothHighwayCells(
      free, {}, labeled,
      semaforr::spatial::HighwaySmoothingPolicy::DirectionalGapFill);
  EXPECT_TRUE(compatible.contains({0, 0}));
  EXPECT_TRUE(adapted.contains({0, 0}));
  const CellSet non_gap_labels{{-1, 0}};
  const auto non_gap = semaforr::spatial::smoothHighwayCells(
      free, {}, non_gap_labels,
      semaforr::spatial::HighwaySmoothingPolicy::DirectionalGapFill);
  EXPECT_FALSE(non_gap.contains({0, 0}));
}

TEST(HighwayLearning, ComponentPolicyCanPreferIntersectionCountOrSize) {
  semaforr::domain::Graph<semaforr::domain::Intersection,
                          semaforr::domain::HighwayEdge> graph;
  for (std::size_t id = 0U; id < 7U; ++id)
    graph.vertices.push_back(
        {id, {}, {static_cast<double>(id), 0.0}, id != 4U});
  graph.edges = {{0U, 1U, 0U, 1.0, {}, {}},
                 {1U, 2U, 1U, 1.0, {}, {}},
                 {2U, 3U, 2U, 1.0, {}, {}},
                 {4U, 5U, 3U, 1.0, {}, {}},
                 {4U, 6U, 4U, 1.0, {}, {}}};
  const auto compatible = semaforr::spatial::selectHighwayComponent(
      graph,
      semaforr::spatial::HighwayComponentSelectionPolicy::MostIntersections);
  const auto adapted = semaforr::spatial::selectHighwayComponent(
      graph,
      semaforr::spatial::HighwayComponentSelectionPolicy::LargestVertexCount);
  EXPECT_EQ(compatible.component_by_vertex[4], compatible.selected_component);
  EXPECT_EQ(adapted.component_by_vertex[0], adapted.selected_component);
  EXPECT_NE(compatible.selected_component, adapted.selected_component);
}

TEST(HierarchicalPlans, HighwayPlanProducesTypedOperationalSteps) {
  semaforr::domain::SpatialModel spatial;
  spatial.highways.nodes = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  spatial.highways.edges = {{0U, 1U}, {1U, 2U}};
  spatial.highways.intersections = {{1U, 3U}};
  spatial.revisions[semaforr::domain::ModelDependency::Highways] = 7U;
  spatial.revisions[semaforr::domain::ModelDependency::HighwayGraph] = 7U;
  semaforr::planning::HighwayPlan planner;
  const auto result =
      planner.plan({{{-1.0, 0.0}, semaforr::domain::Angle::zero()},
                    {3.0, 0.0},
                    &spatial,
                    nullptr});
  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.hierarchical);
  EXPECT_EQ(result.hierarchical->dependency_revisions.at(
                semaforr::domain::ModelDependency::HighwayGraph),
            7U);
  const auto dependencies = planner.dependencies(
      {{{-1.0, 0.0}, semaforr::domain::Angle::zero()}, {3.0, 0.0}, &spatial});
  EXPECT_NE(std::find(dependencies.begin(), dependencies.end(),
                      semaforr::domain::ModelDependency::HighwayGraph),
            dependencies.end());
  EXPECT_TRUE(std::any_of(
      result.hierarchical->steps.begin(), result.hierarchical->steps.end(),
      [](const auto& step) {
        return std::holds_alternative<semaforr::planning::IntersectionStep>(
            step);
      }));
  const auto waypoints =
      semaforr::decision::Enforcer{}.operationalize(*result.hierarchical);
  ASSERT_EQ(waypoints.size(), 1U);
  EXPECT_EQ(waypoints.front(), result.path.front());
}

TEST(HierarchicalPlans, HighwayPlanChoosesBestValidNetworkAlternative) {
  semaforr::domain::SpatialModel spatial;
  spatial.skeleton_nodes = {{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}};
  spatial.skeleton_edges = {{0U, 1U}, {1U, 2U}, {2U, 3U}};
  for (std::size_t id = 0U; id < spatial.skeleton_nodes.size(); ++id)
    spatial.region_skeleton_nodes.push_back(
        {id, id, spatial.skeleton_nodes[id], {}});
  spatial.region_skeleton_edges = {
      {0U, 1U, {}, 10.0, 0U}, {1U, 2U, {}, 10.0, 0U},
      {2U, 3U, {}, 10.0, 0U}};
  spatial.highways.graph.vertices = {{0U, {0, 0}, {0.0, 0.0}, true},
                                     {1U, {0, 10}, {10.0, 0.0}, true}};
  spatial.highways.graph.edges = {
      {0U, 1U, 0U, 10.0, {7U}, {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}}}};
  semaforr::planning::HighwayPlan planner;
  const auto assisted =
      planner.plan({{{-1.0, 0.0}, semaforr::domain::Angle::zero()},
                    {11.0, 0.0},
                    &spatial,
                    nullptr});
  ASSERT_TRUE(assisted.succeeded());
  ASSERT_TRUE(assisted.hierarchical);
  EXPECT_EQ(assisted.hierarchical->planner, "highway_plan");
  EXPECT_NE(assisted.hierarchical->provenance.find("highway_assisted"),
            std::string::npos);
  EXPECT_TRUE(std::any_of(
      assisted.hierarchical->steps.begin(), assisted.hierarchical->steps.end(),
      [](const auto& step) {
        const auto* highway = std::get_if<semaforr::planning::HighwayStep>(&step);
        return highway != nullptr && !highway->fallback_subtrail.empty();
      }));

  spatial.highways.graph.vertices = {{0U, {20, 20}, {20.0, 20.0}, true},
                                     {1U, {20, 30}, {30.0, 20.0}, true}};
  const auto skeleton =
      planner.plan({{{-1.0, 0.0}, semaforr::domain::Angle::zero()},
                    {11.0, 0.0},
                    &spatial,
                    nullptr});
  ASSERT_TRUE(skeleton.succeeded());
  ASSERT_TRUE(skeleton.hierarchical);
  EXPECT_EQ(skeleton.hierarchical->planner, "highway_plan");
  EXPECT_NE(skeleton.hierarchical->provenance.find("skeleton"),
            std::string::npos);
}

TEST(PlanCache, ReusesExactRevisionAndInvalidatesOnConsumedRevision) {
  semaforr::planning::PlanningCoordinator coordinator;
  coordinator.registerPlanner(
      std::make_unique<semaforr::planning::DomainPlanner>(
          "distance", semaforr::planning::PlannerObjective::Distance));
  semaforr::domain::SpatialModel spatial;
  const auto map = planningMap();
  semaforr::planning::PlanningRequest request{
      {{0.5, 0.0}, semaforr::domain::Angle::zero()},
      {2.5, 0.0},
      &spatial,
      nullptr,
      &map};
  ASSERT_TRUE(coordinator.selectPlan(request));
  ASSERT_TRUE(coordinator.selectPlan(request));
  EXPECT_EQ(coordinator.cacheHits(), 1U);
  ++spatial.revisions[semaforr::domain::ModelDependency::SensedOccupancy];
  ASSERT_TRUE(coordinator.selectPlan(request));
  EXPECT_EQ(coordinator.cacheHits(), 1U);
}

TEST(PlannerRegistry, ClassifiesGridAffordanceAndFreespacePlanners) {
  const auto registry = semaforr::planning::defaultPlannerRegistry();
  EXPECT_EQ(registry.inputModel("density"),
            semaforr::planning::PlannerInputModel::Grid);
  EXPECT_EQ(registry.inputModel("region"),
            semaforr::planning::PlannerInputModel::AffordanceModifiedGrid);
  EXPECT_EQ(registry.inputModel("highway"),
            semaforr::planning::PlannerInputModel::Freespace);
  EXPECT_EQ(registry.mapRequirement("region"),
            semaforr::planning::StaticMapRequirement::Optional);
  EXPECT_EQ(registry.mapRequirement("highway"),
            semaforr::planning::StaticMapRequirement::Independent);
  EXPECT_EQ(registry.create("flow")->objective(),
            semaforr::planning::PlanObjective::FlowOpposition);
  EXPECT_THROW(registry.create("unknown"), std::invalid_argument);
}

TEST(AffordancePlanner, RegionCostModificationChangesTheChosenRoute) {
  semaforr::domain::SpatialModel spatial;
  auto map = planningMap();
  map.occupancy.cells[4U] =
      semaforr::domain::StaticOccupancyState::StaticOccupied;
  spatial.learned_regions.push_back(
      {{1.5, 1.0}, semaforr::domain::Distance(0.6)});
  semaforr::planning::DomainPlanner planner(
      "region", semaforr::planning::PlanObjective::RegionPreference);
  const auto result =
      planner.plan({{{0.5, 0.0}, semaforr::domain::Angle::zero()},
                    {2.5, 0.0},
                    &spatial,
                    nullptr,
                    &map});
  ASSERT_TRUE(result.succeeded());
  EXPECT_TRUE(std::any_of(result.path.begin(), result.path.end(),
                          [](const auto& point) { return point.y_m > 0.5; }));
  EXPECT_TRUE(result.objective_costs.contains(
      semaforr::planning::PlanObjective::RegionPreference));
}

TEST(PlanSelection, SupportsEveryExplicitPolicyName) {
  using semaforr::planning::PlanSelectionPolicy;
  EXPECT_EQ(semaforr::planning::planSelectionPolicyFromString("single"),
            PlanSelectionPolicy::Single);
  EXPECT_EQ(semaforr::planning::planSelectionPolicyFromString("range_vote"),
            PlanSelectionPolicy::RangeVote);
  EXPECT_EQ(
      semaforr::planning::planSelectionPolicyFromString("pareto_then_vote"),
      PlanSelectionPolicy::ParetoThenVote);
  EXPECT_EQ(semaforr::planning::planSelectionPolicyFromString("shortest_valid"),
            PlanSelectionPolicy::ShortestValid);
  EXPECT_THROW(semaforr::planning::planSelectionPolicyFromString("raw_cost"),
               std::invalid_argument);
}
