#include <gtest/gtest.h>

#include <algorithm>
#include <numbers>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <semaforr/spatial/learners/circumstance_learner.hpp>
#include <semaforr/spatial/representations/highway_model.hpp>
#include <semaforr/spatial/representations/known_grid.hpp>
#include <string>
#include <variant>
#include <vector>

namespace {

semaforr::spatial::NavigationEpisode episode(std::size_t sequence, double x_m,
                                             bool task_started = false) {
  using namespace semaforr;
  domain::LaserObservation laser;
  laser.angle_min = domain::Angle(-0.2);
  laser.angle_increment = domain::Angle(0.1);
  laser.minimum_range = domain::Distance(0.1);
  laser.maximum_range = domain::Distance(5.0);
  laser.ranges_m = {1.0, 1.0, 2.0, 1.0, 1.0};

  domain::RobotObservation observation;
  observation.pose = {{x_m, 0.0}, domain::Angle::zero()};
  observation.laser = std::move(laser);
  spatial::NavigationEpisode result;
  result.sequence = sequence;
  result.observation = std::move(observation);
  result.selected_action =
      domain::Action(domain::ActionType::Forward, 1U);
  result.active_task = domain::TaskId{1U};
  result.task_started = task_started;
  result.action_completed = true;
  domain::ActionExecutionResult execution;
  execution.decision_id = sequence;
  execution.action_id = sequence;
  execution.task_id = domain::TaskId{1U};
  execution.status = domain::ExecutionCompletionStatus::Succeeded;
  execution.start_pose = result.observation.pose;
  if (!task_started) execution.start_pose.position.x_m -= 0.3;
  execution.final_pose = result.observation.pose;
  result.execution_result = execution;
  return result;
}

class CapturingLearner final : public semaforr::spatial::SpatialLearner {
 public:
  void observe(const semaforr::spatial::NavigationEpisode& episode) override {
    captured = episode;
    ++update.observed_episodes;
    update.status = semaforr::spatial::ModelStatus::Fresh;
  }

  void rebuild() override { ++update.revision; }
  semaforr::spatial::SpatialModelUpdate snapshot() const override {
    return update;
  }
  semaforr::spatial::SpatialRepresentation representation()
      const noexcept override {
    return semaforr::spatial::SpatialRepresentation::Trails;
  }
  std::string_view name() const noexcept override { return "capture"; }
  const semaforr::spatial::ObservationContract& contract()
      const noexcept override {
    return observation_contract;
  }

  std::optional<semaforr::spatial::NavigationEpisode> captured;
  semaforr::spatial::SpatialModelUpdate update = [] {
    semaforr::spatial::SpatialModelUpdate value;
    value.representation = semaforr::spatial::SpatialRepresentation::Trails;
    value.learner = "capture";
    return value;
  }();
  semaforr::spatial::ObservationContract observation_contract{
      true, true, true, true, "after terminal execution", {"test"},
      semaforr::spatial::UpdateSchedule::AfterSuccessfulActionCompletion};
};

}  // namespace

TEST(SpatialLearning, DefaultModulesDeclareLifecycleAndConsumers) {
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);

  EXPECT_EQ(coordinator.learnerCount(), 12U);
  EXPECT_EQ(coordinator.enabledCount(), 12U);
  const auto inspection = coordinator.inspect();
  ASSERT_EQ(inspection.size(), 12U);
  for (const LearnerInspection& learner : inspection) {
    EXPECT_FALSE(learner.name.empty());
    EXPECT_FALSE(learner.contract.update_trigger.empty());
    EXPECT_FALSE(learner.contract.consumers.empty());
    EXPECT_EQ(learner.update.status, ModelStatus::Empty);
  }
}

TEST(SpatialLearning, LearnersCanBeEnabledAndObservedIndependently) {
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.setEnabled(SpatialRepresentation::Hallways, false);

  coordinator.observe(episode(1U, 0.0, true));
  coordinator.observe(episode(2U, 0.3));

  ASSERT_TRUE(coordinator.snapshot(SpatialRepresentation::Trails));
  EXPECT_EQ(coordinator.snapshot(SpatialRepresentation::Trails)->status,
            ModelStatus::Fresh);
  EXPECT_FALSE(coordinator.snapshot(SpatialRepresentation::Hallways));
  EXPECT_FALSE(coordinator.enabled(SpatialRepresentation::Hallways));

  const auto inspection = coordinator.inspect();
  const auto hallway = std::find_if(
      inspection.begin(), inspection.end(), [](const LearnerInspection& value) {
        return value.representation == SpatialRepresentation::Hallways;
      });
  ASSERT_NE(hallway, inspection.end());
  EXPECT_FALSE(hallway->enabled);
  EXPECT_EQ(hallway->update.observed_episodes, 0U);
}

TEST(SpatialLearning, DisablingALearnerRemovesOnlyItsProjectedModel) {
  using namespace semaforr;
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0, true));
  coordinator.observe(episode(2U, 0.3));

  domain::SpatialModel projected;
  coordinator.applyTo(projected);
  ASSERT_FALSE(projected.trails.empty());

  coordinator.setEnabled(SpatialRepresentation::Trails, false);
  coordinator.applyTo(projected);
  EXPECT_TRUE(projected.trails.empty());
}

TEST(SpatialLearning, AutomaticRebuildUsesAcceptedEpisodeCount) {
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(2U);

  coordinator.observe(episode(7U, 0.0, true));
  const auto first_revision =
      coordinator.snapshot(SpatialRepresentation::Regions)->revision;
  EXPECT_GT(first_revision, 0U);

  coordinator.observe(episode(8U, 0.3));
  EXPECT_GT(coordinator.snapshot(SpatialRepresentation::Regions)->revision,
            first_revision);
}

TEST(SpatialLearning, RegionAndSkeletonModelsUpdateIncrementally) {
  using namespace semaforr;
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0, true));
  coordinator.observe(episode(2U, 0.3));
  coordinator.observe(episode(3U, 0.6));

  const auto fresh = coordinator.snapshot(SpatialRepresentation::Regions);
  ASSERT_TRUE(fresh);
  ASSERT_EQ(fresh->status, ModelStatus::Fresh);

  domain::SpatialModel projected;
  coordinator.applyTo(projected);
  ASSERT_FALSE(projected.learned_regions.empty());
  const auto region_revision = fresh->revision;

  coordinator.observe(episode(4U, 0.9));
  EXPECT_EQ(coordinator.snapshot(SpatialRepresentation::Regions)->status,
            ModelStatus::Fresh);
  EXPECT_GT(coordinator.snapshot(SpatialRepresentation::Regions)->revision,
            region_revision);
  coordinator.applyTo(projected);
  EXPECT_GT(projected.revision, 0U);
  EXPECT_FALSE(projected.skeleton_edges.empty());
}

TEST(SpatialLearning, EveryRepresentationRebuildsAndSerializesIndependently) {
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  const std::vector<double> positions{0.0, 0.3, 0.6, 1.2, 1.8};
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    coordinator.observe(episode(index + 1U, positions[index], index == 0U));
  }
  coordinator.rebuildAll();

  for (const SpatialModelUpdate& update : coordinator.snapshots()) {
    EXPECT_GT(update.revision, 0U);
    EXPECT_NE(update.status, ModelStatus::Empty);
    const std::string encoded = coordinator.serialize(update.representation);
    EXPECT_NE(encoded.find("\"representation\":\"" +
                           std::string(toString(update.representation)) + "\""),
              std::string::npos);
    EXPECT_NE(encoded.find("\"payload\":"), std::string::npos);
  }

  EXPECT_TRUE(std::holds_alternative<TrailModel>(
      coordinator.snapshot(SpatialRepresentation::Trails)->payload));
  EXPECT_TRUE(std::holds_alternative<ConveyorModel>(
      coordinator.snapshot(SpatialRepresentation::Conveyors)->payload));
  EXPECT_TRUE(std::holds_alternative<RegionModel>(
      coordinator.snapshot(SpatialRepresentation::Regions)->payload));
  EXPECT_TRUE(std::holds_alternative<DoorExitModel>(
      coordinator.snapshot(SpatialRepresentation::DoorsAndExits)->payload));
  EXPECT_TRUE(std::holds_alternative<HallwayModel>(
      coordinator.snapshot(SpatialRepresentation::Hallways)->payload));
  EXPECT_TRUE(std::holds_alternative<BarrierModel>(
      coordinator.snapshot(SpatialRepresentation::Barriers)->payload));
  EXPECT_TRUE(std::holds_alternative<PassageSkeletonModel>(
      coordinator.snapshot(SpatialRepresentation::PassagesAndSkeleton)
          ->payload));
  EXPECT_TRUE(std::holds_alternative<KnownGridModel>(
      coordinator.snapshot(SpatialRepresentation::KnownGrid)->payload));
  EXPECT_TRUE(std::holds_alternative<SensedOccupancyModel>(
      coordinator.snapshot(SpatialRepresentation::SensedOccupancy)->payload));
  EXPECT_TRUE(std::holds_alternative<InclusionGridModel>(
      coordinator.snapshot(SpatialRepresentation::InclusionGrid)->payload));
  EXPECT_TRUE(std::holds_alternative<CircumstanceModel>(
      coordinator.snapshot(SpatialRepresentation::Circumstances)->payload));
  const std::string all = coordinator.serializeAll();
  EXPECT_NE(all.find("\"schema\":\"semaforr.spatial.v1\""),
            std::string::npos);
}

TEST(SpatialLearning, PublishesImmutableRevisionedSparseSnapshots) {
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0, true));
  const auto first =
      coordinator.snapshot(SpatialRepresentation::KnownGrid);
  ASSERT_TRUE(first);
  ASSERT_TRUE(std::holds_alternative<KnownGridModel>(first->payload));
  const auto& grid = std::get<KnownGridModel>(first->payload);
  EXPECT_TRUE(grid.observations.empty());
  EXPECT_FALSE(grid.sparse_observations.empty());

  coordinator.rebuild(SpatialRepresentation::KnownGrid);
  const auto rebuilt =
      coordinator.snapshot(SpatialRepresentation::KnownGrid);
  ASSERT_TRUE(rebuilt);
  EXPECT_EQ(rebuilt->revision, first->revision);
  EXPECT_EQ(std::get<KnownGridModel>(rebuilt->payload).sparse_observations.size(),
            grid.sparse_observations.size());
}

TEST(SpatialLearning, SkeletonCachesStableConnectedComponents) {
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0, true));
  coordinator.observe(episode(2U, 0.6));
  const auto snapshot =
      coordinator.snapshot(SpatialRepresentation::PassagesAndSkeleton);
  ASSERT_TRUE(snapshot);
  const auto& skeleton =
      std::get<PassageSkeletonModel>(snapshot->payload);
  ASSERT_EQ(skeleton.component_by_node.size(), skeleton.nodes.size());
  EXPECT_EQ(skeleton.component_by_node[0], skeleton.component_by_node[1]);
  EXPECT_GT(skeleton.connectivity_revision, 0U);
}

TEST(SpatialLearning, EpisodeOrderingIsValidatedPerLearner) {
  auto coordinator =
      semaforr::spatial::SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0));
  EXPECT_THROW(coordinator.observe(episode(1U, 0.2)), std::invalid_argument);
}

TEST(SpatialLearning, NavigationEngineObservesCompletePostDecisionEpisodes) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {2.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2}, {0.5});
  decision::DecisionCoordinator decisions(
      {1.0e-9, decision::UnscoredActionPolicy::Exclude, 0.0,
       domain::Action(domain::ActionType::Forward, 1U), 0U});
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  auto learner = std::make_unique<CapturingLearner>();
  CapturingLearner* capture = learner.get();
  learning.addLearner(std::move(learner));
  decision::NavigationEngine engine(world, action_space, decisions, mission,
                                    planning, learning);

  const auto input = episode(1U, 0.0, true).observation;
  const decision::DecisionResult result = engine.decide(input);

  EXPECT_FALSE(capture->captured);
  EXPECT_EQ(world.decision_history.entries().size(), 1U);
  EXPECT_TRUE(world.navigation_history.entries().empty());
  const auto now = std::chrono::steady_clock::now();
  EXPECT_EQ(engine.onActionStarted(
                {result.decision_id, result.action_id, now, input.pose}),
            domain::FeedbackDisposition::Accepted);
  domain::ActionExecutionResult execution;
  execution.decision_id = result.decision_id;
  execution.action_id = result.action_id;
  execution.task_id = domain::TaskId{1U};
  execution.finished_at = now;
  execution.status = domain::ExecutionCompletionStatus::Succeeded;
  execution.start_pose = input.pose;
  execution.final_pose = {{0.2, 0.0}, domain::Angle::zero()};
  execution.distance_achieved_m = 0.2;
  EXPECT_EQ(engine.onActionCompleted(execution),
            domain::FeedbackDisposition::Accepted);

  ASSERT_TRUE(capture->captured);
  ASSERT_TRUE(capture->captured->selected_action);
  EXPECT_EQ(*capture->captured->selected_action, result.action);
  ASSERT_TRUE(result.task);
  EXPECT_EQ(result.task->decision_count, 1U);
  EXPECT_TRUE(capture->captured->task_started);
  EXPECT_EQ(capture->captured->active_task, domain::TaskId{1U});
  EXPECT_EQ(world.navigation_history.entries().size(), 1U);
  EXPECT_EQ(world.completed_path_history.entries().size(), 1U);
}

TEST(SpatialLearning, InitialExplorationFinalizationPublishesGraphModels) {
  auto coordinator =
      semaforr::spatial::SpatialLearningCoordinator::defaults();
  for (std::size_t sequence = 1U; sequence <= 3U; ++sequence) {
    auto input = episode(sequence, static_cast<double>(sequence));
    input.initial_exploration = true;
    coordinator.observe(input);
  }
  coordinator.finalizeInitialExploration();
  const auto highway =
      coordinator.snapshot(
          semaforr::spatial::SpatialRepresentation::Highways);
  const auto skeleton =
      coordinator.snapshot(
          semaforr::spatial::SpatialRepresentation::PassagesAndSkeleton);
  ASSERT_TRUE(highway);
  ASSERT_TRUE(skeleton);
  EXPECT_GT(highway->revision, 0U);
  EXPECT_GT(skeleton->revision, 0U);
}

TEST(CircumstanceLearning, NormalizesSettingsAndLearnsQualifiedCases) {
  using namespace semaforr;
  using namespace semaforr::spatial;
  CircumstanceLearningConfiguration configuration;
  configuration.setting_resolution_m = 1.0;
  configuration.setting_radius_m = 5.0;
  configuration.minimum_cluster_size = 2U;
  configuration.reclustering_threshold = 2U;
  configuration.minimum_case_evidence = 2U;
  configuration.assignment_confidence_threshold = 0.8;
  CircumstanceLearner learner(configuration);

  for (std::size_t index = 0U; index < 4U; ++index) {
    NavigationEpisode input = episode(index + 1U, 0.4 * index,
                                      index == 0U);
    input.active_target = domain::Point2D{4.0, 0.0};
    input.viable_actions = {
        domain::Action(domain::ActionType::Forward, 1U),
        domain::Action(domain::ActionType::TurnLeft, 1U)};
    input.move_distances_m = {0.25};
    input.rotation_angles_rad = {0.2};
    input.task_finished = index == 3U;
    learner.observe(input);
  }
  learner.rebuild();

  const auto update = learner.snapshot();
  ASSERT_TRUE(std::holds_alternative<CircumstanceModel>(update.payload));
  const auto& model = std::get<CircumstanceModel>(update.payload);
  ASSERT_EQ(model.clusters.size(), 1U);
  EXPECT_EQ(model.clusters.front().evidence, 4U);
  EXPECT_EQ(model.clusters.front().centroid.side_cells, 11U);
  ASSERT_FALSE(model.cases.empty());
  EXPECT_GE(model.cases.front().evidence, 2U);
  EXPECT_GE(model.cases.front().accuracy, configuration.accuracy_threshold);
  EXPECT_FALSE(model.cases.front().confidence.empty());
}
