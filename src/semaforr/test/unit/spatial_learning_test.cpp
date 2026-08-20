#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <numbers>
#include <sstream>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <semaforr/spatial/learners/circumstance_learner.hpp>
#include <semaforr/spatial/learners/grid_learners.hpp>
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

class CountingReactive final : public semaforr::planning::ReactivePlanner {
 public:
  explicit CountingReactive(semaforr::domain::Action action)
      : action_(action) {}
  std::string_view name() const noexcept override { return "counting_reactive"; }
  std::vector<std::string_view> dependencies() const override { return {}; }
  semaforr::planning::TriggerEvaluation evaluateTrigger(
      const semaforr::decision::DecisionContext&) const override {
    ++trigger_count;
    return {triggered, triggered ? "active for ordering test"
                                 : "inactive for ordering test"};
  }
  semaforr::planning::ReactivePlanUpdate update(
      const semaforr::decision::DecisionContext&) override {
    ++update_count;
    semaforr::planning::ReactivePlanUpdate result;
    result.status = semaforr::planning::ReactiveStatus::Action;
    result.action = action_;
    result.explanation = "ordering test action";
    return result;
  }
  void cancel(semaforr::planning::InterruptionReason) override {
    ++cancel_count;
  }

  mutable std::size_t trigger_count = 0U;
  std::size_t update_count = 0U;
  std::size_t cancel_count = 0U;
  bool triggered = true;

 private:
  semaforr::domain::Action action_;
};

class PassiveReactive final : public semaforr::planning::ReactivePlanner {
 public:
  explicit PassiveReactive(std::string name) : name_(std::move(name)) {}
  std::string_view name() const noexcept override { return name_; }
  std::vector<std::string_view> dependencies() const override { return {}; }
  semaforr::planning::TriggerEvaluation evaluateTrigger(
      const semaforr::decision::DecisionContext&) const override {
    return {false, "ordering fixture inactive"};
  }
  semaforr::planning::ReactivePlanUpdate update(
      const semaforr::decision::DecisionContext&) override {
    return {};
  }
  void cancel(semaforr::planning::InterruptionReason) override {}

 private:
  std::string name_;
};

class TraceMandatory final : public semaforr::decision::MandatoryRule {
 public:
  explicit TraceMandatory(std::string name) : name_(std::move(name)) {}
  std::string_view name() const noexcept override { return name_; }
  std::optional<semaforr::decision::Decision> evaluate(
      const semaforr::decision::DecisionContext&) const override {
    return std::nullopt;
  }

 private:
  std::string name_;
};

class TraceVeto final : public semaforr::decision::VetoRule {
 public:
  explicit TraceVeto(std::string name) : name_(std::move(name)) {}
  std::string_view name() const noexcept override { return name_; }
  std::vector<semaforr::decision::Veto> evaluate(
      const semaforr::decision::DecisionContext&) const override {
    return {};
  }

 private:
  std::string name_;
};

class FixedPlanner final : public semaforr::planning::Planner {
 public:
  explicit FixedPlanner(bool succeeds) : succeeds_(succeeds) {}
  std::string_view name() const noexcept override { return "fixed_planner"; }
  semaforr::planning::PlanResult plan(
      const semaforr::planning::PlanningRequest& request) override {
    ++calls;
    if (!succeeds_)
      return {semaforr::planning::PlanStatus::NoPath, {}, 0.0, "no path"};
    return {semaforr::planning::PlanStatus::Success, {request.goal},
            semaforr::domain::distance(request.start.position, request.goal)
                .meters(),
            "test plan"};
  }

  std::size_t calls = 0U;

 private:
  bool succeeds_;
};

void completeSelectedAction(semaforr::decision::NavigationEngine& engine,
                            const semaforr::decision::DecisionResult& result,
                            const semaforr::domain::Pose2D& pose) {
  const auto now = std::chrono::steady_clock::now();
  ASSERT_EQ(engine.onActionStarted(
                {result.decision_id, result.action_id, now, pose}),
            semaforr::domain::FeedbackDisposition::Accepted);
  semaforr::domain::ActionExecutionResult execution;
  execution.decision_id = result.decision_id;
  execution.action_id = result.action_id;
  execution.task_id = semaforr::domain::TaskId{1U};
  execution.finished_at = now;
  execution.status = semaforr::domain::ExecutionCompletionStatus::Succeeded;
  execution.start_pose = pose;
  execution.final_pose = pose;
  ASSERT_EQ(engine.onActionCompleted(execution),
            semaforr::domain::FeedbackDisposition::Accepted);
}

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

  // Clearing a projection also releases its retained snapshot and revision so
  // re-enabling can project the learner's unchanged immutable publication.
  coordinator.setEnabled(SpatialRepresentation::Trails, true);
  coordinator.applyTo(projected);
  EXPECT_FALSE(projected.trails.empty());
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

TEST(SpatialLearning,
     EveryLearnerHasDeterministicSerializationAndNoOpRevisionBehavior) {
  using namespace semaforr::spatial;
  auto first = SpatialLearningCoordinator::defaults(100U);
  auto second = SpatialLearningCoordinator::defaults(100U);
  for (std::size_t index = 0U; index < 6U; ++index) {
    const auto value = episode(index + 1U, static_cast<double>(index) * 0.3,
                               index == 0U);
    first.observe(value);
    second.observe(value);
  }
  first.finalizeTarget();
  second.finalizeTarget();
  first.finalizeInitialExploration();
  second.finalizeInitialExploration();
  first.rebuildAll();
  second.rebuildAll();

  const auto first_models = first.snapshots();
  const auto second_models = second.snapshots();
  ASSERT_EQ(first_models.size(), 12U);
  ASSERT_EQ(second_models.size(), first_models.size());
  for (std::size_t index = 0U; index < first_models.size(); ++index) {
    SCOPED_TRACE(std::string(toString(first_models[index].representation)));
    EXPECT_EQ(first_models[index].representation,
              second_models[index].representation);
    EXPECT_EQ(first_models[index].revision, second_models[index].revision);
    EXPECT_EQ(first.serialize(first_models[index].representation),
              second.serialize(second_models[index].representation));
  }

  std::vector<std::size_t> revisions;
  std::vector<std::string> encodings;
  for (const auto& model : first_models) {
    revisions.push_back(model.revision);
    encodings.push_back(first.serialize(model.representation));
  }
  first.rebuildAll();
  const auto rebuilt = first.snapshots();
  ASSERT_EQ(rebuilt.size(), revisions.size());
  for (std::size_t index = 0U; index < rebuilt.size(); ++index) {
    SCOPED_TRACE(std::string(toString(rebuilt[index].representation)));
    EXPECT_EQ(rebuilt[index].revision, revisions[index]);
    EXPECT_EQ(first.serialize(rebuilt[index].representation),
              encodings[index]);
  }
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

TEST(InclusionGrid, RepresentsRegionsAndSupportingSubtrailsNotObservations) {
  using namespace semaforr;
  spatial::InclusionGridLearner learner(8U, 8U, 1.0, {-4.0, -4.0},
                                        spatial::GridExtentPolicy::Expand);
  spatial::RegionModel regions;
  domain::LearnedRegion region;
  region.id = 1U;
  region.boundary = {{0.0, 0.0}, domain::Distance(1.1)};
  regions.learned_regions.push_back(region);
  spatial::PassageSkeletonModel skeleton;
  domain::RegionSkeletonEdge edge;
  edge.from = 0U;
  edge.to = 1U;
  edge.supporting_subtrail = {{0.0, 0.0}, {3.0, 0.0}};
  skeleton.region_edges.push_back(edge);
  learner.replaceRepresented(regions, skeleton);
  const auto snapshot = learner.snapshot();
  const auto& inclusion =
      std::get<spatial::InclusionGridModel>(snapshot.payload);
  EXPECT_GT(inclusion.sparse_included.size(), 3U);
  const auto region_cell = inclusion.geometry.index({0.0, 0.0});
  const auto trail_cell = inclusion.geometry.index({2.5, 0.0});
  ASSERT_TRUE(region_cell);
  ASSERT_TRUE(trail_cell);
  EXPECT_TRUE(std::any_of(inclusion.sparse_included.begin(),
                          inclusion.sparse_included.end(),
                          [&](const auto& cell) {
                            return cell.index == *region_cell;
                          }));
  EXPECT_TRUE(std::any_of(inclusion.sparse_included.begin(),
                          inclusion.sparse_included.end(),
                          [&](const auto& cell) {
                            return cell.index == *trail_cell;
                          }));
}

TEST(InclusionGrid, AddsOnlySuccessfulLowLevelExplorationTranslation) {
  using namespace semaforr;
  spatial::InclusionGridLearner learner(8U, 8U, 1.0, {-4.0, -4.0});
  auto failed = episode(1U, 0.0);
  failed.selection = domain::SelectedActionRecord{};
  failed.selection->provenance = "reactive:LLE:no_plan_available";
  failed.execution_result = domain::ActionExecutionResult{};
  failed.execution_result->status =
      domain::ExecutionCompletionStatus::ControllerFailure;
  failed.execution_result->start_pose = failed.observation.pose;
  failed.execution_result->final_pose =
      {{2.0, 0.0}, domain::Angle::zero()};
  learner.observe(failed);
  EXPECT_FALSE(learner.snapshot().usable());

  auto succeeded = episode(2U, 0.0);
  succeeded.selection = domain::SelectedActionRecord{};
  succeeded.selection->provenance = "reactive:LLE:no_plan_available";
  succeeded.execution_result = domain::ActionExecutionResult{};
  succeeded.execution_result->status =
      domain::ExecutionCompletionStatus::Succeeded;
  succeeded.execution_result->start_pose = succeeded.observation.pose;
  succeeded.execution_result->final_pose =
      {{2.0, 0.0}, domain::Angle::zero()};
  learner.observe(succeeded);
  EXPECT_FALSE(
      std::get<spatial::InclusionGridModel>(learner.snapshot().payload)
          .sparse_included.empty());
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

TEST(NavigationEngine, VictoryPrecedesLowLevelExplorationWhenNoPlanExists) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {2.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2, 1.0}, {0.5});
  decision::DecisionCoordinator decisions;
  decisions.addMandatoryRule(std::make_unique<decision::VictoryRule>(
      domain::Distance(0.2), action_space));
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  decision::NavigationEngine engine(world, action_space, decisions, mission,
                                    planning, learning);
  auto input = episode(1U, 0.0, true).observation;
  input.laser.ranges_m = {5.0, 5.0, 2.0, 5.0, 5.0};
  const auto result = engine.decide(input);
  EXPECT_EQ(result.selected_policy, "mandatory_rule:Victory");
  EXPECT_EQ(result.selected_policy.find("LLE"), std::string::npos);
}

TEST(NavigationEngine, VictoryStopsBeforeEnforcerAndReactivePlanners) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {2.0, 0.0}}}, 10U);
  ASSERT_TRUE(world.mission.activate_next());
  world.mission.install_active_plan({{1.0, 0.0}});
  const domain::ActionSpace action_space({0.2, 1.0}, {0.5});
  decision::DecisionCoordinator decisions;
  decisions.addMandatoryRule(std::make_unique<decision::VictoryRule>(
      domain::Distance(0.2), action_space));
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  auto reactive = std::make_unique<CountingReactive>(
      domain::Action(domain::ActionType::TurnLeft, 1U));
  CountingReactive* reactive_observer = reactive.get();
  std::vector<std::unique_ptr<planning::ReactivePlanner>> reactives;
  reactives.push_back(std::move(reactive));
  decision::NavigationEngine engine(
      world, action_space, decisions, mission, planning, learning, nullptr,
      domain::Distance(0.2), nullptr, nullptr, {}, {}, std::move(reactives),
      true, true);
  auto input = episode(1U, 0.0, true).observation;
  input.laser.ranges_m = {5.0, 5.0, 2.0, 5.0, 5.0};

  const auto result = engine.decide(input);
  EXPECT_EQ(result.selected_policy, "mandatory_rule:Victory");
  EXPECT_EQ(reactive_observer->trigger_count, 0U);
  EXPECT_EQ(reactive_observer->update_count, 0U);
  ASSERT_FALSE(result.decision_cycle.empty());
  EXPECT_EQ(result.decision_cycle.front().component, "Victory");
  EXPECT_EQ(result.decision_cycle.front().final_attribution,
            decision::DecisionTier::TierOne);
}

TEST(NavigationEngine, CognitiveTraceFollowsSemanticOrderBeforeTierThree) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {4.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2, 1.0}, {0.5});
  decision::DecisionCoordinator decisions;
  decisions.addMandatoryRule(std::make_unique<TraceMandatory>("Victory"));
  decisions.addVetoRule(std::make_unique<TraceVeto>("AvoidObstacles"));
  decisions.addVetoRule(std::make_unique<TraceVeto>("NotOpposite"));
  decisions.addVetoRule(std::make_unique<TraceVeto>("Forward"));
  decisions.addVetoRule(std::make_unique<TraceVeto>("Precedent"));
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  std::vector<std::unique_ptr<planning::ReactivePlanner>> reactives;
  reactives.push_back(std::make_unique<PassiveReactive>("Thru"));
  reactives.push_back(std::make_unique<PassiveReactive>("Behind"));
  reactives.push_back(std::make_unique<PassiveReactive>("Out"));
  decision::NavigationEngine engine(
      world, action_space, decisions, mission, planning, learning, nullptr,
      domain::Distance(0.2), nullptr, nullptr, {}, {}, std::move(reactives),
      true, true);
  auto input = episode(1U, 0.0, true).observation;
  input.laser.ranges_m.assign(input.laser.ranges_m.size(), 1.0);

  const auto result = engine.decide(input);
  std::vector<std::string> components;
  for (const auto& event : result.decision_cycle)
    components.push_back(event.component);
  const std::vector<std::string> expected{
      "Victory",          "AvoidObstacles", "NotOpposite",
      "Enforcer",         "Thru",           "Behind",
      "Out",              "LLE",            "Forward",
      "Precedent",        "PlanningCoordinator",
      "tier3_fallback"};
  EXPECT_EQ(components, expected);
  EXPECT_EQ(result.selected_policy, "no_advisor_score");
}

TEST(NavigationEngine, ReactiveMandateStopsLleLateVetoesAndLowerTiers) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {4.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2, 1.0}, {0.5});
  decision::DecisionCoordinator decisions;
  decisions.addVetoRule(std::make_unique<TraceVeto>("Forward"));
  decisions.addVetoRule(std::make_unique<TraceVeto>("Precedent"));
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  auto reactive = std::make_unique<CountingReactive>(
      domain::Action(domain::ActionType::TurnLeft, 1U));
  CountingReactive* observer = reactive.get();
  std::vector<std::unique_ptr<planning::ReactivePlanner>> reactives;
  reactives.push_back(std::move(reactive));
  decision::NavigationEngine engine(
      world, action_space, decisions, mission, planning, learning, nullptr,
      domain::Distance(0.2), nullptr, nullptr, {}, {}, std::move(reactives),
      true, true);
  auto input = episode(1U, 0.0, true).observation;

  const auto result = engine.decide(input);
  EXPECT_EQ(result.selected_policy, "reactive:counting_reactive");
  EXPECT_EQ(observer->trigger_count, 1U);
  EXPECT_EQ(observer->update_count, 1U);
  EXPECT_TRUE(std::none_of(
      result.decision_cycle.begin(), result.decision_cycle.end(),
      [](const auto& event) {
        return event.component == "LLE" || event.component == "Forward" ||
               event.component == "Precedent" || event.tier == "tier2" ||
               event.tier == "tier3";
      }));
}

TEST(NavigationEngine, TierTwoPlanCreationEndsCycleBeforeEnforcer) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {4.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2, 1.0}, {0.5});
  decision::DecisionCoordinator decisions;
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  auto planner = std::make_unique<FixedPlanner>(true);
  FixedPlanner* planner_observer = planner.get();
  planning.registerPlanner(std::move(planner));
  spatial::SpatialLearningCoordinator learning(100U);
  auto reactive = std::make_unique<CountingReactive>(
      domain::Action(domain::ActionType::TurnLeft, 1U));
  CountingReactive* reactive_observer = reactive.get();
  reactive_observer->triggered = false;
  std::vector<std::unique_ptr<planning::ReactivePlanner>> reactives;
  reactives.push_back(std::move(reactive));
  decision::NavigationEngine engine(
      world, action_space, decisions, mission, planning, learning, nullptr,
      domain::Distance(0.2), nullptr, nullptr, {}, {}, std::move(reactives),
      true, true);
  auto input = episode(1U, 0.0, true).observation;
  input.laser.ranges_m.assign(input.laser.ranges_m.size(), 1.0);

  const auto result = engine.decide(input);
  EXPECT_EQ(planner_observer->calls, 1U);
  EXPECT_EQ(result.selected_policy, "tier2:plan_created_cycle_end");
  EXPECT_EQ(result.tier, decision::DecisionTier::TierTwo);
  EXPECT_EQ(reactive_observer->trigger_count, 1U);
  EXPECT_EQ(reactive_observer->update_count, 0U);
  const auto tier_two = std::find_if(
      result.decision_cycle.begin(), result.decision_cycle.end(),
      [](const auto& event) { return event.tier == "tier2"; });
  const auto enforcer = std::find_if(
      result.decision_cycle.begin(), result.decision_cycle.end(),
      [](const auto& event) { return event.component == "Enforcer"; });
  ASSERT_NE(tier_two, result.decision_cycle.end());
  ASSERT_NE(enforcer, result.decision_cycle.end());
  EXPECT_EQ(enforcer->outcome, "no_active_plan_continue");
  EXPECT_LT(enforcer->order, tier_two->order);
  EXPECT_TRUE(std::none_of(
      std::next(tier_two), result.decision_cycle.end(),
      [](const auto& event) { return event.component == "Enforcer"; }));
  EXPECT_TRUE(std::none_of(
      std::next(tier_two), result.decision_cycle.end(),
      [](const auto& event) { return event.tier == "tier3"; }));
  EXPECT_EQ(tier_two->outcome.find("plan_created_cycle_end"), 0U);
  EXPECT_NE(tier_two->outcome.find("attempt=1"), std::string::npos);

  completeSelectedAction(engine, result, input.pose);
  reactive_observer->triggered = true;
  const auto next = engine.decide(input);
  EXPECT_EQ(planner_observer->calls, 1U);
  EXPECT_EQ(next.selected_policy, "mandatory_rule:Enforcer");
  EXPECT_EQ(next.tier, decision::DecisionTier::TierOne);
  EXPECT_EQ(reactive_observer->trigger_count, 1U);
  const auto next_enforcer = std::find_if(
      next.decision_cycle.begin(), next.decision_cycle.end(),
      [](const auto& event) { return event.component == "Enforcer"; });
  ASSERT_NE(next_enforcer, next.decision_cycle.end());
  EXPECT_EQ(next_enforcer->outcome, "legacy_custom_plan_action_selected");
}

TEST(NavigationEngine, RepeatedImmediateTierTwoFailureIsBounded) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {4.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2, 1.0}, {0.5});
  decision::DecisionCoordinator decisions;
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  auto planner = std::make_unique<FixedPlanner>(false);
  FixedPlanner* planner_observer = planner.get();
  planning.registerPlanner(std::move(planner));
  spatial::SpatialLearningCoordinator learning(100U);
  decision::NavigationEngine engine(
      world, action_space, decisions, mission, planning, learning, nullptr,
      domain::Distance(0.2), nullptr, nullptr, {}, {}, {}, false, true, {},
      nullptr, nullptr, {}, 2U);
  auto input = episode(1U, 0.0, true).observation;
  input.laser.ranges_m.assign(input.laser.ranges_m.size(), 1.0);

  const auto first = engine.decide(input);
  EXPECT_FALSE(world.recovery.plan_abandoned);
  const auto first_failure = std::find_if(
      first.decision_cycle.begin(), first.decision_cycle.end(),
      [](const auto& event) {
        return event.outcome.find("no_valid_plan_tier3_eligible") == 0U;
      });
  const auto first_tier_three = std::find_if(
      first.decision_cycle.begin(), first.decision_cycle.end(),
      [](const auto& event) { return event.tier == "tier3"; });
  ASSERT_NE(first_failure, first.decision_cycle.end());
  ASSERT_NE(first_tier_three, first.decision_cycle.end());
  EXPECT_LT(first_failure->order, first_tier_three->order);
  completeSelectedAction(engine, first, input.pose);
  const auto second = engine.decide(input);
  EXPECT_TRUE(world.recovery.plan_abandoned);
  EXPECT_EQ(world.recovery.tier_two_attempts, 2U);
  // The second Tier-2 attempt reuses the revision-keyed NoPath result; the
  // attempt counter still advances even though the planner body need not run.
  EXPECT_EQ(planner_observer->calls, 1U);
  const auto failure = std::find_if(
      second.decision_cycle.begin(), second.decision_cycle.end(),
      [](const auto& event) {
        return event.outcome.find("no_valid_plan_abandoned_tier3_eligible:attempt=2") !=
               std::string::npos;
      });
  ASSERT_NE(failure, second.decision_cycle.end());
  completeSelectedAction(engine, second, input.pose);
  const auto third = engine.decide(input);
  EXPECT_EQ(planner_observer->calls, 1U);
  EXPECT_TRUE(std::any_of(
      third.decision_cycle.begin(), third.decision_cycle.end(),
      [](const auto& event) {
        return event.component == "LLE";
      }));
  EXPECT_TRUE(std::none_of(
      third.decision_cycle.begin(), third.decision_cycle.end(),
      [](const auto& event) {
        return event.tier == "tier2" &&
               event.outcome.find("attempt=3") != std::string::npos;
      }));
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
  configuration.minimum_action_evidence = 2U;
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
    input.event = LearningEvent::SensorObservation;
    input.sequence = index * 3U + 1U;
    learner.observe(input);
    input.selection = domain::SelectedActionRecord{
        index + 1U, index + 1U, domain::TaskId{1U}, {},
        input.observation.pose, *input.selected_action, "tier_three", "test",
        0.25, 0.0, 0.0};
    input.event = LearningEvent::DecisionSelected;
    input.sequence = index * 3U + 2U;
    learner.observe(input);
    input.event = LearningEvent::ActionTerminal;
    input.sequence = index * 3U + 3U;
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
  ASSERT_EQ(model.cases.front().actions.size(), 1U);
  EXPECT_EQ(model.cases.front().actions.front().successful, 4U);
  EXPECT_EQ(model.cases.front().actions.front().failed, 0U);
  EXPECT_EQ(model.cases.front().actions.front().last_outcome,
            domain::CaseOutcome::Successful);
  EXPECT_EQ(model.learning_mode,
            domain::CircumstanceLearningMode::AdaptedThreshold);
  EXPECT_EQ(model.similarity_metric, "normalized_l1");
}

TEST(CircumstanceLearning, UsesOnlyTerminalOutcomesAndDistinguishesResults) {
  using namespace semaforr;
  using namespace semaforr::spatial;
  CircumstanceLearningConfiguration configuration;
  configuration.minimum_cluster_size = 1U;
  configuration.reclustering_threshold = 1U;
  configuration.assignment_confidence_threshold = 0.5;
  configuration.minimum_case_evidence = 1U;
  configuration.minimum_action_evidence = 1U;
  CircumstanceLearner learner(configuration);

  auto input = episode(1U, 0.0, true);
  input.active_target = domain::Point2D{4.0, 0.0};
  input.event = LearningEvent::SensorObservation;
  input.sequence = 1U;
  learner.observe(input);
  input.selection = domain::SelectedActionRecord{
      1U, 1U, domain::TaskId{1U}, {}, input.observation.pose,
      *input.selected_action, "tier_three", "test", 0.25, 0.0, 0.0};
  input.event = LearningEvent::DecisionSelected;
  input.sequence = 2U;
  learner.observe(input);
  learner.rebuild();
  EXPECT_TRUE(std::get<CircumstanceModel>(learner.snapshot().payload)
                  .cases.empty());

  input.execution_result->status =
      domain::ExecutionCompletionStatus::PartialMovement;
  input.event = LearningEvent::ActionTerminal;
  input.sequence = 3U;
  learner.observe(input);
  input.sequence = 4U;
  learner.observe(input);  // duplicate terminal feedback is idempotent
  learner.rebuild();
  const auto& model =
      std::get<CircumstanceModel>(learner.snapshot().payload);
  ASSERT_EQ(model.cases.size(), 1U);
  ASSERT_EQ(model.cases.front().actions.size(), 1U);
  const auto& action = model.cases.front().actions.front();
  EXPECT_EQ(action.selected, 1U);
  EXPECT_EQ(action.executed, 1U);
  EXPECT_EQ(action.partial, 1U);
  EXPECT_DOUBLE_EQ(action.success_credit, 0.5);
  EXPECT_EQ(action.successful, 0U);
}

TEST(CircumstanceLearning, PersistencePreservesStableIdsCasesAndVersions) {
  using namespace semaforr::domain;
  CircumstanceModel model;
  model.learning_mode = CircumstanceLearningMode::DissertationCompatible;
  model.model_version = "circumstance_case_v2";
  model.classifier_version = "centroid_softmax_v1";
  model.feature_version = "robot_centered_heading_normalized_freespace_v1";
  NormalizedSetting setting;
  setting.side_cells = 1U;
  setting.freespace = {1.0};
  model.clusters.push_back(
      {42U, setting, 75U, 0.99,
       CircumstanceCreationMethod::OfflineSimilarityGraph, 3U, 100U, 2U,
       false});
  model.next_circumstance_id = 43U;
  CircumstanceCaseEvidence evidence;
  evidence.key = {42U, 1U, 2U};
  evidence.evidence = 12U;
  evidence.accuracy = 0.8;
  ActionCaseEvidence action;
  action.action = Action(ActionType::Forward, 1U);
  action.selected = 12U;
  action.executed = 12U;
  action.successful = 10U;
  action.failed = 2U;
  action.effective_evidence = 12.0;
  action.success_credit = 10.0;
  action.confidence = 11.0 / 14.0;
  action.accuracy = 10.0 / 12.0;
  action.last_outcome = CaseOutcome::Successful;
  evidence.actions.push_back(action);
  model.cases.push_back(evidence);
  model.migrations.push_back({7U, 42U, "merge", 12U, 4U});

  std::stringstream encoded;
  saveCircumstanceModel(model, encoded);
  const auto restored = loadCircumstanceModel(
      encoded, model.model_version, model.feature_version,
      model.classifier_version);
  ASSERT_EQ(restored.clusters.size(), 1U);
  EXPECT_EQ(restored.clusters.front().id, 42U);
  EXPECT_EQ(restored.next_circumstance_id, 43U);
  ASSERT_EQ(restored.cases.size(), 1U);
  ASSERT_EQ(restored.cases.front().actions.size(), 1U);
  EXPECT_EQ(restored.cases.front().actions.front().successful, 10U);
  ASSERT_EQ(restored.migrations.size(), 1U);
  EXPECT_EQ(restored.migrations.front().previous_id, 7U);

  std::stringstream incompatible(encoded.str());
  EXPECT_THROW(loadCircumstanceModel(incompatible, "different_model"),
               std::runtime_error);
}
