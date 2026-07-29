#include <gtest/gtest.h>

#include <algorithm>
#include <numbers>
#include <string>
#include <variant>
#include <vector>

#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>

namespace {

semaforr::spatial::NavigationEpisode episode(
  std::size_t sequence,
  double x_m,
  bool task_started = false)
{
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
  return {
    sequence,
    std::move(observation),
    domain::Action(domain::ActionType::Forward, 1U),
    domain::TaskId{1U},
    task_started,
    false};
}

class CapturingLearner final : public semaforr::spatial::SpatialLearner {
public:
  void observe(
    const semaforr::spatial::NavigationEpisode& episode) override
  {
    captured = episode;
    ++update.observed_episodes;
    update.status = semaforr::spatial::ModelStatus::Fresh;
  }

  void rebuild() override { ++update.revision; }
  semaforr::spatial::SpatialModelUpdate snapshot() const override
  {
    return update;
  }
  semaforr::spatial::SpatialRepresentation representation() const
    noexcept override
  {
    return semaforr::spatial::SpatialRepresentation::Trails;
  }
  std::string_view name() const noexcept override { return "capture"; }
  const semaforr::spatial::ObservationContract& contract() const
    noexcept override
  {
    return observation_contract;
  }

  std::optional<semaforr::spatial::NavigationEpisode> captured;
  semaforr::spatial::SpatialModelUpdate update = [] {
    semaforr::spatial::SpatialModelUpdate value;
    value.representation =
      semaforr::spatial::SpatialRepresentation::Trails;
    value.learner = "capture";
    return value;
  }();
  semaforr::spatial::ObservationContract observation_contract{
    true, true, true, true, "after decision", {"test"}};
};

}  // namespace

TEST(SpatialLearning, DefaultModulesDeclareLifecycleAndConsumers)
{
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);

  EXPECT_EQ(coordinator.learnerCount(), 7U);
  EXPECT_EQ(coordinator.enabledCount(), 7U);
  const auto inspection = coordinator.inspect();
  ASSERT_EQ(inspection.size(), 7U);
  for (const LearnerInspection& learner : inspection) {
    EXPECT_FALSE(learner.name.empty());
    EXPECT_FALSE(learner.contract.update_trigger.empty());
    EXPECT_FALSE(learner.contract.consumers.empty());
    EXPECT_EQ(learner.update.status, ModelStatus::Empty);
  }
}

TEST(SpatialLearning, LearnersCanBeEnabledAndObservedIndependently)
{
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.setEnabled(SpatialRepresentation::Hallways, false);

  coordinator.observe(episode(1U, 0.0, true));
  coordinator.observe(episode(2U, 0.3));

  ASSERT_TRUE(coordinator.snapshot(SpatialRepresentation::Trails));
  EXPECT_EQ(
    coordinator.snapshot(SpatialRepresentation::Trails)->status,
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

TEST(SpatialLearning, DisablingALearnerRemovesOnlyItsProjectedModel)
{
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

TEST(SpatialLearning, AutomaticRebuildUsesAcceptedEpisodeCount)
{
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(2U);

  coordinator.observe(episode(7U, 0.0, true));
  EXPECT_EQ(
    coordinator.snapshot(SpatialRepresentation::Regions)->revision, 0U);

  coordinator.observe(episode(8U, 0.3));
  EXPECT_GT(
    coordinator.snapshot(SpatialRepresentation::Regions)->revision, 0U);
}

TEST(SpatialLearning, DeferredModelsExposeStalenessAndPreserveLastFreshModel)
{
  using namespace semaforr;
  using namespace semaforr::spatial;
  auto coordinator = SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0, true));
  coordinator.observe(episode(2U, 0.3));
  coordinator.observe(episode(3U, 0.6));

  EXPECT_EQ(
    coordinator.snapshot(SpatialRepresentation::Regions)->status,
    ModelStatus::Incomplete);
  coordinator.rebuild(SpatialRepresentation::Regions);
  const auto fresh = coordinator.snapshot(SpatialRepresentation::Regions);
  ASSERT_TRUE(fresh);
  ASSERT_EQ(fresh->status, ModelStatus::Fresh);

  domain::SpatialModel projected;
  coordinator.applyTo(projected);
  ASSERT_FALSE(projected.learned_regions.empty());
  const auto region_count = projected.learned_regions.size();

  coordinator.observe(episode(4U, 0.9));
  EXPECT_EQ(
    coordinator.snapshot(SpatialRepresentation::Regions)->status,
    ModelStatus::Stale);
  coordinator.applyTo(projected);
  EXPECT_EQ(projected.learned_regions.size(), region_count);

  coordinator.rebuildStale();
  EXPECT_EQ(
    coordinator.snapshot(SpatialRepresentation::Regions)->status,
    ModelStatus::Fresh);
}

TEST(SpatialLearning, EveryRepresentationRebuildsAndSerializesIndependently)
{
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
    const std::string encoded =
      coordinator.serialize(update.representation);
    EXPECT_NE(
      encoded.find(
        "\"representation\":\"" +
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
    coordinator.snapshot(
      SpatialRepresentation::PassagesAndSkeleton)->payload));
}

TEST(SpatialLearning, EpisodeOrderingIsValidatedPerLearner)
{
  auto coordinator =
    semaforr::spatial::SpatialLearningCoordinator::defaults(100U);
  coordinator.observe(episode(1U, 0.0));
  EXPECT_THROW(
    coordinator.observe(episode(1U, 0.2)),
    std::invalid_argument);
}

TEST(SpatialLearning, NavigationEngineObservesCompletePostDecisionEpisodes)
{
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {2.0, 0.0}}}, 10U);
  const domain::ActionSpace action_space({0.2}, {0.5});
  decision::DecisionCoordinator decisions({
    1.0e-9,
    decision::UnscoredActionPolicy::Exclude,
    0.0,
    domain::Action(domain::ActionType::Forward, 1U),
    0U});
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  auto learner = std::make_unique<CapturingLearner>();
  CapturingLearner* capture = learner.get();
  learning.addLearner(std::move(learner));
  decision::NavigationEngine engine(
    world, action_space, decisions, mission, planning, learning);

  const auto input = episode(1U, 0.0, true).observation;
  const decision::DecisionResult result = engine.decide(input);

  ASSERT_TRUE(capture->captured);
  ASSERT_TRUE(capture->captured->selected_action);
  EXPECT_EQ(*capture->captured->selected_action, result.action);
  EXPECT_TRUE(capture->captured->task_started);
  EXPECT_EQ(capture->captured->active_task, domain::TaskId{1U});
  EXPECT_EQ(world.navigation_history.entries().size(), 1U);
}
