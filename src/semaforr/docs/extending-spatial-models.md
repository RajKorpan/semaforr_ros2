<!-- File overview: This guide explains the complete extension path for adding a sensor-derived spatial representation to SemaFORR, including domain types, learner lifecycle, immutable publication, world-model projection, configuration, dependency revisions, consumers, serialization, and tests. Its package-relative location is `docs/extending-spatial-models.md`. -->

# Implementing a new spatial model

Use this path for knowledge learned from robot observations or execution history: a new affordance, graph, sparse grid, geometric entity set, or other persistent spatial representation. Static-map priors belong in `domain::StaticMap`; current sensor evidence belongs in observations or occupancy layers; a learned model belongs in the spatial-learning pipeline described here.

## Architecture and ownership

The mutable learner is owned by `spatial::SpatialLearningCoordinator`. It receives `spatial::NavigationEpisode` events, maintains private construction state, and publishes an immutable `SpatialModelUpdate`. `SpatialLearningCoordinator::applyTo()` projects a changed snapshot into `domain::WorldModel::spatial`. Planners and advisors read the projected model or its shared snapshot; they do not mutate the learner.

```text
Robot observation / execution feedback
  -> NavigationEpisode
  -> SpatialLearningCoordinator
  -> SpatialLearner implementation
  -> immutable SpatialModelUpdate + RepresentationChangeSet
  -> WorldModel::spatial + exact ModelDependency revision
  -> planners, advisors, replay, diagnostics, visualization
```

Authoritative extension points:

- Representation types: `include/semaforr/spatial/representations/`
- Shared representation variant and contracts: `include/semaforr/spatial/learner.hpp`
- Learner interfaces: `include/semaforr/spatial/learner.hpp` and `learner_base.hpp`
- Learner declarations: `include/semaforr/spatial/learners/`
- Learner implementations: `src/spatial/`
- Default construction and projection: `src/spatial/spatial_learning_coordinator.cpp`
- Read-only runtime storage: `include/semaforr/domain/world_model.hpp`
- Exact dependency identifiers: `include/semaforr/domain/model_revision.hpp`
- Configuration: `include/semaforr/config/navigation_configuration.hpp`, `src/config/navigation_configuration.cpp`, and `config/semaforr.yaml`

## Step 1: define semantics before storage

Write down these facts first:

1. What evidence the model represents and what it must not be confused with.
2. Whether its storage is a graph, entity collection, polygon set, dense grid, sparse grid, or tiled grid.
3. Its coordinate frame, geometry, resolution, expansion policy, and provenance.
4. Which event schedule updates it.
5. Which failed or partial execution outcomes are admissible evidence.
6. Which consumers require it and which can operate without it.
7. What constitutes a meaningful mutation and therefore a revision increment.

Do not reuse familiarity, occupancy, inclusion, or static-map fields merely because they have convenient geometry. Add a semantically distinct type.

## Step 2: add the representation type

Create `include/semaforr/spatial/representations/<model>_model.hpp` for the public model and add its implementation to `representations/models.hpp` when appropriate. Prefer immutable value types with stable entity IDs. Grid-backed types must use `domain::GridGeometry`; do not add zero-origin or one-metre assumptions.

Then update:

- `SpatialRepresentation` in `spatial/learner.hpp`.
- `SpatialPayload` in the same file.
- `toString(SpatialRepresentation)` in `src/spatial/learner.cpp`.
- `domain::SpatialModel` in `domain/world_model.hpp` with the consumer-facing snapshot or read-only projection.
- `domain::ModelDependency` and its string conversion for exact revision tracking.

If the model has large storage, keep the published payload sparse and use `SharedSpatialSnapshot`. Do not materialize a dense projection unless a particular consumer requests it.

## Step 3: implement the learner

For the normal lifecycle, derive from `SpatialLearnerBase`:

```cpp
class ExampleLearner final : public SpatialLearnerBase {
 public:
  ExampleLearner()
      : SpatialLearnerBase(
            SpatialRepresentation::Example,
            "example_learner",
            UpdateMode::Incremental,
            ObservationContract{
                .pose = true,
                .laser = true,
                .selected_action = false,
                .task_boundaries = false,
                .update_trigger = "every valid sensor observation",
                .consumers = {"example_advisor"},
                .schedule = UpdateSchedule::EveryObservation}) {}

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
};
```

`SpatialLearnerBase` stores accepted episodes and handles snapshot metadata. Your implementation owns only construction state and calls `publish(payload, status, diagnostic)` when content actually changes. Use `markIncomplete()` when evidence is insufficient.

Choose the schedule deliberately:

| Evidence | Typical schedule |
| --- | --- |
| Raw pose/laser evidence | `EveryObservation` |
| Decision-point evidence | `EveryDecisionCycle` |
| Confirmed successful traversal | `AfterSuccessfulActionCompletion` |
| Failures and successes | `AfterAnyTerminalActionResult` |
| Reconciliation across one target | `EndOfTarget` |
| Initial-model finalization | `EndOfInitialExploration` |
| Expensive derived model | `OnDemand` with `RebuildOnDemand` |

Never treat action selection as action completion. For traversal evidence, require `episode.execution_result`, inspect its completion status, and use its actual final pose. Partial motion may contribute only through the reached geometry. A failed action with no movement must not create connectivity.

## Step 4: register and project the model

Add the learner to `SpatialLearningCoordinator::defaults()` in `src/spatial/spatial_learning_coordinator.cpp`. `addLearner()` rejects duplicate names and duplicate `SpatialRepresentation` values.

Extend every exhaustive coordinator switch:

- `dependencyFor()` maps the representation to its exact `ModelDependency`.
- `clearRepresentation()` removes only that model and its snapshot handle.
- `estimateProjection()` accounts for entities, cells, allocations, copied bytes, and shared bytes.
- `applyTo()` copies or shares the payload into the correct `SpatialModel` field only when its revision changes.

If the model contributes multiple independently relevant products, use distinct dependencies. A planner consuming only the graph should not be invalidated by unrelated diagnostic metadata.

## Step 5: add configuration and validation

Add a normal configuration field under the spatial-representation settings, load it in `navigation_configuration.cpp`, provide an executable default in `config/semaforr.yaml`, include it in fingerprints/manifests, and enable or disable the learner through `SpatialLearningCoordinator::setEnabled()`.

Validation must reject impossible combinations with an actionable message. Examples:

- An advisor requires the representation while its learner and loaded snapshot are both disabled.
- A grid model has an invalid resolution or expansion limit.
- A compatibility profile selects a modernized-only learning policy.

Do not create a separate execution path for an ablation profile. Profiles expand into ordinary validated component settings.

## Step 6: expose read-only access to consumers

Consumers normally receive `const domain::WorldModel&` through `DecisionContext` or `PlanningRequest`:

```cpp
const auto& model = context.world.spatial;
const auto revision = model.revisionOf(domain::ModelDependency::Example);
```

An advisor must declare the required representation in `AdvisorMetadata`. A planner must return the exact dependency from `Planner::dependencies()` and call `attachDependencySnapshot()` on successful output. Optional consumers must test availability and decline participation rather than interpreting an empty structure as valid evidence.

## Step 7: serialization, diagnostics, replay, and visualization

Add schema-versioned serialization when the representation survives restart. Persist geometry, coordinate frame, provenance, semantic version, and representation revision. Loading identical content must not fabricate a mutation.

For replay, record the input event and resulting revision/change set. For visualization, publish from the read-only `WorldModel` projection; visualization must never become the owner of learned state.

## Step 8: tests and completion checklist

Add unit tests under `test/unit/` for:

- Empty and minimum evidence.
- Accepted and rejected event schedules.
- Successful, partial, failed, cancelled, and timed-out actions where relevant.
- Meaningful mutation increments exactly one local revision.
- Rewriting identical content does not increment a revision.
- Immutable snapshot reuse and change-set accuracy.
- Negative coordinates and dynamic expansion for grids.
- Serialization round trip.
- Projection into `WorldModel::spatial` and consumer access.
- Configuration enablement, ablation, and invalid dependency combinations.

Add integration tests when a planner, advisor, HLE, LLE, or Enforcer consumes the model. Update `docs/spatial-learning.md`, the representation catalog, compatibility matrix, and documentation index.

The implementation is complete only when the learner is registered, configuration-gated, projected, revisioned, documented, replayable, and tested. A type that exists only in a header is not an operational spatial model.
