<!-- File overview: This guide explains how to add or extend a SemaFORR-owned crowd model learned from validated social observations, including ROS-boundary adaptation, mutable learning, immutable snapshots, revisions, runtime installation, consumers, visualization, replay, and tests. Its package-relative location is `docs/extending-crowd-models.md`. -->

# Implementing a new crowd model

Crowd knowledge is a SemaFORR-learned representation, not an upstream social-context message and not a separate ROS package. Upstream packages provide observations. SemaFORR validates and normalizes them into domain types, learns crowd models, stores the current and learned snapshots in `WorldModel::crowd`, and lets planners and advisors consume them directly.

## Existing ownership boundary

```text
social_context_msgs or hunav_msgs
  -> ROS adapters and SocialObservationBuffer
  -> domain::CrowdObservation
  -> synchronized RobotObservation
  -> social::CrowdFieldLearner (mutable evidence)
  -> domain::CrowdFieldSnapshot (immutable value snapshot)
  -> domain::CrowdModel in WorldModel::crowd
  -> planners, advisors, replay, Why, visualization
```

The current authoritative files are:

- Live social domain types: `include/semaforr/domain/social.hpp`
- Learned crowd domain types and accessors: `include/semaforr/domain/crowd_model.hpp`
- Learner API/state: `include/semaforr/social/crowd_field_learner.hpp`
- Learner implementation: `src/social/crowd_field_learner.cpp`
- ROS message conversion: `include/semaforr/ros/message_adapters.hpp` and `src/ros/message_adapters.cpp`
- Prediction/formation buffering: `include/semaforr/ros/social_observation_buffer.hpp` and its source
- Runtime synchronization and engine handoff: `src/ros/semaforr_node_component.cpp` and `src/ros/navigation_engine_adapter.cpp`
- Runtime owner: `domain::WorldModel::crowd`
- Consumers: `decision/advisors/social/` and crowd objectives in `planning/`
- Visualization: `src/ros/visualization_publisher.cpp`

Do not add a learned result to `social_context_msgs`. Do not publish an intermediate crowd-model ROS message merely so SemaFORR can subscribe to its own learned state.

## Choose whether this is a field, entity model, or live feature

Add a learned model only when evidence is accumulated across observations. A calculation based solely on the current `CrowdObservation` can remain an advisor-local live feature. Persistent learned products should define:

- Evidence source and required freshness.
- Geometry and coordinate frame.
- Update frequency and meaningful-change policy.
- Confidence, decay, staleness, and conflict rules.
- Separate revision dependency.
- Immutable snapshot format and serialization.

If density, encounter risk, and flow can change independently for planning purposes, preserve their distinct `ModelDependency` revisions even if one learner updates them together.

## Step 1: extend domain types

For additional current evidence, extend `PedestrianObservation`, `FormationObservation`, or `CrowdObservation` in `domain/social.hpp`. Keep ROS types out of the domain header. Add validation for finite values, IDs, timestamp ordering, covariance, frame consistency, and cross-references.

For a learned product, add a value snapshot in `domain/crowd_model.hpp`. It should expose:

- `validate()` for structural invariants.
- `available()` to distinguish valid learned content from an empty default.
- Point/entity query methods that are safe on unavailable data.
- `save()` and `load()` when persistence is supported.
- Content equality so identical publication can be detected.

Store the snapshot in `CrowdModel`, add narrowly scoped const accessors, and add the corresponding `ModelDependency` value. Avoid returning mutable references to learned snapshots.

## Step 2: adapt upstream evidence only at the ROS boundary

If the model needs an upstream field not already normalized, modify SemaFORR's adapter—not `social_context`, `social_context_msgs`, `semaforr_bridge`, or `hunav_msgs`.

Tracked mode enters through `trackedPeopleToDomain()`. HuNav mode enters through `hunavAgentsToDomain()`. Formation messages enter through `formationsToDomain()`. The `SocialObservationBuffer` associates predictions and formations, rejects malformed/stale data, and produces a coherent `CrowdObservation`.

Preserve:

- Canonical person IDs.
- Source and prediction provenance.
- Source timestamp and configured coordinate frame.
- Independent current-state and prediction freshness.
- Valid empty observations as negative evidence.

Missing data is not an empty crowd. Do not update a learner when no valid observation was received.

## Step 3: implement mutable learning

Follow `CrowdFieldLearner` as the reference lifecycle. The learner receives synchronized robot pose, laser, and validated crowd data. Mutable evidence remains private; consumers see only a published snapshot.

```cpp
bool NewCrowdLearner::observe(
    const domain::Pose2D& robot,
    const domain::LaserObservation& laser,
    const domain::CrowdObservation& crowd) {
  crowd.validate();
  // Reject wrong frame, stale sequence, or too-frequent duplicate update.
  // Update only cells/entities supported by visible evidence.
  // Rebuild the immutable snapshot only after meaningful content changes.
  return changed;
}
```

Decide explicitly how laser visibility limits evidence. Current crowd-field learning updates visibility exposure, pedestrian hits, encounter experiences, and directional flow. A new learner must not treat occluded or out-of-frame people as directly observed if its semantics require robot visibility.

## Step 4: install snapshots and revisions

The navigation engine or adapter should:

1. Accept a coherent social observation.
2. Synchronize it with robot pose and laser evidence.
3. Update `WorldModel::crowd` live state.
4. Invoke enabled crowd learners.
5. Install a changed immutable snapshot in `CrowdModel`.
6. Increment only the relevant crowd dependency revisions.
7. Import mutations through `WorldModel::synchronizeMutationJournal()`.
8. Record source, prediction source, formation participation, and degraded/stale status.

Do not use one aggregate revision. Plans and decisions must record only the crowd products they consumed.

## Step 5: configuration and capabilities

Add settings under `social.learning` or a clearly named crowd-model subsection in `config/semaforr.yaml`, with matching configuration structures and validation. Include estimator choice, geometry, thresholds, decay, seed, and enablement as applicable.

Validation must ensure that:

- Social-disabled mode remains valid.
- The learner is not advertised as active without a supported input mode.
- Geometry and estimator parameters are valid.
- Consumers requiring the new model are disabled or rejected when the model cannot be produced or loaded.
- Randomized learning has its own reproducibility seed.

## Step 6: connect consumers

Advisors access live evidence through `world.crowd.observations().current()` and learned evidence through const accessors such as `learnedAt()`, `densityAt()`, or a new model-specific query. Planners receive `const CrowdModel*` in `PlanningRequest`.

Declare exact dependencies in advisor metadata and `Planner::dependencies()`. Decline participation or return `PlannerUnavailable` when required evidence is unavailable. Never substitute zero for “unknown” unless zero is semantically a real observation.

## Step 7: visualization, replay, Why, and tests

Publish visualization directly from `WorldModel::crowd`. Add separate topics only for user-facing visualization products; do not recreate an internal transport model.

Replay must capture the input observation, source/provenance, learner settings, resulting revision, and snapshot content needed for deterministic reconstruction. Why consumes recorded decision data, so ensure advisor/plan diagnostics include the model revision and source rather than adding a direct Why dependency on upstream messages.

Required tests include:

- Valid tracked, HuNav, formation, prediction, and valid-empty observations.
- Missing, malformed, stale, duplicate, wrong-frame, and out-of-order input.
- Occlusion/visibility behavior.
- Minimum evidence, repeated evidence, decay, conflicts, and meaningful revisions.
- Snapshot immutability and serialization round trip.
- Social-disabled operation.
- Advisor/planner gating and exact dependencies.
- Replay determinism and visualization generated from the internal snapshot.

Update `social-navigation.md`, `data-storage-and-access.md`, catalogs, configuration reference, and compatibility documentation when the model becomes operational.
