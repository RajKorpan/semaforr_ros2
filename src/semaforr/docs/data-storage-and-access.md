<!-- File overview: This guide maps SemaFORR runtime data to its authoritative owners, domain types, revisions, access paths, mutability rules, ROS adapters, persistence, replay, and safe consumer patterns, with special coverage of social-context data. Its package-relative location is `docs/data-storage-and-access.md`. -->

# Data storage and access guide

The authoritative runtime state is `domain::WorldModel`, owned by `NavigationEngineAdapter`. The ROS node owns transport subscriptions and synchronizers; the navigation engine owns reasoning; spatial and crowd learners own mutable construction state; planners and advisors receive read-only views.

## Ownership map

| Data | Runtime location | Mutable owner | Normal consumer access |
| --- | --- | --- | --- |
| Robot pose and current laser | `WorldModel::robot` | navigation observation pipeline | `context.world.robot` |
| Mission and target | `WorldModel::mission` | `MissionManager`/engine | `context.world.mission` |
| Selected decisions | `decision_history` | engine | append-only `entries()` |
| Started commands | `command_history` | execution feedback pipeline | append-only `entries()` |
| Terminal outcomes | `execution_history` | execution feedback pipeline | append-only `entries()` |
| Reached traversal path | `completed_path_history`, `path_history` | engine/learning lifecycle | execution-confirmed entries |
| Static prior map | `WorldModel::static_map` non-owning const pointer | adapter startup owner | read-only pointer plus capabilities |
| Learned spatial models | `WorldModel::spatial` | spatial learners/coordinator | `context.world.spatial` |
| Live social observations | `WorldModel::crowd.observations()` | social input pipeline | `current()`/`history()` |
| Learned crowd fields | `WorldModel::crowd.learned()` | crowd learner | const query methods/snapshot |
| Revisions and mutations | spatial/crowd local maps plus world journal | representation owner | `revisionOf()` and mutation history |
| Active plan | planning/engine state | planning coordinator and Enforcer | active plan objective/context |

The adapter exposes `NavigationEngineAdapter::worldModel() const` for ROS-side visualization and diagnostics. Production reasoning should use the reference already supplied in `DecisionContext` or `PlanningRequest`, not reach back into the adapter singleton-style.

## Histories: selected is not executed

Use the history that answers the actual question:

- `decision_history`: what reasoning selected.
- `command_history`: what the controller accepted as started.
- `execution_history`: completion, failure, cancellation, timeout, safety interruption, and actual final pose.
- `completed_path_history`: execution-confirmed navigation entries.
- `observation_history`: sensed poses/timestamps.

Never use selected actions as proof of traversal. Join records with stable decision/action/task IDs.

## Spatial data

`WorldModel::spatial` contains projected consumer views: trails, conveyors, regions, doors/exits, hallways, barriers, region skeleton, sampled-path graph, familiarity, sensed occupancy, inclusion, HLE candidates, highways, circumstances, snapshot handles, and exact revisions.

Access a representation and revision together:

```cpp
const auto& regions = context.world.spatial.regions;
const auto revision = context.world.spatial.revisionOf(
    domain::ModelDependency::Regions);
```

Check both content/status semantics and the revision. An empty vector may mean “no evidence,” “not enabled,” or “validly learned empty,” depending on the representation; use configuration/capabilities and snapshot status where that distinction matters.

Grid queries must use their own transforms. Familiarity is observation frequency, sensed occupancy is occupancy evidence, static occupancy comes from `StaticMap`, and traversability is derived policy. Do not use a nonzero familiarity cell as a free planning vertex.

## Static maps and occupancy

`WorldModel::static_map` is immutable after startup and may be null in normal mapless operation. Check `map_capabilities` before use. Learned spatial data must not overwrite the map, and a simulator map is not automatically visible to SemaFORR.

Map-aware planners receive the static map and traversability policy through `PlanningRequest`. Local safety always uses current robot/sensor evidence and remains available without a map.

## Social-context data path

Social data does not enter `WorldModel` as ROS messages.

Tracked mode:

```text
social_context_msgs/TrackedPersonArray
  -> trackedPeopleToDomain()
  -> SocialObservationBuffer
  + PoseStamped prediction stream
  + optional FormationGroupArray
  -> domain::CrowdObservation
```

HuNav mode:

```text
hunav_msgs/Agents
  -> hunavAgentsToDomain()
  + HuNav PoseStamped predictions
  -> SocialObservationBuffer
  -> domain::CrowdObservation
```

The node transforms coherent observations into the configured coordinate frame, synchronizes them with robot observations, and passes `domain::CrowdState` to `NavigationEngineAdapter`. The engine updates `WorldModel::crowd` and, when enabled, the learned `CrowdFieldSnapshot`.

Read current social evidence safely:

```cpp
const auto& current = context.world.crowd.observations().current();
if (!current || !current->fresh(maximum_age)) {
  return no_participation;
}
for (const auto& person : current->pedestrians) {
  // person.id, position, velocity_mps, confidence,
  // predicted_trajectory, prediction_source, formation_index
}
```

A valid empty current observation is different from missing input. `fresh()` accepts valid empty negative evidence; `usable()` also requires at least one sufficiently confident person and is appropriate for live advisors that need a person to score.

Read learned social evidence through semantic queries:

```cpp
const auto sample = context.world.crowd.learnedAt(point, now, maximum_age);
const double risk = context.world.crowd.navigationRiskAt(point);
const double alignment = context.world.crowd.flowAlignmentAt(point, heading);
```

Use `inputSource()`, `predictionSource()`, `inputStatus()`, formation flags, and exact revisions for diagnostics. Do not assume GST predictions are current merely because current tracked people are fresh; those ages are independent.

## Planner access

`PlanningRequest` carries start, goal, `const SpatialModel*`, `const CrowdModel*`, `const StaticMap*`, traversability settings, task identity, and configuration revision. Check pointers and capability requirements before dereference. Declare consumed dependencies through the planner interface and attach them to successful plans.

## Advisor and Tier-1 access

Both receive `DecisionContext`. It is intentionally read-only. Use `active_plan_objective` for the local plan step when present and `viable_actions` for the candidate set at that component's semantic position.

If a component needs persistent state, store it in the component instance or an explicitly owned model—not by casting away constness from `WorldModel`.

## Persistence, replay, and thread safety

Persist model schema, geometry, frame, provenance, and revisions with content. Replay stores deterministic inputs and reconstructs model mutations, decisions, plans, scores, outcomes, and explanations.

Immutable snapshots may be shared across threads and decisions. Mutable learner state remains inside its owner. A consumer may retain a shared snapshot and revision; it must not retain references into a mutable construction buffer.

## Debugging checklist

When expected data is absent, check in order:

1. Feature and input mode are enabled in the expanded configuration.
2. Correct topic and coordinate frame are configured.
3. Adapter accepted the message and IDs/timestamps are valid.
4. Buffer formed a coherent, fresh observation.
5. Engine received and installed the observation.
6. Learner was enabled and accepted its update schedule.
7. Snapshot content changed and its exact revision advanced.
8. Consumer declared the dependency and checked availability.
9. Diagnostics/replay identify source, staleness, and revision.
