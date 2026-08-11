# Modular spatial learning

Spatial representations are split into immutable value snapshots under
`spatial/representations/` and mutable builders under `spatial/learners/`.
`SpatialLearningCoordinator` exposes every representation through the same
ROS-independent lifecycle:

```cpp
class SpatialLearner {
public:
  virtual ~SpatialLearner() = default;
  virtual void observe(const NavigationEpisode&) = 0;
  virtual void rebuild() = 0;
  virtual SharedSpatialSnapshot sharedSnapshot() const = 0;
  // Compatibility API for explicit callers that need an owned value.
  virtual SpatialModelUpdate snapshot() const = 0;
};
```

A `NavigationEpisode` identifies its lifecycle event and may carry both the
immutable `SelectedActionRecord` and the matching `ActionExecutionResult`.
Motion-dependent learners receive terminal results only; they never infer
completion from action selection.

The default `modernized` profile retains incremental approximations. Setting
`features.spatial_learning_profile: chapter3_compatibility` switches the
Chapter 3 representations to their target-boundary compatibility lifecycle;
see [Chapter 3 spatial representation profiles](chapter3-spatial-representations.md).

Every representation declares one `UpdateSchedule`, including
`EveryObservation`, `EveryDecisionCycle`, `AfterActionStart`,
`AfterSuccessfulActionCompletion`, `AfterAnyTerminalActionResult`,
`EndOfTarget`, `EndOfTask`, `EndOfInitialExploration`, HLE/LLE-only,
`Periodic`, `OnShutdown`, or `OnDemand`. Learners retain mutable construction
state and publish immutable shared snapshots only when their payload or status
changes; unchanged publication does not advance the model revision or copy its
payload into the world model.

## Components

| Learner | Observations consumed | Update timing | Incremental | Consumers |
|---|---|---|---|---|
| `TrailLearner` | Execution-confirmed completed paths, poses, and historical views | Compatibility: rebuild by backward visibility at target end; modernized: sample successful final poses | Profile-dependent | `TrailerLinear`, `TrailerRotation`, trail planner |
| `ConveyorLearner` | Successful compatibility trails or execution-confirmed forward displacement | Compatibility: rasterize frequency grid at target end; modernized: reinforce directed segments after success | Profile-dependent | `ConveyLinear`, `ConveyRotation`, conveyor-cost planners |
| `RegionLearner` | Decision poses and complete scans | Compatibility: reconcile minimum-range circles at target end; modernized: incremental spatial-hash clustering | Profile-dependent | region advisors, LLE, skeleton planner |
| `DoorExitLearner` | Completed paths, reconciled regions, and outcomes | Compatibility: derive exits and door arcs at target end; modernized: publish scan discontinuities only as sensor openings | No | enter advisors, region and skeleton planners |
| `HallwayLearner` | Successful execution segments and historical views | Compatibility: indexed pair inference, heatmap, smoothing, components, visible merge at target end; modernized: orientation bins | No | hallway advisors, `hallwayskel`, `skeletonhall` |
| `BarrierLearner` | Pose and laser ranges | Adds deduplicated adjacent obstacle-return segments after every scan | Yes | `AvoidObstacles`, `UnlikelyField`, collision-aware planners |
| `PassageSkeletonLearner` | Completed paths, reconciled regions, visibility, and trails | Compatibility: region graph with shortest subtrails at target end; modernized: increment sampled path graph after success | Profile-dependent | skeleton, highway, LLE, Enforcer, and passage planners |
| `HighwayLearner` | HLE pose and detected passages | Builds touched grid rows/columns incrementally; at finalization performs local smoothing, minimum-extent extraction, intersection/spur conversion, and largest-component selection | Yes | `HighwayPlan`, `Enforcer` |
| `KnownGridLearner` | Pose and laser visibility | Every observation into sparse construction cells | Yes | `Out`, grid planners |
| `InclusionGridLearner` | Learned region area, region-skeleton supporting subtrails, and successful LLE traversal | Reproject after region/skeleton finalization; increment only after successful LLE translation | Yes | LLE and coverage diagnostics |
| `CircumstanceLearner` | Terminal selected actions with explicit success/failure outcome | Collect after any terminal result; validate cases at end of target | No | `Precedent` |

Every learner declares this information at runtime through
`ObservationContract`. `SpatialLearningCoordinator::inspect()` returns the
contract, enabled state, status, revision, observation count, consumers, and
diagnostic for all modules.

## Freshness and incomplete models

Snapshots use four explicit states:

- `Empty`: no observation has been accepted.
- `Incomplete`: observations exist but do not meet the representation's
  minimum evidence requirements.
- `Fresh`: the payload incorporates all accepted observations.
- `Stale`: a rebuild-based learner has accepted observations since its last
  rebuild.

Only `Fresh` updates are projected into `domain::SpatialModel`. A stale or
incomplete update never clears the last usable representation, so advisors and
planners can continue with the last fresh model. Consumers can inspect the
coordinator when they need to distinguish current from retained data.

The known, sensed-occupancy, and inclusion grids use sparse cells while
learning and emit sorted sparse snapshots. The coordinator aliases these const
vectors into `domain::SpatialModel`; it does not densify them. Explicit legacy
consumers can request a cached lazy dense view or a region-of-interest view.
Highway labels rasterize only new path segments and record the affected rows
and columns. Highway snapshots
use schema-versioned first-class highway, intersection, and graph entities;
legacy node/edge projections remain available during consumer migration.
Skeleton connected components are recomputed only after a graph mutation.

Each publication also carries cell/entity/graph change sets. Projection cost,
copy counts, allocations, lock time, and peak retained bytes are exposed by
the coordinator. See [Snapshot sharing and projection performance](snapshot-sharing.md).

Learners are rebuilt explicitly with `rebuild(kind)`, collectively with
`rebuildStale()` or `rebuildAll()`, and at lifecycle boundaries matching their
declared schedule.

## Independent control and serialization

The default coordinator registers one uniquely owned instance of all twelve
modules:

```cpp
auto learning = SpatialLearningCoordinator::defaults(10);
learning.setEnabled(SpatialRepresentation::Hallways, false);
learning.rebuild(SpatialRepresentation::Regions);

const auto region = learning.snapshot(SpatialRepresentation::Regions);
const std::string json =
  learning.serialize(SpatialRepresentation::Regions);
```

Disabling one representation stops only that learner from observing or
rebuilding. Its state remains available through `inspect()` but is excluded
from normal snapshots and serialization; its corresponding world-model
projection is cleared without affecting other representations. Re-enabling
the learner makes its retained state available again. Serialized JSON is
deterministic and includes lifecycle metadata, consumers, diagnostics, and
the representation-specific payload.

The framework is built and exported as `semaforr::spatial`; it has no ROS
dependency or initialization requirement.
