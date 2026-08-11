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
  virtual SpatialModelUpdate snapshot() const = 0;
};
```

A `NavigationEpisode` identifies its lifecycle event and may carry both the
immutable `SelectedActionRecord` and the matching `ActionExecutionResult`.
Motion-dependent learners receive terminal results only; they never infer
completion from action selection.

Every representation declares one `UpdateSchedule`, including
`EveryObservation`, `EveryDecisionCycle`, `AfterActionStart`,
`AfterSuccessfulActionCompletion`, `AfterAnyTerminalActionResult`,
`EndOfTarget`, `EndOfTask`, `EndOfInitialExploration`, HLE/LLE-only,
`Periodic`, `OnShutdown`, or `OnDemand`. Learners retain mutable construction state and publish copied
snapshots only when their payload or status changes; unchanged publication
does not advance the model revision.

## Components

| Learner | Observations consumed | Update timing | Incremental | Consumers |
|---|---|---|---|---|
| `TrailLearner` | Execution-confirmed start/final pose and task boundaries | After successful action completion; failures never enter a trail | Yes | `TrailerLinear`, `TrailerRotation`, trail planner |
| `ConveyorLearner` | Execution-confirmed forward displacement | After successful action completion; adds or reinforces the actual directed segment | Yes | `ConveyLinear`, `ConveyRotation`, conveyor-cost planners |
| `RegionLearner` | Complete pose/scan episodes | Every observation; nearby overlap candidates come from a spatial hash | Yes | region-leaver advisors, skeleton planner |
| `DoorExitLearner` | Terminal pose and laser ranges, including outcome metadata | Collect terminal evidence; rebuild at end of target | No | enter advisors, region and skeleton planners |
| `HallwayLearner` | Terminal poses, outcomes, and nonempty laser scans | Collect terminal evidence; rebuild at end of target; orientation and midpoint bins avoid all-pairs comparison | No | hallway advisors, `hallwayskel`, `skeletonhall` |
| `BarrierLearner` | Pose and laser ranges | Adds deduplicated adjacent obstacle-return segments after every scan | Yes | `AvoidObstacles`, `UnlikelyField`, collision-aware planners |
| `PassageSkeletonLearner` | Execution-confirmed start/final pose, scan, and task boundaries | After successful action completion; stable node IDs and cached connected components | Yes | skeleton, hallway-skeleton, and passage planners |
| `HighwayLearner` | HLE pose and detected passages | Builds touched grid rows/columns incrementally; at finalization performs local smoothing, minimum-extent extraction, intersection/spur conversion, and largest-component selection | Yes | `HighwayPlan`, `Enforcer` |
| `KnownGridLearner` | Pose and laser visibility | Every observation into sparse construction cells | Yes | `Out`, grid planners |
| `InclusionGridLearner` | Pose and laser visibility | Every observation into sparse construction cells | Yes | LLE, exploration |
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

The known and inclusion grids use sparse cells while learning and emit sorted
sparse snapshots. Legacy consumers are densified only when the coordinator
projects a snapshot into `domain::SpatialModel`. Highway labels rasterize only
new path segments and record the affected rows and columns. Highway snapshots
use schema-versioned first-class highway, intersection, and graph entities;
legacy node/edge projections remain available during consumer migration.
Skeleton connected components are recomputed only after a graph mutation.

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
