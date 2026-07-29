# Modular spatial learning

Phase 10 exposes every learned spatial representation through the same
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

A `NavigationEpisode` contains the validated robot observation, selected
action, active task, monotonically increasing sequence number, and task
boundary flags. `NavigationEngine` submits it after decision arbitration, so
action-dependent learners never see a partially constructed cycle.

## Components

| Learner | Observations consumed | Update timing | Incremental | Consumers |
|---|---|---|---|---|
| `TrailLearner` | Pose and task boundaries | Appends each pose beyond the configured spacing; starts a trace for a new task | Yes | `TrailerLinear`, `TrailerRotation`, trail planner |
| `ConveyorLearner` | Pose and selected action | Adds or reinforces a directed segment after a completed forward traversal | Yes | `ConveyLinear`, `ConveyRotation`, conveyor-cost planners |
| `RegionLearner` | Complete pose/scan episodes; clustering currently uses pose samples | Clusters accumulated observations on rebuild | No | region-leaver advisors, skeleton planner |
| `DoorExitLearner` | Pose and laser ranges | Infers bounded scan discontinuities on rebuild | No | enter advisors, region and skeleton planners |
| `HallwayLearner` | Pose and nonempty laser scan | Extracts sufficiently long scan-supported traversed centerlines on rebuild | No | hallway advisors, `hallwayskel`, `skeletonhall` |
| `BarrierLearner` | Pose and laser ranges | Adds deduplicated adjacent obstacle-return segments after every scan | Yes | `AvoidObstacles`, `UnlikelyField`, collision-aware planners |
| `PassageSkeletonLearner` | Pose, scan, and task boundaries | Simplifies task traces into spaced nodes and edges on rebuild | No | skeleton, hallway-skeleton, and passage planners |

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

Incremental learners publish after each observation. Rebuild-based learners
are rebuilt explicitly with `rebuild(kind)`, collectively with `rebuildStale()`
or `rebuildAll()`, and automatically at the coordinator's configured episode
interval.

## Independent control and serialization

The default coordinator registers one uniquely owned instance of every module:

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
