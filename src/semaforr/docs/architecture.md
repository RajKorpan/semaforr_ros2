# Architecture overview

SemaFORR is split at the ROS boundary. Navigation state, decisions, planning,
geometry, social observations, and learned spatial models are ordinary C++20
types. ROS messages, clocks, parameters, TF, QoS, and publishers remain in the
adapter library.

```text
semaforr_node
  -> semaforr::ros_adapters
       -> SensorSynchronizer / CommandExecutor / VisualizationPublisher
       -> domain message and parameter adapters
       -> semaforr::navigation
            -> NavigationEngine / MissionManager
            -> semaforr::exploration
            -> semaforr::advisors
            -> semaforr::planning
            -> semaforr::spatial
                 -> semaforr::domain / WorldModel
```

The public targets are `semaforr::domain`, `semaforr::planning`,
`semaforr::spatial`, `semaforr::exploration`, `semaforr::advisors`,
`semaforr::navigation`, and `semaforr::ros_adapters`. ROS-independent code
must not include ROS headers.

## One cognitive cycle

1. `SensorSynchronizer` accepts stamped pose and scan messages, transforms the
   pose to the global frame, and emits only coherent, fresh observations.
2. `SemaFORRNode` passes the observation and the latest valid social snapshot
   to `NavigationEngine`.
3. `MissionManager` activates or advances tasks. `PlanningCoordinator`
   generates typed `PlanResult` values and installs the selected waypoints.
4. `DecisionCoordinator` applies Tier 1 mandatory rules and vetoes, then Tier 3
   arbitration when no mandatory decision wins. Tier 3 explicitly selects
   unweighted `[0,10]` compatibility comments with exact ties or normalized,
   weighted scoring with tolerance ties. Plan-sensitive advisors receive the
   current Enforcer operational target rather than silently using the final
   mission target.
5. The engine records a selection under stable decision/action IDs. The
   `CommandExecutor` reports start, progress, and exactly one terminal result
   without blocking the ROS executor.
6. The engine correlates terminal feedback, records executed history, and only
   then dispatches completed-action learners. `VisualizationPublisher`
   publishes the same IDs with the outcome.

Plans and plan-cache entries carry exact named representation revisions.
Unrelated model updates do not invalidate them, and stale diagnostics name the
dependency and old/new revisions. The world mutation sequence exists only for
ordered diagnostics. See
[revision-dependencies.md](revision-dependencies.md).

## Ownership and failure boundaries

The navigation engine owns its mission and world model. Coordinators own
exclusive polymorphic components with `std::unique_ptr`; required dependencies
are references. Decision APIs return values and do not expose mutable
side-channel pointers. Configuration is fully parsed and validated before the
engine is constructed.

Invalid configuration is a startup error. Missing or stale sensors produce a
zero velocity command. Missing or stale social data disables social
participation but does not stop geometric navigation. Empty candidate sets and
non-finite advice produce deterministic safe fallbacks.

The ROS1 `Controller`, `AgentState`, raw-pointer planners, and monolithic
spatial representations have been removed. Retained legacy configuration is
handled only by the offline migration tool and is never part of runtime
construction.

See [decision-tiers.md](decision-tiers.md), [topics-and-frames.md](topics-and-frames.md),
[spatial-learning.md](spatial-learning.md), and
[social-navigation.md](social-navigation.md) for subsystem contracts.
