<!-- File overview: This file documents or configures architecture behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/architecture.md`. -->

# Architecture overview

SemaFORR is split at the ROS boundary. Navigation state, decisions, planning,
geometry, social observations, and learned spatial models are ordinary C++20
types. ROS messages, clocks, parameters, TF, QoS, and publishers remain in the
adapter library.

```text
semaforr_node
  owns SensorSynchronizer / SocialObservationBuffer / CommandExecutor
  owns VisualizationPublisher / NavigationEngineAdapter
       NavigationEngineAdapter owns Configuration / ActionSpace / WorldModel
       owns optional immutable StaticMap and all coordinators
       owns NavigationEngine
            borrows WorldModel, ActionSpace, and coordinator references
            owns active exploration/reactive/Enforcer execution state
       -> exploration / advisors / planning / spatial / validation
            -> domain value types and immutable snapshots
```

The public targets are `semaforr::domain`, `semaforr::planning`,
`semaforr::spatial`, `semaforr::exploration`, `semaforr::advisors`,
`semaforr::navigation`, `semaforr::validation`, and
`semaforr::ros_adapters`. ROS-independent code
must not include ROS headers.

## One cognitive cycle

1. `SensorSynchronizer` accepts stamped pose and scan messages, transforms the
   pose to the global frame, and emits only coherent, fresh observations.
2. `SemaFORRNode` passes the observation and latest valid social snapshot to
   `NavigationEngineAdapter`, which owns the world model and invokes
   `NavigationEngine`.
3. `MissionManager` activates or advances tasks. `NavigationEngine` executes
   the semantic Tier-1 sequence across `DecisionCoordinator`, Enforcer, and
   reactive-planner boundaries without regrouping components by interface.
4. If Tier 1 does not decide and no plan exists, `PlanningCoordinator`
   generates typed `PlanResult` values. Successful plan installation ends the
   cycle; Enforcer first consumes the plan on the next cycle. Tier 3 runs only
   after Tier 1 declines and planning is unnecessary, unavailable, or failed.
   Tier 3 normalizes every production advisor's complete raw score set to
   `[0,10]`, then selects unweighted compatibility comments with exact ties or
   weighted comments with tolerance ties. Plan-sensitive advisors receive the
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

`SemaFORRNode::Impl` owns ROS subscriptions, publishers, TF, synchronization,
social buffering, command execution, visualization, and one
`NavigationEngineAdapter`. The adapter owns the configuration, action space,
shared world model, mission/planning/decision/spatial/phase coordinators, hard
safety filter, optional crowd learner, optional immutable static map, replay
recorder, and the engine instance. `MissionManager` references the mission
inside that world model. `NavigationEngine` references adapter-owned objects
whose lifetime exceeds its own; it owns only its exploration coordinator,
reactive planners, Enforcer, pending execution state, and explanation history.

Spatial learners exclusively mutate their construction state and publish
immutable snapshots through `SpatialLearningCoordinator`; planners and
advisors read the adapter-owned world projection. The simulator owns its
environment geometry independently. It may parse the same file as the adapter,
but simulator map access never supplies a static map to the robot implicitly.
The adapter's static map is immutable after successful startup.

Polymorphic components use `std::unique_ptr`; intentional shared ownership is
limited to immutable snapshot handles and ROS APIs. Decision APIs return
values and do not expose mutable side-channel pointers. Configuration is
parsed and statically validated before adapter construction, while map and
planner capability validation completes during adapter startup.

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
