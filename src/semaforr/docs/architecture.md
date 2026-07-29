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
  -> semaforr::domain
       -> NavigationEngine / MissionManager / WorldModel
       -> semaforr::advisors
       -> semaforr::planning
       -> semaforr::spatial
```

The public targets are `semaforr::domain`, `semaforr::advisors`,
`semaforr::planning`, `semaforr::spatial`, and `semaforr::ros_adapters`.
`semaforr::core` and `semaforr::ros` are compatibility targets. New
ROS-independent code must not include ROS headers.

## One cognitive cycle

1. `SensorSynchronizer` accepts stamped pose and scan messages, transforms the
   pose to the global frame, and emits only coherent, fresh observations.
2. `SemaFORRNode` passes the observation and the latest valid social snapshot
   to `NavigationEngine`.
3. `MissionManager` activates or advances tasks. `PlanningCoordinator`
   generates typed `PlanResult` values and installs the selected waypoints.
4. `DecisionCoordinator` applies Tier 1 mandatory rules and vetoes, then Tier 3
   weighted scoring when no mandatory decision wins.
5. `CommandExecutor` executes the selected discrete action without blocking the
   ROS executor. It reports completion, timeout, odometry reset, or shutdown.
6. `VisualizationPublisher` publishes one structured `DecisionRecord` plus
   navigation and crowd diagnostics. The spatial learners observe the completed
   episode through `SpatialLearningCoordinator`.

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

The legacy `Controller` facade and several legacy advisor implementations
remain behind these interfaces for behavioral compatibility. New work belongs
in the focused libraries above, not in the facade.

See [decision-tiers.md](decision-tiers.md), [topics-and-frames.md](topics-and-frames.md),
[spatial-learning.md](spatial-learning.md), and
[social_navigation.md](social_navigation.md) for subsystem contracts.
