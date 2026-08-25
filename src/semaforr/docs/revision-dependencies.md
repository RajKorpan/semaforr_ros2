<!-- File overview: This file documents or configures revision dependencies behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/revision-dependencies.md`. -->

# Representation revisions and dependency tracking

Model validity is based on exact named dependencies. The diagnostic world
mutation sequence and the legacy `SpatialModel::revision` field are not cache
keys and must not be used to decide whether a plan is stale.

## Revision ownership

Each mutable representation has a monotonic entry in
`DependencyRevisions`: static geometry and occupancy, sensed occupancy,
familiarity, inclusion, trails, conveyors, regions, doors/exits, hallways,
barriers, skeleton, highways, highway graph, circumstances, and the three
learned crowd fields. Planner configuration has its own revision. Static-map
revisions are immutable after startup.

Spatial learners publish only when their content signature changes. Applying
the same published snapshot again is a no-op. Crowd density, risk, and flow
are compared independently; changing a timestamp, snapshot version, or other
diagnostic metadata without changing planner-visible values does not advance
their revisions. Revisions currently have whole-representation granularity.
Regional invalidation is not implemented or claimed.

Every accepted representation mutation also enters `WorldModel`'s monotonic
diagnostic journal with the global sequence, representation, local revision,
timestamp, and summary. The journal explains ordering; the local revisions
remain authoritative for consumers.

## Plan dependencies

Planners declare the representations they read. `PlanningCoordinator` stores
their exact revision snapshot with every `PlanResult` and
`HierarchicalPlan`, together with task, start, target, and planner-policy
revision.

| Planner input | Dependencies |
| --- | --- |
| Static-map grid | static geometry, static occupancy, sensed occupancy |
| Partial sensor grid | sensed occupancy |
| Region affordance | base graph plus regions and doors/exits |
| Trail, hallway, conveyor | base graph plus the named affordance |
| Skeleton | skeleton and regions |
| Highway | skeleton, highways, highway graph, regions, and trails |
| Crowd density/risk/flow | base graph plus only the selected crowd field |

A cached plan is reused only when planner, task, start/target surrogates, and
all declared revisions match. An unrelated layer change therefore leaves the
cache entry valid. Validation diagnostics use
`dependency_changed:<name>:<old>-><new>` and separately report task changes,
start or target tolerance failures, policy changes, and execution
invalidation.

## Operationalization dependencies

Enforcer validates only the spatial dependencies present on the plan. Each
shortcut or repair also appends an operationalization record with its own
inputs: visibility geometry, regions, trails, highways, highway graph, or
familiarity as applicable. The navigation engine validates non-spatial plan
dependencies before handing the plan to Enforcer. A failed terminal execution
marks the remaining hierarchy stale.

The focused acceptance tests are in
`test/unit/revision_dependency_test.cpp`.
