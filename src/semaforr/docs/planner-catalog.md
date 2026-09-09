<!-- File overview: This file documents or configures planner catalog behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/planner-catalog.md`. -->

# Planner catalog

Planner names are case-sensitive and selected through `planners.enabled`.
Unknown names fail at startup.

`planners.enabled` is an opt-in catalog, not an independent capability list.
Tier 2 and each producer representation are authoritative: normalization
removes a requested planner when Tier 2 or any required producer is disabled.
This preserves planner ablations without allowing contradictory effective
states. Every removal is reported in startup diagnostics.

Planner output is classified as `Grid`, `AffordanceModifiedGrid`, or
`Freespace`. Grid families derive topology from traversability; learned
representations modify edge cost but never replace occupancy. SkeletonPlan and
HighwayPlan are freespace planners and retain typed learned-spatial structure. See
`tier-two-planning-and-enforcement.md` for enforcement and provenance.

| Name | Family | Cost | Required topology / fields |
|---|---|---|---|
| `distance` | Grid | Metric path length | Valid static-map traversability; map required |
| `sensor_distance` | Grid | Metric path length | Partial sensed traversability; registered dormant until sensed free cells exist |
| `density` | Grid | Length plus learned density | Valid static-map traversability and crowd density; map required |
| `risk` | Grid | Length plus encounter risk | Valid static-map traversability and crowd risk; map required |
| `flow` | Grid | Length plus opposing flow | Valid static-map traversability and crowd flow; map required |
| `region` | AffordanceModifiedGrid | Region/door/exit affordance cost | Static or partial sensed traversability plus regions and exits |
| `hallway` | AffordanceModifiedGrid | Hallway affordance cost | Static or partial sensed traversability plus hallways |
| `trail` | AffordanceModifiedGrid | Trail affordance cost | Static or partial sensed traversability plus trails |
| `conveyor` | AffordanceModifiedGrid | Traversal-frequency cost | Static or partial sensed traversability plus conveyors |
| `skeleton` | Freespace | Learned transition distance | Region skeleton, visibility, and supporting subtrails |
| `highway` | Freespace | Highway-assisted distance | Highways, intersections, region skeleton, and supporting subtrails |

Each planner implements the typed `Planner` interface and returns a
`PlanResult` with `Success`, `NoPath`, `InvalidRequest`, or
`PlannerUnavailable`. Graph storage is immutable during search; A* keeps all
search state locally. A stale crowd sample contributes no social penalty.
No planner silently switches to a different topology when its required
occupancy or learned model is unavailable.

The registry may accept an affordance planner in mapless mode, but that planner
returns `PlannerUnavailable` until the sensor-derived occupancy snapshot has a
usable partial graph. Static-map planners are not registered without the map
planning capability. Highway planning requires online HLE/highway learning;
the exposed loaded-highway-model field is currently rejected because no model
loader is implemented.

The selected planner and ordered alternatives are included in structured
decision diagnostics. Crowd learning strategies (`count_exposure`, `discount`,
and `cusum`) are configured under `social.learning.estimator`; they are not
planner names.
