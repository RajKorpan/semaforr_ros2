# Planner catalog

Planner names are case-sensitive and selected through `planners.enabled`.
Unknown names fail at startup.

Planner output is explicitly classified as `grid` or `model`. Grid planners
derive topology from traversability; learned representations modify edge cost
but never replace occupancy. SkeletonPlan and HighwayPlan are the only model
planners and retain typed learned-spatial structure. See
`tier-two-planning-and-enforcement.md` for enforcement and provenance.

| Name | Family | Cost | Required topology / fields |
|---|---|---|---|
| `distance` | grid | Metric path length | Static-map traversability |
| `sensor_distance` | grid | Metric path length | Partial sensed traversability |
| `density` | grid | Length plus learned density | Static traversability and crowd density |
| `risk` | grid | Length plus encounter risk | Static traversability and crowd risk |
| `flow` | grid | Length plus opposing flow | Static traversability and crowd flow |
| `region` | grid | Region/door/exit affordance cost | Static or partial sensed traversability; regions and exits |
| `hallway` | grid | Hallway affordance cost | Static or partial sensed traversability; hallways |
| `trail` | grid | Trail affordance cost | Static or partial sensed traversability; trails |
| `conveyor` | grid | Traversal-frequency cost | Static or partial sensed traversability; conveyors |
| `skeleton` | model | Learned transition distance | Region skeleton, visibility, subtrails |
| `highway` | model | Highway-assisted distance | Highways, intersections, region skeleton, subtrails |

Each planner implements the typed `Planner` interface and returns a
`PlanResult` with `Success`, `NoPath`, `InvalidRequest`, or
`PlannerUnavailable`. Graph storage is immutable during search; A* keeps all
search state locally. A stale crowd sample contributes no social penalty.
No planner silently switches to a different topology when its required
occupancy or learned model is unavailable.

The selected planner and ordered alternatives are included in structured
decision diagnostics. Crowd learning strategies (`count_exposure`, `discount`,
and `cusum`) are configured under `social.learning.estimator`; they are not
planner names.
