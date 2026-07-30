# Planner catalog

Planner names are case-sensitive and selected through `planners.enabled`.
Unknown names fail at startup.

| Name | Cost | Required model |
|---|---|---|
| `distance` | Metric path length | None; always registered as the safe baseline |
| `skeleton` | Metric A* over learned passages | Passage/skeleton snapshot |
| `density` | Length plus learned density | Skeleton and crowd field |
| `risk` | Length plus learned encounter risk | Skeleton and crowd field |
| `flow` | Length plus opposing-flow penalty | Skeleton and crowd field |

Each planner implements the typed `Planner` interface and returns a
`PlanResult` with `Success`, `NoPath`, `InvalidRequest`, or
`PlannerUnavailable`. Graph storage is immutable during search; A* keeps all
search state locally. A stale crowd sample contributes no social penalty, and
the distance planner remains available as a deliberate fallback.

The selected planner and ordered alternatives are included in structured
decision diagnostics. Crowd learning strategies (`count_exposure`, `discount`,
and `cusum`) are configured under `social.learning.estimator`; they are not
planner names.
