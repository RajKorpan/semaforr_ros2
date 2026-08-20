# Tier-2 planning and Tier-1 plan enforcement

Tier-2 produces one of two explicit plan families. The family is stored in
`PlanResult::family` and `HierarchicalPlan::family`; it is never inferred from
the planner's display name.

## Grid plans

Distance, crowd-density, crowd-risk, crowd-flow, RegionPlan, HallwayPlan,
TrailPlan, and ConveyorPlan produce grid plans. Static-map planners require
map-derived traversability. The `sensor_distance` planner supports partial
sensor occupancy. Affordance planners use static occupancy when available and
otherwise require partial sensed occupancy. Familiarity never creates a graph
vertex, and the learned skeleton is never silently substituted for occupancy.

Occupancy decides whether an edge exists. Crowd and learned affordances modify
the cost of an otherwise traversable edge. The planner registry exposes each
planner's input model, plan family, map/occupancy requirements, partial-map
support, objective, and revision dependencies.

`GridPlanEnforcer` advances past completed cells, detects path deviation,
derives current traversability, and chooses the farthest remaining waypoint
whose complete segment is traversable. Static, sensed, inflated, unknown, and
outside-extent cells retain their configured meaning. A shortcut is recorded
rather than causing a replan. Only consumed dependency revisions can stale the
plan.

## Model plans

SkeletonPlan and HighwayPlan produce model plans. SkeletonPlan accepts only the
region skeleton: region nodes, direct transition edges, visibility, and
visibility-compressed execution-supported edge Trails. It chooses contained,
visible, then degree/distance region surrogates and retains any exact visibility
ray as an operational step. The sampled path graph and nearest-node attachment
are not fallbacks. HighwayPlan applies intersection membership, highway
membership with closer-endpoint selection, and finally region/skeleton access
in that order. Dijkstra retains every skeleton transition between the region
surrogate and the first region overlapping a highway or intersection, on both
the start and goal sides. Highway graph edges carry operational subtrails.

Model plans may contain region, visibility-connection, subtrail,
skeleton-transition, highway-entry, highway, intersection, highway-exit,
waypoint, and final-target steps.
`ModelPlanEnforcer` interprets those types, applies visible-step and subtrail
lookahead, performs explicit repairs, and records shortcuts and substitutions
against their exact model revisions.

## Shared local action selection

Both Enforcers supply an operational target to `LocalActionEvaluator`. It
predicts every viable action, records its predicted pose and progress metric,
and mandates the best action that makes positive progress. The mandate is a
Tier-1 decision. Victory runs first; reactive planners and Tier 3 run only when
the applicable Enforcer cannot produce a viable action.

Decision records identify plan ID, plan family, planner, Enforcer mode, active
step, operational target, and reason code. Grid waypoint, region, subtrail,
highway, intersection, shortcut, repair, and invalidation traces remain
distinct.

## Planning provenance and range voting

Every plan records its ID, family, planner, start, goal, creation time,
geometric and typed contents, objective costs, dependency revisions,
configuration revision, operating mode, and whether static-map knowledge
contributed. Range voting retains every candidate's raw costs under every
objective, normalized costs under participating objectives, summed score, tie
membership, the winning plan, and the explicit tie-break reason. The current
tie rule is lexicographically smallest planner name; no random tie break is
used.
