# Chapter 3 spatial representation profiles

`features.spatial_learning_profile` selects the algorithms used by trails,
conveyors, regions, exits/doors, hallways, and the skeleton:

- `modernized` keeps the incremental approximations used by the default
  runtime.
- `chapter3_compatibility` enables the historical algorithms described below.

This is a component-scoped profile. It does not make a whole run an exact
historical reproduction; `experiment.behavior_mode: compatibility` remains
the authority for that stronger claim and stays fail-closed while unrelated
compatibility blockers exist. The selected spatial profile is included in the
configuration fingerprint and component manifest.

## Execution-confirmed paths

`PathHistory` owns active and completed `CompletedPath` values. A path point
links a stable decision/action/task ID to selection provenance, its historical
pose and laser view, the action actually accepted by the controller, terminal
status, actual final pose, achieved translation/rotation, and interruption or
partial-motion metadata. Target and task boundaries are explicit. Selection
does not create completed traversal evidence; only terminal execution feedback
does. Failed and partial actions remain in the path, but only successful or
explicitly permitted partial movement contributes geometry.
Pure rotation is an executed action but not a traversed path element. All
geometry consumers use the execution start and actual reached pose; intended
waypoints and selected-action projections are never substituted for them.

## Visibility trails

At target completion the compatibility learner starts at the final executed,
target-relevant marker. It searches the completed path from its beginning for
the earliest pose whose stored laser ray could see that marker, records the
ray index/range as evidence, and repeats backward until the path start is
reached. Visibility uses the pose-relative historical scan, its declared
maximum range, and a configurable metric tolerance; it does not consult a
future static map. Controller rejection and no-movement events do not become
trail geometry. Partial movement is configurable and defaults to retained
actual geometry. Simplification is off by default and, when enabled, uses a
metric point-to-segment tolerance after visibility selection.

## Conveyor frequency

The compatibility conveyor is a sparse grid independent of familiarity and
occupancy. Successful completed trails are rasterized at the configured
resolution; each visited cell increments traversal frequency and accumulates
direction. Maximum frequency and normalized strength are published for
planner use. Decay defaults to none and is explicit when enabled. Geometry is
derived from evidence and may contain negative world cells, so it is not
clipped to map bounds. `ConveyorPlan` uses cell frequency; failed target
traversals contribute no trail and therefore no conveyor strength.

## Regions, exits, and doors

Each decision observation proposes a region centered on the decision pose with
radius equal to the minimum valid obstacle return. At target finalization,
overlap reconciliation is deterministic: candidate IDs are stable, supporting
observations are retained, and the reconciled radius is recalculated from the
contributing visible endpoints rather than averaged upward. Every region also
publishes a 360-bin, one-degree visibility model and its evidence revision.

Successful traveled segments are intersected with region circumferences to
create first-class exits. Exit identity, direction, supporting paths,
traversal count, and confidence survive later revisions. Exits are ordered by
angle around each region. Consecutive exits are grouped into door arcs using
the region-dependent angular-nearness threshold
`(-1.3 * exit_count) / (exit_count + 2*pi) + 1.35`. Scan discontinuities remain
available as `SensorOpening`; they are not compatibility doors.

## Hallways

Compatibility hallway learning partitions successful travel into horizontal,
vertical, and the two diagonal categories. A spatial index limits candidate
pair comparisons without dropping the inference stages. Similarity statistics
identify exceptional parent pairs while sigma is reduced deterministically;
stored views validate generated child segments. Candidate evidence is
rasterized into a configurable heatmap, smoothed with the 70-percent neighbor
rule, split into eight-connected components, and mutually visible aggregates
are joined. Each immutable hallway publishes direction, centerline, connected
cell area, width, extent, support, stable identity, and revision.

## Region skeleton and consumers

Compatibility skeleton nodes are regions, never sampled path points. A direct
region transition extracts the contiguous successful execution records and
runs that raw segment through the ordinary backward visibility Trail learner.
Each edge retains every learned traversal as evidence and selects the shortest
valid learned Trail as its direction-normalized operational label. Failed and
partial terminal actions remain in path history but cannot create a traversable
edge. Connected components are cached by model revision. The retained modernized sampled graph is explicitly named
`sampled_path_nodes` and `sampled_path_edges`.

`SkeletonPlan` selects start and goal regions by retained containment, then
learned 360-degree visibility, then the deterministic
`distance / (degree + 1)` fallback. Visibility evidence becomes a typed plan
step rather than disappearing during surrogate selection. `HighwayPlan` first
tests intersection and highway membership, then uses the same region surrogate
and Dijkstra over learned skeleton edges to reach an overlapping intersection
or highway. The complete start and reversed goal attachment routes remain in
the plan. LLE creates candidates from
known region-visibility endpoints. Enforcer visibility shortcuts consume the
published region evidence and still apply obstacle checks. Planner provenance
and explanations identify region-skeleton/subtrail use.

## Verification

`semaforr_chapter3_representations_test` contains hand-computed fixtures for
completed-path outcomes, backward visibility markers, conveyor reinforcement
and failure exclusion, region reconciliation, exit-derived doors, the complete
hallway inference pipeline, region skeleton topology, subtrail
operationalization, and planner objective consumption.
