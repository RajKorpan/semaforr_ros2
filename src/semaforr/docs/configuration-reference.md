<!-- File overview: This file documents or configures configuration reference behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/configuration-reference.md`. -->

# Configuration reference

`config/semaforr.yaml` is the canonical ROS 2 configuration. The node name is
`semaforr`, so parameters belong below `semaforr.ros__parameters`. Launch files
load this file first and override only installed example paths.

All distances are metres, angles are radians, velocities are SI units, and
durations are seconds.

## Parameter groups

| Group | Purpose |
|---|---|
| `experiment.behavior_mode` | Behavioral claim: supported `modernized` runtime or reserved, fail-closed `compatibility` target. |
| `experiment.mode`, `experiment.random_seed` | Named ablation expansion and the deprecated aggregate seed. When every scoped seed is zero, a nonzero aggregate seed initializes all scoped streams for migration compatibility. |
| `experiment.seeds.*` | Independent seeds for `tier_three_ties`, `lle_fallback`, `planner_ties`, `clustering`, and `simulation_noise`. |
| `reproducibility.*` | Enables versioned run traces and records the trace path plus source/test-suite revisions. |
| `explanations.mode` | `disabled`, `why`, or `comparison`; comparison requires candidate-plan retention. |
| `phases.*` | Independent initial-exploration and target-navigation lifecycle controls. |
| `tiers.tier1.rules` | Ordered, individually enabled cognitive Tier-1 rules. |
| `tiers.tier1.reactive_planners` | Individually enabled `thru`, `behind`, `out`, and `low_level_exploration` planners. |
| `tiers.tier{1,2,3}.enabled` | Cognitive-tier ablation switches; these do not bypass command execution validation. |
| `tiers.tier3.scoring_policy` | `profile`, unweighted `compatibility_comments`, or `weighted_normalized`. |
| `tiers.tier3.tie_policy` | `profile`, `exact`, or `tolerance`; tie candidates and seeded selection are recorded. |
| `tiers.tier3.tie_tolerance` | Finite nonnegative tolerance used only by tolerance tie resolution. |
| `tiers.tier2.tie_policy` | `profile`, `deterministic`, or `seeded_exact`; the latter consumes only `experiment.seeds.planner_ties`. |
| `tiers.tier2.maximum_planning_attempts_per_task` | Positive consecutive immediate-planning failure limit before the current plan attempt is abandoned and LLE becomes eligible; default `3`. |
| `topics.*` | Relative pose, scan, command, state, decision, social, and crowd-field topic names. |
| `qos.sensors.*`, `qos.command.*` | Queue depth, `reliable`/`best_effort`, and `volatile`/`transient_local`. |
| `frames.global`, `frames.scan` | Navigation and laser frame contract. |
| `frames.transform_timeout_s` | Maximum TF lookup wait. |
| `timing.control_rate_hz` | Nonblocking command/state-machine timer rate. |
| `timing.sensor_timeout_s` | Age after which the node publishes zero velocity. |
| `timing.sensor_sync_tolerance_s` | Maximum pose/scan timestamp separation. |
| `map.mode` | `mapless` (default) or `map_enabled`. |
| `map.path` | Absolute, `package://`, package-relative, or named example map. Required only in map-enabled mode. |
| `map.on_load_failure` | `fail_startup` (default) or `disable_map`. |
| `map.origin_x_m`, `map.origin_y_m` | Lower map bound; negative origins are supported. |
| `map.occupancy_resolution_m` | Resolution of occupancy derived from static walls. |
| `map.obstacle_inflation_m` | Nonnegative static obstacle inflation radius. |
| `map.planning.enabled` | Enables the map-based-planning capability after a valid load. |
| `map.visualizations.enabled` | Publishes static map geometry separately from learned models. |
| `map.length_m`, `map.height_m`, `map.granularity_m` | Validated map extent and discretization. |
| `mission.tasks_path` | Mission target file path resolved by launch. |
| `mission.decision_limit` | Maximum decisions before the active task is skipped. |
| `actions.move_distances_m` | Sorted, positive, finite forward magnitudes. |
| `actions.rotation_angles_rad` | Sorted, positive, finite turn magnitudes. |
| `safety.*` | Robot footprint, laser range, obstacle buffer, and sweep limits. |
| `command.*` | Execution velocities, tolerances, timeout policy, and odometry-reset thresholds. |
| `features.*` | Independent spatial and recovery feature switches. |
| `planners.enabled` | Registered planner names; see `planner-catalog.md`. |
| `advisors.*` | Parallel names/enabled/weights arrays and four parameters per advisor. |
| `social.*` | Live-data age/confidence gates and crowd-field learner settings. |

## Named ablation modes

Behavior mode and ablation mode are orthogonal. `experiment.behavior_mode`
defaults to `modernized`. `compatibility` is intentionally rejected until the
blockers and acceptance suite in `compatibility-matrix.md` are resolved. An
ablation profile name never asserts algorithm fidelity.

`experiment.mode` accepts `full`, `tier1_only`, `tier1_tier3`,
`tier3_only`, `tier1_tier2_tier3`, `no_initial_exploration`,
`no_opportunistic_exploration`, `no_spatial_model`, `no_social`, or `custom`.
Modes expand into the same tier, phase, planner, advisor, social, and
representation fields used by `custom`; they do not select alternate runtime
code paths.

The historical evaluation profile `original` means the feature combination
used for that study comparison. It does not mean that every selected learner
or decision procedure is the original algorithm.

HLE is controlled only by `phases.initial_exploration.*`. LLE is controlled by
`exploration.reactive.*` together with the
`low_level_exploration` reactive-planner registration. Exploration-oriented
Tier-3 advisors remain independent entries in `advisors.*`.

`exploration.reactive.behavior_policy` accepts `profile`, `compatibility`, or
`modernized`. Compatibility triggers only when no plan is available or when a
plan completes short of its target, and uses seeded random selection within
the closest target-distance bin. Modernized mode may additionally enable the
explicit `stalled_history_extension`. `closest_target_bin_m` sets the bin
width, and `experiment.seeds.lle_fallback` makes compatibility selection
replayable.

HLE policy thresholds are typed parameters rather than embedded constants:
`minimum_clearance_m`, `heading_tolerance_rad`,
`candidate_completion_distance_m`, `cue_similarity_radius_m`,
`passage_grid_resolution_m`, and `minimum_bundle_beams`. Termination uses
`time_limit_s` and `decision_budget`; `observation_budget` remains the outer
phase-coordinator safeguard.

`phases.initial_exploration.behavior_policy` accepts `profile`, `modernized`,
or `compatibility`. `left_focus_min_rad`, `left_focus_max_rad`,
`right_focus_min_rad`, and `right_focus_max_rad` define the narrow cue sectors;
the corresponding `left_open_*` and `right_open_*` fields define independent
wide openness sectors. These finite ordered robot-relative intervals replace
beam-count-specific slices. Both policies use the configured length-to-width,
threshold-based large-room, cue-clearance, extension, width-change, hard-turn,
and end-clearance thresholds. See [High-level
exploration](high-level-exploration.md).

`social.enabled: false` disables social observation subscription, learning,
advisors, and crowd planners. The subordinate
`social.enabled`, `social.input.mode`, `social.learning.enabled`,
`social.advisors.enabled`, and `social.planners.enabled` switches support
narrower ablations when the master switch is enabled.

`social.input.mode` accepts `none`, `tracked`, or `hunav`. Tracked mode requires
the tracked people and prediction topics; HuNav mode requires the agents and
HuNav prediction topics. Formation input is an optional tracked-mode
dependency. Current and prediction freshness are independent, and
`fallback_prediction` accepts `constant_velocity` or `none`. Social-disabled
and `none` configurations do not require social topics. Crowd learning requires
an active input mode. Direct crowd visualization is controlled by
`social.visualizations.enabled` and `topics.crowd_*`.

Spatial representations are individually controlled by `features.trails`,
`conveyors`, `regions`, `doors`, `hallways`, `barriers`, `known_grid`,
`sensed_occupancy`, `inclusion_grid`, `highways`, and `circumstances`. Global
planners are individually selected in `planners.enabled`, including
`distance`, `sensor_distance`, `skeleton`, `highway`, `density`, `risk`, and
`flow`, plus the affordance-modified `region`, `hallway`, `trail`, and
`conveyor` planners.

These settings describe requested components, but true dependencies are not
independent effective switches. Configuration normalization derives the
runtime component set before validation:

- Disabling a spatial representation deactivates its dependent advisors and
  planners.
- Disabling known-map planning deactivates static-map grid planners.
- Disabling sensed occupancy deactivates the partial sensor-grid planner.
- Disabling crowd learning deactivates learned crowd-field planners.
- Disabling circumstances removes Precedent and circumstance Tier-3
  weighting.
- Disabling HLE removes an unpersisted highway model and its consumers.
- LLE is removed unless Tier 1, Tier 2, inclusion, and a replanning strategy
  are all available.

Each derived removal appears in startup diagnostics, for example
`advisor_disabled_missing_representation:prefer_highways:highways` or
`component_disabled_missing_dependency:planner:region:regions`. An enabled
learned-model advisor also abstains until its representation has published
actual evidence. Unknown component names and malformed numeric values remain
hard configuration errors.

`features.spatial_learning_profile` is `modernized` (incremental adapted
learners) or `chapter3_compatibility` (target-boundary compatibility learners).
It is component-scoped and does not assert whole-system compatibility. The
value participates in both the configuration fingerprint and component
manifest.

`grids.extent_policy` is `expand` or `fixed`. `grids.sensed` controls evidence
needed to clear and expire sensed obstacles. `grids.planning` controls separate
map and partial-sensor unknown-space policies, footprint inflation margins, and
the unknown-cell cost multiplier. `grids.frame_id`, `resolution_m`,
`mapless.initial_width_m`, and `mapless.initial_height_m` define the initial
mapless allocation. `grids.expansion` defines its trigger margin, aligned cell
increment, optional maximum dimensions (zero means unbounded), and hard memory
limit. Learned grids are centered on the first robot pose.

`grids.highway.origin_x_m` and `origin_y_m` define the highway lattice origin.
`smoothing_policy` accepts `profile`, `von_neumann_three_of_four`, or
`directional_gap_fill`; `component_selection_policy` accepts `profile`,
`most_intersections`, or `largest_vertex_count`. `profile` resolves through the
selected spatial-learning profile. See [Highway and passage
models](highway-model.md) for exact semantics.

`map.bounds_policy` is `require_declared`, `infer`, or `infer_expandable`.
Inference uses obstacle geometry and `map.inferred_bounds_padding_m`; static
occupancy remains fixed, while `infer_expandable` permits the separate sensed
overlay to extend beyond that prior. See [Grid layers](grid-layers.md) and
[Grid geometry](grid-geometry.md).

## Invariant safety boundary

Safety is deliberately outside cognitive Tier 1. Before arbitration,
`HardSafetyFilter` rejects motion without a usable laser view, invalid action
indices, and forward actions that violate collision clearance. The
configurable Tier-1 `avoid_obstacles` rule remains available for cognitive
behavior and diagnostics, but is not the platform safety boundary.

The complete order is: generate typed candidates; run hard safety; run the
cognitive tiers; validate the selected action while constructing the execution
request; enforce velocity, acceleration, finite-command, timeout, and odometry
invariants inside `CommandExecutor`; then perform the publisher's final finite
and velocity-bound check. Thus `avoid_obstacles` is ablatable, while the hard
filter and command/controller checks are not.

Immediately before command publication, the sensor synchronizer cancels
execution when pose or laser data is stale or incoherent. `CommandExecutor`
then validates action indices and finite targets, ramps commands within
`command.maximum_{linear,angular}_acceleration_*`, and enforces
`command.maximum_{linear,angular}_velocity_*`. The publishing boundary performs
a final finite-value and velocity-bound check. Emergency cancellation and
shutdown always publish zero velocity immediately.

## Social-learning parameters

`social.learning.estimator` accepts `count_exposure`, `discount`, or `cusum`.
Resolution and origins define the crowd grid in `frames.global`.
`minimum_update_period_s` bounds update frequency, `encounter_radius_m`
defines proximity evidence, and `minimum_flow_speed_mps` suppresses unstable
directions. `confidence_exposures` controls evidence confidence.
`discount_factor` is used by the discount estimator; the three `cusum_*`
values configure CUSUM. `random_seed` makes stochastic estimators reproducible.

## Startup validation

Startup fails with a parameter name and actionable reason when:

- a required mission path is empty, missing, or malformed, or map access was
  requested and the map cannot be resolved under its configured failure policy;
- numeric values are non-finite or outside their allowed range;
- action magnitudes are empty, non-positive, or unsorted;
- advisor arrays have inconsistent sizes, a name is unknown, or no active
  decision-producing advisor remains;
- an enabled planner is unknown or lacks its required supporting model;
- HighwayPlan is enabled without online highway learning, HLE is unavailable
  to produce the graph, or `features.loaded_highway_model` is nonempty (the
  field is rejected because no highway-model loader is implemented);
- LLE lacks the inclusion grid, Tier 2, its reactive registration, or any
  global replanning strategy;
- the invariant `safety.command_envelope.enabled` boundary is disabled;
- a spatial or social advisor lacks its declared representation or social
  subsystem;
- crowd-cost planning is enabled without map planning, social planners, or
  the corresponding crowd-learning capability;
- QoS policy, frame name, topic name, or estimator is invalid;
- map dimensions or granularity are inconsistent with the parsed map.

`map.on_load_failure: disable_map` is the one explicit degradation policy: a
failed requested map is reported, static-map planners are disabled, and the
adapter continues with mapless capabilities. No planner is registered over an
empty static grid.

## Overriding a deployment

Copy the installed YAML into your own package and pass it as a parameter file:

```python
Node(
    package="semaforr",
    executable="semaforr_node",
    name="semaforr",
    parameters=[my_yaml, {
        "map.path": installed_map,
        "mission.tasks_path": installed_mission,
    }],
)
```

Prefer launch-time package-share resolution over absolute paths embedded in
YAML. Command-line overrides are useful for experiments:

```bash
ros2 launch semaforr example_simulation.launch.py \
  duration:=30.0 sensor_cutoff:=12.0
```
