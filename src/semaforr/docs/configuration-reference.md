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
| `experiment.mode`, `experiment.random_seed` | Named ablation expansion and the reproducible decision seed. `experiment.profile` is a deprecated alias for `mode`. |
| `phases.*` | Independent initial-exploration and target-navigation lifecycle controls. |
| `tiers.tier1.rules` | Ordered, individually enabled cognitive Tier-1 rules. |
| `tiers.tier1.reactive_planners` | Individually enabled `thru`, `behind`, `out`, and `low_level_exploration` planners. |
| `tiers.tier{1,2,3}.enabled` | Cognitive-tier ablation switches; these do not bypass command execution validation. |
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

HLE policy thresholds are typed parameters rather than embedded constants:
`minimum_clearance_m`, `heading_tolerance_rad`,
`candidate_completion_distance_m`, `cue_similarity_radius_m`,
`passage_grid_resolution_m`, and `minimum_bundle_beams`. Termination uses
`time_limit_s` and `decision_budget`; `observation_budget` remains the outer
phase-coordinator safeguard.

`social.enabled: false` disables social observation subscription, learning,
advisors, and crowd planners. The subordinate
`social.observations.enabled`, `social.learning.enabled`,
`social.advisors.enabled`, and `social.planners.enabled` switches support
narrower ablations when the master switch is enabled.

Spatial representations are individually controlled by `features.trails`,
`conveyors`, `regions`, `doors`, `hallways`, `barriers`, `known_grid`,
`sensed_occupancy`, `inclusion_grid`, `highways`, and `circumstances`. Global
planners are individually selected in `planners.enabled`, including
`distance`, `sensor_distance`, `skeleton`, `highway`, `density`, `risk`, and
`flow`.

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

- map or mission paths are empty, missing, or malformed;
- numeric values are non-finite or outside their allowed range;
- action magnitudes are empty, non-positive, or unsorted;
- advisor arrays have inconsistent sizes, a name is unknown, or no active
  decision-producing advisor remains;
- an enabled planner is unknown or lacks its required supporting model;
- HighwayPlan is enabled without the highway graph, or highway learning has
  neither HLE output nor a configured loaded model;
- LLE lacks the inclusion grid, Tier 2, its reactive registration, or any
  global replanning strategy;
- the invariant `safety.command_envelope.enabled` boundary is disabled;
- a spatial or social advisor lacks its declared representation or social
  subsystem;
- crowd-cost planning is enabled without the skeleton and crowd learner;
- QoS policy, frame name, topic name, or estimator is invalid;
- map dimensions or granularity are inconsistent with the parsed map.

The node never continues with partially initialized configuration.

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
