# Configuration reference

`config/semaforr.yaml` is the canonical ROS 2 configuration. The node name is
`semaforr`, so parameters belong below `semaforr.ros__parameters`. Launch files
load this file first and override only installed example paths.

All distances are metres, angles are radians, velocities are SI units, and
durations are seconds.

## Parameter groups

| Group | Purpose |
|---|---|
| `configuration.use_legacy_files` | Compatibility switch. Keep `false` for YAML deployments. |
| `topics.*` | Relative pose, scan, command, state, decision, social, and crowd-field topic names. |
| `qos.sensors.*`, `qos.command.*` | Queue depth, `reliable`/`best_effort`, and `volatile`/`transient_local`. |
| `frames.global`, `frames.scan` | Navigation and laser frame contract. |
| `frames.transform_timeout_s` | Maximum TF lookup wait. |
| `timing.control_rate_hz` | Nonblocking command/state-machine timer rate. |
| `timing.sensor_timeout_s` | Age after which the node publishes zero velocity. |
| `timing.sensor_sync_tolerance_s` | Maximum pose/scan timestamp separation. |
| `map.path` | XML map path; launch files should resolve it from package share. |
| `map.length_m`, `map.height_m`, `map.granularity_m` | Validated map extent and discretization. |
| `mission.tasks_path` | Mission target file path resolved by launch. |
| `mission.decision_limit`, `mission.plan_limit` | Per-task decision and planning limits. |
| `actions.move_distances_m` | Sorted, positive, finite forward magnitudes. |
| `actions.rotation_angles_rad` | Sorted, positive, finite turn magnitudes. |
| `safety.*` | Robot footprint, laser range, obstacle buffer, and sweep limits. |
| `command.*` | Execution velocities, tolerances, timeout policy, and odometry-reset thresholds. |
| `learning.*` | Legacy highway learning thresholds. |
| `features.*` | Independent spatial and recovery feature switches. |
| `planners.enabled` | Registered planner names; see `planner-catalog.md`. |
| `advisors.*` | Parallel names/enabled/weights arrays and four parameters per advisor. |
| `social.*` | Live-data age/confidence gates and crowd-field learner settings. |

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
