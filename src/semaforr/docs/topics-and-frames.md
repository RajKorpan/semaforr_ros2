# Topic and frame contract

Default topic names are relative, so a namespace applies consistently. They
are parameterized under `topics.*`; remapping is optional, not required by the
installed example.

## Inputs

| Parameter / default | Type | Contract |
|---|---|---|
| `topics.pose`: `pose` | `geometry_msgs/msg/PoseStamped` | Stamped robot pose. TF converts non-global frames to `frames.global`. |
| `topics.scan`: `scan_raw` | `sensor_msgs/msg/LaserScan` | Range scan in `frames.scan`; NaN, infinity, and out-of-range beams are classified and ignored or integrated according to the laser contract rather than rejecting the whole scan. Its timestamp must synchronize with pose. |
| `topics.tracked_people`: `/human_poses_3d_tracked_global` | `social_context_msgs/msg/TrackedPersonArray` | Primary current-state input when `social.input.mode=tracked`. |
| `topics.tracked_predictions`: `/pedestrian_predictions_tracked` | `geometry_msgs/msg/PoseStamped` | GST predictions encoded by the collaborator's `<id>_pred_<step>` convention. |
| `topics.hunav_agents`: `/human_states` | `hunav_msgs/msg/Agents` | Optional simulation current-state input when `social.input.mode=hunav`. |
| `topics.hunav_predictions`: `/pedestrian_predictions` | `geometry_msgs/msg/PoseStamped` | HuNav-mode GST prediction stream. |
| `topics.formations`: `/formation_groups` | `social_context_msgs/msg/FormationGroupArray` | Optional group context associated by tracked ID. |

Pose and scan subscriptions always exist and use `qos.sensors.*`. The social
subscription is constructed only when `social.enabled` and
`social.observations.enabled` are both true. A pose/scan pair is coherent only when both are fresh
and their timestamps differ by no more than
`timing.sensor_sync_tolerance_s`. Stale or missing data changes the state to
`WaitingForSensors` and publishes a zero command.

## Outputs

| Parameter / default | Type | Meaning |
|---|---|---|
| `topics.command`: `cmd_vel` | `geometry_msgs/msg/Twist` | Current velocity command; zero on timeout, completion, shutdown, or invariant failure. |
| `topics.navigation_state`: `navigation_state` | `semaforr_msgs/msg/NavigationState` | Node state machine and action execution status. |
| `topics.decision_records`: `decision_records` | `semaforr_msgs/msg/DecisionRecord` | Replayable reasoning and execution trace for one stable decision ID; lifecycle updates reuse that ID. |

The separately installed `why` node consumes `decision_records`, accepts
`semaforr_msgs/msg/ExplanationQuestion` on `why_questions`, and publishes
`semaforr_msgs/msg/ExplanationResponse` on `why_responses`. Its parameters are
`records_topic`, `questions_topic`, and `responses_topic`; they are not
navigation-engine input parameters.

## Visualization outputs

These are the visualization publishers currently constructed by
`VisualizationPublisher`. Names without a `topics.*` parameter are fixed
relative topic names and may still be remapped by ROS.

| Topic | Type | Publication condition |
|---|---|---|
| `target_point` | `geometry_msgs/msg/PointStamped` | An active mission target exists. |
| `waypoint` | `geometry_msgs/msg/PointStamped` | The active mission has an installed waypoint. |
| `plan` | `nav_msgs/msg/Path` | A plan geometry is available. |
| `decision_pose` | `geometry_msgs/msg/PoseStamped` | A decision is published. |
| `static_map_geometry` | `visualization_msgs/msg/Marker` | `map.visualizations.enabled` and static geometry are available. |
| `static_map_occupancy` | `visualization_msgs/msg/Marker` | Map visualization is enabled and static occupancy is available. |
| `familiarity_grid` | `visualization_msgs/msg/Marker` | `grids.visualizations.enabled` and familiarity cells exist. |
| `sensed_occupancy_free` | `visualization_msgs/msg/Marker` | Grid visualization is enabled and sensed-free cells exist. |
| `sensed_occupancy_occupied` | `visualization_msgs/msg/Marker` | Grid visualization is enabled and sensed-occupied cells exist. |

Other learned-representation marker publishers are not part of the current
runtime topic contract.

## Frames and TF

- `frames.global` defaults to `map`. Missions, maps, plans, crowd fields, and
  spatial models use this frame.
- `frames.scan` defaults to `base_laser_link`. Laser messages must declare this
  frame; static sensor mounting belongs in TF.
- Pose, current-person, and formation messages in another frame are transformed to the global frame
  with the message timestamp and `frames.transform_timeout_s`.
- If a transform is unavailable, that message is rejected and a warning is
  emitted. Coordinate offsets are not applied as a fallback.
- Publishers must use one ROS clock consistently. Do not mix wall time with
  simulation time; set `use_sim_time` across every participating node when a
  simulator provides `/clock`.

For a real robot, either publish `PoseStamped` directly or run the bridge:

```bash
ros2 run semaforr_bridge odom_to_pose_bridge
```

Then set/remap the configured relative topics in one launch file. The installed
deterministic example already supplies pose and scan without manual publishers.
