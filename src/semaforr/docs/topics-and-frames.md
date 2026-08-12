# Topic and frame contract

Default topic names are relative, so a namespace applies consistently. They
are parameterized under `topics.*`; remapping is optional, not required by the
installed example.

## Inputs

| Parameter / default | Type | Contract |
|---|---|---|
| `topics.pose`: `pose` | `geometry_msgs/msg/PoseStamped` | Stamped robot pose. TF converts non-global frames to `frames.global`. |
| `topics.scan`: `scan_raw` | `sensor_msgs/msg/LaserScan` | Finite ranges in `frames.scan`; its timestamp must synchronize with pose. |
| `topics.social_observations`: `social_observations` | `social_context_msgs/msg/SocialObservation` | Canonical pedestrian IDs, positions, velocities, stamped predictions, confidence/covariance, and source age. |

Pose and scan use `qos.sensors.*`. A pair is coherent only when both are fresh
and their timestamps differ by no more than
`timing.sensor_sync_tolerance_s`. Stale or missing data changes the state to
`WaitingForSensors` and publishes a zero command.

## Outputs

| Parameter / default | Type | Meaning |
|---|---|---|
| `topics.command`: `cmd_vel` | `geometry_msgs/msg/Twist` | Current velocity command; zero on timeout, completion, shutdown, or invariant failure. |
| `topics.navigation_state`: `navigation_state` | `semaforr_msgs/msg/NavigationState` | Node state machine and action execution status. |
| `topics.decision_records`: `decision_records` | `semaforr_msgs/msg/DecisionRecord` | Replayable reasoning and execution trace for one stable decision ID; lifecycle updates reuse that ID. |
| `topics.crowd_field`: `crowd_field` | `social_context_msgs/msg/CrowdField` | Derived learned crowd diagnostic, never a second navigation input. |

The separate `why` node consumes `decision_records`, accepts
`semaforr_msgs/msg/ExplanationQuestion` on `why_questions`, and publishes
`semaforr_msgs/msg/ExplanationResponse` on `why_responses`. These explanation
topic names are parameters of the Why node rather than navigation-engine input
topics.

Visualization topics include `target_point`, `waypoint`, `all_targets`,
`remaining_targets`, `plan`, `original_plan`, `decision_pose`,
`decision_laser`, `region`, `door`, `trail`, `conveyor`, `hallway1` through
`hallway4`, `barrier`, `skeleton`, and graph marker topics. They are diagnostic
outputs in `frames.global`.

## Frames and TF

- `frames.global` defaults to `map`. Missions, maps, plans, crowd fields, and
  spatial models use this frame.
- `frames.scan` defaults to `base_laser_link`. Laser messages must declare this
  frame; static sensor mounting belongs in TF.
- Pose and social messages in another frame are transformed to the global frame
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
