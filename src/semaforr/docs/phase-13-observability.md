# Phase 13: logging, diagnostics, and observability

SemaFORR emits one `semaforr_msgs/msg/DecisionRecord` on the configured
`topics.decision_records` topic after each discrete action reaches a terminal
outcome. The same sequence number follows a decision from arbitration through
execution, so a subscriber or rosbag recording contains the complete
explanation in one message.

Each record contains the robot pose, task target and optional waypoint,
candidate actions, typed vetoes, raw advisor scores, weights and weighted
contributions, selected tier and policy, selected planner and action, decision
source, latency, action progress and target, execution duration, and terminal outcome.
Candidate, veto, and contribution ordering is deterministic.

Navigation state changes use `semaforr_msgs/msg/NavigationState` on
`topics.navigation_state`; they are no longer colon-encoded strings. Normal
state, task, and mission transitions are logged at INFO. Advisor calculations
are logged at DEBUG. Sensor loss, stale input, transform failure, recovery, and
action timeout paths use WARN. Startup configuration failures use ERROR.
Legacy direct `cout`/`cerr` diagnostics in the navigation, advisor, exploration,
and spatial-learning paths have been removed so console output is controlled by
ROS logger severity.

To inspect a run:

```bash
ros2 topic echo /decision_records semaforr_msgs/msg/DecisionRecord
ros2 topic echo /navigation_state semaforr_msgs/msg/NavigationState
```

For a reproducible diagnostic artifact, record both topics along with pose,
scan, social observation, and velocity command topics in the Phase 0 rosbag.
