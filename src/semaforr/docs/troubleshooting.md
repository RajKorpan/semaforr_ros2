# Troubleshooting

## The node exits during configuration

Read the complete `ERROR` line; configuration diagnostics include the parameter
or source line. Confirm the map and mission are installed, not source-tree
paths:

```bash
ros2 pkg prefix semaforr
ros2 launch semaforr stage_tutorial.launch.py
```

Unknown advisor/planner names are case-sensitive. Crowd-cost planners require
`skeleton` and `social.learning.enabled`.

## It stays in `WaitingForSensors`

Inspect topic types, timestamps, frames, and QoS:

```bash
ros2 topic info pose --verbose
ros2 topic info scan_raw --verbose
ros2 topic echo navigation_state
```

Pose and scan must be newer than `timing.sensor_timeout_s`, close within
`timing.sensor_sync_tolerance_s`, and use compatible ROS time. The scan frame
must equal `frames.scan`. A pose in another frame needs a valid timestamped TF
transform to `frames.global`.

## The robot repeatedly stops

This is deliberate safety behavior when sensors are stale, an action times out,
odometry resets, no candidate survives, or shutdown begins. Inspect
`navigation_state`, the latest `decision_records`, and `WARN` logs. Check that
action velocity, tolerance, and timeout settings match the robot rather than
increasing the timeout blindly.

## Social advisors do not participate

Echo `social_observations` and verify stable IDs, confidence, timestamps,
prediction timestamps, and frame. Data older than `social.maximum_age_s`,
below `social.minimum_confidence`, or lacking TF is ignored. `crowd_field` is an
output; publishing it does not feed navigation.

## RViz is empty or reports frame errors

Start the supplied layout with:

```bash
ros2 launch semaforr example_simulation.launch.py rviz:=true
```

Set RViz Fixed Frame to the configured global frame. Real deployments must
publish their TF tree. The deterministic example emphasizes plan, pose, target,
and learned-model outputs and does not emulate a complete robot TF tree.

## Docker build fails

Docker Desktop must use Linux containers and have network access for apt and
rosdep. Rebuild without stale layers:

```bash
docker compose build --pull
docker compose up
```

The Docker build compiles the copied workspace; it does not depend on a host
`build/` or `install/` directory. Superseded ROS 1 crowd packages have been
removed from the active tree.

## Tests fail only after an incremental build

Remove only generated build products, then rebuild:

```bash
rm -rf build install log
colcon build
colcon test --packages-select semaforr
colcon test-result --verbose
```

Never delete or edit source fixtures to make a regression comparison pass.
Classify intentional differences in the Phase 14 regression report.
