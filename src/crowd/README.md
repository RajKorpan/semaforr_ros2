# SemaFORR crowd diagnostics

This ROS 2 package replaces the six legacy ROS 1 crowd learners with one
parameterized implementation. It subscribes only to
`social_context_msgs/msg/SocialObservation` and publishes `crowd_density`,
`crowd_risk`, and `crowd_flow` for visualization and explanation consumers.
SemaFORR navigation itself consumes `CrowdState` directly and does not ingest
these diagnostic grids.

The historical executable names remain as aliases to the new node so old
launch scripts can transition without restoring the removed `CrowdModel`
message. The nested ROS 1 sources are retained as migration reference, but the
top-level `semaforr_crowd` package is the only package discovered and installed
by colcon.

Launch the complete diagnostic sidecar (crowd grids plus action and plan
explanations) with:

```bash
ros2 launch semaforr_crowd social_diagnostics.launch.py
```

The navigation core remains independently launchable and has no dependency on
these diagnostic nodes.
