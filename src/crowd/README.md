# SemaFORR crowd diagnostics

`semaforr_crowd` is now a diagnostic adapter, not a second crowd learner.
SemaFORR consumes the one canonical
`social_context_msgs/msg/SocialObservation`, learns a visibility-normalized
crowd field inside the ROS-independent navigation core, and publishes the
derived `social_context_msgs/msg/CrowdField` snapshot.

This package converts that snapshot into:

- `crowd_density` (`nav_msgs/msg/OccupancyGrid`)
- `crowd_risk` (`nav_msgs/msg/OccupancyGrid`)
- `crowd_flow` (`visualization_msgs/msg/MarkerArray`)

The grids are visualization products and are never read back into navigation.
The legacy count, discount, CUSUM, and Thompson variants are selected through
the SemaFORR `social.learning.estimator` parameter; they are not separate ROS
nodes.

Run the diagnostic consumers alongside a running SemaFORR node with:

```bash
ros2 launch semaforr_crowd social_diagnostics.launch.py
```

The superseded ROS 1 packages have been removed. Their history remains
available in Git; active source contains only the ROS 2 diagnostic adapter.
