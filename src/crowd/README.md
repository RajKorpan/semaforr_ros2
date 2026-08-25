# SemaFORR crowd diagnostics

`semaforr_crowd` is retained temporarily as a compatibility notice. SemaFORR
owns social-observation adaptation, visibility-normalized crowd learning,
immutable crowd snapshots, planners, advisors, replay, and diagnostics. No
learned crowd model crosses a ROS message boundary.

Launching the former executable prints the migration notice and exits:

```bash
ros2 launch semaforr_crowd social_diagnostics.launch.py
```

New deployments should launch `semaforr` directly.
