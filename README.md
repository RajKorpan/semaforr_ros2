# Social-SemaFORR for ROS 2

Social-SemaFORR is a cognitively inspired navigation workspace for ROS 2
Humble. It combines deterministic decision tiers, graph planning, modular
spatial learning, structured diagnostics, and a stable social-observation API.

## Build and run the example

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
colcon test --packages-select semaforr
colcon test-result --verbose
ros2 launch semaforr example_simulation.launch.py
```

The installed example supplies its own map, mission, pose, and laser stream. It
exits after 20 seconds and writes a structured trace to
`~/.ros/semaforr/example-simulation.json`; no source-tree path or manual topic
publisher is required. Add `rviz:=true` to open the installed RViz layout.

## Docker

```bash
docker compose build
docker compose up
```

The image builds the copied ROS 2 workspace and runs the same example. Its
trace is written to `baseline-results/example-simulation.json`.

## Workspace packages

- `semaforr`: navigation domain, decision tiers, planners, spatial learning,
  ROS adapters, launch files, and tests
- `semaforr_msgs`: structured navigation and decision diagnostics
- `social_context_msgs`: canonical social observation and crowd-field messages
- `social_context`: social observation and trajectory-prediction producers
- `semaforr_bridge`: odometry and tracked-person adapters
- `semaforr_crowd`: optional crowd-field diagnostics
- `semaforr_examples`: installed retained maps and scenarios
- `why`, `why_plan`: decision and plan explanation tools

Start with the
[documentation index](src/semaforr/docs/README.md), especially the
[architecture overview](src/semaforr/docs/architecture.md),
[configuration reference](src/semaforr/docs/configuration-reference.md),
[topic/frame contract](src/semaforr/docs/topics-and-frames.md), and
[troubleshooting guide](src/semaforr/docs/troubleshooting.md).

ROS 1 crowd estimator sources are retained for reference but excluded from ROS
2 package discovery with `COLCON_IGNORE`.
