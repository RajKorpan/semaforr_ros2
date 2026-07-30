# Build and run

## Native ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
ros2 launch semaforr example_simulation.launch.py
```

The example loads the installed default YAML, compact open-room map, mission,
and deterministic
pose/scan simulator. It needs no manual publishers or source paths. After 20
seconds it exits and writes `~/.ros/semaforr/example-simulation.json`.

Use `rviz:=true`, change `duration`, or exercise sensor-loss handling:

```bash
ros2 launch semaforr example_simulation.launch.py \
  rviz:=true duration:=30.0
ros2 launch semaforr example_simulation.launch.py \
  duration:=8.0 sensor_cutoff:=3.0
```

`stage_tutorial.launch.py` launches navigation without the fixture for a real
robot or external simulator. It expects the documented pose, scan, TF, and
optional social inputs.

## Docker

```bash
docker compose build
docker compose up
```

The image copies and builds the ROS 2 workspace during `docker build`, sources
both Humble and the installed overlay in its entrypoint, and runs the same
example. The trace appears at
`baseline-results/example-simulation.json` on the host.

To run another installed command:

```bash
docker compose run --rm semaforr \
  ros2 launch semaforr example_simulation.launch.py duration:=5.0
```

## Installed example assets

The compact example map and mission live in
`share/semaforr/config/example`; the retained tutorial lives under
`share/semaforr/config/stage_tutorial`. Launch files locate both through the
ament index. `share/semaforr/rviz/semaforr.rviz` is the installed RViz layout.
The larger retained scenario collection is installed by the
`semaforr_examples` package.

## CI

`.github/workflows/ros2-humble.yml` builds every ROS 2 package in Humble, runs
source-quality, formatting, static-analysis, unit, contract, integration,
sanitizer, and modified-code coverage gates, checks all colcon results, and
builds the deployment image. Local commands above are the same commands used
by CI.
