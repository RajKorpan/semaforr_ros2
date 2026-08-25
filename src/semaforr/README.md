<!-- File overview: This file documents or configures readme behavior for the SemaFORR navigation package. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `README.md`. -->


# SemaFORR Navigation Package

The core navigation system for Social-SemaFORR, providing cognitively-inspired robot navigation in ROS2.

## Overview

SemaFORR is designed to enable robots to navigate complex environments using cognitive principles. It leverages a set of advisors and configurable parameters for flexible, intelligent path planning.

## Features

- Cognitive navigation algorithms
- ROS-independent domain model with explicit ROS message adapters
- RAII ownership for tasks, planners, learners, graphs, and search state
- Typed, validated configuration with source-and-line diagnostics
- Focused mission, learning, decision, and planning coordinators with no
  monolithic controller facade
- Replaceable, value-returning interfaces for all three decision tiers
- Deterministic tier arbitration with safe empty and non-finite fallbacks
- Decomposed path planning and robust geometry edge-case handling
- Independently enabled, inspected, rebuilt, and serialized spatial learners
- Responsive callback/timer ROS 2 node with synchronized sensors and safe-stop
  action execution
- Unified live-and-learned crowd model with visibility-normalized density,
  encounter-risk, and directional-flow evidence
- One canonical social observation input and a separate derived crowd-field
  diagnostic output
- Example configuration files for quick setup

## Usage

### Build

Make sure your workspace is built:

```bash
colcon build --packages-select semaforr
source install/setup.bash
colcon test --packages-select semaforr
colcon test-result --verbose
```

### Run SemaFORR Node

Run the complete deterministic example from installed package data:

```bash
ros2 launch semaforr example_simulation.launch.py
```

It supplies simulator geometry, a mission, pose, and scan inputs and writes a
structured trace to `~/.ros/semaforr/example-simulation.json`. By default the
simulator map is hidden from SemaFORR; use the documented map-mode launch
arguments to grant the robot static-map access. Use `rviz:=true` for the
installed RViz layout. `stage_tutorial.launch.py` is the navigation-only launch
for a real robot or external simulator.

## Configuration

`config/semaforr.yaml` is the supported runtime configuration. It defines typed
action magnitudes, safety limits, mission policy, feature flags, planners,
advisors, and installed map/task paths as ROS parameters. Configuration parsing
is ROS-independent after the parameter boundary. Static validation completes
before adapter construction; map and planner capability validation completes
during adapter startup. Missing files, unknown names, duplicate settings,
non-finite or unsorted values, inconsistent array sizes, and malformed map/task
data fail at startup with an actionable diagnostic.

Convert a retained legacy experiment once with:

```bash
ros2 run semaforr semaforr_convert_legacy_config \
  --advisors old/advisors.conf \
  --parameters old/params.conf \
  --map old/map.xml \
  --tasks old/target.conf \
  --dimensions old/dimensions.conf \
  --output converted.yaml
```

## Architecture and verification

The pre-refactor characterization harness, runtime scenario, sanitizer profile,
coverage profile, and known-behavior inventory are documented in
`test/fixtures/baseline/README.md`.

The ROS-independent `semaforr::domain` target, message adapters, build profiles,
installed package layout, and downstream-consumer checks are documented in
`docs/build-and-package.md`.

The per-representation observation, update, and consumer contracts are
documented in `docs/spatial-learning.md`. The event-driven ROS node
architecture, parameters, state machine, and sensor-loss behavior are covered
by `docs/architecture.md`, `docs/topics-and-frames.md`, and `docs/testing.md`.

The canonical social API, unified `CrowdModel`, learning strategies, planner
and advisor consumers, stale-data fallback, persistence, and diagnostic
projection are documented in `docs/social-navigation.md`.

## Documentation

The maintained documentation starts at `docs/README.md` and includes the
architecture, decision tiers, advisor and planner catalogs, full configuration
reference, topic/frame contract, spatial learning, troubleshooting,
contributor, deployment, and legacy migration guides.
