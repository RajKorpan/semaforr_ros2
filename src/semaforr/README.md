
# SemaFORR Navigation Package

The core navigation system for Social-SemaFORR, providing cognitively-inspired robot navigation in ROS2.

## Overview

SemaFORR is designed to enable robots to navigate complex environments using cognitive principles. It leverages a set of advisors and configurable parameters for flexible, intelligent path planning.

## Features

- Cognitive navigation algorithms
- ROS-independent domain model with explicit ROS message adapters
- RAII ownership for controllers, tasks, planners, graphs, and search state
- Typed, validated configuration with source-and-line diagnostics
- Controller facade decomposed into focused mission, learning, decision, and
  planning implementation units
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

Launch the tutorial configuration from the installed package:

```bash
ros2 launch semaforr stage_tutorial.launch.py
```

## Configuration

`config/semaforr.yaml` is the supported runtime configuration. It defines typed
action magnitudes, safety limits, mission policy, feature flags, planners,
advisors, and installed map/task paths as ROS parameters. Configuration parsing
is ROS-independent after the parameter boundary and completes before the
controller is constructed. Missing files, unknown names, duplicate settings,
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

## Refactoring baseline

The pre-refactor characterization harness, runtime scenario, sanitizer profile,
coverage profile, and known-behavior inventory are documented in
`test/fixtures/baseline/README.md`.

The ROS-independent `semaforr::domain` target, message adapters, build profiles,
installed package layout, and downstream-consumer checks are documented in
`docs/build-and-package.md`.

The implementation and verification status of Phases 0-9 is recorded in
`docs/phases-0-9-completion.md`.

Phase 10 and the per-representation observation/update/consumer contracts are
documented in `docs/phase-10-completion.md` and
`docs/spatial-learning.md`.

The event-driven ROS node architecture, parameters, state machine, and
sensor-loss verification are documented in `docs/phase-11-completion.md`.

The canonical social API, unified `CrowdModel`, learning strategies, planner
and advisor consumers, stale-data fallback, persistence, and diagnostic
projection are documented in `docs/social_navigation.md`.
