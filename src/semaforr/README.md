
# SemaFORR Navigation Package

The core navigation system for Social-SemaFORR, providing cognitively-inspired robot navigation in ROS2.

## Overview

SemaFORR is designed to enable robots to navigate complex environments using cognitive principles. It leverages a set of advisors and configurable parameters for flexible, intelligent path planning.

## Features

- Cognitive navigation algorithms
- ROS-independent domain model with explicit ROS message adapters
- RAII ownership for controllers, tasks, planners, graphs, and search state
- Typed, validated configuration with source-and-line diagnostics
- Controller façade decomposed into focused mission, learning, decision, and
  planning implementation units
- Replaceable, value-returning interfaces for all three decision tiers
- Deterministic tier arbitration with safe empty and non-finite fallbacks
- ROS2 node integration
- Example configuration files for quick setup

## Usage

### Build

Make sure your workspace is built:

```bash
colcon build
source install/setup.bash
```

### Run SemaFORR Node

Launch the tutorial configuration from the installed package:

```bash
ros2 launch semaforr stage_tutorial.launch.py
```

### Node

- **semaforr_node**: Main entry point for navigation. Requires 6 parameters for configuration.

## Configuration

- `target_set`: List of navigation targets
- `map_config`: XML map configuration
- `map_dimensions`: Map size and boundaries
- `advisors`: Advisor configuration file
- `params`: General parameters for navigation

Example configuration files are provided in the `config/` directory.
Configuration parsing is ROS-independent and happens before the controller is
constructed. Missing files, unknown or duplicate settings, invalid values, and
malformed advisor, task, or dimensions rows fail fast with an actionable error.

## Refactoring baseline

The pre-refactor characterization harness, runtime scenario, sanitizer profile,
coverage profile, and known-behavior inventory are documented in
`test/baseline/README.md`.

The ROS-independent `semaforr::domain` target, message adapters, build profiles,
installed package layout, and downstream-consumer checks are documented in
`docs/build-and-package.md`.
