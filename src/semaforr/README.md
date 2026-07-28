
# SemaFORR Navigation Package

The core navigation system for Social-SemaFORR, providing cognitively-inspired robot navigation in ROS2.

## Overview

SemaFORR is designed to enable robots to navigate complex environments using cognitive principles. It leverages a set of advisors and configurable parameters for flexible, intelligent path planning.

## Features

- Cognitive navigation algorithms
- ROS-independent domain model with explicit ROS message adapters
- RAII ownership for controllers, tasks, planners, graphs, and search state
- Highly configurable via parameters and advisor files
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

## Refactoring baseline

The pre-refactor characterization harness, runtime scenario, sanitizer profile,
coverage profile, and known-behavior inventory are documented in
`test/baseline/README.md`.

The ROS-independent `semaforr::domain` target, message adapters, build profiles,
installed package layout, and downstream-consumer checks are documented in
`docs/build-and-package.md`.
