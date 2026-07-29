# Build and package structure

SemaFORR targets ROS 2 Humble and C++14. The package separates its
ROS-independent navigation model from message transport and the ROS executable:

- `semaforr::domain` is the canonical shared library for navigation logic. Its
  sources and public model headers contain no ROS includes or message types.
- `semaforr::ros_adapters` converts ROS messages to the domain value types at
  the process boundary.
- `semaforr::core` is a compatibility target that forwards to
  `semaforr::domain`.
- `semaforr_node` owns subscriptions, publishers, and visualization and links
  the domain and adapter targets.
- `CrowdModel.msg` is consumed through its generated C++ API.

## Source layout

Implementation and public headers are grouped by responsibility:

| Area | Responsibility |
| --- | --- |
| `config` | Typed configuration values, parsers, and validation |
| `core` | Actions, positions, and geometry primitives |
| `domain` | ROS-independent sensor, pose, and crowd value types |
| `decision` | Agent state, beliefs, controller, tasks, and advisors |
| `exploration` | Local, frontier, highway, and circumnavigation strategies |
| `navigation` | Map, graph, A*, and path-planning infrastructure |
| `spatial` | Regions, trails, conveyors, barriers, doors, and hallways |
| `ros` | ROS node and visualization adapter |
| `vendor/tinyxml` | Isolated bundled TinyXML implementation |

Headers use package-qualified paths, for example:

```cpp
#include <semaforr/core/FORRAction.h>
#include <semaforr/domain/SensorTypes.h>
#include <semaforr/navigation/PathPlanner.h>
```

Domain components accept `semaforr::domain::LaserScan`, `PoseArray`, and
`CrowdModel`. ROS callbacks convert incoming messages with
`semaforr::ros::toDomain`; ROS publishers convert outbound scans with
`semaforr::ros::toRos`.

## Controller decomposition

`Controller` remains the public façade used by the ROS node and downstream
consumers. Its implementation is grouped into focused translation units:

| File | Responsibility |
| --- | --- |
| `Controller.cpp` | Construction and configuration-driven assembly |
| `ControllerMission.cpp` | Sensor updates, task lifecycle, and top-level action orchestration |
| `ControllerLearning.cpp` | Spatial learning and navigation-graph maintenance |
| `ControllerDecision.cpp` | Tier-one enforcement and tier-three advisor voting |
| `ControllerPlanning.cpp` | Tier-two plan generation, scoring, and selection |

This keeps existing call sites and decision state intact while making each
workflow independently discoverable and reducing the original monolithic
implementation to bounded, responsibility-specific files. A source contract
checks method placement, file size, and the stable public façade.

## Configuration boundary

`semaforr::config::loadConfiguration` reads the five runtime files into one
ROS-independent `Configuration` value before constructing `Controller`.
Controller parameters, planner switches, dimensions, advisors, and tasks each
have explicit types. The original five-path `Controller` constructor remains
as a compatibility adapter and delegates to the same loader.

The existing file formats and tutorial values are unchanged. Blank lines,
whole-line comments, and inline `#` comments are accepted. Validation rejects:

- missing or unreadable files;
- missing, unknown, or duplicate parameter keys;
- values that are not finite numbers, integer limits, or `0`/`1` flags;
- duplicate advisors or advisor rows with states other than `t` and `f`;
- malformed dimensions and task rows; and
- empty or oversized action lists.

Errors identify the source file and line when a row is responsible. The
configuration parser is part of `semaforr::domain`, has no ROS dependencies,
and can also parse streams directly for tests and embedding.

## Ownership model

Project code uses values for small, mandatory components and
`std::unique_ptr` for polymorphic or dynamically assembled ownership:

- `RobotDriver` uniquely owns its controller and visualizer.
- `Controller` uniquely owns beliefs, explorers, planners, and advisors.
- `AgentState` uniquely owns tasks while agenda and current-task pointers are
  non-owning views.
- `PathPlanner` can uniquely own its graphs; `Graph` uniquely owns nodes and
  edges.
- A* owns all virtual search nodes for the duration of the search object.

Raw pointers remain only as non-owning observers where existing algorithms
expect pointer syntax. Bundled TinyXML retains its upstream memory-management
implementation.

The C++ node does not embed Python. Python remains a runtime dependency only for
the optional baseline recorder installed as `semaforr_record_baseline`.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select semaforr --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Optional instrumentation profiles are mutually exclusive:

```bash
colcon build --packages-select semaforr \
  --cmake-args -DSEMAFORR_ENABLE_SANITIZERS=ON

colcon build --packages-select semaforr \
  --cmake-args -DSEMAFORR_ENABLE_COVERAGE=ON
```

## Test

```bash
colcon test --packages-select semaforr
colcon test-result --verbose
```

The test suite includes behavior characterization, domain-value and
message-adapter checks, ownership/destruction checks, configuration parser and
validation checks, source/build boundary contracts, launch-file checks, and a
fixed-timestep contract for the runtime baseline driver.
`test/downstream` verifies both the canonical `semaforr::domain` target and the
`semaforr::core` compatibility target from an installed package:

```bash
cmake -S src/semaforr/test/downstream -B /tmp/semaforr-downstream
cmake --build /tmp/semaforr-downstream
/tmp/semaforr-downstream/semaforr_downstream_smoke
/tmp/semaforr-downstream/semaforr_core_compat_smoke
```

Run those commands only after sourcing the workspace install.

## Installed resources

Headers, the domain library, and the ROS adapter library are installed under
the package prefix. Runtime configuration, launch files, and this documentation
are installed beneath `share/semaforr`; executables are available through
`ros2 run`.
