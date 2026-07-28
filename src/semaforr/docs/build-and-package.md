# Build and package structure

SemaFORR targets ROS 2 Humble and C++14. The package now separates the reusable
navigation implementation from the ROS executable:

- `semaforr::core` is the shared library exported to downstream CMake projects.
- `semaforr_node` contains only the ROS node entry point and links to the core.
- `CrowdModel.msg` is consumed through its generated C++ API.

## Source layout

Implementation and public headers are grouped by responsibility:

| Area | Responsibility |
| --- | --- |
| `core` | Actions, positions, and geometry primitives |
| `decision` | Agent state, beliefs, controller, tasks, and advisors |
| `exploration` | Local, frontier, highway, and circumnavigation strategies |
| `navigation` | Map, graph, A*, and path-planning infrastructure |
| `spatial` | Regions, trails, conveyors, barriers, doors, and hallways |
| `ros` | ROS node and visualization adapter |
| `vendor/tinyxml` | Isolated bundled TinyXML implementation |

Headers use package-qualified paths, for example:

```cpp
#include <semaforr/core/FORRAction.h>
#include <semaforr/navigation/PathPlanner.h>
```

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

The test suite includes behavior characterization, source/build contract checks,
launch-file checks, and a fixed-timestep contract for the runtime baseline
driver. `test/downstream` is a small consumer project used to verify that an
installed SemaFORR package can be found and linked:

```bash
cmake -S src/semaforr/test/downstream -B /tmp/semaforr-downstream
cmake --build /tmp/semaforr-downstream
/tmp/semaforr-downstream/semaforr_downstream_smoke
```

Run those commands only after sourcing the workspace install.

## Installed resources

Headers and the core library are installed under the package prefix. Runtime
configuration, launch files, and this documentation are installed beneath
`share/semaforr`; executables are available through `ros2 run`.
