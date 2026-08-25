<!-- File overview: This file documents or configures map operation behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/map-operation.md`. -->

# Optional static maps

SemaFORR defaults to **mapless** operation. A static map is optional prior
knowledge for comparison planners; it is not the learned world model.

## Modes

| Mode | Simulator geometry | SemaFORR static map | Grid planners |
|---|---|---|---|
| Mapless/live | independent or absent | absent | Static-map planners disabled; partial sensor-grid planners optional |
| Hidden simulation map | loaded by simulator | absent | Static-map planners disabled; partial sensor-grid planners optional |
| Map-enabled simulation | loaded by simulator | loaded independently | configurable |
| Known real-world map | not applicable | loaded | configurable |

`map.mode=mapless` is a normal startup mode. The resolver and parser are not
called, no static-map raster or index is allocated, and learned grids, HLE,
LLE, trails, regions, highways, skeletons, local safety, and sensor processing
continue according to their own switches.

`map.mode=map_enabled` resolves and parses `map.path`, validates geometry
against the configured origin and bounds, rasterizes immutable occupancy, and
publishes the four capabilities `map_available`, `map_geometry_available`,
`map_occupancy_available`, and `map_based_planning_available`. `distance` and
the crowd grid planners require the last capability. `sensor_distance` uses
only partial sensed occupancy. Region, hallway, trail, and conveyor planners
use static occupancy when available and otherwise may use partial sensed
occupancy. Skeleton and highway planners consume learned representations and
remain map-independent.

## Resolution and formats

Resolution is deterministic:

1. an absolute path;
2. `package://<package>/<relative-path>`;
3. a relative path under a configured package share or working directory;
4. a name under the installed `semaforr_examples/core` tree.

For example, `map-a` resolves to `core/map-a/map-aS.xml`, and
`package://semaforr_examples/core/map-a/map-aS.xml` is explicit and portable.
Custom maps outside the repository should use an absolute path or be installed
in another ROS package and referenced with `package://`.

The legacy example tree contains project XML files, Menge scene (`*S.xml`),
behavior (`*B.xml`), view (`*V.xml`), roadmap text, PNG, target, and dimensions
files. Static-map loading currently supports the `ObstacleSet` geometry in
Menge scene XML. Behavior, view, image, and roadmap files are intentionally not
accepted as static geometry. Bounds, origin, occupancy resolution, and
inflation are explicit SemaFORR parameters rather than inferred from a
developer-specific file.

## Ownership and provenance

`NavigationEngineAdapter` resolves and exclusively owns the optional robot map
before planner registration. `WorldModel` holds a non-owning
`const StaticMap*` view
whose lifetime is bounded by that composition root. The object contains its canonical
source, format, bounds, wall segments, polygons, occupancy, revision, and
`StaticMap` provenance. It cannot be mutated through the world model.

Spatial learners mutate only sensor-derived representations in
`WorldModel::spatial`; they never overwrite the static map. Live scans remain
current local evidence. A simulator owns its environment map separately and
may read the same file independently without granting SemaFORR access.
Static-map geometry does not change after successful startup. Map-aware
visualization publishes a separate `static_map_geometry` marker when enabled.

## Failure and diagnostics

`map.on_load_failure=fail_startup` rejects an unresolved, unsupported,
malformed, out-of-bounds, or empty requested map. `disable_map` continues in a
mapless capability state and suppresses every planner requiring a static map.
It never registers a grid planner over an empty grid. Startup logs and decision
manifests report map mode/status/source, planning capability, and planners that
were enabled or disabled due to map availability.

The deterministic simulation launch keeps these settings independent:

```bash
# A: simulator map, robot map hidden (the default)
ros2 launch semaforr example_simulation.launch.py

# B: simulator and robot independently load the map
ros2 launch semaforr example_simulation.launch.py \
  semaforr_map_mode:=map_enabled map_based_planning:=true \
  map_planners:=distance,skeleton

# C: no simulator map and no robot map
ros2 launch semaforr example_simulation.launch.py \
  simulator_environment_map:='' semaforr_map_mode:=mapless

# D: robot uses a custom known map
ros2 launch semaforr example_simulation.launch.py \
  semaforr_map_mode:=map_enabled semaforr_map_path:=/data/site.xml \
  map_based_planning:=true map_planners:=distance
```
