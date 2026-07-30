# Phase 0-9 completion audit

This records the phase audit performed in the ROS 2 Humble container on
2026-07-29. The final convergence pass subsequently removed the compatibility
implementation; current commands and results are recorded in
`phase-14-testing.md`.

## Verification snapshot

- Normal, sanitizer, and coverage builds complete.
- Normal, sanitizer, and coverage profiles each pass 67 tests with no failures.
- The 20-second deterministic ROS scenario records 400 poses, the repeated
  laser fixture, 11 decisions, command transitions, task transitions, planner
  data, vetoes, advisor diagnostics, and timing.
- Normal and sanitizer runtime traces match
  `test/fixtures/baseline/stage_tutorial.expected.json`.
- The sanitizer runtime exits cleanly with AddressSanitizer,
  UndefinedBehaviorSanitizer, and leak detection enabled and emits no report.
- Production coverage is 11.9% of lines and 36.5% of functions.

## Phase status

### Phase 0: behavioral baseline

Complete. The source contract, recorded sensor fixture, golden decision trace,
runtime verifier, known-defect inventory, warning inventory, performance
measurements, sanitizer profiles, and coverage report are repeatable. The
recorder uses deterministic 50 ms simulation steps so wall-clock jitter is not
part of the semantic comparison.

### Phase 1: build and package definition

Complete. CMake uses explicit source lists and C++20, exports real
`domain`, `planning`, `advisors`, and ROS-adapter libraries, uses supported
ROSIDL targets, installs headers/configuration/launch/docs, enables strict
warnings and `BUILD_TESTING`, and has a downstream consumer contract. The
manifest has audited ROS dependencies, version, description, and license
metadata. Inherited-source provenance is explicitly recorded in
`docs/licensing.md` because it cannot be legally confirmed from this repository
alone. Python is used only for installed launch/test/conversion utilities;
it is not embedded in the C++ package.

Custom diagnostics now live in the separate `semaforr_msgs` interface package,
so the C++ navigation package no longer owns ROSIDL generation.

### Phase 2: directory organization

Complete. Public headers use `.hpp` under responsibility-oriented
`include/semaforr` areas. Implementations live in matching `src` areas, tests
are split into `unit`, `integration`, `contracts`, and `fixtures`, and private
factory code remains private. Vendored TinyXML was removed; map parsing now has
a validated domain parser.

The inherited monolithic headers and ROS1 implementation were removed in the
final convergence pass. Public headers now describe only the focused system.

### Phase 3: ROS-independent domain model

Complete. `Action`, typed geometry, observations, velocity command, action
space, mission/task state, histories, recovery/crowd state, and `WorldModel`
are ROS-independent values. ROS conversion is isolated in adapters. Domain,
decision, graph, and geometry tests run without ROS initialization.

### Phase 4: ownership

Complete for exercised ownership paths. Decision results and actions are
returned by value, exclusive polymorphic ownership uses `std::unique_ptr`,
interfaces have virtual destructors, graph search owns temporary search state,
and nullable state uses explicit optional values. No allocation is performed
for the returned decision in the modern decision cycle. Sanitizer unit and
runtime profiles report no leak, invalid access, or undefined behavior.

ROS-managed nodes and publishers continue to use `std::shared_ptr`, as required
by ROS ownership conventions. Non-ROS polymorphic ownership uses
`std::unique_ptr`; production source contracts reject owning `new`/`delete`.

### Phase 5: configuration

Complete. ROS parameters are backed by YAML and converted into a fully
initialized typed configuration. Paths, finite sorted action arrays, bounds,
advisor registration/weights/parameters, planner construction, dimensions, and
task rows are validated with actionable diagnostics. Legacy `.conf` experiments
have a one-time converter.

### Phase 6: controller decomposition

Complete. `NavigationEngine`, `MissionManager`, `DecisionCoordinator`,
`SpatialLearningCoordinator`, and `PlanningCoordinator` have focused
interfaces. A value `DecisionResult` carries its source, vetoes, advisor
contributions, planner, and explanation. The former `Controller` and mutable
decision-statistics side channel were removed.

### Phase 7: decision tier interfaces

Complete. Mandatory and veto rules are distinct value-returning interfaces;
planners return typed status/results; advisors evaluate candidate spans and
return raw scores, weights, participation, and explanations. Advisor and
planner registries reject duplicate and unknown names.

### Phase 8: deterministic arbitration

Complete. Empty candidates produce safe stop, no participation uses a
configured fallback, non-finite values are rejected, the unscored-action policy
is explicit, vetoed actions cannot re-enter, diagnostics are sorted, raw and
weighted scores remain distinct, tie tolerance is configured, and an injected
RNG is seeded once. Equal inputs and seeds are covered by deterministic tests.

### Phase 9: planning and geometry

Complete. Typed meter/radian geometry centralizes angle normalization,
tolerances, distances, intersections, and coordinate assumptions. Domain graph
storage is separate from immutable A* search state; costs are named and paths
have typed status. Tests cover repeated searches, identical endpoints,
disconnected graphs, invalid vertices, and malformed maps. The validated map
parser is separate from navigation semantics and keeps units explicit.

## Commands

```bash
docker compose run --rm semaforr bash -lc \
  'source /opt/ros/humble/setup.bash &&
   colcon build --packages-select semaforr &&
   colcon test --packages-select semaforr &&
   colcon test-result --verbose'

src/semaforr/scripts/run_phase0.sh normal
src/semaforr/scripts/run_phase0.sh sanitizer
src/semaforr/scripts/run_phase0.sh coverage
```

The detailed warning, coverage, known-defect, and replay instructions are under
`test/fixtures/baseline`.
