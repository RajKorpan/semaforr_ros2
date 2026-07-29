# Phase 14 testing strategy

The test suite is split into unit, component, integration, regression, and
quality gates. All random arbitration tests use explicit seeds, and integration
fixtures use meters, radians, and the `map` frame.

## Coverage map

| Requirement | Test |
|---|---|
| Action construction and ordering | `domain_types_test`, `characterization_test` |
| Angles, expected poses, laser endpoints, goal tolerance | `navigation_behavior_test` |
| Obstacle vetoes | `navigation_behavior_test`, `navigation_scenario_test` |
| Mission and task transitions | `domain_types_test`, `component_strategy_test` |
| Advisor scoring, weighting, tie/fallback safety | `decision_coordinator_test`, `component_strategy_test` |
| A* and unreachable graphs | `domain_planning_test` |
| Region and door geometry | `navigation_behavior_test` |
| Configuration validation | `configuration_test` |
| Tier 1, Tier 3, planning, spatial learning, execution | component GTests |
| Nine navigation situations | `navigation_scenario_test`, `ros_execution_test` |
| Baseline decision trace | `phase14_strategy_test` |

The integration scenarios are deterministic equivalents to ROS bags. Their
catalog is `test/fixtures/scenarios/phase14_scenarios.json`; recorded bags may
replace an equivalent fixture without changing the assertions.

## Regression policy

Run `scripts/compare_decision_traces.py BASELINE CURRENT ANNOTATIONS`.
Unchanged decisions are `exact_match`. Every changed decision must be annotated
as `acceptable_intentional_improvement` or
`regression_requiring_correction`, with a nonempty rationale. Unclassified
differences and regressions make the command fail.

## Quality gates

Normal ROS 2 Humble verification:

```sh
colcon build --packages-up-to semaforr
colcon test --packages-select semaforr
colcon test-result --verbose
```

Sanitizer verification:

```sh
colcon build --packages-select semaforr \
  --cmake-args -DSEMAFORR_ENABLE_SANITIZERS=ON
colcon test --packages-select semaforr
```

Coverage verification after a coverage-instrumented test run:

```sh
python3 scripts/check_coverage.py coverage/semaforr.filtered.info
```

The coverage gate applies an 80% line threshold to the files listed in
`config/coverage_thresholds.json`. The source-quality manifest is checked during
`colcon test`; `.clang-format` and `.clang-tidy` define the formatting and
static-analysis policy. Modern component targets and Phase 14 production files
compile with warnings promoted to errors. Legacy headers remain tracked by the
warning baseline until their extraction is complete.

## Verified result

Verified in ROS 2 Humble on 2026-07-29:

- Normal profile: 158 tests, 0 errors, 0 failures, 0 skipped.
- ASan/UBSan/LeakSanitizer profile: 158 tests, 0 errors, 0 failures, 0 skipped.
- Regression trace: 11 exact matches and no classified or unclassified
  differences.
- Modified-code line coverage:
  - `DecisionCoordinator.cpp`: 92.4%
  - `MissionManager.cpp`: 80.0%
  - `ObstacleVetoRule.cpp`: 86.1%
  - `MotionModel.cpp`: 95.0%
  - `PlanningCoordinator.cpp`: 100.0%

The refactored component libraries pass their warning-as-error build. The
whole-package build still emits warnings from inherited legacy headers,
principally `PathPlanner.hpp`; those warnings remain recorded in
`test/fixtures/baseline/build_warnings.md` and prevent claiming the
whole-repository zero-warning goal. The Humble image used for verification did
not contain `clang-format` or `clang-tidy`, so their checked-in policies were
not executed; the manifest-based static source gate did pass.
