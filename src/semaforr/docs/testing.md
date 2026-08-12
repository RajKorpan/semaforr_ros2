# Testing strategy

The test suite is split into unit, component, integration, regression, and
quality gates. All random arbitration tests use explicit seeds, and integration
fixtures use meters, radians, and the `map` frame.

Every CTest is labeled with its behavioral claim. Current unit, component,
integration, and regression tests carry `behavior_mode:modernized`. The matrix
and fail-closed configuration test carries
`behavior_mode:compatibility-contract`. No test currently carries the
`behavior_mode:compatibility` label, so the repository does not claim exact
reproduction. The machine-readable suite declaration is
`test/compatibility_modes.yaml`.

Run a claim-specific suite with:

```sh
ctest -L behavior_mode:modernized
ctest -L behavior_mode:compatibility-contract
```

## Coverage map

| Requirement | Test |
|---|---|
| Action construction and ordering | `domain_types_test`, `characterization_test` |
| Angles, expected poses, laser endpoints, goal tolerance | `navigation_behavior_test` |
| Victory, AvoidObstacles/hard safety, NotOpposite, Behind, Out, and Forward | `tier_one_component_test` |
| Obstacle vetoes | `tier_one_component_test`, `navigation_behavior_test`, `navigation_scenario_test` |
| Mission and task transitions | `domain_types_test`, `component_strategy_test` |
| Advisor scoring, weighting, tie/fallback safety | `decision_coordinator_test`, `component_strategy_test` |
| A* and unreachable graphs | `domain_planning_test` |
| Region and door geometry | `navigation_behavior_test` |
| Configuration validation | `configuration_test` |
| Tier 1, Tier 3, planning, spatial learning, execution | component GTests |
| Action feedback lifecycle, stable IDs, failure learning, duplicates, preemption, and controller restart | `semaforr_action_execution_lifecycle_test` |
| Nine navigation situations | `navigation_scenario_test`, `ros_execution_test` |
| Baseline decision trace | `navigation_strategy_test` |

The integration scenarios are deterministic equivalents to ROS bags. Their
catalog is `test/fixtures/scenarios/navigation_scenarios.json`; recorded bags may
replace an equivalent fixture without changing the assertions.

## Regression policy

Run `scripts/compare_decision_traces.py BASELINE CURRENT ANNOTATIONS`.
Unchanged decisions are `exact_match`. Every changed decision must be annotated
as `acceptable_intentional_improvement` or
`regression_requiring_correction`, with a nonempty rationale. Unclassified
differences and regressions make the command fail.

A trace comparison must also reject or separately classify runs whose behavior
mode, configuration fingerprint, or component manifest differs.

## Quality gates

Normal ROS 2 Humble verification:

```sh
colcon build --packages-up-to semaforr
colcon test --packages-select semaforr
colcon test-result --verbose
```

Sanitizer verification:

```sh
scripts/run_quality_checks.sh sanitizer
```

Coverage verification after a coverage-instrumented test run:

```sh
scripts/run_quality_checks.sh coverage
```

The coverage gate applies an 80% line threshold to the files listed in
`config/coverage_thresholds.json`. The source-quality manifest is checked during
`colcon test`; `.clang-format` and `.clang-tidy` define the formatting and
static-analysis policy. Modern component targets and production files
compile with warnings promoted to errors.

## Verified result

Verified in ROS 2 Humble on 2026-07-29:

- Normal profile: 29 CTest targets containing 131 test cases, 0 errors,
  0 failures, 0 skipped.
- Fresh-image whole workspace: 10 packages and 193 test cases, 0 errors,
  0 failures, 4 intentional skips.
- ASan/UBSan/LeakSanitizer profile: 29 CTest targets containing 131 test
  cases, 0 errors, 0 failures, 0 skipped.
- Regression trace: 11 exact matches and no classified or unclassified
  differences.
- Overall instrumented production line coverage: 57.5%; the enforced
  modified-core threshold is 80%.
- Modified-code line coverage:
  - `navigation_configuration.cpp`: 94.6%
  - `decision_coordinator.cpp`: 92.2%
  - `mission_manager.cpp`: 85.0%
  - `navigation_advisor.cpp`: 95.8%
  - `navigation_engine.cpp`: 81.8%
  - `obstacle_veto_rule.cpp`: 86.1%
  - `motion_model.cpp`: 94.7%
  - `domain_planner.cpp`: 86.5%
  - `planning_coordinator.cpp`: 100.0%

Every production library now uses the warning-as-error profile. The warnings
in `test/fixtures/baseline/build_warnings.md` describe the removed ROS1
baseline and remain only as characterization evidence. Formatting, static
analysis, sanitizer, coverage, and replay commands are enforced by the CI
profiles documented above.
