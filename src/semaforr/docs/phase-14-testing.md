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
scripts/run_phase0.sh sanitizer
```

Coverage verification after a coverage-instrumented test run:

```sh
scripts/run_phase0.sh coverage
```

The coverage gate applies an 80% line threshold to the files listed in
`config/coverage_thresholds.json`. The source-quality manifest is checked during
`colcon test`; `.clang-format` and `.clang-tidy` define the formatting and
static-analysis policy. Modern component targets and Phase 14 production files
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
  - `Configuration.cpp`: 94.6%
  - `DecisionCoordinator.cpp`: 92.2%
  - `MissionManager.cpp`: 85.0%
  - `NavigationAdvisor.cpp`: 95.8%
  - `NavigationEngine.cpp`: 81.8%
  - `ObstacleVetoRule.cpp`: 86.1%
  - `MotionModel.cpp`: 94.7%
  - `DomainPlanner.cpp`: 86.5%
  - `PlanningCoordinator.cpp`: 100.0%

Every production library now uses the warning-as-error profile. The warnings
in `test/fixtures/baseline/build_warnings.md` describe the removed ROS1
baseline and remain only as characterization evidence. Formatting, static
analysis, sanitizer, coverage, and replay commands are enforced by the CI
profiles documented above.
