# Testing strategy

Replay round-trip, deterministic reproduction, divergence attribution, and
configuration dependency checks are covered by `semaforr_replay_test` and
`semaforr_configuration_test`. See `reproducibility.md` for the trace contract.

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

`tier_two_enforcer_test.cpp` is the focused planning/enforcement suite. It verifies grid
lookahead, obstacle-safe shortcuts, deviation and completion, exact dependency
invalidation, typed skeleton/highway execution, planner declarations, and full
range-vote evidence. The exploration, Chapter-3 representation, spatial
learning, and revision-dependency suites cover the connected lifecycle and
Tier ordering.

Run a claim-specific suite with:

```sh
ctest -L behavior_mode:modernized
ctest -L behavior_mode:compatibility-contract
```

## Dependency-ordered coverage map

Tests follow production dependencies. A failure in an earlier row is
resolved before interpreting results from a later row.

| Order | Boundary | Executable evidence |
|---:|---|---|
| 1 | Geometry: parsing, installation, visibility, occupancy, negative coordinates, inflation | `semaforr_map_parser_test`, `semaforr_map_runtime_test`, `semaforr_grid_geometry_test`, `semaforr_grid_layers_test` |
| 2 | Action lifecycle: selection, start, success, partial movement, failure, cancellation, preemption, duplicate/stale feedback | `semaforr_action_execution_lifecycle_test`, `semaforr_ros_execution_test` |
| 3 | Learned representations | `semaforr_spatial_learning_test`, `semaforr_chapter3_representations_test`, `semaforr_exploration_planning_test`, `semaforr_crowd_model_test` |
| 4 | Planning, voting, operationalization, repair, and exact invalidation | `semaforr_domain_planning_test`, `semaforr_tier_two_enforcer_test`, `semaforr_revision_dependency_test` |
| 5 | Tier-cycle control flow | `semaforr_decision_coordinator_test`, `semaforr_spatial_learning_test`, `semaforr_tier_one_component_test` |
| 6 | Explanation golden fixtures | `why_system_test` and `why_explanations.golden` |
| 7 | Comparable performance measurements | `semaforr_performance_regression_test`, `semaforr_snapshot_projection_test` |
| 8 | End-to-end deterministic environments | `semaforr_navigation_scenario_test`, `semaforr_environment_regression_test` |
| 9 | Documentation/catalog/topic/configuration consistency | `semaforr_documentation_contract_test` |

### Representation evidence matrix

The shared learner suite checks that all twelve learners start empty, declare
minimum-evidence schedules, publish independently, increment revisions only on
meaningful changes, serialize with a representation payload, and produce the
same revision and byte-for-byte serialization from the same fixture. A second
rebuild proves that an unchanged model neither advances its revision nor
changes its archive. Specialized merge and minimum-evidence assertions are:

| Representation | Minimum, incremental, and merge evidence |
|---|---|
| Trails | `TrailCompatibility.SelectsHandComputedHistoricalVisibilityMarkers`; completed and failed paths |
| Conveyors | repeated-success strengthening and failed-traversal exclusion |
| Regions | minimum-range creation and deterministic overlap reconciliation |
| Doors/exits | first-class exit merge and exit-derived door arcs |
| Hallways | directional parent inference, heatmap/component merge, width and area |
| Barriers | laser wall evidence through grid/obstacle geometry fixtures |
| Region skeleton | region nodes, direct transition merge, shortest operational subtrail, stable components |
| Familiarity grid | empty sparse state, incremental ray integration, expansion, revision and geometry archive round trip |
| Sensed occupancy | free/hit/min/max/invalid evidence, conflict merge, decay and expansion |
| Inclusion grid | region and subtrail projection plus successful-LLE-only updates |
| Highways | incremental labels, smoothing, intersections, graph edges, schema archive, negative coordinates |
| Circumstances | minimum evidence gate, normalized case merge, outcomes, and save/load round trip |

Grid geometry, HLE passage grids, crowd fields, and circumstance cases have
public load APIs and therefore use full save/load/save round trips. Other
learner snapshots currently have a deterministic schema archive rather than a
runtime restore API; their tests validate payload identity and stable archives
without claiming that diagnostic JSON can be reloaded as a world model.

| Requirement | Test |
|---|---|
| Action construction and ordering | `semaforr_domain_types_test` |
| Angles, expected poses, laser endpoints, goal tolerance | `navigation_behavior_test` |
| Victory, AvoidObstacles/hard safety, NotOpposite, Behind, Out, and Forward | `tier_one_component_test` |
| Obstacle vetoes | `tier_one_component_test`, `navigation_behavior_test`, `navigation_scenario_test` |
| Mission and task transitions | `domain_types_test`, `component_strategy_test` |
| Advisor scoring, weighting, tie/fallback safety | `decision_coordinator_test`, `component_strategy_test` |
| A* and unreachable graphs | `domain_planning_test` |
| Region and door geometry | `navigation_behavior_test` |
| Configuration validation | `semaforr_configuration_test`, `semaforr_configuration_contract_test` |
| Tier 1, Tier 3, planning, spatial learning, execution | component GTests |
| Action feedback lifecycle, stable IDs, failure learning, duplicates, preemption, and controller restart | `semaforr_action_execution_lifecycle_test` |
| Deterministic navigation situations | `navigation_scenario_test`, `environment_regression_test`, `ros_execution_test` |
| Baseline decision trace | `navigation_strategy_test` |

The integration scenarios are deterministic equivalents to ROS bags. Their
catalog is `test/fixtures/scenarios/navigation_scenarios.json`; recorded bags may
replace an equivalent fixture without changing the assertions.

The environment regression suite covers a simple corridor, a doorway, a
hallway network, crossing highways, a dead end, a large room, a dynamic
obstacle, failed movement, and a negative-coordinate map. Simulator-independent
fixtures invoke the same learners, occupancy fusion, planners, and lifecycle
objects used at runtime.

## Golden explanations

`src/why/test/fixtures/why_explanations.golden` contains stable semantic
fragments, not brittle full paragraphs. The fixture covers Tier-1 mandates,
Tier-3 support and opposition, confidence qualification, counterfactuals,
recorded plan comparison, alternative routes, and egocentric direction words.
The test constructs one trace and verifies every public explanation category
against those claims.

`semaforr_tier_advisor_test` exercises commonsense and learned-spatial
rationales with favored, disfavored, and abstention or ambiguous cases. It
also checks every registered production advisor's ten-point normalization
contract, raw-comment preservation, `0`/`10` extrema, neutral `5` for an
all-equal set, and representation provenance. Social-navigation and
learned-crowd rationale fixtures remain in `semaforr_social_navigation_test`
and `semaforr_crowd_model_test` and declare the same normalization contract.

## Performance measurements

Run the non-flaky measurement suites with:

```sh
ctest -L test_kind:performance --output-junit performance.xml
```

GoogleTest properties record mean decision latency, allocation count and bytes,
large-grid represented versus stored cells, projection cost, HLE cue-processing
time, hallway pair-processing time, serialization time and size, and plan-cache
hit rate. Functional invariants are hard gates; timing values are observations
for same-host regression comparison rather than universal wall-clock limits.

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

## Reporting verification

Do not copy a historical test count or coverage percentage into a publication
as if it described the current checkout. Record the source revision and attach
the output of `colcon test-result --verbose`, the claim-specific CTest labels,
the replay/configuration fingerprint, and the coverage artifact produced by
that revision. The CI workflow and test inventory in `CMakeLists.txt` are the
authority for the current suite.

Production libraries use the warning-as-error profile. The warnings in
`test/fixtures/baseline/build_warnings.md` describe the removed baseline and
remain characterization evidence only.
