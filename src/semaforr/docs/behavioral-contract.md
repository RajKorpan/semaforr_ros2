# Navigation behavioral contract

This document defines the required runtime behavior independently of its
research history. A component satisfies the contract only when its acceptance
tests pass; sharing a historical name is not sufficient.

This is the modernized runtime contract. Algorithm-fidelity status, intentional
deviations, and the unavailable compatibility target are defined in the
[behavioral compatibility matrix](compatibility-matrix.md).

## Runtime lifecycle

1. Construct an empty `domain::WorldModel` with no active target.
2. When initial exploration is enabled, enter
   `NavigationPhase::InitialExploration` before activating a mission target.
3. Finalize regions, skeleton connectivity, passage data, highways,
   intersections, and the highway graph after initial exploration.
4. Emit `initial_model_finalized` and `target_navigation_started`, then allow
   `MissionManager` to activate the first target.
5. Process each target through hard safety; the interleaved semantic Tier-1
   order (Victory, early vetoes, Enforcer, reactive planners, LLE, late
   vetoes); bounded Tier-2 planning; eligible Tier-3 arbitration; and command
   validation. Successful Tier-2 plan creation ends its cycle, so Enforcer
   first consumes that plan on the next cycle.
6. Allow reactive planners to interrupt action selection without blocking.
   Low-level exploration returns control after requesting replanning or
   reaching an explicit terminal condition.
7. Update every spatial representation on its declared `UpdateSchedule` and
   publish a new immutable revision only when its model changes.
8. Complete or skip the active target, publish target-boundary updates, and
   activate the next target.
9. Enter `MissionComplete` and return a typed pause after the mission ends.

## Component contracts

| Component | Implementation | Configuration | Required data | Produced data | Update schedule | Consumers | Acceptance criteria |
|---|---|---|---|---|---|---|---|
| Initial exploration | `ExplorationCoordinator`, `HighLevelExplorer` | `phases.initial_exploration` | pose, laser, candidate heap, passage grid, budgets | action or subgoal, candidate event, grid revision, completion reason | one nonblocking update per decision | spatial finalization | deterministic candidate lifecycle; mission remains inactive |
| Reactive exploration | `LowLevelExplorer` implementing `ReactivePlanner` | `exploration.reactive` | target, laser rays, unfinished cues, visibility, inclusion gaps | temporary action, cancellation reason, or replan request | when guidance is missing | navigation engine and Tier 2 | deterministic triggers, interruption, pursuit, and replanning |
| Regions | `RegionLearner` | `features.regions` | pose and laser | immutable region snapshot | observation and finalization | Enforcer, planners, advisors | revision changes only after mutation |
| Skeleton | `PassageSkeletonLearner` | `features.astar` or a planner that requires the learned skeleton | completed paths, regions, visibility, and trails | region skeleton or distinctly named sampled-path graph, plus component cache | profile-dependent successful-action or target-finalization schedule | skeleton, highway, and affordance planners | stable IDs and invalidation tests |
| Known grid | `KnownGridLearner` | `features.known_grid` | scan rays | sparse observation grid | every observation | Out and reactive exploration | only intersected cells change |
| Sensed occupancy | `SensedOccupancyLearner` | `features.sensed_occupancy` | classified scan rays | free/occupied evidence, confidence, conflicts and provenance | every observation | occupancy fusion and partial sensor-grid planning | invalid rays ignored; hit endpoints occupied |
| Inclusion grid | `InclusionGridLearner` | `features.inclusion_grid` | learned regions, region-skeleton supporting subtrails, successful LLE traversal | sparse learned-inclusion cells | target/model finalization and successful LLE motion | reactive exploration and coverage diagnostics | observations alone add no inclusion; region/subtrail projection and LLE growth are tested |
| Highways | `HighwayLearner`, `domain::HighwayGraph` | `features.highways` | passage grid and trails | highways, intersections, connected graph | incremental grid and exploration finalization | highway planner | extraction, serialization, component, and route-choice tests |
| Trails and conveyors | corresponding spatial learners | representation switches | completed motion | immutable snapshots | completed action | planners and advisors | incremental update tests |
| Doors and hallways | corresponding spatial learners | representation switches | regions and travel | immutable snapshots | target boundary or on demand | planners and advisors | rebuild and revision tests |
| Hard safety | `HardSafetyFilter` and command validation | safety envelope | observation and candidate actions | safe candidates and executable commands | every decision and command | all tiers | unsafe actions never reach publication |
| Tier 1 | named rules in `TierOneRegistry` | individual rule switches | decision context and plans | mandatory action, veto, operationalized step, or replan request | configured deterministic order | navigation engine | per-rule behavior and registry tests |
| Tier 2 | planner registry and `PlanningCoordinator` | individual planner switches | target and model snapshots | typed hierarchical plans | mission activation and replanning | Enforcer | planner validity, caching, and revision invalidation |
| Tier 3 | advisor registry | individual advisor switches | declared model dependencies | normalized action scores and explanations | after Tier-1 constraints | decision coordinator | metadata, dependency, directionality, and arbitration tests |
| Social navigation | live and learned crowd advisors | `social.*` master and component switches | fresh observations or learned crowd field | density, encounter-risk, and flow preferences | every eligible decision | Tier-3 arbitration | freshness, confidence, prediction, and ablation tests |

## Ordering and safety invariants

The validated Tier-1 order is `victory`, `avoid_obstacles`, `not_opposite`,
`enforcer`, `thru`, `behind`, `out`, `low_level_exploration`, `forward`, and
`precedent`. Hard safety is outside this configurable list and cannot be
disabled. Initial exploration owns its phase, reactive exploration is an
interruptible planner, and exploration-oriented advisors remain ordinary
Tier-3 heuristics.

At runtime, Victory receives the first mandated-action opportunity. Early
AvoidObstacles and NotOpposite vetoes are retained when Enforcer gets its
plan-operationalization opportunity. Only then may Thru, Behind, Out, and LLE
act; Forward and Precedent run afterward. Tier 2 runs only after the complete
Tier-1 pass and only without an active plan. A newly installed plan ends the
cycle instead of being operationalized immediately. Tier 3 is unreachable
after any successful Tier-1 selection or successful Tier-2 plan creation.
Immediate Tier-2 failures are limited by
`tiers.tier2.maximum_planning_attempts_per_task`; reaching the limit abandons
the plan attempt and makes LLE eligible. `DecisionCycleEvent` records make
every run, continuation, Tier-2 return, and final attribution observable.
