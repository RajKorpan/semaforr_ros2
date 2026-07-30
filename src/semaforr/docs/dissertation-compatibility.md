# Dissertation compatibility specification

This is the behavioral contract for Chapters 3 and 4. A component is
compatible only when its acceptance tests pass; sharing a historical name is
not sufficient.

## Required lifecycle

1. Construct an empty `domain::WorldModel`; no target is active.
2. If configured, enter `NavigationPhase::InitialExploration`. HLE exclusively
   owns actions during this phase.
3. HLE finalizes regions, skeleton, passage/highway grids, intersections, and
   the highway graph. Finalization publishes new immutable revisions.
4. Emit `initial_model_finalized` and `target_navigation_started`, then permit
   `MissionManager` to activate the first target.
5. For each target: hard safety → ordered Tier 1/reactive control → Tier 2
   planning/replanning → Tier 3 arbitration → command safety validation.
6. A reactive planner may temporarily control actions. LLE hands control back
   after requesting Tier-2 replanning or reaching a typed terminal reason.
7. Update each representation only on its declared `UpdateSchedule`.
8. Complete or skip the target, publish target-boundary models, then activate
   the next target.
9. Enter `MissionComplete` and return a typed pause forever.

## Component matrix

| Dissertation component | Current equivalent | Missing implementation | Intended package / namespace | Switch | Required data | Produced data | Schedule | Consumers | Tests and acceptance criteria |
|---|---|---|---|---|---|---|---|---|---|
| HLE | `HighwayExplorer` | Full dissertation candidate-return/pursuit policy remains an acceptance gap | `semaforr::exploration` | `phases.initial_exploration` | pose, laser, budgets | actions, candidates, passages, events | initial phase | finalizer | deterministic actions; mission remains inactive |
| LLE | `LowLevelExplorer` | Candidate-pursuit detail remains an acceptance gap | `semaforr::planning` | `exploration.reactive` | target, connectivity, progress, inclusion gaps | reactive action or replan request | missing guidance | Tier 2 | missing connectivity invokes one revision-scoped replan |
| Passage | `PassageCandidate` | Stable completed-passage record missing | `semaforr::exploration` | HLE | candidate pursuit | passage cue | candidate completion | highway learner | candidate tests |
| Regions | `RegionLearner` | None in lifecycle scope | `semaforr::spatial` | `features.regions` | pose, laser | region snapshot | observation/finalization | Enforcer, advisors | revision changes after mutation |
| Skeleton | `PassageSkeletonLearner` | Connected-component cache deferred | `semaforr::spatial` | `features.astar` | exploration path | stable graph | incremental/finalization | SkeletonPlan, HighwayPlan | finalization test |
| Known grid | `KnownGridLearner` | Sparse storage deferred | `semaforr::spatial` | `features.known_grid` | scan rays | observation grid | every observation | Out, LLE | intersected cells change |
| Inclusion grid | `InclusionGridLearner` | Completed-command callback remains separate | `semaforr::spatial` | `features.inclusion_grid` | accepted poses | inclusion counts | every accepted episode | Out, LLE | independent switches |
| Highways/intersections | `HighwayLearner` | Morphological extraction remains an algorithm gap | `semaforr::spatial` | `features.highways` | HLE observations | highway graph | HLE/finalization | HighwayPlan | graph finalized before target |
| Trails/conveyors | corresponding learners | Command-outcome scheduling is an integration gap | `semaforr::spatial` | representation switches | completed motion | snapshots | accepted episode | planners/advisors | incremental tests |
| Doors/hallways | corresponding learners | Spatial-index optimization deferred | `semaforr::spatial` | representation switches | regions/travel | snapshots | target/on demand | planners/advisors | rebuild tests |
| Circumstances | none | Entire component missing | `semaforr::spatial` | `features.circumstances` | context/outcome | statistics | completed action | Precedent | future acceptance test |
| Hard safety | `HardSafetyFilter` | Freshness/kinematic checks remain at command layer | `semaforr::decision` | non-disableable | observation/actions | safe candidates | every command | all tiers | unsafe action absent |
| Victory | `VictoryRule` | Visibility-specific direct motion incomplete | `semaforr::decision` | Tier-1 rule | active target | action | ordered Tier 1 | engine | deterministic rule test |
| NotOpposite | `NotOppositeRule` | None | `semaforr::decision` | Tier-1 rule | orientation history | veto | ordered Tier 1 | arbitration | reversal veto test |
| Enforcer | `Enforcer` | Per-step repair diagnostics incomplete | `semaforr::decision` | Tier-1 rule | hierarchical plan | waypoints | plan installation | engine | operationalization test |
| Thru/Behind/Out | reactive planners | Stateful cancellation API incomplete | `semaforr::planning` | individual switches | waypoint/FOV/grids | temporary action | interrupt | engine | trigger tests |
| Forward | `ForwardRule` | Full regression proof incomplete | `semaforr::decision` | Tier-1 rule | plan progress | action | ordered Tier 1 | engine | forward test |
| Precedent | none | Entire component missing | `semaforr::decision` | Tier-1 rule | circumstances | vetoes | ordered last | arbitration | future statistical test |
| Tier-3 catalog | partial named factories | Several dissertation advisors missing | `semaforr::decision` | individual advisors | declared dependencies | normalized scores | Tier 3 | arbitration | registry/dependency tests |

## Ordering and safety invariants

The validated Tier-1 order is `victory`, `avoid_obstacles`, `not_opposite`,
`enforcer`, `thru`, `behind`, `out`, `low_level_exploration`, `forward`,
`precedent`. Hard safety is outside this list and cannot be disabled. HLE is a
phase owner, LLE is reactive exploration, and Novelty/Curiosity/SpatialLearner/
Enfilade/VisualScan are Tier-3 heuristics; these mechanisms are never selected
by a generic exploration flag.
