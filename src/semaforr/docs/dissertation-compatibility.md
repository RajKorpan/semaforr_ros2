# Dissertation compatibility matrix

This matrix is the restoration contract for Chapters 3 and 4 of *Metareasoning,
Opportunistic Exploration, and Explanations for Autonomous Indoor Navigation*.
`Implemented` means the capability is present in the current modular C++ core;
`foundation` means its lifecycle and data boundary exist but its dissertation
strategy is scheduled for a later restoration phase.

| Dissertation capability | Current component | Status | Update or execution schedule | Consumers |
|---|---|---:|---|---|
| Paths/navigation history | `domain::NavigationHistory` | Implemented | Every decision | advisors, learners |
| Trails | `TrailLearner` | Implemented | Incremental | trail consumers |
| Conveyors | `ConveyorLearner` | Implemented | Incremental | advisors, planners |
| Regions | `RegionLearner` | Implemented | Incremental spatial merge | region consumers |
| Doors/exits | `DoorExitLearner` | Implemented | Rebuild on demand | region planners |
| Hallways | `HallwayLearner` | Implemented | Rebuild on demand | hallway consumers |
| Skeleton | `PassageSkeletonLearner` | Implemented | Incremental graph append | skeleton planner |
| Known grid | `KnownGridLearner` | Implemented | Every coherent laser view | Out, LLE |
| Inclusion grid | `InclusionGridLearner` | Implemented | Every accepted pose | LLE, coverage |
| Highways/intersections | `HighwayLearner` | Implemented | Incremental during HLE | `HighwayPlan` |
| Circumstances/settings | none | Missing | Every decision / learned batch | Precedent |
| HLE preliminary phase | `HighwayExplorer` + phase coordinator | Implemented | Before mission activation | highway learner |
| LLE | none | Missing | Reactive Tier 1 | replanning |
| Hard collision safety | `HardSafetyFilter` | Implemented | Before cognitive arbitration | command selection |
| Victory/NotOpposite | tier interfaces exist | Missing | Ordered Tier 1 | action selection |
| Enforcer operationalization | `Enforcer` | Implemented | Plan installation/progress | mission |
| Tier-2 hierarchical planning | `SkeletonPlan`, `HighwayPlan`, cached coordinator | Implemented | Plan creation | mission |
| Tier-3 range voting | `DecisionCoordinator` | Implemented | When enabled | action selection |
| Dissertation advisor catalog | navigation/social advisors | Partial | Tier 3 | action selection |
| Configuration fingerprint | `configurationFingerprint` | Implemented | Startup | diagnostics |
| Component manifest | `componentManifest` | Implemented | Startup | diagnostics |
| Named ablation profiles | `AblationProfile` | Implemented | Before validation | experiments |
| Versioned model persistence | `serializeAll` | Implemented (write) | On demand | checkpoints |

## Safety and ablation rule

The hard safety filter is not a cognitive tier. It remains active in every
ablation and records its vetoes in the decision record. This permits meaningful
`tier3_only` experiments without allowing collision-producing commands.

## Initial-exploration boundary

When initial exploration is enabled, `NavigationPhaseCoordinator` prevents
`MissionManager` from activating a target until the configured exploration
budget is complete. The current foundation returns safe exploration pauses;
the HLE strategy will replace that placeholder without changing the engine or
ROS node lifecycle.
