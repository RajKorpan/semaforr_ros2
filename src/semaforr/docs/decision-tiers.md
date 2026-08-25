<!-- File overview: This file documents or configures decision tiers behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/decision-tiers.md`. -->

# Decision-tier guide

SemaFORR uses a strict hierarchy. Higher tiers constrain or bypass lower tiers;
an action vetoed by Tier 1 cannot re-enter Tier 3 aggregation.

## Compatibility decision cycle

Target-navigation decisions execute this observable cycle:

1. Hard safety removes actions that violate the platform envelope.
2. Tier 1 runs in semantic order, independent of C++ interface category:
   `Victory`, `AvoidObstacles`, `NotOpposite`, Enforcer, `Thru`, `Behind`,
   `Out`, LLE, `Forward`, and `Precedent`. A mandate ends the cycle
   immediately; a veto modifies the viable set and processing continues.
3. After the complete Tier-1 pass, no survivors produces a safe stop and one
   survivor is selected as Tier 1.
4. Tier 2 runs only when Tier 1 made no decision, no plan is active, and at
   least one planner is registered. Successful plan creation stores the plan
   and ends the cycle with a typed pause; the next cycle restarts at hard
   safety and Enforcer can then operationalize the plan. Enforcer is never
   rerun on a newly created plan in the planning cycle.
5. A failed Tier-2 attempt records that no valid plan was produced and may
   fall through to Tier 3. With an existing plan, or with no registered Tier-2
   planners, Tier 3 is also eligible after Tier 1 declines to decide.
6. Tier 3 never runs in the cycle that successfully creates a plan.

Every step appends a `DecisionCycleEvent` with its ordinal, tier, component,
input action set, mandate, vetoes, continuation/return reason, and final tier
attribution. The same compact trace is exposed in runtime phase diagnostics as
`decision_cycle:*` records.

## Tier 1: mandatory rules and vetoes

Every configured Tier-1 name is resolved by `TierOneRegistry`; the adapter
contains no name-specific construction branches. The validated execution
order is `victory`, `avoid_obstacles`, `not_opposite`, `enforcer`, `thru`,
`behind`, `out`, `low_level_exploration`, `forward`, `precedent`.

| Registered name | Contract | Current role |
|---|---|---|
| `victory` | MandatoryRule | Pauses within target tolerance or turns/moves directly toward a sensed target. |
| `avoid_obstacles` | VetoRule | Applies the configurable cognitive obstacle veto; it is distinct from non-ablatable hard safety. |
| `not_opposite` | VetoRule | Rejects turns whose predicted heading repeats either of the two latest execution-confirmed orientations. |
| `enforcer` | PlanOperationalizer | Converts the active grid or typed hierarchical plan step into a local mandate. |
| `thru` | ReactivePlanner | Pursues the clearer side of a tight opening with interruption and budget handling. |
| `behind` | ReactivePlanner | Recovers a nearby unseen waypoint by preferring an available quarter turn right, then left. |
| `out` | ReactivePlanner | Surveys confinement, constructs an execution-confirmed reverse subtrail, and prepends it for Enforcer. |
| `low_level_exploration` | ReactivePlanner and ReplanningTrigger | Searches missing knowledge and requests Tier-2 replanning after inclusion/connectivity growth. |
| `forward` | VetoRule | Rejects projected turns into target-local one-metre cells marked only by Enforcer decisions. |
| `precedent` | VetoRule | Rejects actions only after circumstance assignment, case evidence, accuracy, and action-confidence gates pass. |

Mandatory rules return an optional decision. They are evaluated in configured
order and the first applicable result wins. `Victory` either stops within goal
tolerance or directly turns/moves toward a visible unobstructed target. Its
stable reasons are `victory:target_within_tolerance`,
`victory:turn_toward_visible_target`, and
`victory:move_toward_visible_target`.

Veto rules return zero or more action/reason pairs. `AvoidObstacles` removes
unsafe motions, `NotOpposite` suppresses immediate orientation reversal, and
`Forward` prevents orientation regression along the installed plan. Vetoes are
accumulated before Tier 3 runs and are copied into the decision record.
`NotOpposite` reads only terminal, execution-confirmed orientations.
`Forward` owns a sparse target-local grid with logical one-metre cells. Each
Enforcer selection marks the current cell and all eight neighbors. Other
Tier-1 and Tier-3 selections and terminal execution records do not update this
grid. Only viable rotations are checked; translations are never vetoed by
Forward. If all remaining rotations would be vetoed, the grid is cleared and
the vetoes are withdrawn.

`Thru` runs only after Victory and Enforcer decline. It first tests the mission
target, then the active waypoint, using the closest laser direction, a
configurable neighboring-beam clear-count, and a narrow ellipse around that
ray. Sensor-range membership alone is insufficient. It starts only when the
forward corridor is obstructed and no forward action survives the earlier
AvoidObstacles veto. Rays immediately left and right of the objective ray are
averaged as vectors; Thru pursues the endpoint of the longer average ray in
bounded increments. Endpoint arrival, sensor or mission loss, an invalid local
action, the decision limit, Victory, and an Enforcer mandate all terminate the
state. The beam neighborhood, bundle size, ellipse dimensions, step,
tolerance, and budget are typed constructor parameters.

`Behind` uses a distance threshold of 1.5 metres plus the radius of a region
containing the waypoint. When the waypoint is absent from the current and
previous executed views, it prefers an available 90-degree right turn, then
an available left turn. An execution-confirmed quarter turn suppresses an
immediate repeat. History retains the laser observation pose independently
from the terminal action pose, so the previous visibility test uses the frame
in which that scan was actually observed.

`Out` reconstructs a temporary familiarity grid from only the current target's
most recent `10+n/50` decision observations. It triggers when at least 75% of
the nonzero recent cells have count four or greater and the current view adds
at most one cell. The cumulative familiarity model is not consulted. During
its four-right-turn survey, each new current view is compared with the updated
recent grid and more than one new cell abandons recovery. If the survey finds
no new space, Out locates the latest execution-confirmed point outside recent
coverage, runs the visibility-based Trail learner on the contiguous successful
path suffix, and prepends that typed subtrail for Enforcer. Out never directly
pursues its escape markers; failed or partial suffixes are not operationalized.

The separate Tier-1 contracts are `MandatoryRule`, `VetoRule`,
`PlanOperationalizer`, `ReactivePlanner`, and `ReplanningTrigger`. `Enforcer`
implements plan operationalization. `Thru`, `Behind`, `Out`, and LLE implement
interruptible reactive control; LLE also implements the replanning trigger.

## Tier 2: planning

Tier 2 is mission-level deliberation, not a competing motor vote. Registered
planners receive a `PlanningRequest` and return a typed result:
`Success`, `NoPath`, `InvalidRequest`, or `PlannerUnavailable`.
`PlanningCoordinator` evaluates successful candidates deterministically,
records the selected planner, and installs waypoints. A failed planner cannot
silently leave a partially mutated plan.

Immediate planning failure is bounded by
`tiers.tier2.maximum_planning_attempts_per_task` (default `3`). Each failure
is explicitly recorded and can make Tier 3 eligible for that cycle. Once the
consecutive-failure limit is reached, the plan is marked abandoned and LLE
becomes eligible on the next cycle. A successful plan resets consecutive
failures. A task transition resets all attempt state; an LLE
connectivity-triggered replan explicitly starts a fresh attempt series. This
prevents an unbounded planning loop while preserving a traceable recovery
point.

Reactive planners use a common trigger/update/cancel contract. LLE is stateful
and temporarily owns Tier-1 actions while it assembles and pursues candidates
from unfinished HLE cues, the current scan, stored region visibility, and
inclusion-grid gaps. A new connectivity revision produces an explicit Tier-2
replanning request. Target sensing, a new plan, candidate exhaustion, absence
of candidates, budget exhaustion, sensor loss, and mission changes remain
distinct completion or cancellation reasons.

Registered mandatory rules and a successful Enforcer action are evaluated
before LLE. Victory therefore owns direct visible-target motion, and an
Enforcer-produced waypoint counts as available guidance. LLE's selected-policy diagnostic includes its trigger
reason (`no_plan_available`, `completed_plan_failed_target`, or the explicitly
modernized `stalled_history_extension`).

## Tier 3: advisor aggregation

After Tier 1 vetoes, each enabled advisor may score the remaining actions.
Diagnostics retain raw scores and weighted contributions separately.
Aggregation rejects NaN and infinity, uses a defined floating-point tie
tolerance, sorts diagnostics, and draws ties from a coordinator-owned random
generator seeded once. A deterministic seed therefore reproduces action and
diagnostic selection.

An advisor may decline to participate, particularly when its required spatial
or social model is unavailable. If no advisor participates, the configured
fallback is used. If no candidate survives, the result is a safe `Pause`.

## Decision result

Every cycle returns a value containing the action, `DecisionSource`, Tier 1
vetoes, Tier 3 contributions, complete `decision_cycle` trace, optional
planner, sequence number, latency, and execution outcome. The ROS adapter projects it to
`semaforr_msgs/msg/DecisionRecord`; decision logic never depends on that ROS
message.

Mandatory trace events expose a stable `reason_code`; every Tier-1 veto stores
its stable reason code in the veto explanation field. Human-facing prose may
be layered on these codes without making experiment analysis depend on prose.

To add a rule, planner, or advisor, implement its narrow interface, register
the factory name, add validated configuration, and add deterministic unit
tests. Do not add dispatch branches to the ROS node.

Enforcer dispatches by the active plan's explicit family. `GridPlanEnforcer`
interprets occupancy-derived geometric paths, while `ModelPlanEnforcer`
interprets SkeletonPlan and HighwayPlan typed steps. Both reuse one predictive
local-action evaluator and directly mandate Tier-1 actions. See
`tier-two-planning-and-enforcement.md` for the full contract.
