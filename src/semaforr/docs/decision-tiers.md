# Decision-tier guide

SemaFORR uses a strict hierarchy. Higher tiers constrain or bypass lower tiers;
an action vetoed by Tier 1 cannot re-enter Tier 3 aggregation.

## Compatibility decision cycle

Target-navigation decisions execute this observable cycle:

1. Hard safety removes actions that violate the platform envelope.
2. Tier-1 mandatory rules run in registered order. The first viable mandate
   ends the cycle immediately.
3. Tier-1 veto rules successively reduce the viable action set. No survivors
   produces a safe stop; exactly one survivor selects that action as Tier 1.
4. If the task has no waypoint, Tier 2 gets one planning attempt and returns
   control to Tier 1. Mandatory rules and vetoes are evaluated again against
   the same hard-safe candidate set.
5. When a plan exists, Enforcer gets the first opportunity to operationalize
   its current waypoint.
6. Only if Enforcer cannot act are `Thru`, `Behind`, `Out`, and then LLE
   evaluated. Reactive state persists between cycles, but does not preempt a
   newly applicable Victory or Enforcer decision.
7. Tier 3 scores the surviving actions only when Tier 1 has not selected one.

Every step appends a `DecisionCycleEvent` with its ordinal, tier, component,
input action set, mandate, vetoes, continuation/return reason, and final tier
attribution. The same compact trace is exposed in runtime phase diagnostics as
`decision_cycle:*` records.

## Tier 1: mandatory rules and vetoes

Every configured Tier-1 name is resolved by `TierOneRegistry`; the adapter
contains no name-specific construction branches. The validated execution
order is `victory`, `avoid_obstacles`, `not_opposite`, `enforcer`, `thru`,
`behind`, `out`, `low_level_exploration`, `forward`, `precedent`.

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
`Forward` stores successful executed plan motion in footprint-sized cells;
selected or failed actions do not mark cells.

`Behind` uses a distance threshold of 1.5 metres plus the radius of a region
containing the waypoint. When the waypoint is absent from the current and
previous executed views, it prefers an available 90-degree right turn, then
an available left turn. An execution-confirmed quarter turn suppresses an
immediate repeat. History retains the laser observation pose independently
from the terminal action pose, so the previous visibility test uses the frame
in which that scan was actually observed.

`Out` evaluates the most recent `10+n/50` execution records, where `n` is the
available navigation-history length. It surveys with four right quarter turns,
builds a reverse subtrail from successful executed motion, prepends that
subtrail to the remaining mission plan, and returns control to Enforcer. It
never directly pursues the escape points.

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
returns to Tier 1 for the current cycle. Once the consecutive-failure limit is
reached, the plan is marked abandoned and LLE becomes eligible. A successful
plan resets consecutive failures. A task transition resets all attempt state;
an LLE connectivity-triggered replan explicitly starts a fresh attempt series.
This prevents an unbounded Tier-2/Tier-1 loop while preserving a traceable
recovery point.

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
