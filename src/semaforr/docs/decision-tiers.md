# Decision-tier guide

SemaFORR uses a strict hierarchy. Higher tiers constrain or bypass lower tiers;
an action vetoed by Tier 1 cannot re-enter Tier 3 aggregation.

## Tier 1: mandatory rules and vetoes

Every configured Tier-1 name is resolved by `TierOneRegistry`; the adapter
contains no name-specific construction branches. The validated dissertation
order is `victory`, `avoid_obstacles`, `not_opposite`, `enforcer`, `thru`,
`behind`, `out`, `low_level_exploration`, `forward`, `precedent`.

Mandatory rules return an optional decision. They are evaluated in configured
order and the first applicable result wins. `Victory` either stops within goal
tolerance or directly turns/moves toward a visible unobstructed target.

Veto rules return zero or more action/reason pairs. `AvoidObstacles` removes
unsafe motions, `NotOpposite` suppresses immediate orientation reversal, and
`Forward` prevents orientation regression along the installed plan. Vetoes are
accumulated before Tier 3 runs and are copied into the decision record.

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

Reactive planners use a common trigger/update/cancel contract. LLE is stateful
and temporarily owns Tier-1 actions while it assembles and pursues candidates
from unfinished HLE cues, the current scan, stored region visibility, and
inclusion-grid gaps. A new connectivity revision produces an explicit Tier-2
replanning request. Target sensing, a new plan, candidate exhaustion, absence
of candidates, budget exhaustion, sensor loss, and mission changes remain
distinct completion or cancellation reasons.

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
vetoes, Tier 3 contributions, optional planner, sequence number, latency, and
execution outcome. The ROS adapter projects it to
`semaforr_msgs/msg/DecisionRecord`; decision logic never depends on that ROS
message.

To add a rule, planner, or advisor, implement its narrow interface, register
the factory name, add validated configuration, and add deterministic unit
tests. Do not add dispatch branches to the ROS node.
