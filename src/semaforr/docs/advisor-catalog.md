# Advisor catalog

This catalog uses the same names accepted by configuration and registered by
`registerAdvisorCatalog`. Names are case-sensitive. An unknown or duplicate
name, missing required representation, or enabled Tier 3 with no active
advisor is a startup error.

The table describes the current implementation. A familiar historical name is
not a fidelity claim; see [the compatibility matrix](compatibility-matrix.md)
for algorithm status and known deviations.

## Baseline and generic navigation advisors

| Name | Action set | Current behavior |
|---|---|---|
| `random` | All | Assigns zero to every viable action so the configured seeded tie policy chooses the baseline action. |
| `goal_progress` | All | Rewards predicted progress toward Enforcer's active plan-step objective, then the installed waypoint or mission target. |
| `goal_progress_linear` | Forward and pause | Applies the same progress calculation only to translational candidates. |
| `clearance` | All | Rewards forward magnitude and uses the current scan's maximum usable range as a small turn/pause clearance preference. |
| `clearance_rotation` | Turns and pause | Applies the clearance preference only to rotational candidates. |
| `exploration` | All | Rewards the anticipated pose's distance from execution-confirmed navigation history. |

## Commonsense advisors

| Name | Required evidence | Current behavior |
|---|---|---|
| `big_step` | Active local objective | Prefers longer safe forward actions; turns receive half the longest-step utility. |
| `elbow_room` | Current laser | Maximizes distance between the anticipated pose and current laser endpoints. |
| `novelty` | Active-task execution history | Prefers anticipated positions farther from positions executed for the current task. |
| `go_around` | Current laser | Prefers headings separated from the nearest observed obstacle endpoint. |
| `greedy` | Active local objective | Maximizes distance reduction toward the active plan step, waypoint, or target. |
| `curiosity` | Execution history | Prefers anticipated positions farther from all previously executed positions. |
| `enfilade` | Recent execution history | Prefers returning toward distinct positions in the last ten history entries. |
| `visual_scan` | Laser and view history | Scores turns by sampled angular coverage not present in the current or nearby historical views. |

## Learned-spatial advisors

| Name | Required representation | Current behavior |
|---|---|---|
| `avoid_revisit` | Navigation history | Prefers anticipated poses farther from all recorded executed positions. |
| `prefer_regions` | Regions | Prefers anticipated poses deeper inside a learned region. |
| `prefer_highways` | Highways | Prefers anticipated poses nearer learned highway nodes. |
| `prefer_doors` | Doors | Prefers anticipated poses nearer the midpoint of a learned door segment. |
| `follow_trails` | Trails | Prefers anticipated poses nearer any trail marker. |
| `convey` | Conveyors | Combines traversal frequency, robot distance, and approach distance to prefer frequent useful conveyor segments. |
| `enter` | Regions and active local objective | Prefers entering a region containing the active plan step, waypoint, or target. |
| `exit` | Regions and active local objective | Prefers leaving the current region when it does not contain the local objective. |
| `trailer` | Trails and active local objective | Selects a trail segment with objective progress and prefers joining its useful endpoint. |
| `unlikely` | Regions and doors | Penalizes anticipated poses in non-target regions having zero or one nearby door. |
| `access` | Regions and doors | Prefers approaching regions with more nearby learned doors. |
| `crossroads` | Hallways | Prefers hallway segments with more geometric overlap with other hallways. |
| `follow` | Hallways and active local objective | Selects the hallway nearest the local objective and prefers following it toward the useful endpoint. |
| `least_angle` | Region skeleton and active local objective | Chooses the adjacent skeleton branch best aligned with the local objective and prefers approaching that branch. |
| `spatial_learner` | Inclusion, regions, conveyors | Prefers cells absent from inclusion and penalizes membership in learned regions or frequent conveyors. |
| `stay` | Hallways | Prefers remaining within 0.75 m of the hallway currently nearest the robot. |

## Social advisors

| Name | Required evidence | Current behavior |
|---|---|---|
| `social_navigation` | Fresh live social observation | Scores predicted separation, encounter geometry, velocity, covariance, and confidence for observed pedestrians. |
| `crowd_avoid` | Crowd-density snapshot | Penalizes anticipated cells with learned crowd density. |
| `risk_avoid` | Crowd-risk snapshot | Penalizes anticipated cells with learned encounter and predictive-collision risk. |
| `flow_follow` | Crowd-flow snapshot | Rewards alignment with learned pedestrian flow. |

The social master switch disables observation subscription, learning, these
advisors, and crowd planners together. Missing or stale social evidence causes
the relevant advisor to abstain; geometric navigation continues.

## Configuration and arbitration

`advisors.names`, `advisors.enabled`, and `advisors.weights` must have equal
length. `advisors.parameters` contains four finite reserved values per advisor
because ROS 2 parameters cannot represent an array of mappings. The canonical
configuration intentionally enables a subset of the registered catalog.

Tier-3 arbitration has two policies. `compatibility_comments` transforms each
participating advisor's raw ordering into unweighted comments in `[0,10]` and
normally uses exact ties. `weighted_normalized` transforms scores to the
advisor's declared range, normally `[-1,1]`, applies its weight, and normally
uses tolerance ties. `profile` resolves from the behavior mode. The decision
record preserves raw and transformed scores, weight, weighted contribution,
viability, total, tie candidates, seed, and selected tie index.

When a hierarchical plan is active, `goal_progress`, `goal_progress_linear`,
`greedy`, `enter`, `exit`, `trailer`, `follow`, and `least_angle` use
Enforcer's current operational target. They fall back to the installed
waypoint or mission target only when no typed local objective is available.
