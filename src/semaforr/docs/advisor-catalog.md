# Advisor catalog

Advisor names are case-sensitive and registered centrally. Configuration fails
at startup for an unknown name or when no enabled advisor remains.

| Name | Candidates | Evidence and intent |
|---|---|---|
| `goal_progress` | All actions | Rewards reduction in waypoint or target distance |
| `goal_progress_linear` | Forward actions | Linear-only goal progress |
| `clearance` | All actions | Rewards predicted laser clearance |
| `clearance_rotation` | Turns | Rotation-only clearance |
| `exploration` | All actions | Rewards distance from recent navigation history |
| `social_navigation` | All actions | Live pedestrian position, velocity, prediction, covariance, and confidence |
| `crowd_avoid` | All actions | Learned crowd-field density |
| `risk_avoid` | All actions | Learned encounter and predictive collision risk |
| `flow_follow` | All actions | Alignment with learned directional pedestrian flow |

The live social advisor participates only when data passes the configured age,
frame, covariance, and confidence gates. Learned advisors participate only
when their crowd-field sample exists and is fresh. Geometric navigation
continues when social evidence is unavailable.

`advisors.names`, `advisors.enabled`, and `advisors.weights` must have equal
length. `advisors.parameters` contains four finite reserved values per advisor
because ROS 2 parameters cannot represent an array of mappings. Raw score,
weight, and weighted contribution are published separately in each structured
decision record.

The offline legacy converter collapses historical advisor families into these
registered contracts. Runtime code contains no ROS1 advisor factory or alias
fallback.
