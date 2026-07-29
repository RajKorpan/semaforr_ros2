# Advisor catalog

Advisor names are case-sensitive. Configuration fails when a name is unknown
or when no enabled advisor can produce a decision. Linear advisors score
forward actions; `Rotation` variants score turns.

## Default advisor set

| Family | Configured names | Evidence and intent |
|---|---|---|
| Goal seeking | `Greedy`, `GreedyRotation`, `LeastAngle`, `LeastAngleRotation` | Target and waypoint bearing; rewards direct progress |
| Clearance | `ElbowRoom`, `ElbowRoomRotation`, `BigStep`, `BigStepRotation`, `GoAroundRotation` | Laser clearance and legal action magnitude |
| Exploration | `Explorer`, `ExplorerRotation`, `UnlikelyField`, `UnlikelyFieldRotation` | Navigation history and under-visited space |
| Regions | `RegionLeaverLinear`, `RegionLeaverRotation`, `EnterLinear`, `EnterRotation` | Fresh or retained region/exit model |
| Trails and conveyors | `TrailerLinear`, `TrailerRotation`, `ConveyLinear`, `ConveyRotation` | Learned trails and directional traversals |
| Live interpersonal space | `Interpersonal`, `InterpersonalRotation` | Current valid pedestrians and predicted trajectories |
| Learned density | `CrowdAvoid`, `CrowdAvoidRotation` | Unified crowd-field density |
| Learned encounter risk | `RiskAvoid`, `RiskAvoidRotation` | Unified crowd-field encounter evidence |
| Learned flow | `FlowAvoid`, `FlowAvoidRotation` | Directional crowd flow; penalizes opposing movement |

The four social families participate only when their required data is valid.
Live predictions older than `social.maximum_age_s`, below
`social.minimum_confidence`, or in an untransformable frame are ignored.
Learned crowd costs remain available when a valid crowd-field snapshot exists.

## Retained compatibility families

The factory also retains legacy experiment names for exits, doors, spatial
learning, wall following, visibility, and social behaviors:

- `ExitLinear`, `ExitRotation`, `ExitFieldLinear`, `ExitFieldRotation`,
  `ExitClosest`, `ExitClosestRotation`
- `EnterExit`, `EnterExitRotation`, `EnterDoorLinear`, `EnterDoorRotation`,
  `ExitDoorLinear`, `ExitDoorRotation`, `AccessLinear`, `AccessRotation`
- `LearnSpatialModel`, `LearnSpatialModelRotation`, `Curiosity`,
  `CuriosityRotation`
- `Enfilade`, `EnfiladeRotation`, `Thigmotaxis`, `ThigmotaxisRotation`,
  `VisualScanRotation`
- `BaseLine`, `BaseLineRotation`, `ExplorerEndPoints`,
  `ExplorerEndPointsRotation`, `Unlikely`, `UnlikelyRotation`
- `Front`, `FrontRotation`, `Rear`, `RearRotation`, `Side`, `SideRotation`,
  `Visible`, `VisibleRotation`
- `FindTheCrowd`, `FindTheCrowdRotation`, `FindTheRisk`,
  `FindTheRiskRotation`, `FindTheFlow`, `FindTheFlowRotation`, `Follow`,
  `FollowRotation`, `Crossroads`, `CrossroadsRotation`, `Stay`,
  `StayRotation`

Some historical class declarations are intentionally not registered. The
factory is the source of truth; commented-out legacy names are not supported.

## Weights and parameters

`advisors.names`, `advisors.enabled`, and `advisors.weights` must have equal
length. `advisors.parameters` contains four finite values per advisor because
ROS 2 parameters cannot represent an array of mappings. Raw score, configured
weight, and final weighted contribution appear separately in each
`DecisionRecord`, which makes tuning observable.

For a minimal non-social robot, keep goal-seeking and clearance advisors and
disable the social families. For social navigation, retain both geometric
safety rules and the relevant live/learned social advisors; social scores do
not replace obstacle vetoes.
