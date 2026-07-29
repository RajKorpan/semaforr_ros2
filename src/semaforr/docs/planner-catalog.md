# Planner catalog

Planner names are case-sensitive and are selected through
`planners.enabled`. Each planner changes graph edge cost or graph structure;
the selected planner and ordered alternatives are recorded with the decision.

| Name | Primary preference | Required model |
|---|---|---|
| `distance` | Shortest metric path | Parsed navigation graph |
| `smooth` | Smooth, low-turn paths | Navigation graph |
| `novel` | Less frequently visited space | Navigation history |
| `density` | Lower learned crowd density | `skeleton` and crowd learning |
| `risk` | Lower learned encounter risk | `skeleton` and crowd learning |
| `flow` | Compatible directional flow | `skeleton` and crowd learning |
| `combined` | Composite distance, social, novelty, safety, and spatial costs | `skeleton`, crowd learning, available spatial models |
| `explore` | Exploration-oriented graph cost | Navigation history |
| `spatial` | Region, door, and exit structure | Spatial model |
| `hallwayer` | Learned hallway preference | Fresh or retained hallway model |
| `trailer` | Previously successful trails | Trail model |
| `barrier` | Learned barrier-aware cost | Barrier model |
| `conveys` | Learned conveyor direction | Conveyor model |
| `safe` | Obstacle/wall clearance | Map and graph clearance |
| `skeleton` | Passage/skeleton navigation graph | Passage/skeleton representation |
| `hallwayskel` | Hallway-enhanced skeleton graph | Hallway and skeleton models |

`CUSUM` and `discount` are rejected as planner names: they are crowd-learning
estimators selected by `social.learning.estimator`. Crowd-cost planners are
also rejected unless `skeleton` is enabled and `social.learning.enabled` is
true.

The modern planner interface returns `PlanResult` rather than mutating output
arguments. Repeated A* searches use local search state, so one failed or
successful search cannot contaminate the next. Empty graphs, identical
start/goal, disconnected components, and unreachable goals are covered by the
ROS-independent planning tests.

The default YAML enables `skeleton`, `density`, `risk`, and `flow`. On a robot
without social observations, the crowd learner simply has no current evidence;
use `distance` or `skeleton` alone if learned crowd costs are not part of the
deployment.
