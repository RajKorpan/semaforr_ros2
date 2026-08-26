<!-- File overview: This guide introduces SemaFORR to new users and contributors. It explains the central concepts, recommends an ordered reading path, maps concepts to source code, and provides hands-on exercises for tracing navigation, planning, learning, social input, execution feedback, replay, and explanations. Its package-relative location is `docs/getting-started-with-semaforr.md`. -->

# Getting started with SemaFORR

This is the starting point for someone who has not worked with SemaFORR
before. You do not need to understand every advisor or learned representation
before running the system. First learn the runtime loop and the ownership
boundaries; then study the subsystem related to your work.

## SemaFORR in one paragraph

SemaFORR is a ROS 2 navigation system whose decisions combine immediate
rules, plans, and heuristic advice. It can navigate without a prior map by
learning spatial structure from sensor observations and execution-confirmed
experience. It can also use an optional static map for explicitly map-dependent
planners. A non-configurable safety boundary checks actions independently of
the cognitive tiers. Plans, decisions, controller outcomes, learned-model
revisions, and explanations are recorded with stable identifiers so behavior
can be diagnosed and replayed.

## The six ideas to understand first

### 1. Navigation has phases

The runtime can begin with initial high-level exploration, then enter
target-directed navigation, and finally enter mission complete. Initial
exploration owns navigation before the first target is activated. Reactive
low-level exploration is different: it can temporarily take control during
target navigation when useful guidance is missing.

### 2. Decisions use three cognitive tiers

The simplified decision sequence is:

```text
current observation
  -> hard safety
  -> ordered Tier 1 rules, Enforcer, and reactive planners
  -> Tier 2 planning when no usable plan exists
  -> Tier 3 advisor voting when earlier stages do not decide
  -> selected action
  -> command execution
  -> terminal execution feedback
  -> learning
```

- Tier 1 contains immediate mandates, vetoes, plan operationalization, and
  reactive behaviors.
- Tier 2 produces typed plans from occupancy or learned representations.
- Tier 3 scores the remaining viable actions with commonsense, spatial, and
  social advisors.

A plan created by Tier 2 is installed at the end of that decision cycle.
Enforcer begins operationalizing it on the next cycle.

### 3. The world model has several distinct kinds of knowledge

Do not treat “the map” as one universal data structure. SemaFORR distinguishes:

- an optional immutable static map;
- current pose, laser, and social observations;
- sensed occupancy;
- familiarity or observation frequency;
- learned trails, conveyors, regions, exits, doors, hallways, skeletons,
  inclusion, passages, highways, and circumstances;
- learned crowd density, encounter risk, and flow;
- active mission, plan, histories, revisions, and diagnostics.

Each representation has its own semantics, update schedule, revision, and
consumers.

### 4. Selected actions are not executed actions

Selection records what reasoning intended. Command and execution records show
what the controller actually started and where the robot actually arrived.
Completed-action learners run only after terminal feedback. Failed, cancelled,
timed-out, partially completed, and safety-interrupted actions remain visible
without being learned as successful traversal.

### 5. ROS is an adapter boundary

Navigation algorithms use ordinary C++ domain types. ROS messages, topics,
TF, clocks, QoS, and publishers are handled under `src/ros`. Spatial learners,
planners, and advisors should not depend directly on ROS message types.

### 6. Current behavior is defined by contracts and tests

The maintained documentation and executable tests describe the current
system. Historical publications are valuable background, but a familiar
component name does not guarantee that its current algorithm is an exact
historical reproduction. Check the compatibility matrix before making a
fidelity claim.

## Recommended reading order

### Minimum path for every newcomer

Read these documents in order:

1. This introduction.
2. [Architecture overview](architecture.md) — ownership and one cognitive
   cycle.
3. [Navigation behavioral contract](behavioral-contract.md) — the required
   lifecycle and ordering invariants.
4. [Decision-tier guide](decision-tiers.md) — what Tier 1, Tier 2, Tier 3,
   Enforcer, and reactive planners do.
5. [Data storage and access](data-storage-and-access.md) — where runtime state
   lives and how to read it safely.
6. [Configuration reference](configuration-reference.md) — how a concrete run
   selects modes and components.
7. [Testing strategy](testing.md) — which claims have executable evidence.

After this path, you should be able to explain who owns the world model, when
each tier runs, why learning waits for execution feedback, and how mapless and
map-enabled operation differ.

### Read next according to your task

| If you need to understand… | Read… |
|---|---|
| All registered Tier-1 and Tier-3 components | [Advisor catalog](advisor-catalog.md) |
| Tier-2 planners and selection | [Planner catalog](planner-catalog.md), [Tier-2 planning and enforcement](tier-two-planning-and-enforcement.md) |
| Spatial learning | [Spatial learning](spatial-learning.md), [spatial representation profiles](chapter3-spatial-representations.md) |
| Familiarity, occupancy, or grid expansion | [Grid layers](grid-layers.md), [grid geometry](grid-geometry.md) |
| Mapless versus known-map navigation | [Optional static maps](map-operation.md) |
| Initial and reactive exploration | [High-level exploration](high-level-exploration.md), then the LLE section of [decision tiers](decision-tiers.md) |
| Regions, passages, highways, and graph planning | [Highway and passage models](highway-model.md) |
| Execution feedback and path learning | [Action execution and learning lifecycle](action-execution-lifecycle.md) |
| Social input and learned crowd models | [Social navigation](social-navigation.md) |
| Explanations | [Why explanations](explanations.md) |
| Revisions, caches, and immutable snapshots | [Revision dependencies](revision-dependencies.md), [snapshot sharing](snapshot-sharing.md) |
| Reproducible experiments | [Replay and reproducibility](reproducibility.md), [experimental validation](experimental-validation.md) |
| Known adaptations and fidelity status | [Compatibility matrix](compatibility-matrix.md) |

## First hands-on session

### 1. Build and test the package

Use ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to semaforr why
source install/setup.bash
colcon test --packages-select semaforr why
colcon test-result --verbose
```

The complete environment and container instructions are in
[deployment.md](deployment.md).

### 2. Run the installed example

```bash
ros2 launch semaforr example_simulation.launch.py
```

The default example keeps the simulator's environment map hidden from the
robot. This is a useful first run because it demonstrates that simulator
geometry and SemaFORR map access are independent. Enable RViz with
`rviz:=true` when a graphical display is available.

### 3. Observe the run in this order

Do not begin by inspecting every topic. Answer these questions:

1. Which navigation phase is active?
2. Is a mission target active?
3. Which candidates survived hard safety and Tier-1 vetoes?
4. Did Tier 1 decide, did Tier 2 create a plan, or did Tier 3 vote?
5. Which action and stable IDs were selected?
6. What terminal outcome did the controller report?
7. Which learned representations changed revision afterward?

The installed example writes a structured trace under
`~/.ros/semaforr/example-simulation.json`. Use the stable decision, action,
task, plan, and execution IDs to connect the records.

### 4. Change one thing at a time

Copy `config/semaforr.yaml`, then try one controlled change:

- disable initial exploration;
- run with social input disabled;
- enable a single Tier-3 advisor;
- switch between mapless and map-enabled operation;
- enable one learned representation or planner.

Validate startup diagnostics before comparing behavior. Named ablation
profiles expand into ordinary configuration; they do not create separate
runtime implementations.

## Source-code tour: follow one observation

Read code in runtime order rather than alphabetically:

1. `src/ros/main.cpp` starts the executable.
2. `src/ros/semaforr_node_component.cpp` owns subscriptions, synchronization,
   social buffering, command execution, and publishers.
3. `src/ros/navigation_engine_adapter.cpp` constructs configuration, the world
   model, coordinators, optional static map, learners, planners, and advisors.
4. `src/decision/navigation_engine.cpp` runs phases, mission progression,
   semantic tier ordering, plan installation, feedback correlation, and
   learning dispatch.
5. `src/decision/decision_coordinator.cpp` applies Tier-1 rule stages and
   Tier-3 arbitration.
6. `src/planning` contains planner registries, plan types, selection, and
   Enforcer behavior.
7. `src/spatial` and `src/social` contain mutable learners that publish
   revisioned snapshots.
8. `src/ros/command_executor.cpp` executes typed actions without blocking and
   returns terminal feedback.
9. `src/validation/replay.cpp` records and replays deterministic inputs and
   outcomes.
10. The `why` package consumes decision records to explain the same evidence.

Public types and contracts are under `include/semaforr`. When reading an
implementation, open its public header first, then its focused test, and only
then its `.cpp` file.

## A productive way to study one component

For any advisor, planner, or representation, use this checklist:

1. Find its catalog or compatibility-matrix row.
2. Find its configuration switch and declared dependencies.
3. Read its public interface.
4. Read its focused unit tests to see concrete inputs and expected behavior.
5. Read the implementation.
6. Find where it is registered.
7. Find its diagnostics, revisions, and replay fields.
8. Run only its focused test executable while experimenting.

This avoids mistaking a helper, legacy name, or diagnostic projection for the
authoritative implementation.

## Package boundaries

| Package | Responsibility |
|---|---|
| `semaforr` | Navigation, world model, exploration, planning, advisors, spatial and crowd learning, ROS adapters, diagnostics, replay |
| `semaforr_msgs` | SemaFORR-owned ROS diagnostics and explanation records |
| `why` | Explanation service over recorded SemaFORR decisions and plans |
| `social_context` and `social_context_msgs` | Upstream tracked people, predictions, and formation context |
| `hunav_msgs` | Optional HuNav simulation input messages |
| `semaforr_bridge` | Upstream bridge integration |

Social-context and HuNav messages are converted at the ROS boundary into the
same internal crowd observation model. Navigation advisors and planners read
`WorldModel::crowd`; they do not consume those ROS messages directly.

## Where to go when implementing something

- New spatial representation: [Implementing a spatial model](extending-spatial-models.md)
- New crowd model: [Implementing a crowd model](extending-crowd-models.md)
- New Tier-1 component: [Implementing Tier 1](extending-tier-one.md)
- New Tier-2 planner: [Implementing Tier 2](extending-tier-two.md)
- New Tier-3 advisor: [Implementing Tier 3](extending-tier-three.md)
- Social-data advisor: [Using social-context data](social-context-advisor-guide.md)
- General contribution rules: [Contributor guide](contributing.md)

## Suggested learning milestones

You are ready to modify the system safely when you can:

- draw the observation-to-decision-to-feedback loop;
- distinguish hard safety from cognitive AvoidObstacles;
- distinguish HLE, LLE, and exploration-oriented Tier-3 advisors;
- explain why familiarity is not occupancy;
- identify the active local plan objective versus the final target;
- locate authoritative state in `WorldModel` and its representation revision;
- tell selected, started, partial, failed, and completed actions apart;
- identify whether a claim is current behavior, an intentional adaptation, or
  an unsupported compatibility claim;
- name the focused tests that protect the behavior you intend to change.

