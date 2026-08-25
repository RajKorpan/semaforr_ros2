<!-- File overview: This guide explains how to implement and register a new cognitive Tier-1 component in its correct semantic category and execution position, including dependencies, configuration, diagnostics, safety boundaries, lifecycle state, and tests. Its package-relative location is `docs/extending-tier-one.md`. -->

# Implementing a new Tier-1 component

Tier 1 is not a single advisor interface. Choose the interface based on the behavior's control authority:

| Behavior | Interface | Result |
| --- | --- | --- |
| Must choose an immediate action | `MandatoryRule` | `optional<Decision>` |
| Must remove unsafe or inappropriate candidates | `VetoRule` | `vector<Veto>` |
| Converts a hierarchical plan step to local action guidance | `PlanOperationalizer` | waypoint/operational step |
| Temporarily owns navigation across cycles | `planning::ReactivePlanner` | trigger and stateful plan updates |
| Requests replanning without choosing ordinary motion | `ReplanningTrigger` | replanning request |

Interfaces are in `include/semaforr/decision/rules.hpp` and `include/semaforr/planning/reactive_planner.hpp`. Existing concrete Tier-1 classes are declared in `decision/tier_registry.hpp`, `decision/obstacle_veto_rule.hpp`, and the reactive-planner header.

## Preserve the safety and ordering contract

`HardSafetyFilter` is outside the ablatable cognitive tier. A new Tier-1 rule must not replace final command validation or assume that disabling the rule disables platform safety.

The compatibility order is:

```text
HardSafety
-> Victory
-> AvoidObstacles
-> NotOpposite
-> Enforcer
-> Thru / Behind / Out
-> LLE
-> Forward
-> Precedent
-> Tier 2
-> Tier 3
```

The coordinator follows registered semantic order. Adding a component requires an explicit placement decision. Do not append it arbitrarily or group it by C++ base class if that changes behavior. A mandate ends the cycle immediately; a veto changes the viable action set and allows the next ordered component to run.

## Step 1: select and implement the interface

For a mandatory rule:

```cpp
class ExampleRule final : public MandatoryRule {
 public:
  std::string_view name() const noexcept override { return "example"; }
  std::vector<std::string_view> dependencies() const override;
  std::optional<Decision> evaluate(
      const DecisionContext& context) const override;
};
```

Return `nullopt` when the trigger is not satisfied. A returned `Decision` must contain the selected `domain::Action`, stable rule name, and a precise explanation/reason code.

For a veto rule, return one `Veto` per removed viable action and implement `lastReason()` when the existing diagnostics expect an aggregate rationale. Only veto actions present in `context.viable_actions`; do not invent a second action space.

For stateful reactive behavior, implement `evaluateTrigger()`, `update()`, and `cancel()`. Define completion, cancellation, task-change, sensor-loss, and budget reasons. Persistent reactive state may survive an interruption, but the component must yield each cycle so Victory and a successful Enforcer retain priority.

## Step 2: read data through `DecisionContext`

Tier-1 evaluation receives a read-only `DecisionContext` containing:

- `context.world`: robot, mission, histories, recovery, crowd, spatial model, and optional static map.
- `context.action_space`: configured distances and rotation angles.
- `context.viable_actions`: candidates after earlier vetoes.
- `context.active_plan_objective`: local plan objective when one exists.

Use execution-confirmed histories for claims about actual motion. `decision_history` means selected, `command_history` means started, `execution_history` contains outcomes, and `completed_path_history` contains reached traversal geometry. Do not infer success from selection.

Declare required model names through `dependencies()`. If evidence is optional, check availability and yield without mutating world state.

## Step 3: register the factory and order

Add the factory in `registerTierFactories()` in `src/decision/tier_registry.cpp` using one of:

```cpp
tier_one.registerMandatory("example", factory);
tier_one.registerVeto("example", factory);
tier_one.registerOperationalizer("example", factory);
tier_one.registerReactive("example", factory, requests_replanning);
```

Then update the authoritative ordered Tier-1 configuration and validation in `navigation_configuration.cpp`. The registered name, configured name, diagnostics, and documentation must match exactly. Validation should reject duplicates, unknown names, and invalid order relative to required predecessors.

Do not add a hard-coded branch in the ROS node. The ROS layer executes the typed action returned by the navigation engine.

## Step 4: configuration, ablation, and state reset

Add parameters for behavior thresholds as typed configuration rather than constants in `evaluate()`. Include the component in the appropriate named profiles only when intended. The master Tier-1 switch may disable the cognitive rule, but hard safety remains enabled.

If the component keeps task-local state, reset it on task/target change. If it consumes callbacks, protect against duplicate or stale execution IDs. Cancellation must leave the robot and plan state coherent.

## Step 5: diagnostics and explanations

Every evaluation should make observable:

- Whether the component ran.
- Its trigger result.
- Input and output viable sets.
- Mandate or veto reason codes.
- Model revisions used.
- Why control continued or terminated.
- Final tier and component attribution.

Avoid explanations based on data the rule did not consume. Social rules should record live versus learned crowd evidence and staleness.

## Step 6: tests

Add dedicated tests under `test/unit/tier_one_component_test.cpp` or a focused file. Cover:

- Trigger false, trigger true, and ambiguous boundary cases.
- Every mandate/veto reason.
- Correct use of viable actions and configured action magnitudes.
- Missing/stale dependencies.
- Task reset, cancellation, and execution feedback for stateful rules.
- Exact ordering: earlier components can preempt it and its mandate prevents later stages.
- Tier-2/Tier-3 are not invoked after its mandate.
- Hard safety can still reject its returned command.
- Configuration enable/disable, profile expansion, and validation.

Update `decision-tiers.md`, the compatibility matrix, configuration reference, and any Why templates. The component is not complete until the runtime trace proves its intended position.
