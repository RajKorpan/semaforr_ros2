<!-- File overview: This guide explains how to implement, declare, register, configure, select, cache, operationalize, explain, and test a new Tier-2 planner using SemaFORR's current planning interfaces and exact model dependencies. Its package-relative location is `docs/extending-tier-two.md`. -->

# Implementing a new Tier-2 planner

Tier 2 creates plans; it does not directly command the robot. A successful planning cycle stores the selected plan and ends. On the next decision cycle Tier 1 runs again, and Enforcer operationalizes the plan. Preserve that lifecycle when adding a planner.

## Planner families and source layout

Choose the input family before implementing the algorithm:

| Family | Registry input | Typical source |
| --- | --- | --- |
| Known/sensed occupancy graph | `Grid` | static or partial occupancy |
| Occupancy plus learned costs | `AffordanceModifiedGrid` | regions, trails, hallways, conveyors, crowd fields |
| Learned topological/geometric model | `Freespace` | skeleton or highway graph |

Core files:

- Planner API, requests, results, objectives, dependencies: `include/semaforr/planning/planner.hpp`
- Registry declarations and capability requirements: `include/semaforr/planning/planner_registry.hpp`
- Default registration: `src/planning/planner_registry.cpp`
- Existing grid/affordance planner: `include/semaforr/planning/domain_planner.hpp` and its source
- Hierarchical planners: `include/semaforr/planning/hierarchical_plan.hpp` and `src/planning/hierarchical_plan.cpp`
- Selection, caching, and voting: `planning_coordinator.hpp/.cpp`
- Enforcer: `include/semaforr/decision/enforcer.hpp` and `src/decision/plan_enforcer.cpp`

## Step 1: define objective and output contract

Decide whether an existing `PlanObjective` represents the planner's cost. Add a new objective only when its units and rationale are genuinely distinct. Extend string conversion, normalization/range voting, diagnostics, and tests together.

A planner returns `PlanResult` with:

- `PlanStatus`.
- Geometric path where applicable.
- Primary cost and `ObjectiveCosts` suitable for cross-planner comparison.
- Explanation and diagnostics.
- Optional typed `HierarchicalPlan`.

Do not compare incompatible raw units. Populate objective-specific costs so `minimum_normalized_cost`, `range_vote`, and `pareto_then_vote` can operate correctly.

## Step 2: implement `planning::Planner`

```cpp
class ExamplePlanner final : public Planner {
 public:
  std::string_view name() const noexcept override { return "example"; }
  PlanObjective objective() const noexcept override;
  PlanFamily planFamily() const noexcept override;
  PlannerMetadata metadata() const override;
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest& request) const override;
  PlanResult plan(const PlanningRequest& request) override;
};
```

Validate the request before planning: finite start/goal, task identity where needed, required model pointers, map/occupancy capabilities, geometry compatibility, and usable representation content. Return a precise non-success status instead of constructing an empty successful plan.

`dependencies()` must list exactly what this invocation consumed. For example, a distance planner should not depend on crowd flow, while a highway plan normally depends on regions/skeleton, highways, and highway graph. Conditional dependencies are permitted when a fallback path uses fewer models.

After creating a successful result, call `attachDependencySnapshot()` so the plan stores current revisions. If constructing `HierarchicalPlan` directly, also fill planned start/goal, task ID, planner configuration revision, operating mode, provenance, and typed steps.

## Step 3: produce operationalizable steps

Use the `PlanStep` variant instead of smuggling domain meaning through untyped waypoints. Available steps include waypoint, subtrail, region, visibility connection, highway, intersection, highway entry/exit, skeleton transition, and final target.

Every nontrivial step needs enough geometry or IDs for Enforcer to execute it after planning. Examples:

- A skeleton edge carries its learned supporting subtrail.
- A visibility surrogate includes the visibility connection.
- A highway edge or entry includes an operational fallback subtrail.
- A region step identifies the region and its usable center/visibility evidence.

If a new step type is required, update `PlanStep`, `stepTarget()`, serialization/replay, Enforcer's visitor, visualization, Why explanations, and stale-plan repair tests.

## Step 4: register capability requirements

Register the factory in `defaultPlannerRegistry()`:

```cpp
registry.add(
    "example",
    PlannerInputModel::AffordanceModifiedGrid,
    [] { return std::make_unique<ExamplePlanner>(); },
    StaticMapRequirement::Optional,
    OccupancyRequirement::StaticOrSensedPartial);
```

Choose requirements accurately:

- `Required`: the planner cannot operate without a loaded static map.
- `Optional`: a static map may supplement another valid source.
- `Independent`: no static map is consumed.
- `SensedPartial`: requires a valid partial sensed-occupancy graph.
- `StaticOrSensedPartial`: either explicit source is accepted.
- `None`: the planner uses learned geometric/topological knowledge without an occupancy graph.

Registration is gated by actual capabilities, not a nonempty filename. A map planner must never initialize against empty geometry.

## Step 5: configuration and selection

Add the planner name to configuration validation's registered catalog, the default planner list only if appropriate, and `config/semaforr.yaml`. Include planner-specific settings and their revision in the configuration fingerprint.

Verify behavior under all supported selection policies:

- `single`
- `minimum_normalized_cost`
- `range_vote`
- `pareto_then_vote`
- `shortest_valid`

Planner ties use the configured planner seed/policy, separate from Tier-3 and LLE randomness. Record candidates, normalized costs, votes, Pareto membership, and final selection.

## Step 6: caching and invalidation

The cache key must include the start surrogate, target surrogate, planner identity, relevant model revisions, and relevant policy/configuration revision. A cached plan becomes stale only if a consumed dependency changes, task/start/goal tolerance is violated, policy changes, or execution invalidates the remaining route.

Use `dependencyChangeReasons()` and `stalePlanReasons()` to produce exact diagnostics. Do not invalidate a pure distance plan because an unrelated crowd layer changed.

## Step 7: tests

Add tests for:

- Valid, invalid, unavailable, no-path, and disconnected cases.
- Required map/occupancy/representation gating.
- Obstacle and bounds compliance.
- Objective costs and normalization across competing planners.
- Every selection policy and deterministic ties.
- Typed hierarchical steps and Enforcer operationalization.
- Exact dependency snapshots and unrelated-change cache reuse.
- Relevant-change invalidation with an exact stale reason.
- Start/goal surrogate branches where applicable.
- Plan replay, Why comparison, and alternate-route explanation.

Update `planner-catalog.md`, `tier-two-planning-and-enforcement.md`, configuration reference, compatibility matrix, and the documentation index. Configuration must not accept the new planner until its runtime factory is operational.
