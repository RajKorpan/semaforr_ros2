<!-- File overview: This guide explains how to implement and register a read-only Tier-3 scoring advisor, including metadata, local objectives, action prediction, score normalization, weighting policies, model dependencies, reproducible ties, diagnostics, configuration, and tests. Its package-relative location is `docs/extending-tier-three.md`. -->

# Implementing a new Tier-3 advisor

Tier 3 scores the actions that remain viable after Tier 1. It runs only when Tier 1 did not decide and the Tier-2 lifecycle permits fallback. A Tier-3 advisor must be read-only: it evaluates `DecisionContext`, returns an `AdvisorEvaluation`, and never mutates the world model, installs a plan, or commands ROS directly.

## Source layout and examples

- Interface and metadata: `include/semaforr/decision/advisor.hpp`
- Generic registry: `include/semaforr/decision/registry.hpp`
- Catalog construction: `src/decision/advisors/catalog_registry.cpp`
- Commonsense/spatial implementation: `heuristic_advisor.hpp/.cpp`
- Live social example: `social/social_navigation_advisor.hpp/.cpp`
- Learned crowd example: `social/learned_crowd_advisor.hpp/.cpp`
- Decision voting and trace records: `decision_coordinator.hpp/.cpp` and `decision_result.hpp/.cpp`

Use a dedicated class when the advisor has substantial geometry, prediction, or configuration. Adding another enum branch to `HeuristicAdvisor` is acceptable only when it genuinely shares the same prediction and evaluation machinery.

## Step 1: define rationale, actions, and evidence

Write one testable sentence: “Prefer/avoid X because Y.” Then identify:

- Which action types it scores.
- Whether it can run without an active mission target.
- Whether it should use the final target or current operational plan step.
- Required live/learned representations.
- Behavior when evidence is absent or stale.
- Raw score meaning before policy transformation.

During plan execution, goal-progress advisors normally use `context.active_plan_objective->target`, not the final mission target. Fall back to the mission target only when no local objective is present.

## Step 2: implement `Advisor`

```cpp
class ExampleAdvisor final : public Advisor {
 public:
  std::string_view name() const noexcept override { return "example"; }
  std::vector<std::string_view> dependencies() const override;
  AdvisorMetadata metadata() const override;
  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;
};
```

Metadata must declare:

- Required representation names.
- Scored action types.
- Target-independent participation.
- Expected normalization.
- Human-readable rationale.

In `evaluate()`:

1. Validate that required evidence exists and is fresh.
2. Filter to the advertised action types without adding candidates.
3. Predict each candidate using the configured `ActionSpace` and current robot pose.
4. Compute raw scores for all participating viable actions.
5. Return `participated=false` when the rationale cannot be evaluated.
6. Set explanation, weight, and exact model revision used.

Do not normalize each action independently. Produce one raw score set; the decision system normalizes the advisor's complete set according to the selected policy.

## Step 3: support both scoring policies correctly

Compatibility policy expects unweighted comments normalized to `[0,10]` and seeded random selection only among exact total-score ties. Modernized policy may transform to `[-1,1]`, apply configured weights, and use tolerance ties.

The decision trace preserves raw score, normalized score, weight, weighted contribution, viability, total, tie candidates, and random selection. Your advisor should not pre-apply compatibility weights or hide a second normalization inside its raw calculation.

When all raw scores are identical, normalization must remain finite and must not arbitrarily manufacture a preference.

## Step 4: register and configure

Register the factory in `registerAdvisorCatalog()` in `src/decision/advisors/catalog_registry.cpp`:

```cpp
registry.registerFactory(
    "example",
    [configuration] {
      return std::make_unique<ExampleAdvisor>(configuration);
    });
```

Then add the name to the authoritative registered-advisor set in `navigation_configuration.cpp`, configuration defaults or profiles as intended, `config/semaforr.yaml`, `advisor-catalog.md`, and validation of dependencies. Do not modify ROS dispatch logic.

Social master disablement must disable all social advisors through configuration. A spatial advisor cannot be enabled without its required representation unless it is explicitly optional and declines participation when absent.

## Step 5: exact data and revision access

Read through `context.world`:

```cpp
const auto& spatial = context.world.spatial;
const auto& crowd = context.world.crowd;
const auto revision = crowd.revisionOf(
    domain::ModelDependency::LiveCrowdObservation);
```

For multi-model advisors, record every dependency in the decision trace where supported; do not report the maximum revision as though it represented all inputs. Query helpers such as `CrowdModel::learnedAt()` or `flowAlignmentAt()` preserve model semantics better than indexing raw cells in advisor code.

## Step 6: tests

For every advisor add:

- A scenario strongly favoring one action.
- A scenario strongly disfavoring one action.
- An ambiguous/equal-score scenario.
- Raw-score assertions before normalization.
- Best maps to 10 and worst to 0 under compatibility normalization.
- Identical scores remain finite and unbiased.
- Correct local plan objective versus final target.
- Missing, stale, and disabled dependency behavior.
- Compatibility and modernized scoring/weighting.
- Exact versus tolerance ties and deterministic seeded choice.
- Replay of scores and explanation trace.

Update the advisor catalog and Why rationale vocabulary. The catalog description must match the calculation actually executed in code.
