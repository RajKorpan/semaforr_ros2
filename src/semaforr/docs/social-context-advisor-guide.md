<!-- File overview: This guide gives a step-by-step implementation recipe for consuming normalized social-context data in a new Tier-1 rule or Tier-3 advisor, covering input configuration, freshness, predictions, formations, action prediction, registration, gating, diagnostics, replay, and tests. Its package-relative location is `docs/social-context-advisor-guide.md`. -->

# Using social-context data in a Tier-1 or Tier-3 component

This guide starts after upstream ROS messages have entered SemaFORR. New reasoning components should consume `domain::CrowdObservation` or `domain::CrowdModel` through `DecisionContext`; they should not subscribe directly to `social_context_msgs` and should not add ROS message types to decision code.

## 1. Select the evidence and control authority

First choose live versus learned evidence:

- **Live people:** current positions, velocities, confidence, trajectories, and formation membership from `world.crowd.observations().current()`.
- **Learned density/risk/flow:** accumulated field queries from `world.crowd`.
- **Hybrid:** fresh predictions plus learned historical risk, as used by risk avoidance.

Then choose Tier 1 or Tier 3:

- Use a Tier-1 mandatory rule only when social evidence requires one immediate action.
- Use a Tier-1 veto rule when an action must be removed from the viable set.
- Use a Tier-1 reactive planner only when social interaction needs multi-cycle state and interruption semantics.
- Use a Tier-3 advisor when social evidence expresses a preference among otherwise viable actions.

HardSafety remains authoritative regardless of the choice. A social Tier-1 veto can be ablated; collision-safe command execution cannot.

## 2. Enable and verify the input pipeline

The relevant executable configuration is under `social` in `config/semaforr.yaml`:

```yaml
social:
  enabled: true
  input:
    mode: tracked                 # tracked, hunav, or none
    tracked_people_topic: /human_poses_3d_tracked_global
    tracked_predictions_topic: /pedestrian_predictions_tracked
    hunav_agents_topic: /human_states
    hunav_predictions_topic: /pedestrian_predictions
    formations_topic: /formation_groups
    coordinate_frame: map
    current_maximum_age_s: 0.75
    prediction_maximum_age_s: 6.0
    minimum_confidence: 0.25
    fallback_prediction: constant_velocity
  formations:
    enabled: true
    minimum_confidence: 0.5
  learning:
    enabled: true
```

Tracked mode converts `TrackedPersonArray`; HuNav mode converts `hunav_msgs/Agents`. Both prediction topics consist of `PoseStamped` messages accumulated by `SocialObservationBuffer`. Formation data is optional. After conversion, both modes produce the same domain type.

Before debugging the component, confirm startup diagnostics report social enabled, the intended input mode/topic, accepted current observations, prediction source (`gst`, HuNav, or fallback), and non-stale status.

## 3. Add component configuration

Keep thresholds in a typed configuration structure:

```cpp
struct ExampleSocialConfiguration {
  std::chrono::nanoseconds maximum_age{std::chrono::milliseconds(750)};
  double minimum_confidence{0.25};
  double prediction_horizon_s{2.0};
  double interaction_distance_m{1.2};
  double weight{1.0};  // Tier 3 only in adapted scoring.
};
```

Validate finite, nonnegative durations/distances, confidence in `[0,1]`, positive horizon, and supported modes. Add parameters to configuration loading, fingerprints, defaults, and documentation. The social master switch must disable the component.

## 4. Acquire current data safely

Use the read-only world model:

```cpp
const auto& current = context.world.crowd.observations().current();
if (!current ||
    !current->fresh(configuration_.maximum_age)) {
  // Tier 1: return no decision/no veto.
  // Tier 3: return participated=false.
}
```

Choose `fresh()` versus `usable()` deliberately:

- `fresh(maximum_age)` accepts a valid empty observation. Use it when “no people currently observed” is meaningful negative evidence.
- `usable(maximum_age, minimum_confidence)` additionally requires at least one sufficiently confident person. Use it when the algorithm cannot score without a person.

Do not convert missing input into an empty observation. Do not use old predictions merely because the current-state message is fresh.

Filter pedestrians explicitly:

```cpp
for (const auto& person : current->pedestrians) {
  if (person.confidence < configuration_.minimum_confidence) continue;
  // person.position is already in the configured world frame.
}
```

No TF lookup belongs inside the advisor/rule. The ROS boundary has already normalized the frame.

## 5. Evaluate predicted people

Each `PedestrianObservation` provides current position, velocity, ordered future `PredictedPosition` values, covariance, and `prediction_source`.

For a requested future time:

1. Prefer a valid trajectory sample/interpolation covering that time.
2. Otherwise use constant-velocity extrapolation only when configuration permits it.
3. Bound extrapolation to the configured horizon.
4. Increase conservatism for low confidence or large covariance.
5. Record whether GST/HuNav or fallback evidence was used.

The existing `SocialNavigationAdvisor` is the reference for sampling robot and pedestrian trajectories across a horizon and computing minimum separation. Reuse or extract shared geometry rather than implementing inconsistent timestamp rules in multiple components.

## 6. Use formations when relevant

Formation membership is optional. `person.formation_index` indexes `current->formations`; every accepted formation references known current person IDs. Check the optional before access:

```cpp
if (person.formation_index) {
  const auto& formation = current->formations[*person.formation_index];
  // formation.member_ids, formation_type, center, confidence
}
```

Ignore formations below the component's confidence threshold. If formation evidence changes a mandate, veto, or score, set/record formation participation so replay and Why can distinguish availability from actual use.

## 7. Predict robot outcomes consistently

Use the shared motion model for one-step outcomes:

```cpp
if (!context.action_space) return no_participation;
const auto predicted = domain::expectedPoseAfterAction(
    context.world.robot.pose, action, *context.action_space);
```

For translation over time, interpolate from the current pose to the expected pose using the configured action duration or a documented horizon. Rotation actions change heading without fabricating translational displacement. Evaluate only `context.viable_actions` at the component's point in the Tier-1/Tier-3 cycle.

## 8A. Implement a Tier-1 social veto

A typical collision-prediction veto looks like:

```cpp
class PredictedSocialVeto final : public VetoRule {
 public:
  std::string_view name() const noexcept override {
    return "predicted_social_veto";
  }

  std::vector<std::string_view> dependencies() const override {
    return {"live_crowd_observations"};
  }

  std::vector<Veto> evaluate(
      const DecisionContext& context) const override {
    std::vector<Veto> vetoes;
    const auto& current = context.world.crowd.observations().current();
    if (!current || !current->usable(maximum_age_, minimum_confidence_)) {
      return vetoes;
    }
    for (const auto& action : context.viable_actions) {
      if (predictedMinimumSeparation(context, *current, action) <
          minimum_separation_m_) {
        vetoes.push_back({action, "predicted_social_separation"});
      }
    }
    return vetoes;
  }
};
```

Adapt the `Veto` construction to the exact current type in `decision_result.hpp`. Place the rule explicitly in the Tier-1 order and define what happens if it vetoes every candidate. Do not silently convert that case into an arbitrary mandate.

Register it with `TierOneRegistry::registerVeto()` in `registerTierFactories()`, add the name to ordered configuration validation, and add it to profiles only where intended.

## 8B. Implement a Tier-3 social advisor

```cpp
class GroupCourtesyAdvisor final : public Advisor {
 public:
  std::string_view name() const noexcept override {
    return "group_courtesy";
  }

  std::vector<std::string_view> dependencies() const override {
    return {"live_crowd_observations", "social_formations"};
  }

  AdvisorMetadata metadata() const override {
    return {dependencies(),
            {domain::ActionType::Forward,
             domain::ActionType::TurnLeft,
             domain::ActionType::TurnRight,
             domain::ActionType::Pause},
            true,
            ScoreNormalization::TenPoint,
            "prefer actions that avoid crossing a social formation"};
  }

  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;
};
```

Inside `evaluate()`:

1. Initialize weight and explanation.
2. Check current evidence and formation availability.
3. Return `participated=false` when the rationale is not applicable.
4. Predict each candidate action.
5. Compute raw formation-crossing cost/support for all candidates.
6. Store raw scores without per-action normalization.
7. Record the live-crowd revision and whether formations participated.

Register the factory in `registerAdvisorCatalog()`, add it to the registered-advisor configuration set, and document it in `advisor-catalog.md`.

## 9. Use learned crowd data instead of live data when appropriate

For a historical-risk advisor or rule:

```cpp
const auto predicted = domain::expectedPoseAfterAction(
    context.world.robot.pose, action, *context.action_space);
const double risk = context.world.crowd.navigationRiskAt(predicted.position);
const auto revision = context.world.crowd.revisionOf(
    domain::ModelDependency::CrowdRisk);
```

Use the actual dependency enum names in `model_revision.hpp`. Check `learnedAvailable()` before interpreting a query. If live risk is also used, record both the live observation and learned-risk revisions.

## 10. Diagnostics, replay, and Why

Record at minimum:

- Input source (`tracked` or `hunav`).
- Prediction source and fallback use.
- Current-data freshness/degraded status.
- Live and learned revisions consumed.
- Formation available versus formation participated.
- Trigger/participation outcome.
- Per-action raw values and final mandate, veto, or normalized contribution.

Why should learn this through `DecisionRecord`; do not give Why a direct `social_context_msgs` dependency.

## 11. Test in dependency order

Adapter/buffer tests:

- Tracked and HuNav conversion.
- Prediction association, fallback, malformed IDs, duplicates, and staleness.
- Formation association and unknown/stale members.
- Correct frame normalization.

Component tests:

- Missing observation versus valid empty observation.
- Fresh versus stale current data and independently stale predictions.
- Confidence/covariance boundaries.
- Person crossing, following, approaching, and moving away.
- Formation crossing versus passing around a group.
- Rotation, pause, and translation predictions.
- Tier-1 ordering and all-actions-vetoed behavior.
- Tier-3 raw/normalized scoring and equal-score behavior.
- Social master disablement and individual ablation.
- Exact model revisions, replay, and explanation fields.

Integration tests should run the same geometry in tracked and HuNav modes and verify equivalent domain observations produce equivalent reasoning. Also test social-disabled navigation to ensure the component declines cleanly and mapless navigation remains operational.
