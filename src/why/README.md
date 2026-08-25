# Why

`why` is the single request-driven explanation package for navigation
decisions, action execution, plan selection, and plan execution. It replaces
the former split decision/plan explanation adapters.

## Interface

The node retains `semaforr_msgs/DecisionRecord` messages from
`decision_records` and answers `semaforr_msgs/ExplanationQuestion` requests on
`why_questions`. Structured `semaforr_msgs/ExplanationResponse` answers are
published on `why_responses`. Topic names are parameters.

Questions may identify a task, planning episode, plan, decision, action, or
execution by stable ID. With no identifier, the latest decision is used. The
supported question types cover decision rationale, hypothetical poses,
decision confidence, action counterfactuals, plan rationale, route comparison,
recorded alternatives, plan confidence, route description, and combined
action/plan explanations. Natural-language questions are routed internally;
callers can also provide an explicit question type.

The response contains both natural language and replayable evidence: linked
IDs, confidence inputs, advisors, planners, typed plan steps, model revisions,
source provenance, exact route geometry, exact turns and distances, and their
natural-language categories.

## Trace and mutation contract

The trace store upserts lifecycle records for the same decision. It indexes
decisions by decision, action, execution, task, plan, and planning-episode ID.
The store is in-memory for the active run and never changes navigation state.
Ordinary questions only read recorded evidence. Hypothetical and user-route
evaluation use injected immutable evaluator callbacks; without one the system
returns an explicit unavailable response instead of inventing results.

Decision records distinguish physical safety rejection, cognitive veto, lack
of viability, and lower preference. They also distinguish selected, commanded,
started, completed, partial, cancelled, timed-out, failed,
safety-interrupted, and preempted actions. A selected action is never described
as completed before terminal execution feedback.

Planner objective text comes from planner registration metadata. Plan answers
use the actual candidate cost matrix and range-voting totals. Alternative-plan
answers only return recorded candidates unless a caller explicitly invokes a
separate replanning control interface.

## Chapter 5 compatibility behavior

For Tier-3 decisions, the compatibility explanation path uses normalized
advisor comments in `[0,10]` and sample standard deviations. The decision trace
retains the selected-action comment sum, advisor count, normalized support
proportion, every action total, action-total mean and deviation, and the three
separate Chapter 5 statistics: agreement `gamma`, standardized overall support
`zeta`, and confidence `lambda = (0.5 - gamma) * zeta`. The natural-language
intervals and the omission threshold for weak advisor comments follow the
Chapter 5 tables. Adapted voting may still use configured weights to choose an
action, but it does not change the unweighted comments used by this explanation
calculation.

Selected and already-recorded alternative plans share one route-description
pipeline. It converts the plan to locations, applies the eight allocentric
angle bins, turns consecutive directions into egocentric turns, inserts and
collapses straight travel, and translates accumulated distances using the
documented interval upper limits. Highway turns are marked as intersections
only when the corresponding typed plan location is an intersection.

Plan confidence is available only when a selected plan and a recorded
comparison plan both expose costs for their respective supported objectives.
It uses the objective-specific difference intervals and the categorical
Table 5.10 mapping (`really`, `only somewhat`, or `not`). It does not derive a
new scalar confidence and does not generate a comparison plan. Crowd-specific
objectives are engineering extensions and therefore return an unavailable
Chapter 5 comparison unless a compatible dissertation objective pair is also
recorded.

See `semaforr/docs/explanations.md` for the complete core trace ownership and
reasoning vocabulary.
