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

See `semaforr/docs/explanations.md` for the complete core trace ownership and
reasoning vocabulary.
