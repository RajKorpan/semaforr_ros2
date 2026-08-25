<!-- File overview: This file documents or configures explanations behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/explanations.md`. -->

# Unified explanation architecture

## Ownership and data flow

The navigation adapter owns the shared mutable world model and coordinators.
The navigation engine references that state, owns pending execution and its
bounded explanation history, and produces immutable decision records. It
retains the most recent 4096 records in the active process and indexes them by
stable decision and action IDs. The ROS adapter publishes
the same decision ID as its lifecycle advances through selected, commanded,
started, and a terminal execution state. The Why package upserts those records
and indexes task, planning episode, plan, decision, action, and execution IDs.

Explanation generation is observational. It cannot advance a plan, invoke
learning, update circumstances, alter advisor weights, or issue commands.
Hypothetical and user-route evaluations are dependency-injected pure callbacks
over immutable snapshots. If a callback is unavailable, the response says so;
missing planner or advisor reasoning is never reconstructed after the fact.

## Decision trace

Each decision record contains the complete generated and viable action sets,
predicted poses, evidence provenance, Tier-1 evaluation sequence, mandates,
vetoes, the actions remaining after each stage, Tier-3 comments and totals, tie
policy and candidates, confidence inputs, selected tier and policy, active plan
context, and execution lifecycle. Veto producers provide a reason code, a
rejection kind, and an explanation category.

The shared rejection vocabulary is:

- `safety`: a command cannot be executed within the non-ablatable envelope.
- `cognitive`: an executable action conflicts with a behavioral rule.
- `not_viable`: the action is unavailable under the current state.
- lower preference: the action stays viable but loses arbitration.

Tier-3 records retain raw comments, normalized `[0,10]` comments, advisor mean
and sample standard deviation, relative support, configured weight, weighted
contribution, final action total, and viability. Decision confidence retains
the selected comment sum, advisor count, support proportion, every action
total, action-total mean and sample deviation, agreement `gamma`, standardized
overall support `zeta`, confidence `lambda`, and all interval translations.
`gamma`, `zeta`, and `lambda` follow Equations 5.2--5.4; zero action-total
deviation produces finite `zeta = 0`. Every production advisor normalizes its
complete viable-action raw set to `[0,10]`; an all-equal set becomes neutral
`5` without division by zero. In compatibility scoring, those comments are
unweighted and exact ties use the seeded random policy. In adapted scoring,
configured weights and tolerance ties remain visible in the trace.

## Planning trace

A planning episode has its own ID and records every valid candidate, selected
plan ID, policy, objective-cost matrix, normalized costs, final vote, tie set,
tie rule, complete geometry, typed steps, planner configuration revision,
operating mode, static-map use, and exact representation revisions. Planner
metadata defines the plan family, primary objective, human-readable objective,
representation dependencies, map requirement, and mapless support.

An active hierarchical plan also has an execution revision. Cursor changes,
repairs, shortcuts, substitutions, completion, and invalidation advance or are
recorded with that execution state. Grid Enforcer explanations cite the path
index, operational waypoint, and traversability/shortcut reason. Model Enforcer
explanations cite the typed region, subtrail, highway, intersection, entry, or
exit step.

## Question routing

The unified request supports:

- why this decision;
- what would happen at a hypothetical pose;
- decision confidence;
- why an alternative action was not selected;
- why a plan won;
- comparison with a recorded or supplied route;
- another recorded plan;
- plan confidence;
- a natural route description;
- a combined action and plan explanation.

Counterfactuals report whether an action was never generated, not viable,
rejected by safety, cognitively vetoed, tied, inconsistent with Enforcer, or
merely lower-ranked. Plan comparisons use the same recorded objective set.
They acknowledge objective tradeoffs and do not claim unqualified superiority.

Route descriptions retain exact points, segment distances, and egocentric turn
angles while producing concise direction and distance categories. One shared
pipeline handles selected (`We will`) and recorded alternative (`We could`)
plans using the eight allocentric bins, modulo-eight egocentric turns, inserted
and collapsed straight travel, and interval-upper-limit distance wording.
Model-plan steps are reduced to meaningful operational locations; HighwayPlan
omits unnecessary highway structures but preserves intersection locations so
turns at those joins say `at an intersection`.

Plan confidence is emitted only when an existing selected plan and an existing
comparison plan both retain the required cross-objective costs. Objective
differences use their objective-family Chapter 5 intervals, and the two
translations index the categorical Table 5.10 result. Why does not synthesize
an alternative or replace the table with a scalar confidence composite.

## Provenance

Every answer can report whether evidence came from current sensors, static-map
geometry, accumulated sensed occupancy, familiarity, trails, regions,
hallways, highways, skeleton connectivity, circumstances, crowd fields, or the
active plan. Model revisions are copied from the plan or decision that consumed
them. A mapless trace therefore cannot be described as map knowledge merely
because the simulator used hidden environment geometry.
