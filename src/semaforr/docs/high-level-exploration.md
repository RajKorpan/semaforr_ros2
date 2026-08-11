# High-level exploration

HLE owns navigation before mission activation. It remains nonblocking: each
call consumes one observation and returns one discrete action or typed subgoal.
`phases.initial_exploration.behavior_policy` selects `modernized`,
`compatibility`, or `profile`. `profile` follows the whole-system behavior mode;
the explicit `compatibility` value permits isolated HLE experiments while
other whole-system compatibility blockers remain unresolved.

## Compatibility cue semantics

Compatibility discovery examines exactly two fixed 41-ray focus bundles: the
first 41 rays form `RightOpen`, and the final 41 rays form `LeftOpen`. Each cue
records passage length, geometric width, length-to-width ratio, confidence,
global start and endpoint, global direction, discovery observation ID, current
extension, passage ID, and lifecycle state. A bundle is accepted when it passes
the configured passage-length and length-to-width test, or the configured
large-room length and width test.

Every cue receives an explicit validation result containing:

- start, midpoint, and endpoint clearance;
- the number of existing passage identities crossed;
- geometric reachability sampled along the segment; and
- an acceptance flag and stable reason string.

Cues crossing more than one passage identity are rejected. Existing and new
cues are compared by angular agreement, segment distance, and projected
interval overlap. Similar cues merge into the stable candidate rather than
being suppressed solely by endpoint hashing. Endpoint hashing remains only in
the named modernized policy.

## Pursuit and termination

Pursuit always steers toward the candidate's global endpoint, so moving or
rotating after discovery cannot reinterpret the original relative heading.
New compatible views can extend the endpoint along the persistent passage
direction. Progress, current width, and extension remain attached to the
candidate.

Compatibility termination reasons are `endpoint_reached`,
`end_of_passage_clearance`, `width_changed`, `hard_turn`, `large_room`, and
`candidate_unreachable`. Width changes and hard turns suspend the candidate for
later reactive use; loss of reachability abandons it; ordinary passage ends and
large-room entry complete it. Time, decision-budget, and explicit-finish
reasons remain engineering safeguards and abandon an active candidate.

## Passage evidence, diagnostics, and replay

The passage grid keeps free and obstructed sensor evidence separate from
numbered passage centerlines. Passage cells record both passage and candidate
IDs plus `in_progress`, `suspended`, `completed`, or `abandoned` state.
Execution-confirmed centerline traversal wins when an obstacle endpoint is
quantized into the same coarse cell; it does not clear obstruction in other
cells. Passage-grid snapshots can be restored with geometry and revision
validation.

Every candidate transition records a sequence number, candidate snapshot,
reason, and one of `created`, `merged`, `rejected`, `selected`, `suspended`,
`completed`, or `abandoned`. The coordinator exports these as diagnostic
events. Every update also stores the complete input observation and exact
`ExplorationResult`; `HighLevelExplorer::replay` reproduces the ordered result
stream without recomputation.

