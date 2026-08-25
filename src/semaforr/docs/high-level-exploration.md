<!-- File overview: This file documents or configures high level exploration behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/high-level-exploration.md`. -->

# High-level exploration

HLE owns navigation before mission activation. It remains nonblocking: each
call consumes one observation and returns one discrete action or typed subgoal.
`phases.initial_exploration.behavior_policy` selects `modernized`,
`compatibility`, or `profile`. `profile` follows the whole-system behavior mode;
the explicit `compatibility` value permits isolated HLE experiments while
other whole-system compatibility blockers remain unresolved.

## Compatibility cue semantics

HLE measures four robot-relative angular sectors: narrow 15-degree
`LeftFocus` and `RightFocus` sectors and wide 90-degree `LeftOpen` and
`RightOpen` sectors. Beam membership comes from each beam's reported angle,
never its array index. Every sector averages valid Cartesian beam endpoints;
the Focus mean defines cue direction and length while the corresponding Open
mean and endpoint spread provide independent width and openness evidence.
Consequently equivalent geometry produces equivalent cues for different beam
counts and angular resolutions. Each cue records passage length, geometric
width, length-to-width ratio, confidence,
global start and endpoint, global direction, discovery observation ID, current
extension, passage ID, and lifecycle state. A bundle is accepted when it passes
the configured passage-length and length-to-width test, or the configured
large-room length and width test.

Large-room classification deliberately remains the configurable threshold
test. The historical environment-specific trained classifier is not loaded;
this is an intentional modernized behavior in both isolated HLE policies.

Every cue receives an explicit validation result containing:

- start, midpoint, and endpoint clearance;
- the number of existing passage identities crossed;
- geometric reachability sampled along the segment; and
- an acceptance flag and stable reason string.

Cues crossing more than one passage identity are rejected. Existing and new
cues are compared by angular agreement, segment distance, and projected
interval overlap. Similar cues merge into the stable candidate. The spatial
hash only accelerates lookup; it does not define cue geometry or equivalence.

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

## Low-level exploration handoff

LLE keeps cue start `alpha(kappa)` separate from cue endpoint `omega(kappa)`.
It first records whether the start is already satisfied, reached through direct
visibility or an inclusion-grid route, unreachable, or invalidated. Only after
reaching the start does it install 20 evenly spaced cue waypoints. An
unreachable or invalidated start discards that cue; it is never treated as an
exploration traversal.

If no validated cue remains, uncovered current rays retain closest-target-bin
selection and seeded compatibility randomness. When every current ray endpoint
is already included, LLE routes through the inclusion grid to the included
cell closest to the mission target, then rebuilds candidates from the new pose.
A valid cue seen during relocation abandons the relocation immediately.
Inclusion persists across temporary interruption, and roughly 10-percent
inclusion growth or new connectivity still requests Tier-2 replanning.
