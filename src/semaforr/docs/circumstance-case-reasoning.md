# Circumstance case reasoning

Circumstance learning, action cases, Precedent, and optional Tier-3 weighting
share one revisioned `CircumstanceModel`. There is no second Precedent case
store.

## Learning modes

- `adapted_threshold` uses robot-centered, heading-normalized freespace cells,
  normalized L1 similarity, a configured assignment threshold, deterministic
  similarity-graph grouping, online centroid updates, and stable monotonic IDs.
- `dissertation_compatible` builds initial and unmatched-setting clusters as
  connected components of the thresholded similarity graph and classifies
  against the published prototypes with a softmax over negative normalized
  distances. Its confidence is a classifier probability; adapted confidence
  is normalized centroid similarity. The active mode, feature version,
  classifier version, thresholds, and reclustering policy are serialized and
  emitted in decision traces.

## Execution lifecycle

At selection, the learner retains the decision ID, action ID, task, starting
pose and setting, circumstance ID/revision, assignment confidence, target, and
action. It does not update case counts. Matching terminal feedback updates that
original context. Duplicate results are idempotent and task mismatches are not
attributed.

Successful and failed executions contribute full positive or negative
evidence. Partial movement contributes the configured fractional success
credit. Timeouts are negative evidence. Safety interruptions are counted
separately and contribute negative evidence only when configured. Cancellation,
preemption, clock/odometry resets, and missing results never fabricate success
or failure evidence.

For action `a`:

```text
accuracy(a)   = success_credit(a) / effective_evidence(a)
confidence(a) = (1 + success_credit(a)) / (2 + effective_evidence(a))
```

The confidence is a Laplace-smoothed historical success estimate. Evidence
sufficiency is a separate gate. Case accuracy is the strongest supported
action accuracy, allowing a reliable action to serve as contrast for a poor
one. Raw outcome counts and all derived values remain available.

## Precedent and Tier 3

Precedent is a cognitive Tier-1 veto, never a physical-safety veto. It requires
a confident circumstance match, minimum circumstance and case evidence,
minimum action-specific effective evidence, and a sufficiently accurate case.
Unseen or sparse actions are left viable. Decision-cycle diagnostics record
the precise abstention or veto reason and every threshold used.

Tier-3 circumstance weighting is independently enabled. Reliable actions use
an evidence-blended multiplier around neutral `1.0`, bounded by
`tier3_maximum_influence`. Sparse or ambiguous evidence always uses `1.0`.
Traces retain advisor comments, ordinary weighted totals, pre-circumstance
totals, multipliers, post-circumstance totals, and whether the winner changed.

## Persistence and identities

`session_only` performs no file I/O. `load_only`, `save_only`, and `load_save`
use the configured model path and schema version 2. Loading validates model,
feature, and (in compatibility mode) classifier versions. Stable IDs,
centroids, action outcome counts, migrations, and evaluation metrics survive a
round trip. Reclustering allocates monotonic IDs; migrations provide aliases
for future merge/retirement operations without duplicating case evidence.
