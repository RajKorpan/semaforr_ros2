<!-- File overview: This file documents or configures action execution lifecycle behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/action-execution-lifecycle.md`. -->

# Action execution and learning lifecycle

Action selection is an intention, not evidence that motion occurred. Each
selection receives monotonically increasing `DecisionId` and `ActionId` values
and is stored in `decision_history`. The record includes its task, expected
start pose, tier/provenance, timestamp, and intended motion.

The ROS-independent engine accepts four feedback forms:

- `onActionStarted`: records the command start and actual start pose.
- `onActionProgress`: records the latest pose and achieved translation/rotation.
- `onActionCompleted`: accepts a successful terminal result.
- `onActionFailed` and `onActionCancelled`: accept unsuccessful terminal results.

Terminal results contain actual timestamps and poses, achieved translation and
rotation, timeout/cancellation/safety/controller details, and collision flags.
They are written to `execution_history`; terminal attempts are written to
`navigation_history`; only successful attempts enter `completed_path_history`.
Completed-action learners are dispatched only after the terminal result is
accepted. Failed and partial movement remains available to circumstances,
diagnostics, and explanations without becoming successful trail, conveyor, or
skeleton evidence.

Every `PathDecisionPoint` exposes selection, controller-start, successful
completion, partial completion, failure, cancellation, timeout, safety
interruption, preemption, and `actualReachedPose()` independently. A
successful rotation is a completed action but is not traversed path geometry.
`successfulTraversal()` therefore requires a controller-started, successfully
completed translation. `partialTraversal()` requires a controller-started,
actually achieved partial translation. The terminal learning episode carries
the accepted start event explicitly; controller rejection cannot be inferred
as a started command from its terminal status.

The engine rejects unknown action IDs, stale decision IDs, task mismatches,
success before a start event, and duplicate terminal callbacks. A new decision
cannot be selected while terminal feedback is missing. Recently completed IDs
are retained in a bounded duplicate-detection window. Controller restarts
terminate any pending action as a controller failure.

## Learner event schedule

| Learner | Event received | Success policy |
|---|---|---|
| Region, barrier, familiarity, sensed occupancy | Every sensor observation or the representation's profile schedule | Independent of selected-action outcome |
| Inclusion | Region/skeleton finalization and successful LLE translation | Failed, cancelled, and rotation-only actions do not add inclusion |
| Trail, conveyor, passage/skeleton | Terminal action result | Successful completion only |
| Door/exit, hallway, circumstance | Terminal evidence accumulated; publish/rebuild at target boundary | Outcome remains attached; successful traversal is distinguishable from failures |
| Highway | Successful HLE terminal evidence; final rebuild at initial-exploration boundary | Successful HLE motion only |

Trail candidates use execution start and actual final poses from contiguous
eligible translations; failed/no-motion records cannot insert points or bridge
discontinuities. Conveyor frequency accepts only successfully completed
trails. Region-skeleton edges accept only contiguous successful translations.
Exits may accept a configured partial traversal only when its actual segment
crosses the circumference. Out searches and reverses only a contiguous suffix
of successfully reached poses and uses the execution start pose rather than
the selected action's expected start.

Replay stores the selected action separately from the full controller outcome,
including start/final poses, achieved motion, terminal status, interruption
flags, and detail. Why records the same terminal object and reports the actual
reached pose without rewriting the attempted action as completed motion.

Decision-cycle, action-start, periodic, shutdown, task-boundary, HLE-only, and
LLE-only schedules are explicit event types even when no default learner is
currently registered for one of them.
