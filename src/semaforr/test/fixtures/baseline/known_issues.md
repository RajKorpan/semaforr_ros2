# Known behavior at the Phase 0 boundary

These observations are recorded so that characterization does not turn known
defects into permanent requirements.

- `Position()` invokes a temporary three-argument constructor instead of
  initializing its members. Default-constructed coordinates are indeterminate.
- Tier 3 assigns `PAUSE` when no advisor supplies a candidate, but then still
  performs modulo by the empty candidate count.
- Tier 3 seeds the process-global pseudo-random generator on every decision
  using one-second wall-clock resolution.
- Configuration readers do not reliably reject missing, empty, or malformed
  files, and some controller fields can remain uninitialized.
- `RobotDriver` does not validate all six required path parameters before
  constructing `Controller`.
- Crowd subscriptions are commented out, so social observations do not reach
  the controller in the checked-in runtime.
- Action execution uses pose displacement and elapsed-time thresholds rather
  than acknowledgement from a lower-level controller.
- The command executor adds `0.01 m/s` of forward motion during both left and
  right turns.
- No explicit final zero-velocity publication is guaranteed on every shutdown
  or exceptional exit path.
- The default configuration enables no A* planner and no Tier 2 planner variant.
- The compatibility implementation still contains substantial legacy
  signed/unsigned, initialization-order, and possibly-uninitialized warnings.
  They are not part of the golden behavioral contract.

## Sanitizer runtime

The initial five-second instrumented scenario on 2026-07-27 terminated before
leak reporting and exposed these earlier failures:

- Undefined behavior in `FORRRegion.h:18`: an invalid value was loaded as a
  `bool`, indicating uninitialized or corrupted region state.
- A null `Task` was used by `Visualizer.h:1357`.
- The null access reached `Task::getDecisionCount()` in `Task.h:187` and caused
  an AddressSanitizer segmentation fault.

The final 20-second scenario on 2026-07-29 found and fixed two additional
boundary defects: a string-array reference obtained from a temporary ROS
parameter and an unused out-of-bounds third action read in
`advisorNotOpposite`. After those repairs, all 67 tests and the complete runtime
scenario pass with AddressSanitizer, UndefinedBehaviorSanitizer, and leak
detection enabled. No sanitizer report is emitted, and the resulting decision
trace matches the approved golden trace.

Refactoring may fix these items, but each intentional behavior change should be
covered by a new expectation and called out in the corresponding change.

## Resolved after the baseline

Phase 8 replaced Tier 2 and Tier 3 time-seeded tie selection with stable
ordering. Tier 3 now returns `PAUSE` when no finite candidate exists instead of
performing modulo by zero. Tier 2 rejects empty plans and safely declines
selection when all candidate costs are non-finite.

Phase 4 replaced the tested owning raw-pointer paths with values and
`std::unique_ptr`. Phase 5 made configuration fields initialized and validated.
The ROS shutdown path now exits cleanly after the baseline recorder sends
`SIGINT`.
