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
- When the baseline launch sends `SIGINT` after recording, `semaforr_node`
  currently throws `rclcpp::exceptions::RCLError` while creating a guard
  condition against an invalid context and exits with `SIGABRT`.
- The default configuration enables no A* planner and no Tier 2 planner variant.
- Extensive owning raw pointers make leaks likely; the sanitizer profile is
  expected to expose them.

## Initial sanitizer runtime

A five-second instrumented ROS 2 Humble scenario was run on 2026-07-27. It
terminated before leak reporting and exposed these earlier failures:

- Undefined behavior in `FORRRegion.h:18`: an invalid value was loaded as a
  `bool`, indicating uninitialized or corrupted region state.
- A null `Task` was used by `Visualizer.h:1357`.
- The null access reached `Task::getDecisionCount()` in `Task.h:187` and caused
  an AddressSanitizer segmentation fault.

The characterization unit tests themselves pass under AddressSanitizer and
UndefinedBehaviorSanitizer. The crash is specific to the integrated runtime and
should become a regression test when its initialization path is corrected.

Refactoring may fix these items, but each intentional behavior change should be
covered by a new expectation and called out in the corresponding change.
