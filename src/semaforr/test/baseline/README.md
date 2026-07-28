# SemaFORR Phase 0 Baseline

This directory captures observable behavior before the SemaFORR refactor.
It contains two complementary baselines:

1. `source_contract.json` records behavior that can be characterized without
   starting ROS: topics, velocity conversion, action completion thresholds,
   default action magnitudes, feature flags, and tutorial targets.
2. A runtime trace records `cmd_vel` transitions and `decision_log` messages
   while a deterministic virtual robot publishes pose and open-space laser
   observations.

The source contract is verified by `test_source_contract.py`. It is deliberately
strict: if a refactor changes a value, the test should fail until the change is
reviewed and the contract is intentionally updated.

## Capture the runtime golden trace

Build and source the workspace in ROS 2 Humble:

```bash
src/semaforr/scripts/run_phase0.sh normal
source install/setup.bash
mkdir -p baseline-results
ros2 launch semaforr stage_tutorial_baseline.launch.py \
  output:="$(pwd)/baseline-results/stage_tutorial.actual.json"
```

Inspect the complete trace before approving it. Once accepted, copy it to:

```text
src/semaforr/test/baseline/stage_tutorial.expected.json
```

Do not approve a trace merely because the process exited successfully. Review
the action sequence, decision diagnostics, final pose, ROS warnings, and known
issues first.

## Compare a later run

```bash
python3 src/semaforr/scripts/verify_baseline.py \
  src/semaforr/test/baseline/stage_tutorial.expected.json \
  baseline-results/stage_tutorial.actual.json
```

Timestamps, detailed advisor comments, measured computation times, and the
final floating-point pose are not compared. The scenario metadata, velocity
transition sequence, and semantic decision sequence must match. The complete
actual trace retains these volatile details for inspection and performance
comparison.

## Profiles

```bash
src/semaforr/scripts/run_phase0.sh normal
src/semaforr/scripts/run_phase0.sh sanitizer
src/semaforr/scripts/run_phase0.sh coverage
```

The sanitizer profile enables AddressSanitizer, UndefinedBehaviorSanitizer, and
leak detection. The coverage profile instruments the package and creates an
HTML report when `lcov` and `genhtml` are installed.

## Local environment status

The baseline infrastructure was introduced on 2026-07-27 from a Windows host.
That host did not provide ROS 2 or `colcon`, and its Docker daemon was stopped.
The ROS-independent C++ characterization executable was validated locally, but
the first authoritative ROS trace and coverage report must be captured in a
ROS 2 Humble environment. An unobserved trace has intentionally not been
invented or committed.
