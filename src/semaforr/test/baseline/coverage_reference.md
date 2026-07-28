# Initial coverage reference

The Phase 0 coverage profile was run under ROS 2 Humble on 2026-07-27.

The initial characterization suite directly executes two production translation
units:

- `src/core/FORRAction.cpp`
- `src/core/Position.cpp`

The HTML report is generated under `coverage/html/index.html`. The report is
filtered to SemaFORR production files; system headers and characterization test
code are excluded.

For this deliberately narrow initial scope, the measured result is:

- Lines: 100.0% (`50/50`)
- Functions: 100.0% (`15/15`)
- Branches: not collected

This is intentionally a starting measurement, not a claim of meaningful
whole-controller coverage. The integrated baseline scenario characterizes ROS
behavior, but the current node aborts during launch shutdown and therefore
cannot reliably flush coverage data. Later phases should increase domain-level
coverage as components are separated from ROS and should add a clean shutdown
integration test.
