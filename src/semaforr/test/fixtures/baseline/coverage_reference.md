<!-- File overview: This file exercises coverage reference behavior for automated verification and regression testing. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `test/fixtures/baseline/coverage_reference.md`. -->

# Coverage reference

The baseline profile was regenerated against the current package structure
under ROS 2 Humble on 2026-07-29. All 67 tests passed before capture.

The HTML report is generated at `coverage/html/index.html`. System headers and
test sources are excluded; headers and implementation files in the SemaFORR
package are included.

- Lines: 11.9% (`1650/13876`)
- Functions: 36.5% (`472/1292`)
- Branches: not collected

The percentage is intentionally a behavioral baseline, not a coverage target.
It exposes the amount of unexercised inherited planning and spatial-learning
code while directly covering the extracted domain values, configuration
validation, ownership model, decision coordinator, geometry, graph/A*, map
parser, ROS adapters, and build contracts.
