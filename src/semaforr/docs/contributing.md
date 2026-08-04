# Contributor guide

## Build and test

Use ROS 2 Humble and install dependencies from manifests:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select semaforr
colcon test-result --verbose
```

Run `python3 src/semaforr/scripts/check_source_quality.py` before submitting.
The sanitizer, coverage, and regression commands are in
`testing.md`.

## Design rules

- Domain, decision, planning, spatial, and social code must compile without ROS
  headers or initialization. Convert messages only in `src/ros`.
- Put stable public interfaces under `include/semaforr`; keep helpers private to
  `src`.
- Use values, references, `std::unique_ptr`, and `std::optional` according to
  ownership. Do not allocate in a decision cycle.
- Return typed results and preserve explicit units (`_m`, `_rad`, `_s`, `_mps`).
- Make randomness injectable and seed it once. Sort externally visible
  diagnostics.
- Reject invalid configuration at startup. Do not add silent defaults after
  validation.
- Add components through registries and tests, not dispatch edits in the ROS
  node.

## Tests required with changes

Add the narrowest ROS-independent unit test first. Add a component test when
ownership or lifecycle spans classes, and a fixture-driven integration test
when decisions or mission progression can change. Update a golden trace only
after classifying and documenting every difference.

New or modified modern code is built with strict warnings. Coverage thresholds
apply to files listed in `config/coverage_thresholds.json`; extend that
manifest when adding production components.

## Style and review

Use the package `.clang-format` and `.clang-tidy` files where the tools are
available. Keep changes focused, remove dead code only after preserving history
in version control, and update the relevant catalog/configuration contract in
the same change. A review should be able to trace behavior from configuration
through a typed result to a structured diagnostic.
