# Build and package contract

SemaFORR targets ROS 2 Humble and C++20. From a sourced Humble workspace:

```bash
colcon build --packages-up-to semaforr \
  --cmake-args -DBUILD_TESTING=ON -DSEMAFORR_STRICT_MODERN_CODE=ON
colcon test --packages-select semaforr
colcon test-result --verbose
```

The build uses explicit source lists and exports:

- `semaforr::domain`: value types, configuration, motion, crowd model;
- `semaforr::planning`: graph, A*, map parser, typed planners;
- `semaforr::advisors`: rules, advisors, deterministic arbitration;
- `semaforr::spatial`: independently enabled spatial learners;
- `semaforr::navigation`: mission and cognitive-cycle coordination;
- `semaforr::ros_adapters`: ROS messages, TF, parameters, execution, and
  visualization.

There are no compatibility targets. Downstream packages should link only the
smallest required target. The smoke project in `test/integration/downstream`
checks all exported targets from an installed package.

All production libraries compile with `-Wall -Wextra -Wpedantic -Werror`.
`SEMAFORR_ENABLE_SANITIZERS=ON` enables AddressSanitizer and
UndefinedBehaviorSanitizer; leak detection is enabled through
`ASAN_OPTIONS=detect_leaks=1` on supported Linux builds.
`SEMAFORR_ENABLE_COVERAGE=ON` enables the coverage profile.

Configuration is loaded exclusively from ROS parameters backed by YAML.
`map.path` and `mission.tasks_path` are resolved by launch files from the
installed package share. The offline `semaforr_convert_legacy_config` utility
converts retained experiment files once; the node does not parse them.

The Dockerfile builds the complete workspace from a Humble base. The
reproducible example is:

```bash
docker compose build semaforr
docker compose run --rm semaforr
```

See [architecture.md](architecture.md),
[configuration-reference.md](configuration-reference.md), and
[troubleshooting.md](troubleshooting.md).
