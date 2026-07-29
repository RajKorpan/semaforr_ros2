# Migrating legacy configuration

Legacy SemaFORR split runtime settings across positional `.conf` files and
embedded source paths. ROS 2 deployments should convert an experiment once,
review the generated YAML, and then maintain YAML as the source of truth.

## Convert

After building and sourcing the workspace:

```bash
ros2 run semaforr semaforr_convert_legacy_config \
  --advisors legacy/advisors.conf \
  --parameters legacy/params.conf \
  --map legacy/map.xml \
  --tasks legacy/target.conf \
  --dimensions legacy/dimensions.conf \
  --output converted.yaml
```

The converter parses exact keys and positions and reports malformed lines
instead of continuing with uninitialized values. It does not modify the legacy
files.

## Review

1. Move the YAML, map, and mission into a ROS package.
2. Resolve map and mission paths with `get_package_share_directory` in a launch
   file; do not commit workstation-specific absolute paths.
3. Verify all advisor names against `advisor-catalog.md`. Remove historical
   names that are declared but not registered.
4. Replace `CUSUM` or `discount` planner entries with
   `social.learning.estimator`; enable `density`, `risk`, `flow`, or `combined`
   only when needed.
5. Confirm dimensions, metre/radian units, action ordering, weights, feature
   prerequisites, frames, topics, QoS, and social age/confidence policy.
6. Launch once with the deterministic fixture and compare the structured trace
   with the retained baseline.

## ROS 1 interfaces

Do not recreate `CrowdModel`, `crowd_pose`, and `crowd_pose_all` as parallel
inputs. Convert the upstream producer to the one
`social_context_msgs/msg/SocialObservation` contract. The learned
`CrowdField` is a SemaFORR output and diagnostic projection.

ROS 1 crowd estimator source is retained under ignored directories for
reference. Its count/exposure, discount, and CUSUM semantics now live behind
`social.learning.estimator`; those packages are not built in ROS 2.

## Acceptance

The migrated launch must start with `configuration.use_legacy_files: false`,
contain no source-tree paths, pass startup validation, and reproduce or
intentionally explain the legacy decision trace.
