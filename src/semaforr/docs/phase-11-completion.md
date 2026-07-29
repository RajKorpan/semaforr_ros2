# Phase 11 completion: event-driven ROS node

Phase 11 replaces the blocking `spin_some()` action loop with subscriptions and
a ROS-clock timer. The executor advances one step per timer callback, so sensor,
shutdown, and monitoring callbacks remain serviceable while an action is in
progress.

## Component boundaries

| Component | Responsibility |
| --- | --- |
| `SemaFORRNode` | Declares and validates runtime parameters, owns ROS entities, transforms poses, and advances the node state machine |
| `SensorSynchronizer` | Validates pose/scan frames and values and exposes only fresh, time-coherent observations |
| `NavigationEngineAdapter` | Isolates the established `Controller` facade and maps selected actions to configured distance/angle targets |
| `CommandExecutor` | Executes one discrete action without blocking and returns typed completion or failure status |
| `VisualizationPublisher` | Translates decision/model snapshots into the established ROS visualization topics |

`NavigationEngineAdapter` is a compatibility boundary. The ROS-independent
`semaforr::decision::NavigationEngine` remains the target architecture; no ROS
message type enters that decision interface.

## State machine

```text
WaitingForSensors
        |
        v
  ReadyToDecide
        |
        v
 ExecutingAction ---- completed ----> ReadyToDecide
        |                                  |
        +---- timeout/reset/cancel --------+
        |
        +---- shutdown -------------------> Stopped
```

The node publishes each transition on `topics.navigation_state`. A failure is
included after the state name, for example
`WaitingForSensors:sensor_pose_stale` or
`ReadyToDecide:action_timed_out`.

`start()` creates subscriptions, TF listening, and the ROS-clock control timer.
`stop()` cancels the timer and action, publishes zero velocity, and transitions
to `Stopped`. It is registered as a pre-shutdown callback, so the final command
is published before the ROS context invalidates its publishers. This gives
explicit activation and shutdown semantics without
requiring lifecycle-manager infrastructure from deployments that currently
launch a regular `rclcpp::Node`.

## Observation and frame policy

- Pose and scan source timestamps must be within
  `timing.sensor_sync_tolerance_s`.
- Arrival and source timestamps are checked against
  `timing.sensor_timeout_s`.
- Non-finite poses, invalid scan metadata, clock resets, empty frames, and
  unexpected scan frames cannot produce a decision observation.
- Poses already in `frames.global` are accepted directly. Other pose frames are
  transformed through TF with `frames.transform_timeout_s`; there are no
  configured coordinate offsets.
- Scan data must use `frames.scan`. Transforming every range into another scan
  plane is deliberately not approximated.
- All timeouts, stamps, action deadlines, and the control timer use the node's
  ROS clock.

## Discrete action execution

Action targets and command rates are independent:

- forward actions use configured action-space distances and
  `command.linear_velocity_mps`;
- turns use configured action-space angles and
  `command.angular_velocity_radps`;
- `command.turn_linear_velocity_mps` preserves the established small linear
  component during turns;
- progress is accumulated from successive odometry samples, with angle
  differences normalized at every update;
- discontinuities exceeding the odometry-reset thresholds stop the action with
  a typed `OdometryReset` result;
- deadlines derive from target/rate, a multiplier, and a minimum timeout.

Small turns must show measured angular progress. The effective completion
tolerance is capped relative to the requested angle, preventing a target
smaller than the general tolerance from completing immediately.

## Runtime parameters

The following groups are in `config/semaforr.yaml`:

- `topics.pose`, `topics.scan`, `topics.command`, and
  `topics.navigation_state`;
- sensor and command queue depth, reliability, and durability under `qos`;
- `frames.global`, `frames.scan`, and transform timeout;
- control frequency, sensor age, and pose/scan skew under `timing`; and
- linear/angular rates, tolerances, deadline policy, and odometry-reset
  thresholds under `command`.

Empty topics/frames, non-positive depths or numeric limits, and unsupported QoS
strings fail node construction with a precise error.

## Verification

Normal and sanitizer builds use the existing package commands:

```bash
colcon build --packages-select semaforr --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select semaforr
colcon test-result --verbose
```

The stale-sensor behavior is reproducible:

```bash
source install/setup.bash
ros2 launch semaforr stage_tutorial_baseline.launch.py \
  duration:=4.0 sensor_cutoff:=1.5 \
  output:="$(pwd)/baseline-results/sensor-timeout.actual.json"
python3 src/semaforr/scripts/verify_sensor_timeout.py \
  baseline-results/sensor-timeout.actual.json
```

The verified trace first contains a non-zero turn command, then records
`WaitingForSensors:sensor_pose_stale`, and ends with an exact zero velocity
command.

ROS 2 Humble results for this phase:

- normal build: successful;
- normal suite: 94 tests, zero failures;
- AddressSanitizer, UndefinedBehaviorSanitizer, and leak-enabled suite:
  94 tests, zero failures and no sanitizer report;
- downstream installed-package build: successful for the domain, core,
  planning, advisors, spatial, and ROS targets;
- two 20-second callback-node replays: identical eight-action decision and
  five-command semantic sequences; and
- live sensor-cutoff replay: stale data detected and zero command published.

The Phase 0 golden trace is intentionally retained rather than overwritten.
The callback executor produces eight decisions in the 20-second scenario
instead of eleven because the old loop treated the smallest turn as complete
without observed angular progress. That behavior change is the reviewed safety
fix described above; all decisions through the common eight-decision prefix
remain identical.
