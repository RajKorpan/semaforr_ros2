# SemaFORR bridge package

ROS 2 adapters that translate upstream pose and people-tracking messages into
the interfaces consumed by SemaFORR. The package does not own navigation or
social learning state.

## Overview

SemaFORR Bridge provides nodes that convert and relay messages between different ROS topics and message types, ensuring compatibility between robot controllers and the SemaFORR navigation system.

## Installed adapters

| Executable | Input | Output | Purpose |
| --- | --- | --- | --- |
| `odom_to_pose_bridge` | `nav_msgs/msg/Odometry` on `odometry_topic` (default `/mobile_base_controller/odom`) | `geometry_msgs/msg/PoseStamped` on `pose_topic` (default `/pose`) | Copy odometry pose and header into the navigation pose interface. |
| `tracked_people_to_social_observation` | `social_context_msgs/msg/TrackedPersonArray` on `tracked_people_topic` (default `/human_poses_3d_tracked_global`) | `social_context_msgs/msg/SocialObservation` on `social_observation_topic` (default `/social_observations`) | Validate tracked people and add constant-velocity predictions for the canonical social interface. |

The people adapter also exposes `prediction_steps`, `prediction_step_s`,
`history_step_s`, and `default_position_variance`. It rejects messages without
a coordinate frame and ignores non-finite tracks.

## Usage

### Build

Make sure your workspace is built and sourced:

```bash
colcon build
source install/setup.bash
```

### Run an adapter

Launch the bridge node to convert Odometry to PoseStamped:

```bash
ros2 run semaforr_bridge odom_to_pose_bridge
```

or:

```bash
ros2 run semaforr_bridge tracked_people_to_social_observation
```

Override topics with ROS parameters, for example:

```bash
ros2 run semaforr_bridge odom_to_pose_bridge --ros-args \
  -p odometry_topic:=/odom -p pose_topic:=/robot_pose
```
