
# Social Context Package

Provides social-observation producers and the optional camera/lidar
pose-estimation pipeline used with Social-SemaFORR.

## Overview

The primary HuNav adapter converts HuNavSim agent states to the canonical
`SocialObservation` interface and produces deterministic constant-velocity
predictions. Optional pose-estimation executables include learned 2D pose
detectors; they are separate from the HuNav adapter.

## Features

- HuNavSim-to-`SocialObservation` conversion with constant-velocity prediction
- Optional camera/lidar human detection, localization, and tracking tools
- ROS 2 node for real-time social-context updates
- Integration with HuNavSim and SemaFORR
- Configurable and extensible model support

## Usage

### Build

Make sure your workspace is built and sourced:

```bash
colcon build
source install/setup.bash
```

### Run Social Context Node

Launch the main node for social context prediction:

```bash
ros2 run social_context social_context_hunav
```

## Node

- **social_context_hunav**
  - Subscribes to `/human_states` (`hunav_msgs/msg/Agents`).
  - Publishes the canonical
    `social_context_msgs/msg/SocialObservation` on `/social_observations`.
  - Topic names and prediction horizon are parameters: `agents_topic`,
    `social_observation_topic`, `prediction_steps`, and `prediction_step_s`.
  - Uses the top-level `hunav_msgs` interface package; no private message copy
    is bundled in this package.

## Virtual Environment & Dependencies

The HuNav adapter uses the package's declared ROS dependencies. Optional
camera-based pose estimation requires the additional packages listed in the
trajectory-prediction requirements file.

1. Create a Python 3.10 virtual environment (recommended: conda).
2. Install dependencies:

	 ```bash
	 pip install -r src/social_context/trajectory_prediction/requirements.txt
	 ```
