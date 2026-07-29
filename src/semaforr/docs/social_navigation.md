# Social navigation API

SemaFORR accepts exactly one navigation-facing social input:
`social_context_msgs/msg/SocialObservation` on the configurable
`topics.social_observations` topic.

The message header defines the observation time and coordinate frame for every
pedestrian. Each pedestrian has a stable string ID, current position and
velocity, an absolute-time predicted trajectory, confidence, and a row-major
2-by-2 XY position covariance. Prediction position and timestamp arrays must
have the same length. IDs must be unique within an observation, timestamps
must be strictly increasing and later than the observation time, and
covariance must be finite, symmetric, and positive semidefinite.

## Producers

- `social_context/social_context_hunav` converts HuNav simulator agents.
- `semaforr_bridge/tracked_people_to_social_observation` converts the legacy
  tracked-person stream.

Upstream detector, tracker, or simulator messages remain implementation
details of their producer. They are not additional SemaFORR inputs. Producers
must preserve a stable ID across observations and should publish in the
configured global frame. The node uses TF to normalize other frames, including
positions, velocities, predicted positions, and covariance.

## Lifecycle and fallback

The ROS adapter validates each message before it enters the domain. It computes
`CrowdObservation.data_age` from ROS time and filters pedestrians below
`social.minimum_confidence`. `SocialObservationBuffer` rejects invalid frames,
future timestamps, invalid values, clock resets, and malformed trajectories.

An observation older than `social.maximum_age_s` is stale. Stale, invalid, or
missing data clears only the current crowd snapshot; bounded history remains
available for diagnostics. Social advisors opt out and social planner costs
return their neutral fallback when no valid current snapshot exists. Ordinary
navigation therefore continues without treating old predictions as live
people.

## Consumers

`CrowdState` is the only social state stored by the domain and legacy
compatibility layer. It is provided to:

- stateless social arbitration (`SocialNavigationAdvisor`);
- legacy interpersonal and crowd-avoidance advisors, which participate only
  while `AgentState::hasValidCrowd()` is true; and
- density, risk, and flow planning costs through `PathPlanner::setCrowdState`.

`AgentState` and `PathPlanner` contain no ROS message types. Their compatibility
pose views are read-only projections used by existing visualization code, not
parallel inputs.

## Deterministic scenarios

`semaforr_social_navigation_test` covers interpersonal proximity, crossing,
following, opposing flow, stale-data opt-out, frame/confidence validation, and
a recorded opposing-flow trajectory. The recorded case proves that, with the
same state and random seed, adding the trajectory changes advisor scores and
changes the selected action from forward to pause.
