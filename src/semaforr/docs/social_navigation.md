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

An empty but valid observation is retained. It is negative evidence for the
learned density field, but it does not activate live social advisors.

## Unified domain model

`CrowdModel` is the single domain-owned social aggregate. It contains two
parts with intentionally different lifetimes:

- `CrowdState observations`: the current validated observation and bounded
  history. Current positions and predictions expire with
  `social.maximum_age_s`.
- `CrowdFieldSnapshot learned`: persistent, map-aligned density, encounter
  risk, and eight-bin directional-flow evidence. It does not expire merely
  because the latest detector message is stale.

`CrowdFieldLearner` updates the learned part from a coherent robot pose, laser
scan, and valid `CrowdObservation`. Laser visibility supplies the exposure
denominator, including valid observations containing no pedestrians. A person
contributes only when their grid cell is visible. The robot cell records
encounter opportunities and near-person encounters. Learning is rate-limited
by `social.learning.minimum_update_period_s`.

The configured estimator controls how evidence evolves:

- `count_exposure` accumulates hits divided by visible exposures;
- `discounted_count` discounts evidence in cells that are observed again;
- `cusum` resets changed cells using two-sided CUSUM detection; and
- `thompson` draws a deterministic, seed-controlled Gamma sample.

Snapshots retain their raw denominators, confidence, update time, estimator,
and monotonically increasing version. `CrowdFieldSnapshot::save/load`
provides a validated, ROS-independent persistence format.

## Consumers

All consumers receive the same `CrowdModel`:

- stateless social arbitration (`SocialNavigationAdvisor`);
- legacy interpersonal and crowd-avoidance advisors, which participate only
  while `AgentState::hasValidCrowd()` is true; and
- learned density, risk, and flow advisors; and
- density, risk, flow, and combined path planners through
  `PathPlanner::setCrowdModel`.

Live interpersonal calculations use current positions and predicted
trajectories. Learned advisors and planner costs sample the identical grid
cells through `CrowdModel`; they do not maintain private grids. The shared
navigation-risk query takes the maximum of persistent encounter risk and fresh
predicted-collision risk, so the risk advisor and planner agree. When live
predictions disappear it falls back to learned evidence. Missing learned
evidence produces a neutral density/flow score or cost. Missing or stale live
data disables only live advisors and the transient part of composite risk.

`AgentState` and `PathPlanner` contain no ROS message types. Their compatibility
`setCrowdState` and pose views are projections for retained legacy callers, not
parallel state or ROS inputs.

## Derived output and diagnostics

SemaFORR publishes its learned snapshot as
`social_context_msgs/msg/CrowdField` on `topics.crowd_field`. This is a derived,
transient-local diagnostic output, not another social observation input. The
`semaforr_crowd` package subscribes only to this message and projects it into
density/risk occupancy grids and directional-flow markers. Consequently the
bridge/social-context producers own perception, SemaFORR owns learning, and the
crowd package owns visualization; no node learns the same evidence twice.

The learner frame, geometry, estimator, update rate, thresholds, confidence
scale, and random seed are configured under `social.learning.*`. The grid uses
the validated map dimensions and configured origin/resolution. All units are
meters, seconds, and radians.

## Deterministic scenarios

`semaforr_social_navigation_test` covers interpersonal proximity, crossing,
following, opposing flow, stale-data opt-out, frame/confidence validation, and
a recorded opposing-flow trajectory. The recorded case proves that, with the
same state and random seed, adding the trajectory changes advisor scores and
changes the selected action from forward to pause.

`semaforr_crowd_model_test` additionally covers visibility-normalized negative
evidence, seeded Thompson estimates, serialization/restore, persistence after
live data becomes stale, and identical learned-cell use by advisors and
planners.
