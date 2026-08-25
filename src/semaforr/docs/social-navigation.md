# Social navigation API

SemaFORR adapts the collaborator-owned social interfaces at its ROS boundary
and exposes only `domain::CrowdObservation` to navigation. Select `tracked`,
`hunav`, or `none` with `social.input.mode`.

## Producers

- Tracked mode consumes `TrackedPersonArray`, `/pedestrian_predictions_tracked`,
  and optional `FormationGroupArray`.
- HuNav mode consumes `hunav_msgs/Agents` and `/pedestrian_predictions`.

Formation groups are optional in tracked mode. A group is retained only when
all member IDs match fresh current tracks. Its type, center, confidence, and
membership remain available in the internal observation for future advisors.
HuNav mode does not subscribe to tracked-camera formation output.

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

An observation older than `social.current_maximum_age_s` is stale. Predictions
expire independently under `social.prediction_maximum_age_s`; missing or
incomplete GST cycles use configured constant-velocity fallback. Stale, invalid, or
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
  `social.current_maximum_age_s`.
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

- live stateless arbitration (`SocialNavigationAdvisor`);
- learned density, encounter-risk, and flow advisors; and
- typed density, risk, and flow `DomainPlanner` instances.

Live interpersonal calculations use current positions and predicted
trajectories. Learned advisors and planner costs sample the identical grid
cells through `CrowdModel`; they do not maintain private grids. The shared
navigation-risk query takes the maximum of persistent encounter risk and fresh
predicted-collision risk, so the risk advisor and planner agree. When live
predictions disappear it falls back to learned evidence. Missing learned
evidence produces a neutral density/flow score or cost. Missing or stale live
data disables only live advisors and the transient part of composite risk.

No advisor or planner contains a ROS message type or maintains a parallel crowd
grid. ROS conversion occurs once in the adapter layer.

## Ownership and diagnostics

`CrowdFieldSnapshot` is an internal immutable SemaFORR representation. It is
never transported through `social_context_msgs`, and the standalone crowd
package contains no learner or duplicate field model. Social-context producers
own perception; SemaFORR owns adaptation, learning, revisions, planning,
advising, persistence, replay, and diagnostics.

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
