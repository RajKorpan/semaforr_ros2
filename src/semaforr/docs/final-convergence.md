# Final convergence audit

This is the maintained completion record for the phased SemaFORR refactor.
Detailed subsystem contracts remain in the linked architecture, testing,
deployment, social-navigation, and spatial-learning guides.

## Final architecture

The active navigation runtime contains no ROS 1 controller, agent state,
advisor factory, pointer-owned graph, positional decision log, parallel crowd
model input, or private copy of a message package.

The dependency direction is:

```text
domain <- planning / advisors / spatial <- navigation <- ROS adapters <- node
```

`semaforr_msgs/msg/DecisionRecord` is the stable observability and explanation
API. `social_context_msgs/msg/SocialObservation` is the only pedestrian input,
and `social_context_msgs/msg/CrowdField` is a learned-model output used for
diagnostics. The `why` and `why_plan` nodes consume `DecisionRecord` directly.

The superseded ROS 1 crowd learner variants and the private nested
`hunav_msgs` copy were removed after their supported behavior moved into the
ROS-independent crowd learner and top-level interface package. Git history is
the migration archive.

## Implementation sequence status

All requested sequence items are represented in the final tree:

1. Deterministic fixtures, golden traces, runtime metrics, sanitizer profiles,
   coverage reports, and known-defect records establish the baseline.
2. Explicit component libraries, audited manifests, installed resources,
   strict warnings, tests, and downstream target checks repair packaging.
3. Public `.hpp` interfaces and inward dependencies normalize the directory.
4. Typed actions, geometry, observations, mission state, histories, crowd
   state, and `WorldModel` form the ROS-independent core.
5. Values, `unique_ptr`, references, and optionals replace raw ownership.
6. Validated ROS-parameter YAML and the offline legacy converter replace
   runtime positional configuration.
7. Navigation, mission, planning, spatial learning, and decision coordination
   replace the monolithic controller.
8. Mandatory rules, veto rules, planners, and advisors use focused typed
   interfaces and registries.
9. Deterministic arbitration defines safe stop, fallback, tie, finite-score,
   veto, contribution, ordering, and seed behavior.
10. Geometry, immutable A*, graph storage, map parsing, and typed path results
    are ROS-independent and tested.
11. Each spatial representation has an independent learner lifecycle,
    snapshot, enable flag, serialization boundary, and consumer contract.
12. The callback-driven ROS node synchronizes sensors, executes actions
    asynchronously, validates time/frames, and stops safely.
13. Social observations enter one stable API, populate one crowd model, and
    deterministically affect advisors and planners only while valid.
14. One structured decision record explains candidates, vetoes, advisor
    scores/contributions, planner, action, timing, task, and outcome.
15. Unit, component, integration, regression, sanitizer, formatting, static
    analysis, and modified-code coverage gates run in ROS 2 Humble.
16. Architecture, catalogs, configuration, topics/frames, learning,
    troubleshooting, contribution, migration, examples, launch, RViz, Docker,
    and CI documentation support deployment.
17. Compatibility runtime code and quarantined ROS 1 source copies are gone.

## Verification

The final ROS 2 Humble verification results and exact coverage percentages are
recorded in [phase-14-testing.md](phase-14-testing.md). The documented example
launch produces a structured trace from installed resources, and an
independent downstream CMake project links and runs against all six exported
SemaFORR libraries.

Use:

```bash
source /opt/ros/humble/setup.bash
colcon build
colcon test
colcon test-result --verbose

src/semaforr/scripts/run_phase0.sh sanitizer
src/semaforr/scripts/run_phase0.sh coverage
docker compose build semaforr
docker compose run --rm semaforr
```
