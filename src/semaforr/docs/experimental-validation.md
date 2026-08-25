<!-- File overview: This file documents or configures experimental validation behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/experimental-validation.md`. -->

# Experimental validation

Experiments use named profiles that expand into the same validated configuration
objects as custom runs. The evaluation profiles are `purely_reactive`,
`original`, `doors`, `least_angle`, `access`, `tentative`, `hallways`,
`shortest_path`, `cost_graph`, `wander`, `deliberator`, `forward_only`,
`global_exploration`, `local_exploration`, `highway`, `circumstances`, and
`naive`. The general component-ablation profiles remain available alongside
them.

All currently runnable profiles execute with
`experiment.behavior_mode: modernized`. Profile names identify feature
combinations and never imply dissertation-algorithm fidelity. Compatibility
mode remains fail-closed until its acceptance suite exists.

The scenario catalog in
`test/fixtures/scenarios/evaluation_scenarios.json` defines compact artificial,
museum, and office families, repeated target-sequence counts, and parameter
sweeps. Environment geometry and target files remain external inputs so a run
cannot silently substitute one floor plan for another.

Run the installed deterministic matrix with:

```bash
ros2 run semaforr run_experiment_matrix.py \
  --output-directory results/profile-matrix \
  --runs 2 --duration 20 --seed 0
```

The matrix writes one baseline scenario JSON per run plus a manifest containing
profile, seed, output path, and return code. Baseline JSON records selected
policy, decision tier, planning and model-update latency, decision latency,
process allocation count and requested bytes, covered cells, traveled distance,
target outcomes, and intervention frequency. It is a simulator/metrics trace,
not the versioned offline-replay format. Allocation measurements count calls to the process-wide C++
allocation operators during each decision; they are not resident-memory
samples. Coverage counts the union of one-metre cells overlapped by learned
regions and trails. Runtime traces report the numerator; the ROS-independent
collector divides it by the scenario's freespace-cell count. Target success is
derived from explicit `target_completed` events, while activated targets that
do not complete remain failed attempts.

For decision replay, separately enable `reproducibility.recording.enabled` and
set `reproducibility.trace_path`. That versioned trace records the expanded
configuration and fingerprint, scoped seeds, source/test revisions, map
checksum, task sequence, model versions, component manifest, sensor inputs,
controller outcomes, plans, decisions, advisor-score digest, and explanation
digest. See [Replay and experiment reproducibility](reproducibility.md).

For comparisons, retain the behavior mode, profile, random seed, source
revision, test-suite revision, map checksum, ordered target sequence, decision
limit, exploration budget, configuration fingerprint, and component manifest.
Report exploration and target time and distance separately, and aggregate
repeated runs with both a mean and dispersion measure.
