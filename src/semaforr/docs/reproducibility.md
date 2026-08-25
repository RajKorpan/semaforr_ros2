<!-- File overview: This file documents or configures reproducibility behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/reproducibility.md`. -->

# Replay and experiment reproducibility

SemaFORR can record a versioned, self-contained decision trace by setting:

```yaml
reproducibility:
  recording:
    enabled: true
  trace_path: /output/run.trace
  source_revision: <source commit or release>
  test_suite_revision: <scenario/test revision>
```

The trace header records the expanded configuration snapshot and fingerprint,
behavior mode, ablation profile, component manifest, static-map checksum (or
`mapless`), model versions, ordered task sequence, source/test revisions,
compatibility deviations, and all five scoped random seeds. Each cycle records
the original synchronized scan timestamp, pose, complete laser scan, optional
crowd observation, exact representation revisions, plan identity and digest,
Tier decision, advisor-score digest, selected action, explanation digest, and
the terminal controller outcome when one is received.

## Random streams

The following streams are independent:

* `experiment.seeds.tier_three_ties`
* `experiment.seeds.lle_fallback`
* `experiment.seeds.planner_ties`
* `experiment.seeds.clustering`
* `experiment.seeds.simulation_noise`

Tier-3 arbitration and LLE consume their corresponding streams. Tier-2 uses
the planner stream only when `tiers.tier2.tie_policy` resolves to
`seeded_exact`. Current clustering is deterministic and simulation noise is
owned by the simulator, but both seeds are recorded so a later stochastic
implementation or an external simulator can share the same run identity.

`experiment.random_seed` is retained only for migration. If it is nonzero and
all scoped seeds remain zero, it initializes all five streams. New experiments
should set the scoped fields directly.

## Offline replay

`validation::RunRecorder::load()` restores the recorded inputs without ROS.
`validation::OfflineReplay::run()` feeds each observation and preceding
controller result to a caller-provided ROS-independent decision harness. It
compares exact revisions, plan output, tier/source/policy, advisor comments,
action selection, and explanation traces. The report names every differing
field and separately rejects changes in configuration fingerprint, map
checksum, source revision, or seed set. This makes a difference attributable
to a concrete input, model, configuration, or implementation revision.

Modernized runs explicitly record their compatibility deviations and point to
`compatibility-matrix.md`. Compatibility mode remains fail-closed until its
acceptance blockers are resolved, so a trace cannot falsely claim complete
experimental compatibility.

## Runtime validation

Startup validation has two stages. Static validation rejects unsupported
fields and cross-field contradictions. Runtime validation runs after map load
and planner registration and reports the number of active planners, active
explanation mode, scoped seeds, recording state, and a final passed marker.
Map load policy may explicitly disable map components; otherwise a requested
planner set that produces no active planner stops startup.
