# Grid layers and traversability

SemaFORR does not use familiarity as occupancy. The runtime keeps four
different grid concepts because they answer different questions.

| Layer | Ownership | Meaning | Mutable? | Primary consumers |
|---|---|---|---|---|
| Familiarity (`known_grid`) | `KnownGridLearner` | observation count, last observation sequence, and confidence | learned incrementally | Out, novelty, coverage and familiarity diagnostics |
| Sensed occupancy | `SensedOccupancyLearner` | unknown, observed free, or observed occupied with evidence, confidence, conflict, dynamic state, sequence, and source | learned incrementally | sensor-grid planning and occupancy fusion |
| Static occupancy | optional `StaticMap` | static free, static occupied, or format-defined unknown | immutable after startup | known-map grid and crowd planners |
| Traversability | planner request | traversable, blocked, unknown, inflated, and outside-extent states | derived, never learned | the planner that requested it |

Inclusion, highway evidence, hallways, conveyors, regions, crowd fields and
planner costs are separate representations. A positive familiarity count is
never interpreted as a free cell.

The layers share the coordinate and extent contract documented in
[grid geometry and expansion](grid-geometry.md), without sharing cell meaning.

## Range integration

Each scan is transformed using the observed world pose. NaN, negative
infinity, below-minimum and above-maximum beams are ignored. Positive infinity
and a return at `range_max` clear the valid ray without making an occupied
endpoint. A finite return below `range_max` marks cells before the endpoint
free and marks the endpoint occupied. Familiarity is updated independently for
every valid observed cell, including a hit endpoint.

Occupied evidence wins immediately. Free evidence clears an occupied cell only
after `grids.sensed.free_observations_to_clear` additional observations.
Dynamic hits expire after `grids.sensed.dynamic_expiry_observations` updates if
they are not refreshed. Conflicts remain in cell metadata.

With `grids.extent_policy: expand`, learned grids grow in 32-cell chunks and
retain prior evidence when a scan crosses an edge. `fixed` clips evidence at
the configured extent.

## Fusion, inflation, and provenance

Known-map planning fuses the immutable static layer with accumulated sensor
evidence. Static occupied remains blocked when sensed free and records a
conflict. Sensed occupied temporarily blocks static free without changing the
map. Static unknown follows the configured unknown policy.

Inflation combines robot radius, safety clearance, localization uncertainty,
and optional turning margin. Dynamic cells receive an additional margin.
Inflated cells retain source provenance plus the `Inflation` flag. The local
hard-safety filter remains authoritative in both operating modes.

Unknown policies are `prohibited`, `high_cost`, `within_sensor_range`, and
`exploration_only`. Planner diagnostics state the policy and source counts.

## Planner capabilities

- `distance`, `density`, `risk`, and `flow` require valid static occupancy.
- `sensor_distance` is an explicitly partial planner. It remains unavailable
  until sensed occupancy exists and includes only policy-permitted cells.
- Region, hallway, trail, and conveyor planners modify an explicit static or
  partial sensed traversability graph with their learned representation.
- Skeleton and highway planners use their learned graph structures directly;
  occupancy may validate local execution but is not their base graph.
- Reactive safety uses the current sensor view and robot footprint regardless
  of map access.

The simulator map is not a robot information source. In hidden-map simulation,
only received range observations populate familiarity and sensed occupancy.
