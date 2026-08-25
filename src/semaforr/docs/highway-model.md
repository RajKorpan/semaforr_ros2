<!-- File overview: This file documents or configures highway model behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/highway-model.md`. -->

# Highway and passage models

High-level exploration maintains a typed passage grid. Each represented cell is
one of `free`, `obstructed`, or `passage`; a passage cell also carries the
stable exploration-candidate ID that produced it. `evidence_count` records
support for the state but is never interpreted as a passage identity. A valid
laser return marks the cells before its endpoint free and the endpoint
obstructed. A maximum-range return marks only free cells. Obstruction takes
precedence over free evidence. An execution-confirmed passage centerline takes
precedence when an obstacle endpoint is quantized into the same coarse cell;
it does not clear obstruction outside the traversed centerline.

The highway learner uses the shared `GridGeometry` transform. Its frame,
resolution, and origin come from `grids.frame_id`, `grids.resolution_m`, and
`grids.highway.origin_{x,y}_m`. Internal sparse lattice coordinates may be
negative. Published cells are rebased into the snapshot's nonnegative local
indices, while `HighwayModel::geometry` preserves their world coordinates.
Consequently no one-metre, zero-origin, or nonnegative-world-coordinate
assumption is part of the representation.

## Named policies

`grids.highway.smoothing_policy` accepts:

- `von_neumann_three_of_four`: compatibility behavior; an unobstructed free
  cell is labeled when at least three of its four von Neumann neighbors are
  free.
- `directional_gap_fill`: adapted behavior; fills one-cell horizontal or
  vertical gaps between existing highway labels.
- `profile`: selects the first policy for the compatibility learning profile
  and the second for the modernized profile.

`grids.highway.component_selection_policy` accepts:

- `most_intersections`: compatibility behavior; retain the connected component
  containing the most nonterminal intersections, then use vertex count and a
  stable component ID as deterministic tie breaks.
- `largest_vertex_count`: adapted behavior; retain the largest graph component,
  with intersection count as a tie break.
- `profile`: selects the first policy for the compatibility learning profile
  and the second for the modernized profile.

The selected names are stored in every highway snapshot, serialized model,
component manifest, and diagnostic event.

## Graph execution data

Highway graph edges retain both the highway identifier and an
`operational_subtrail`: execution-confirmed historical poses between the edge's
intersections. `HighwayPlan` installs this geometry in its `HighwayStep` and
reverses it when the edge is traversed in the opposite direction. Legacy trail
labels remain a fallback for old serialized models. Highway serialization uses
schema version 2 and includes grid geometry, policy names, and edge subtrails.
