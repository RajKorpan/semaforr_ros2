<!-- File overview: This file documents or configures snapshot sharing behavior for the maintained architecture and user documentation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `docs/snapshot-sharing.md`. -->

# Snapshot sharing and projection performance

Spatial learners publish immutable, revisioned snapshots. The world model
retains shared ownership of the publication instead of requesting and copying
an unchanged value snapshot on every decision. A learner creates a new
publication only after a meaningful payload change; existing readers keep the
previous publication alive until their `shared_ptr<const SpatialModelUpdate>`
is released. This is publication-level copy-on-write: mutable learner state is
never exposed to consumers.

## Copy inventory

`SpatialLearningCoordinator::lastProjectionMetrics()` and
`cumulativeProjectionMetrics()` report snapshots examined and reused,
representations projected, dense and sparse cells copied or shared, entity
copies, estimated allocations, copied/shared bytes, projection time, lock
duration, and peak projection footprint.

Before this change, each coordinator projection requested twelve value
snapshots. Known and inclusion sparse cells were expanded into full dense
world-model arrays; sensed occupancy was copied as a full dense array. This
made unchanged decision cost depend on grid area and destroyed sparse storage
at the learner/world-model boundary.

The current projection rules are:

| Representation | Normal publication | World-model projection |
|---|---|---|
| Familiarity | Sorted sparse cells and metadata | Shared immutable sparse vectors; dense legacy payload only when explicitly supplied |
| Sensed occupancy | Sorted sparse occupied/free evidence | Shared immutable sparse vector |
| Inclusion | Sorted sparse counts | Shared immutable sparse vector |
| Trails, conveyors, regions, doors, hallways | Immutable entity snapshot | Copied once when that representation's revision changes |
| Skeleton, highways, highway graph | Immutable graph snapshot plus node/edge change set | Copied once on the representation boundary when its revision changes |
| Circumstances | Immutable case snapshot | Copied once when its revision changes |

An unchanged revision is pointer-reused and performs no representation
projection. Projection never holds a learner mutex. Lazy dense cache creation
uses a short cache-local lock and does not mutate the sparse snapshot.

## Sparse and dense access

`FamiliarityGrid`, `SensedOccupancyGrid`, and `SparseCountGrid` provide:

- sparse cell iteration for normal consumers;
- `regionOfInterest(...)` for bounded extraction;
- `valueAt(...)` for indexed queries without dense conversion; and
- `denseCells()` for explicit, lazy dense materialization.

Dense materializations are cached by the immutable snapshot's geometry and
content revision. Consumers that genuinely require a dense graph, such as a
grid traversability builder, construct that derived product at their own
boundary. Visualization, LLE, Out, and spatial-advisor queries use sparse
access directly.

## Change sets

Every publication contains a `RepresentationChangeSet` with its new revision.
Grid changes are represented as contiguous changed-cell ranges. Entity and
graph publications report added, removed, or updated entities and nodes or
edges. Rewriting identical content does not publish a revision or a change
set. Change sets are serialized with learner snapshots for diagnostics and
incremental downstream consumers.

## Benchmark and acceptance gate

`semaforr_snapshot_projection_test` exercises immutable lifetime, lazy dense
caching, region extraction, grid change sets, and 20 m versus 2000 m grid
projection. Its allocation probe and GoogleTest properties record projection
time, allocations, allocation bytes, peak retained projection bytes, and the
unchanged second-decision allocation count. The required invariants are:

- no dense cells are copied for ordinary sparse publications;
- first-projection allocation cost does not grow with an empty grid's area;
- the unchanged second projection copies zero bytes, projects zero
  representations, and performs zero allocations; and
- projection lock duration is zero.

The benchmark is a deterministic regression gate rather than a platform
throughput claim. Use its recorded XML properties when comparing builds on the
same toolchain and host.

## Lifetime and thread safety

Published payloads are const and may outlive their learner or coordinator.
Readers may retain them across decision cycles. A new publication never
mutates an old one. The world model owns aliases to sparse vectors through the
publication's shared control block, preventing dangling views. Dense cache
objects are shared and internally synchronized; their returned vectors are
immutable for the lifetime of the grid snapshot.
