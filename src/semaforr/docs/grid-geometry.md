# Grid geometry

All raster layers use `domain::GridGeometry` for coordinate transforms and
extent metadata. The boundary convention is half-open: the minimum corner is
inside a grid and the maximum corner is outside it. Cell centers are returned
in the declared world frame. Negative coordinates and nonzero origins are
normal inputs.

## Operating modes

In map-enabled operation, static occupancy is created with fixed geometry from
declared map bounds, or from obstacle bounds plus `map.inferred_bounds_padding_m`
when `map.bounds_policy` is `infer` or `infer_expandable`. The geometry records
the canonical map path as its identifier and records whether bounds were
declared or inferred. Queries outside static-map geometry are non-traversable.
Static occupancy, map traversability, map crowd fields, and map visualization
use this same transform.

In mapless operation, familiarity, sensed occupancy, and inclusion grids begin
with `grids.mapless.initial_width_m` by
`grids.mapless.initial_height_m`, centered on the first coherent robot pose.
They expand before inserting evidence near or beyond an edge. Expansion is
chunk-aligned and controlled by `grids.expansion.margin_m`,
`increment_cells`, optional maximum dimensions, and `memory_limit_cells`.
Existing sparse evidence is remapped through cell centers, so its world
coordinates do not change. Expansion increments `geometry_revision` and does
not mark newly allocated cells as observed or free.

Runtime transition from mapless to map-enabled operation is intentionally not
supported. Loading a static map requires a restart; this avoids silently
realigning learned evidence. Persisted mapless geometry does not require a map.
Persisted map-derived geometry carries a map identifier for compatibility
validation.

## Layer extent policies

| Layer | Map-enabled | Mapless |
|---|---|---|
| Static occupancy | Fixed map bounds | Absent |
| Sensed occupancy | Expandable sensor overlay | Expandable sparse evidence |
| Familiarity | Expandable learned extent | Expandable sparse evidence |
| Inclusion | Expandable learned extent | Expandable sparse evidence |
| Highway passage | Evidence-local labels/graph | Evidence-local labels/graph |
| Hallways | Sparse segments and orientation bins | Sparse segments and orientation bins |
| Conveyors | Sparse traversal axes | Sparse traversal axes |
| Crowd fields | Fixed map transform when a map is present | Configured raster area |
| Visualization | Source layer geometry | Current represented extent |

Regions, trails, skeletons, highways, doors, hallways, and conveyors remain
sparse geometric or graph models. They are not converted to dense global grids.

## Persistence and diagnostics

Serialized learned-grid snapshots include frame, origin, minimum and maximum,
resolution, cell dimensions, boundary and out-of-bounds policies, fixed versus
expandable state, geometry revision, extent source, and map identifier. Startup
map diagnostics report the static grid extent source and geometry revision.
Expansion refusal due to maximum extent or memory limit is reported as an
explicit error instead of dropping observations silently.
