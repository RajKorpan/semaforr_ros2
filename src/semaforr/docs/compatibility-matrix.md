# Behavioral compatibility matrix

This document defines what implementation claims mean. A component name is
not evidence that a particular algorithm is present. The status and behavior
mode recorded here are the authority for publications, experiments, and
regression review. Status is an implementation target, not proof of
experimental equivalence. A `Dissertation-faithful` label is always qualified
by the row's **Exact reproduction?** evidence and by the global fail-closed
compatibility mode. No current whole-system run may be reported as an exact
dissertation reproduction.

## Status vocabulary

- **Dissertation-faithful** means the component is intended to reproduce the
  published algorithm, inputs, lifecycle, ordering, and observable behavior.
  It is verified only when the row names an adequate oracle or acceptance
  suite; component tests alone do not establish publication-level equivalence.
- **Functionally adapted** intentionally uses a different algorithm while
  preserving the component's purpose. It is supported in modernized mode but
  must not be described as an exact reproduction.
- **Engineering extension** is intentional behavior outside the published
  system.
- **Temporary approximation** is a known simplification with a planned
  faithful or explicitly adapted replacement.
- **Unsupported or incomplete** is exposed by code, configuration, messages,
  or documentation but is not operational end to end.

## Operating modes and claim policy

`experiment.behavior_mode` has two values:

- `modernized` is the supported default. It permits the safety envelope,
  deterministic seeded choices, weighted normalized advisors, incremental
  learners, typed plans, registries, caching, and other documented
  adaptations.
- `compatibility` is reserved for exact experimental reproduction. Startup
  currently rejects it with the unresolved blocker list. This prevents an
  experiment from claiming compatibility merely because historically named
  components were enabled.

The target is to make compatibility mode operational without weakening the
hard command-safety boundary. Safety interventions must be recorded so they
can be separated from cognitive decisions during analysis.

Every reported run must include the behavior mode, ablation profile, random
seed, configuration fingerprint, component manifest, source revision, map,
target sequence, and test-suite revision. Historical profile names such as
`original`, `doors`, or `highway` identify experimental feature sets only;
they are not fidelity claims.

## Representations and learners

| Component | Status | Published algorithm and lifecycle | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Decision path/history | Dissertation-faithful | Record the completed decision point, action, pose, and view after execution | `CompletedPath` preserves selected and executed actions, start/final poses, historical views, stable IDs, target/task boundaries, terminal outcomes, interruptions, and partial motion | Yes; IDs and detailed failure states are an engineering extension | Failed evidence remains visible but is never labeled successful traversal | Yes at representation level | Retain the additional outcome metadata in both profiles |
| Trail / `TrailLearner` | Dissertation-faithful in `chapter3_compatibility`; functionally adapted in `modernized` | At target end, work backward and select the earliest historical point that could sense the current marker | Compatibility uses backward laser-view visibility and preserves evidence/subtrails; modernized retains execution-confirmed distance sampling | Modernized difference is intentional | Compatibility removes visible detours; modernized favors low update cost | Algorithm-level yes; whole-system reproduction remains gated | Maintain hand-computed visibility fixtures |
| Conveyor / `ConveyorLearner` | Dissertation-faithful in `chapter3_compatibility`; functionally adapted in `modernized` | Count successful trail occupancy in a separate frequency grid at target end | Compatibility rasterizes successful trails with direction, normalization, decay policy, and representation-local bounds; modernized retains directed segment flows | Modernized difference is intentional | Compatibility represents repeated successful cell use; failures add no frequency | Algorithm-level yes; whole-system reproduction remains gated | Maintain frequency and planner-objective fixtures |
| Region / `RegionLearner` | Dissertation-faithful in `chapter3_compatibility`; functionally adapted in `modernized` | Per-decision minimum-range circle; deterministically reconcile overlapping candidates after a target | Compatibility retains the supporting observation, recomputes overlap geometry from visible endpoints, assigns stable IDs, and publishes 360-degree visibility; modernized retains incremental spatial-hash clustering | Modernized difference is intentional | Profiles differ in identity and timing, by design | Algorithm-level yes; whole-system reproduction remains gated | Maintain overlap and visibility fixtures |
| Exit | Dissertation-faithful in `chapter3_compatibility` | Traveled location on a region circumference, accumulated at target end | First-class `RegionExit` records region, point, direction, traversal count, supporting paths, and confidence; repeated angular evidence merges deterministically | No material deviation | Downstream doors and transitions consume successful travel evidence | Algorithm-level yes | Add larger multi-target fixtures |
| Door / `DoorExitLearner` | Dissertation-faithful in `chapter3_compatibility`; functionally adapted sensor feature in `modernized` | Order exits, apply the region-dependent angular-nearness formula, and group consecutive exits into revisable arcs | Compatibility derives typed door arcs from exits; scan discontinuities are published separately as `SensorOpening` and are never labeled learned doors | Sensor feature is intentional | Diagnostics can distinguish experienced doors from instantaneous openings | Algorithm-level yes; whole-system reproduction remains gated | Maintain circular-order and later-target merge fixtures |
| Hallway / `HallwayLearner` | Dissertation-faithful in `chapter3_compatibility`; functionally adapted in `modernized` | Four direction classes, similarity statistics, exceptional parent pairs, visible children, heatmap, 70% smoothing, components, mutual-visibility merge, width/extent | Compatibility implements every inference stage while spatial buckets bound parent comparisons; modernized retains orientation-bin centerlines | Spatial indexing is an intentional efficiency adaptation | Compatibility publishes connected area and width without requiring quadratic global scans | Algorithm-level yes; exact historical tie behavior may differ | Maintain scenario fixtures and benchmark indexed candidate reduction |
| Barrier / `BarrierLearner` | Engineering extension | Not a Chapter 3 affordance | Learns barrier segments for modern safety and planning support | Yes | Adds knowledge unavailable to the published controller | No | Retain only in modernized mode and manifest it explicitly |
| Optional static map | Engineering extension | Mapless learning is primary; known-map planners are retained as baselines | Immutable, provenance-tagged wall geometry and occupancy loaded only in `map_enabled` mode and kept separate from learned models | Yes | Enables controlled known-map comparisons without changing mapless learning | Not applicable to mapless reproduction | Retain as an explicitly manifested experimental capability |
| Known grid / `KnownGridLearner` | Dissertation-faithful | Count cells visible along laser rays, at most once per decision | Sparse ray-cell observation counts deduplicated across the complete decision scan before one atomic observation update | Yes, storage optimized | Equivalent familiarity intent with different storage and beam-density-invariant counts | Algorithm-level yes | Maintain single-ray, overlapping-ray, repeated-observation, maximum-range, and invalid-ray fixtures; Out owns a separate target-local recent-window projection |
| Inclusion grid / `InclusionGridLearner` | Dissertation-faithful with incremental extension | Mark cells included in regions and region-connecting subtrails; update during LLE | Projects learned region area and region-skeleton supporting subtrails; successful LLE translations add provisional subtrail inclusion | Intentional incremental publication only | Inclusion growth can request replanning without treating observation locations as inclusion | Algorithm-level yes | Add larger trace-replay fixtures |
| Passage grid | Dissertation-faithful selected semantics in HLE compatibility mode | Free/obstructed/passage-number labels updated locally during HLE | Sparse typed evidence; passage cells carry stable passage and candidate IDs, lifecycle completion state, and separate evidence counts; snapshots restore with geometry/revision validation | Coarse-cell traversal precedence is an intentional reconciliation rule | Downstream code can distinguish obstruction, passage identity, candidate provenance, and completion | Algorithm-level with deterministic grid fixtures | Add recorded-environment trace fixtures |
| Skeleton / `PassageSkeletonLearner` | Dissertation-faithful in `chapter3_compatibility`; functionally adapted graph retained separately | Region nodes, direct region-transition edges, shortest supporting subtrail labels, and stable visibility | Compatibility publishes typed region nodes/edges and component IDs; the former spaced graph is explicitly `sampled_path_*` in modernized mode | Modernized graph is intentional | SkeletonPlan operationalizes edge subtrails; HighwayPlan connects through region nodes | Algorithm-level yes; whole-system reproduction remains gated | Expand surrogate and disconnected-component fixtures |
| Region visibility | Dissertation-faithful in `chapter3_compatibility` | 360 one-degree maximum visibility with source/end points for every region | Each region publishes immutable one-degree bins with evidence and revision; LLE and Enforcer query them | No material deviation | Consumers no longer substitute region centers for visibility | Algorithm-level yes | Add occlusion-specific visibility fixtures |
| Highway grid / `HighwayLearner` | Dissertation-faithful policies in `chapter3_compatibility`; functionally adapted in `modernized` | Interpolate exploration path, 3-of-4 von Neumann smoothing, minimum row/column extents, intersections, spurs, spreading activation, component with most intersections | Shared configurable geometry and sparse signed lattice; compatibility selects 3-of-4 smoothing and most-intersections; modernized selects named directional gap fill and largest-vertex adaptations | Modernized differences are intentional | Compatibility fixes cell and component selection; modernized retains incremental efficiency | Algorithm-level pending published-map oracles | Add published-map extraction fixtures |
| Highway and intersection types | Dissertation-faithful | Long horizontal/vertical extents and their overlap/terminal access points | First-class typed cells, axes, endpoints, intersections, and revisions | Yes | Storage is stronger without changing intent | Pending oracle tests | Add published-map extraction fixtures |
| Highway graph | Dissertation-faithful data contract with adapted deterministic extraction | Intersection nodes; highway edges labeled with path-derived subtrails and centroid distance | Typed graph edges retain execution-confirmed operational subtrails, highway/trail IDs, distance, shared geometry, and schema version 2 | Extraction optimizations are intentional | Highway steps are directly operationalizable and reverse their subtrail when traversed backward | Algorithm-level pending published-map oracles | Add published-map extraction fixtures |
| Setting normalization | Dissertation-faithful | Robot-centered, east-facing discretized freespace grid | Robot-centered heading-normalized `NormalizedSetting` | Yes | Equivalent purpose with configurable geometry | Pending oracle tests | Freeze compatibility parameters and add canonical setting fixtures |
| Circumstance / `CircumstanceLearner` | Functionally adapted | Offline spectral clustering, neural classifier, confidence rejection, then online assignment and periodic reclustering | Deterministic L1 seed grouping, centroid updates, confidence and evidence gates | Yes, to remove offline model/runtime dependency | Cluster identities and assignments cannot reproduce published cases | No | Provide a compatibility model loader and published clustering/classification pipeline |
| Crowd field / learner | Engineering extension | Social planners predate the Chapter 3 model; no unified learned field in these chapters | Learns versioned density, risk, and flow snapshots | Yes | Adds social learning and new objectives | No | Retain in modernized mode and disable through the social master switch for compatibility |

## Exploration, tiers, rules, and reactive planners

| Component | Status | Published algorithm and lifecycle | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Navigation phases | Engineering extension | HLE occurs before target navigation, but phases are implicit | Explicit initial exploration, target navigation, and mission complete phases | Yes | Makes lifecycle testable without changing intended ordering | Yes at lifecycle level | Retain in both modes |
| HLE | Dissertation-faithful selected specification under explicit `compatibility`; functionally adapted under `modernized` | Fixed left/right 41-ray bundles, passage length/width and large-room tests, clear/reachable cue validation, at-most-one passage intersection, segment-distance and interval-overlap merging, persistent global geometry, extension and explicit termination | Compatibility implements those cue, validation, pursuit, numbered-grid, diagnostic, and replay contracts; modernized retains contiguous clear bundles and endpoint hashing | Modernized difference is intentional | Compatibility headings survive robot motion and terminal reasons/grid states are explicit; time and decision budgets are retained safety extensions | Algorithm-level with deterministic scan/state oracles; whole-system reproduction remains gated | Add recorded-environment trace fixtures when available |
| LLE | Dissertation-faithful in `compatibility`; functionally adapted in `modernized` | Trigger on no/finished plan, use HLE cues, path views, region R* visibility, inclusion grid, 20 waypoints, random closest-bin fallback, 10% replanning | Compatibility uses only no-plan/completed-failed-plan triggers and seeded random choice in the closest distance bin; modernized may add stalled-history recovery and deterministic fallback | The modernized stall trigger is an explicit extension | Compatibility reproduces published trigger and candidate semantics; seeded randomness makes runs replayable | Algorithm-level yes | Add end-to-end trace corpus |
| Hard safety filter | Engineering extension | Obstacle avoidance is cognitive Tier 1 | Independent sensor, finite-command, clearance, and action validity filter | Yes | May veto an action the published cognitive system would select | No bit-for-bit; safety interventions separable | Retain and record interventions in both modes |
| Command safety envelope | Engineering extension | Platform execution details not part of tier reasoning | Velocity, acceleration, freshness, bounds, timeout, and final finite checks | Yes | Safer platform behavior and possibly different trajectories | No bit-for-bit | Retain; require intervention-free runs for strict behavioral comparisons |
| Tier ordering | Functionally adapted | Ordered Tier 1; Tier 2 only if no plan; return to Tier 1; Tier 3 only after Tier 1 cannot decide | Hard safety bounds candidates; plan operationalization runs; registered mandatory Tier-1 rules (including Victory) are checked before LLE and other reactive planners; vetoes and Tier 3 follow | Hard safety and eager plan availability are engineering adaptations | LLE no longer preempts Victory or a successful Enforcer waypoint | Behavioral reproduction with documented safety adaptation | Continue expanding ordering scenarios |
| Victory | Dissertation-faithful | Turn or move toward sensed unobstructed target | Direct visibility, turn, move, or pause within tolerance | Yes | Typed actions and tolerance are platform adaptations | Pending scenario oracle | Add published action-repertoire fixtures |
| AvoidObstacles | Functionally adapted | Veto moves that approach within epsilon of sensed obstacle | Swept-corridor laser clearance plus hard safety | Yes, for footprint safety | More conservative than the published forward-distance check | No | Provide published cognitive rule in compatibility mode while retaining hard safety separately |
| NotOpposite | Dissertation-faithful | Avoid return to the last two orientations | Veto turns whose predicted heading matches the final orientations of execution-confirmed terminal records | Yes | Prematurely selected turns cannot affect the orientation history | Algorithm-level | Retain execution-history oracle tests |
| Enforcer | Functionally adapted | Tier-1 action selector with plan-specific operationalization | Dispatches explicit grid plans to `GridPlanEnforcer` and Skeleton/Highway plans to `ModelPlanEnforcer`; both directly mandate through a shared predictive local-action evaluator before reactive planners and Tier 3 | Safety/action typing and explicit plan-family split are intentional | Plan following retains Tier-1 priority while grid and learned-model explanations remain distinct | Algorithm-level, subject to typed action repertoire | Retain ordering, shortcut, repair, and invalidation scenarios |
| Thru | Functionally adapted | After Victory/Enforcer fail, follow clearer side cue in 0.8 m steps until goal, interruption, or budget | Stateful clearer-side pursuit before ordinary Tier-1 arbitration | No for ordering; yes for interface | Can assume control at different times | No | Move trigger behind Victory/Enforcer in compatibility coordinator |
| Behind | Dissertation-faithful with typed availability | If nearby region/waypoint/target was unseen twice, mandate available 90-degree right then left turn | Uses `1.5 m + containing-region radius`, requires absence from current and previous executed views, prefers viable right then left, and suppresses only an execution-confirmed prior quarter turn | Typed availability is intentional | Region geometry and fallback direction affect the trigger exactly as declared | Algorithm-level | Add recorded-view traces when available |
| Out | Dissertation-faithful with nonblocking execution | Recent `10+n/50` known grid, four 90-degree surveys, then prepend reverse subtrail for Enforcer | Evaluates familiarity only at recent executed poses, preserves four-turn survey, constructs an execution-confirmed reverse subtrail, prepends it to the mission plan, and returns to Enforcer | Nonblocking state machine is intentional | Recovery no longer bypasses hierarchical execution | Algorithm-level | Add long recorded confinement traces |
| Forward | Dissertation-faithful with typed footprint cells | Visited footprint grid updated by Enforcer; veto projected rotations; clear if all rotations vetoed | Successful execution-confirmed plan motion populates a footprint-sized cell set; projected rotation lookahead is vetoed on visited cells and the set clears if every rotation is vetoed | Typed cell coordinates are intentional | Failed or merely selected actions cannot fabricate visited space | Algorithm-level | Retain negative-coordinate and task-boundary tests |
| Precedent | Functionally adapted | Circumstance/distance/angle case; published confidence and accuracy thresholds | Same evidence equations plus minimum-evidence gates on adapted circumstances/trails | Yes for evidence safety | More conservative activation and different case identities | No | Use faithful trail/circumstance sources in compatibility mode |

## Global planners

| Planner | Status | Published algorithm | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Distance / A* | Functionally adapted | A* on a metric occupancy cost graph | Dijkstra over explicit static-plus-sensed traversability; unknown-space policy, inflation, and bounds are explicit; no familiarity or skeleton fallback exists | Dijkstra and safety layers are intentional | Equal-cost tie behavior and routes may differ from the published A* implementation | No published-map oracle yet | Add published scenario/path oracles before changing status |
| Crowd density | Functionally adapted | Crowd-sensitive occupancy cost graph | Static-map traversability with a versioned learned-density edge objective | Yes | Uses the unified modern crowd-field estimator | No | Retain modernized; add legacy objective adapter only if reproduction requires it |
| Crowd risk | Functionally adapted | Penalize risky crowd encounters | Static-map traversability with learned encounter-risk edge cost | Yes | Risk estimator and normalization differ | No | Retain modernized |
| Crowd flow | Functionally adapted | Penalize travel against crowd flow | Static-map traversability with learned opposing-flow edge cost | Yes | Flow source and normalization differ | No | Retain modernized |
| RegionPlan | Temporary approximation | Published Table 3.1 weights over an occupancy graph and learned regions/doors/exits | Dijkstra over explicit static or partial sensed traversability with region and door/exit edge costs | Base occupancy correction is complete; objective calibration is not verified | Routes no longer depend on familiarity, but published weights/ties may differ | No published-route oracle | Calibrate objective weights against published fixtures |
| HallwayPlan | Temporary approximation | Published hallway edge weights over occupancy graph | Dijkstra over explicit static or partial sensed traversability with hallway-proximity cost | Base occupancy correction is complete; geometry/profile remains selectable | Compatibility and modernized hallway geometry can yield different routes | No published-route oracle | Validate compatibility hallway routes and objective scale |
| TrailPlan | Temporary approximation | Published trail-marker weights over occupancy graph | Dijkstra over explicit static or partial sensed traversability with trail proximity | Base occupancy correction is complete | The selected trail profile changes preferred routes | No published-route oracle | Validate compatibility trail routes and objective scale |
| ConveyorPlan | Temporary approximation | Published conveyor-cell frequency weights over occupancy graph | Dijkstra over explicit static or partial sensed traversability; uses the conveyor frequency grid when available and segment fallback in modernized mode | Grid support is intentional; segment fallback is adapted | Compatibility and modernized profiles can choose different routes | No published-route oracle | Add published frequency-route fixtures and exclude fallback in compatibility mode |
| SkeletonPlan | Dissertation-faithful with typed execution | Dijkstra/A* over region graph using visibility surrogates and supporting subtrails | Uses only region-skeleton nodes and direct transition edges; emits typed region, skeleton-transition, and final-target steps | Typed schema and deterministic A* are intentional engineering choices | Sampled path geometry can no longer be mislabeled as SkeletonPlan | Algorithm-level | Expand surrogate-selection fixtures |
| HighwayPlan | Functionally adapted | Skeleton connection, shortest highway graph path, skeleton connection; compare with SkeletonPlan | Combines the region skeleton with highway intersections and operational edge subtrails; emits explicit entry, highway, intersection, exit, and final-target steps | Yes at graph-search and typed-interface level | Route may differ through deterministic attachment rules | Algorithm-level | Expand multiple-entry and unusable-connection repairs |
| Tier-2 range voting | Dissertation-faithful target with engineering tie controls | Each objective evaluates every plan, normalizes costs to `[0,10]`, and minimizes summed score | Same range-vote structure; `seeded_exact` records and randomly resolves exact ties, while modernized deterministic mode uses planner-name order | Seeded reproducibility and deterministic option are intentional | Compatibility-target ties are replayable; objective implementations can still differ | Pending numeric and route oracle tests | Keep both policies and validate the compatibility-target matrix against published examples |
| Other selection policies | Engineering extension | Not described | Single, normalized minimum, Pareto-then-vote, shortest-valid | Yes | Enables new experiments | No | Modernized mode only |
| Plan cache and revision invalidation | Engineering extension | Not described | Caches by task and start/target surrogates plus exact declared representation and policy revisions | Yes | Relevant mutations invalidate precisely; unrelated layers remain cached | Potentially | Implemented and covered by exact-dependency regression tests |

## Tier-3 advisors and voting

Unless a row says otherwise, the listed advisor preserves the published
rationale but uses anticipated poses, configurable weights, and per-decision
signed normalization instead of unweighted `[0,10]` comments.

| Advisor | Status | Published rationale | Current consequence / deviation | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|
| BigStep | Functionally adapted | Prefer a long step | Scores safe forward magnitude; turns receive a partial utility | No | Add published comment oracle |
| ElbowRoom | Functionally adapted | Stay far from obstacles | Scores predicted nearest laser endpoint | No | Freeze published metric and scale |
| Novelty | Functionally adapted | Avoid locations visited on current target | Uses target-tagged navigation history | No | Add published distance/comment mapping |
| GoAround | Functionally adapted | Turn away from nearby obstacle | Maximizes heading separation from nearest scan endpoint | No | Add published lookahead and scale |
| Greedy | Functionally adapted | Approach target or active plan step | Uses the active Enforcer objective, then waypoint/target fallback; normalized weighted scoring remains the modernized default | No published comment oracle | Validate the compatibility `[0,10]` transform against published examples |
| Curiosity | Functionally adapted | Visit locations never visited in experiment | Uses all navigation history | No | Compatibility comment scale |
| Enfilade | Functionally adapted | Return toward recent locations | Uses last ten sufficiently distinct positions | No | Match published history window and score |
| VisualScan | Functionally adapted | Rotate toward orientations with least prior view overlap | Samples unseen angular coverage in nearby history | No | Match published overlap calculation |
| Convey | Functionally adapted | Approach frequent distant conveyors | Uses frequency-aware conveyor segments; the compatibility conveyor grid is consumed by ConveyorPlan but not directly by this advisor | No | Add a compatibility advisor path over conveyor cells if required by an oracle |
| Enter | Functionally adapted | Enter target/plan-step region | Uses the active Enforcer plan-step objective, then waypoint/target fallback, with the selected region profile | No published comment oracle | Validate score scale against published examples |
| Exit | Functionally adapted | Leave a region without target/plan step | Uses the active Enforcer plan-step objective and current region membership | No published comment oracle | Validate score scale against published examples |
| Trailer | Functionally adapted | Follow trail segment that approaches target/plan step | Selects the trail segment and endpoint that improve the active local objective | No published comment oracle | Validate score scale with compatibility trails |
| Unlikely | Temporary approximation | Avoid dead-end regions | Infers low-door regions instead of skeleton degree | No | Use region skeleton degree |
| Access | Functionally adapted | Approach regions with many doors | Counts the currently projected door segments near learned regions; compatibility mode supplies exit-derived doors, modernized mode may supply sensor openings | No | Require exit-derived door provenance for a future compatibility runtime |
| Crossroads | Functionally adapted | Approach hallways with many overlaps | Computes centerline overlap dynamically | No | Use faithful hallway aggregate labels |
| Follow | Functionally adapted | Follow target-relevant hallway | Selects the hallway nearest the active local plan objective and follows its useful endpoint | No published comment oracle | Validate with compatibility hallway aggregates |
| LeastAngle | Functionally adapted | Leave a region through the skeleton branch best aligned to target/plan step | Uses the nearest projected region-skeleton node, its adjacent edges, and the active local objective | No published comment oracle | Validate current-region surrogate and score scale against published examples |
| SpatialLearner | Functionally adapted | Prefer locations absent from regions and high conveyors | Uses inclusion, adapted regions, and segment flows | No | Rebase on faithful representations |
| Stay | Functionally adapted | Remain in current hallway | Uses distance to nearest centerline | No | Use faithful hallway area membership |
| Weighted signed voting | Engineering extension | Unweighted `[0,10]` range voting with random tie break | Advisor utilities normalize to `[-1,1]`, receive weights, and use seeded tolerance ties | Yes | Changes coalition strength and decisions | No | Add unweighted `[0,10]` compatibility policy |

The remaining registered advisors are modernized extensions and are classified
individually here so a manifest can never silently imply that they belonged to
the published Tier-3 catalog.

| Advisor | Status | Current rationale | Behavioral consequence | Compatibility policy |
|---|---|---|---|---|
| Random | Engineering extension | Leave all candidates tied for seeded baseline selection | Creates a reproducible random-control condition | Disable unless the compatibility experiment explicitly requires a random baseline |
| GoalProgress | Engineering extension | Maximize predicted progress to waypoint or target over every action | Adds a strong generic navigation vote and partly assumes Enforcer's local role | Disable in compatibility mode |
| GoalProgressLinear | Engineering extension | Score only forward progress | Adds a move-specific target vote | Disable in compatibility mode |
| Clearance | Engineering extension | Maximize anticipated clearance over every action | Adds a continuous safety preference beyond AvoidObstacles | Disable in compatibility mode |
| ClearanceRotation | Engineering extension | Score rotation clearance only | Biases turns independently of the published Advisors | Disable in compatibility mode |
| Exploration | Engineering extension | Prefer less-visited anticipated locations | Overlaps Novelty and Curiosity with different scoring | Disable in compatibility mode |
| AvoidRevisit | Engineering extension | Prefer distance from all recorded positions | Adds a global anti-revisit vote | Disable in compatibility mode |
| PreferRegions | Engineering extension | Prefer anticipated poses deeper in learned regions | Adds a generic region vote not listed in Table 4.3 | Disable in compatibility mode |
| PreferHighways | Engineering extension | Prefer anticipated poses near learned highway nodes | Adds local highway preference to Tier 3 | Disable in compatibility mode |
| PreferDoors | Engineering extension | Prefer anticipated poses near learned openings | Adds a direct door preference distinct from Access | Disable in compatibility mode |
| FollowTrails | Engineering extension | Prefer anticipated poses near any trail marker | Adds generic trail attraction distinct from Trailer | Disable in compatibility mode |
| SocialNavigation | Engineering extension | Use live pedestrian prediction for separation and encounter behavior | Changes action selection from current social observations | Disable with the social master switch in compatibility mode |
| CrowdAvoid | Engineering extension | Avoid cells with learned crowd density | Adds a learned social-field vote | Disable with the social master switch in compatibility mode |
| RiskAvoid | Engineering extension | Avoid cells with learned encounter risk | Adds a learned risk-field vote | Disable with the social master switch in compatibility mode |
| FlowFollow | Engineering extension | Prefer alignment with learned pedestrian flow | Adds a learned directional social vote | Disable with the social master switch in compatibility mode |

## Explanations

The unified request-driven implementation is described in
[`explanations.md`](explanations.md). Its current status is functionally
adapted: compatibility scoring preserves
unweighted comments and exact ties, while the modernized mode retains weighted
scoring. Both modes publish the same replayable trace. Decision and plan
questions, counterfactuals, immutable hypothetical-evaluator hooks, confidence,
recorded alternatives, route comparison, typed route language, provenance, and
stable historical lookup are operational in the single `why` package.

| Component | Status | Published algorithm | Current implementation | Intentional? | Behavioral consequence | Exact reproduction? | Planned resolution |
|---|---|---|---|---|---|---|---|
| Unified Why explanations | Functionally adapted | Question-driven Tier-1, Tier-3, exploration, confidence, counterfactual, hypothetical, plan, alternative, comparison, and route responses | One `why` package routes questions over a shared historical trace and returns structured evidence plus prose | Yes; ROS request/response messages and immutable evaluator callbacks are engineering adaptations | Action and plan reasoning can be combined and replayed without mutating navigation | Algorithm-level; language is intentionally concise | Maintain golden wording and numeric confidence fixtures |
| Tier-1 explanation templates | Functionally adapted | Component-specific action, confidence, and alternative-action phrases | Producer reason codes and typed plan context distinguish safety, cognitive constraints, both Enforcer modes, reactive control, exploration, and precedent | Yes | Physical safety, cognitive rejection, viability, and lower preference remain distinct | Yes at semantic-category level | Expand wording when new Tier-1 components register |
| Tier-3 rationale explanation | Dissertation-faithful in compatibility scoring; functionally adapted in weighted scoring | Support/opposition clauses from relative advisor support with agreement and standardized support confidence | Retains raw/transformed comments, statistics, relative support, weight, contribution, totals, tie policy, seed, Gini agreement, and standardized support | Weighted scoring is intentional | Both scoring modes are replayable and distinct | Yes in compatibility mode | Preserve numeric reference tests |
| HLE explanation | Functionally adapted | Exploration action/confidence/alternative templates | Question-driven explanation consumes HLE policy, candidate lifecycle, passage state, and reason codes from the common trace | Structured diagnostics and safeguards are intentional | HLE choices share the target-navigation explanation interface | Algorithm-level | Add richer cue-comparison prose when traces expose competing scores |
| Why plan comparison | Functionally adapted | Compare routes under participating objectives and explain preference, alternative, and confidence | Recorded alternatives use the exact Tier-2 matrix; supplied routes require an immutable evaluator and otherwise report unavailable | Yes | Tradeoffs are reported without invented costs or silent replanning | Algorithm-level with an evaluator configured | Keep experiment-specific human-route evaluators outside Why |
| Route description | Functionally adapted | Typed-step route narration plus egocentric angle and distance translation | Simplifies geometric paths into turn segments and preserves region, subtrail, highway entry/travel/intersection/exit, exact turn, and exact distance fields | Yes | Routes are concise while quantitative geometry remains verifiable | Algorithm-level | Calibrate distance-language thresholds per experiment if required |
| DecisionRecord diagnostics | Engineering extension | Why reads an internal knowledge store | Versioned record includes actions, predictions, Tier-1 stages, veto semantics, Tier-3 inputs, confidence, candidate plans, typed steps, revisions, provenance, and lifecycle | Yes | Strong auditability and stable historical linking | Not applicable as an extension | Version the message when trace semantics change |

## Compatibility-mode exit criteria

Compatibility mode becomes runnable only when all of the following are true:

1. Every temporary approximation required by a selected compatibility profile
   is either dissertation-faithful or explicitly excluded by validation.
2. Tier execution and action attribution match the published decision cycle.
3. Cost-graph planners use a validated occupancy graph and published
   affordance costs.
4. HLE, LLE, spatial update schedules, and target boundaries pass algorithm
   oracle fixtures.
5. Why answers the Chapter 5 question set from recorded explanation traces.
6. Compatibility tests pass without an unrecorded hard-safety intervention.
7. A replay report includes behavior mode, profile, seed, fingerprint,
   manifest, source revision, and exact-match/divergence annotations.
